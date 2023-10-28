#include <Wire.h>
#include <RTCZero.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <WiFiNINA.h>
#include <ThingSpeak.h>

MAX30105 particleSensor;

const byte RATE_SIZE = 4;  // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];     // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0;  // Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;
long irValue;
int Sms = 0;

float temperature;
float temperatureF;
float latitude;
float longitude;

int year;
int month;
int day;
int hour;
int minute;
int second;

int WhiteLed = 9;
int YellowLed = 10;

// Define the interval for checking status (in milliseconds)
const unsigned long interval = 5000;

// Variable to store the last time the function was called
unsigned long previousMillis = 0;

// Use Serial1 for the GPS module (hardware serial)
#define GPS_SERIAL Serial1

// Use Serial2 for the GSM module (hardware serial)
#define GSM_SERIAL Serial1

RTCZero rtc;


enum State {
  IDLE,
  MEASURE_TEMPERATURE,
  MEASURE_BP,
  WAIT_FOR_GPS
};

State currentState = IDLE;
unsigned long measurementStartTime;

// ThingSpeak settings
// ThingSpeak settings
char thingSpeakApiKey[] = "NGMB5BNSDUIWDED";  // ThingSpeak API Key
unsigned long channelID = 271982733;             // Your ThingSpeak Channel ID
const char* server = "api.thingspeak.com";

// WiFi settings
char ssid[] = "Mwarandus Lab";
char pass[] = "password";

// NTP settings
IPAddress timeServerIP;
const char* ntpServerName = "pool.ntp.org";
const int timeZone = 3;

WiFiClient client;

void setup() {
  Serial.begin(115200);    // Start the serial communication with the computer
  GPS_SERIAL.begin(9600);  // Start the hardware serial communication with the GPS module
  GSM_SERIAL.begin(9600);  // Start the hardware serial communication with the GSM module

  rtc.begin();

  connectWiFi();

  // Set the NTP server IP
  WiFi.hostByName(ntpServerName, timeServerIP);

  // Sync the RTC time with the NTP server
  rtc.begin();
  rtc.setEpoch(syncNTP());

  pinMode(WhiteLed, OUTPUT);
  pinMode(YellowLed, OUTPUT);

  digitalWrite(WhiteLed, LOW);
  digitalWrite(YellowLed, LOW);

  // Initialize the GSM module
  GSM_SERIAL.println("AT");
  delay(1000);
  while (GSM_SERIAL.available()) {
    Serial.write(GSM_SERIAL.read());
  }

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 was not found. Please check wiring/power. ");
    while (1)
      ;
  }

  particleSensor.setup();                     // Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A);  // Turn Red LED to low to indicate the sensor is running
  particleSensor.setPulseAmplitudeGreen(0);   // Turn off Green LED

  ThingSpeak.begin(client);  // Initialize ThingSpeak client
}

void loop() {
  digitalWrite(YellowLed, LOW);

  switch (currentState) {
    case IDLE:
      checkFingerDetection();
      break;

    case MEASURE_TEMPERATURE:
      measureTemperature();
      break;

    case MEASURE_BP:
      measureBP();
      break;

    case WAIT_FOR_GPS:
      // Wait for 10 seconds before measuring and sending GPS data
      if (millis() - measurementStartTime >= 5000) {
        GpsData();
      }
      break;
  }
}

void checkFingerDetection() {
  irValue = particleSensor.getIR();

  if (irValue > 50000) {
    currentState = MEASURE_TEMPERATURE;
    measurementStartTime = millis();
  }// Add an additional condition to transition to MEASURE_TEMPERATURE
}

void measureTemperature() {
  particleSensor.setup(0);  // Configure sensor. Turn off LEDs
  particleSensor.enableDIETEMPRDY();
  temperature = particleSensor.readTemperature();
  Serial.print("temperatureC = ");
  Serial.print(temperature, 2);

  temperatureF = particleSensor.readTemperatureF();
  Serial.print("   temperatureF = ");
  Serial.print(temperatureF, 2);

  sendHealthData(beatsPerMinute, beatAvg, temperature, temperatureF, latitude, longitude);

  Serial.println();

  // Check if 10 seconds have passed
  if (millis() - measurementStartTime >= 10000) {
    // Before transitioning to MEASURE_BP, re-initialize the sensor settings
    particleSensor.setup();                     // Configure sensor with default settings
    particleSensor.setPulseAmplitudeRed(0x0A);  // Turn Red LED to low to indicate the sensor is running
    particleSensor.setPulseAmplitudeGreen(0);   // Turn off Green LED

    currentState = MEASURE_BP;
    digitalWrite(WhiteLed, HIGH);
    delay(2000);
    digitalWrite(WhiteLed, LOW);
  }
}

void measureBP() {
  irValue = particleSensor.getIR();

  if (irValue > 50000) {
    if (checkForBeat(irValue) == true) {
      // We sensed a beat!
      long delta = millis() - lastBeat;
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (byte)beatsPerMinute;  // Store this reading in the array
        rateSpot %= RATE_SIZE;                     // Wrap variable

        // Take the average of readings
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;

        if (irValue < 50000)
          Serial.print(" No finger?");

        Serial.println();
        if (beatAvg > 65 && beatAvg < 80 && beatsPerMinute > 65) {
          // Send the BPM data via GSM
          sendHealthData(beatsPerMinute, beatAvg, temperature, temperatureF, latitude, longitude);
          Serial.print("IR=");
          Serial.print(irValue);
          Serial.print(", BPM=");
          Serial.print(beatsPerMinute);
          Serial.print(", Avg BPM=");
          Serial.print(beatAvg);
          delay(1000);

          // Reset to IDLE state
          currentState = WAIT_FOR_GPS;
          digitalWrite(WhiteLed, HIGH);
          delay(2000);
          digitalWrite(WhiteLed, LOW);
          measurementStartTime = millis();
        }
      }
    }
  }
}
void sendHealthData(float beatsPerMinute, float beatAvg, float temperature, float temperatureF, float latitude, float longitude) {
  if (beatsPerMinute > 0 && beatAvg > 0 && temperature > 0 && temperatureF > 0 && Sms == 0 /*&& isValidLocation(latitude, longitude)*/) {
    digitalWrite(YellowLed, HIGH);
    digitalWrite(WhiteLed, HIGH);

    // // Update ThingSpeak fields
    ThingSpeak.setField(1, temperature);
    ThingSpeak.setField(2, temperatureF);
    ThingSpeak.setField(3, beatsPerMinute);
    ThingSpeak.setField(4, beatAvg);
    ThingSpeak.setField(5, latitude);
    ThingSpeak.setField(6, longitude);

    // Send data to ThingSpeak
    int status = ThingSpeak.writeFields(channelID, thingSpeakApiKey);

    if (status == 200) {
      Serial.println("ThingSpeak update successful.");
    } else {
      Serial.println("Error updating ThingSpeak. HTTP status code: " + String(status));
    }

    GSM_SERIAL.println("AT+CMGF=1");
    GSM_SERIAL.println("AT+CMGS=\"+254748613509\"");
    delay(1000);
    GSM_SERIAL.println("Health Data  ");
    GSM_SERIAL.print("BPM: ");
    GSM_SERIAL.println(beatsPerMinute, 0);
    GSM_SERIAL.print("Avg BPM: ");
    GSM_SERIAL.println(beatAvg, 0);
    GSM_SERIAL.print("Temperature (Celsius): ");
    GSM_SERIAL.println(temperature, 1);
    GSM_SERIAL.print("Temperature (Fahrenheit): ");
    GSM_SERIAL.println(temperatureF, 1);
    GSM_SERIAL.print("Location: https://www.google.com/maps?q=");
    GSM_SERIAL.println(latitude, 6);
    GSM_SERIAL.println(longitude, 6);
    year = rtc.getYear();
    month = rtc.getMonth();
    day = rtc.getDay();
    hour = rtc.getHours();
    minute = rtc.getMinutes();
    second = rtc.getSeconds();
    Serial.print("Current Time: ");
    Serial.print(day);
    Serial.print("/");
    Serial.print(month);
    Serial.print("/");
    Serial.print(year);
    Serial.print(" ");
    Serial.print(hour);
    Serial.print(":");
    Serial.print(minute);
    Serial.print(":");
    Serial.println(second);
    delay(5000);
    currentState = IDLE;
  }
}
void GpsData() {
  if (GPS_SERIAL.available()) {
    String sentence = GPS_SERIAL.readStringUntil('\n');  // Read a line from GPS module

    // Check if the sentence contains latitude and longitude
    if (sentence.startsWith("$GPGGA")) {
      // Split the sentence into individual parts
      char data[100];
      sentence.toCharArray(data, 100);

      // Extract latitude and longitude
      char* token = strtok(data, ",");
      for (int i = 0; i < 6; i++) {
        token = strtok(NULL, ",");
      }
      if (token != NULL) {
        latitude = atof(token);
        token = strtok(NULL, ",");
        if (token != NULL) {
          longitude = atof(token);
          Serial.print("Latitude: ");
          Serial.print(latitude, 6);  // Print latitude with 6 decimal places
          Serial.print(", Longitude: ");
          Serial.println(longitude, 6);  // Print longitude with 6 decimal places
          // Check if latitude and longitude are valid (not zero or near zero)
          if (isValidLocation(latitude, longitude)) {
            sendHealthData(beatsPerMinute, beatAvg, temperature, temperatureF, latitude, longitude);
            Sms = 0;  // Reset the SMS flag
          }
        }
      }
    }
  }
}
bool isValidLocation(float lat, float lon) {
  // Check if latitude and longitude are valid (not zero or near zero)
  return (lat != 0.0 && lon != 0.0);
}
void connectWiFi() {
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(WhiteLed, LOW);
  }

  Serial.println();
  Serial.println("Connected to Wi-Fi");
  digitalWrite(WhiteLed, HIGH);
}
time_t syncNTP() {
  const int NTP_PACKET_SIZE = 48;
  byte packetBuffer[NTP_PACKET_SIZE];

  // Initialize a NTP request packet
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  packetBuffer[0] = 0b11100011;  // LI, Version, Mode
  packetBuffer[1] = 0;           // Stratum, or type of clock
  packetBuffer[2] = 6;           // Polling Interval
  packetBuffer[3] = 0xEC;        // Peer Clock Precision

  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;

  // Send a request to the NTP server
  WiFiUDP udp;
  udp.begin(2390);
  udp.beginPacket(timeServerIP, 123);
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();

  // Wait for the response (up to 1000ms)
  delay(1000);
  if (udp.parsePacket()) {
    udp.read(packetBuffer, NTP_PACKET_SIZE);
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    unsigned long secsSince1900 = highWord << 16 | lowWord;

    const unsigned long seventyYears = 2208988800UL;
    time_t epoch = secsSince1900 - seventyYears;

    return epoch + timeZone * 3600;
  } else {
    Serial.println("Failed to get NTP response");
    return 0;
  }
}