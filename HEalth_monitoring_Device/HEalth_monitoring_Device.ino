#include <SoftwareSerial.h>
#include <BlynkSimpleEsp8266.h>
#include <Wire.h>
#include "MAX30105.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>

// Define the GPS module connection
const int GPS_RX_PIN = 2; // GPS module TX pin to NodeMCU D2 (GPIO2)
const int GPS_TX_PIN = 3; // GPS module RX pin to NodeMCU D3 (GPIO3)

// Define MAX30105 pins
const int MAX30105_SDA_PIN = 4; // NodeMCU D4 (GPIO4)
const int MAX30105_SCL_PIN = 5; // NodeMCU D5 (GPIO5)

// Define DHT11 pin
const int DHT_PIN = 6; // NodeMCU D6 (GPIO6)

SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
MAX30105 particleSensor;
DHT dht(DHT_PIN, DHT11);

// You should get Auth Token in the Blynk App.
char auth[] = "YOUR_BLYNK_AUTH_TOKEN";

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  Blynk.begin(auth, "YOUR_WIFI_SSID", "YOUR_WIFI_PASSWORD");
  
  Wire.begin(MAX30105_SDA_PIN, MAX30105_SCL_PIN);
  particleSensor.begin();
  
  dht.begin();
}

void loop() {
  Blynk.run();
  
  // Check if data is available from the GPS module
  if (gpsSerial.available()) {
    String gpsData = gpsSerial.readStringUntil('\n');
    
    // Check if the data starts with "$GPRMC" (recommended minimum specific GPS/Transit data)
    if (gpsData.startsWith("$GPRMC")) {
      // Split the data into individual parts
      // Format: $GPRMC,hhmmss.sss,A,llll.llll,N,yyyyy.yyyy,E,spd,crs,date,mv,mvEW,posMode,navStatus*cs<CR><LF>
      // Relevant fields: 3 - latitude, 4 - N/S indicator, 5 - longitude, 6 - E/W indicator
      // Note: Additional error checking and validation can be implemented for a production environment.
      String gpsParts[15];
      int gpsPartCount = 0;
      
      int gpsStartIndex = 0;
      int gpsEndIndex = gpsData.indexOf(',');
      
      while (gpsEndIndex != -1) {
        gpsParts[gpsPartCount] = gpsData.substring(gpsStartIndex, gpsEndIndex);
        gpsStartIndex = gpsEndIndex + 1;
        gpsEndIndex = gpsData.indexOf(',', gpsStartIndex);
        gpsPartCount++;
      }
      
      // Extract latitude and longitude values
      String latitude = gpsParts[3];
      String longitude = gpsParts[5];
      
      // Check N/S and E/W indicators and adjust sign accordingly
      if (gpsParts[4] == "S") {
        latitude = "-" + latitude;
      }
      if (gpsParts[6] == "W") {
        longitude = "-" + longitude;
      }
      
      // Print latitude and longitude values
      Serial.print("Latitude: ");
      Serial.println(latitude);
      Serial.print("Longitude: ");
      Serial.println(longitude);
      
      // Update Blynk virtual pins with latitude and longitude values
      Blynk.virtualWrite(V1, latitude.toFloat());
      Blynk.virtualWrite(V2, longitude.toFloat());
    }
  }
  
  // Read temperature and humidity from DHT11
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  
  // Print temperature and humidity values
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C\t");
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
  
  // Update Blynk virtual pins with temperature and humidity values
  Blynk.virtualWrite(V3, temperature);
  Blynk.virtualWrite(V4, humidity);
  
  // Read heart rate and SpO2 from MAX30105
  float heartRate = particleSensor.getHeartRate();
  float spo2 = particleSensor.getSpO2();
  
  // Print heart rate and SpO2 values
  Serial.print("Heart Rate: ");
  Serial.print(heartRate);
  Serial.print(" bpm\t");
  Serial.print("SpO2: ");
  Serial.print(spo2);
  Serial.println(" %");
  
  // Update Blynk virtual pins with heart rate and SpO2 values
  Blynk.virtualWrite(V5, heartRate);
  Blynk.virtualWrite(V6, spo2);
  
  delay(500); // Delay for stability
}