#include <TinyGPS++.h>
#include <SD.h>

HardwareSerial SerialGPS(1);
TinyGPSPlus gps;
File file;

void setup() {
  Serial.begin(115200);
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);
  if (SD.begin()) {
    Serial.println("SD card initialized!");
    file = SD.open("/GPS_TEST.txt", FILE_APPEND);
    if (file) {
      Serial.println("File created successfully!");
      file.close();
    } else {
      Serial.println("Error creating file!");
    }
  }else{
    Serial.println("Error OPEN SD");
  }
}


void loop() {

  // Serial.println(SerialGPS.available());

  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }
  if (gps.time.isUpdated()) {
    Serial.println("~");
  }

  // while (SerialGPS.available() > 0) {
  //   Serial.println("~");
  //   if (gps.encode(SerialGPS.read())) {
  //     if (gps.location.isUpdated()) {
  //       // Get GPS data
  //       String latitude = String(gps.location.lat(), 6);
  //       String longitude = String(gps.location.lng(), 6);
  //       String altitude = String(gps.altitude.meters());
  //       String speed = String(gps.speed.kmph());
  //       String course = String(gps.course.deg());

  //       // Create a data string
  //       String data = "Latitude: " + latitude + " Longitude: " + longitude +
  //                     " Altitude: " + altitude + " Speed: " + speed + " Course: " + course;

  //       // Open a file on SD card in append mode
  //       file = SD.open("/gps_data.txt", FILE_APPEND);
  //       if (file) {
  //         // Append the data to the file
  //         Serial.println(data);
  //         file.println(data);
  //         file.close();
  //       } else {
  //         Serial.println("Failed to open file for writing");
  //       }
  //     }
  //   }
  // }
}


