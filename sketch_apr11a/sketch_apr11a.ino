#include <SD.h>              // Include the SD library
#include <SPI.h>             // Include the SPI library
#include <SoftwareSerial.h>  // Include the SoftwareSerial library

File dataFile;  // File object to handle SD card file

SoftwareSerial mySerial(34, -1); // RX pin: 34, TX pin: -1 (not used)

void setup() {
  Serial.begin(9600);    // Start serial communication with the Arduino IDE
  mySerial.begin(9600);  // Start serial communication with the external serial module

  if (!SD.begin(SS)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }
  // Create a new file on the SD card, or append to an existing file
  dataFile = SD.open("/data.txt", FILE_APPEND);  // Open file in write mode

  if (dataFile) {
    dataFile.println("Data logged:");
    dataFile.close();
  } else {
    Serial.println("Error opening data file");
  }
}

void loop() {

  if (Serial.available()) {
    char data = Serial.read();  // Read data from external serial module
    mySerial.write(data);
    Serial.write(data);
  }

  if (mySerial.available()) {
    char data = mySerial.read();  // Read data from external serial module
    Serial.write(data);
    dataFile = SD.open("/data.txt", FILE_APPEND);  // Open file in write mode
    if (dataFile) {
      dataFile.print(data);  // Write data to SD card
      dataFile.close();
    } else {
      Serial.println("Error opening data file");
    }
  }
}
