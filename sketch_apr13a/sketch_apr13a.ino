#include <SoftwareSerial.h>
#include <SD.h>

SoftwareSerial mySerial(34, -1); // RX pin: 34, TX pin: -1 (not used)
File dataFile;

void setup() {
  Serial.begin(115200);         // Initialize Serial for debugging
  mySerial.begin(9600);        // Initialize SoftwareSerial at 9600 baud rate

  Serial.println(SS);
  if (!SD.begin(SS)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }

  
  dataFile = SD.open("/data.txt", FILE_APPEND);
  
  if (dataFile) {
    Serial.println("File created successfully.");
    dataFile.close();
  } else {
    Serial.println("Setup Error creating file!");
    while (1);
  }

}

void loop() {
  if (mySerial.available()) {    // Check if data is available on SoftwareSerial
    char data = mySerial.read(); // Read the incoming data
    Serial.write(data);          // Write the data to Serial for debugging
  }
}
