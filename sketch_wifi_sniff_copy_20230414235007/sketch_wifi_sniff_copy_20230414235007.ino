#include <SD.h>              // Include the SD library
#include <SPI.h>             // Include the SPI library
#include <SoftwareSerial.h>  // Include the SoftwareSerial library
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>
#define WIFI_PKT_PROBE_REQ 0x40

File dataFile;                    // File object to handle SD card file

SoftwareSerial SerialGPS(34, -1);  // RX pin: 34, TX pin: -1 (not used)
TinyGPSPlus gps;

typedef struct {
  uint8_t frame_control[2];
  unsigned duration_id : 16;
  uint8_t addr1[6]; /* receiver address */
  uint8_t addr2[6]; /* sender address */
  uint8_t addr3[6]; /* filtering address */
  unsigned sequence_ctrl : 16;
  uint8_t addr4[6]; /* optional */
} wifi_ieee80211_mac_hdr_t;
typedef struct {
  wifi_ieee80211_mac_hdr_t hdr;
  uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
} wifi_ieee80211_packet_t;

void promiscuousCallback(void *buf, wifi_promiscuous_pkt_type_t type) {
  wifi_promiscuous_pkt_t *raw_packet = (wifi_promiscuous_pkt_t *)buf;
  if (type == WIFI_PKT_MGMT) {
    printf('%u\n', raw_packet->rx_ctrl.sig_len, raw_packet->rx_ctrl.sig_mode)
  }
}


void setup() {
  Serial.begin(9600);    // Start serial communication with the Arduino IDE
  SerialGPS.begin(9600);  // Start serial communication with the external serial module

  if (!SD.begin(SS)) {
    Serial.println("SD card initialization failed!");
    while (1)
      ;
  }
  // Create a new file on the SD card, or append to an existing file
  dataFile = SD.open("/data.txt", FILE_APPEND);  // Open file in write mode

  if (dataFile) {
    dataFile.println("Data logged:");
    dataFile.close();
  } else {
    Serial.println("Error opening data file");
  }

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

  WiFi.mode(WIFI_MODE_STA);

  esp_wifi_init(&cfg);
  esp_wifi_set_storage(WIFI_STORAGE_RAM);
  esp_wifi_set_mode(WIFI_MODE_NULL);
  esp_wifi_start();

  // wifi_promiscuous_filter_t wifi_filter;
  // wifi_filter.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA_MPDU;
  // wifi_filter.filter_id = WIFI_PROMIS_FILTER_MASK_MGMT;
  // esp_wifi_set_promiscuous_filter(&wifi_filter);
  esp_wifi_set_promiscuous_rx_cb(promiscuousCallback);
  esp_wifi_set_promiscuous(true);
}

void loop() {

  if (Serial.available()) {
    char data = Serial.read();  // Read data from external serial module
    mySerial.write(data);
    Serial.write(data);
  }

  if (mySerial.available()) {
    char data = mySerial.read();  // Read data from external serial module
    // Serial.write(data);
    dataFile = SD.open("/data.txt", FILE_APPEND);  // Open file in write mode
    if (dataFile) {
      dataFile.print(data);  // Write data to SD card
      dataFile.close();
    } else {
      Serial.println("Error opening data file");
    }
  }
}
