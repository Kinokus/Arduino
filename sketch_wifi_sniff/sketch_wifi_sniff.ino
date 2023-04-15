#include <SD.h>              // Include the SD library
#include <SPI.h>             // Include the SPI library
#include <SoftwareSerial.h>  // Include the SoftwareSerial library
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>
#define WIFI_PKT_PROBE_REQ 0x40
File dataFile;                    // File object to handle SD card file
SoftwareSerial mySerial(34, -1);  // RX pin: 34, TX pin: -1 (not used)
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
  wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
  const wifi_ieee80211_packet_t *packet = (const wifi_ieee80211_packet_t *)pkt->payload;
  const wifi_ieee80211_mac_hdr_t *hdr = &packet->hdr;
  if (type == WIFI_PKT_MGMT /*&& hdr->frame_control[0] == WIFI_PKT_PROBE_REQ*/) {
    uint8_t ssid_len = pkt->rx_ctrl.sig_len;
    char mac_addr_str[18];
    snprintf(
      mac_addr_str,
      sizeof(mac_addr_str),
      "%02X:%02X:%02X:%02X:%02X:%02X",
      hdr->addr2[0],
      hdr->addr2[1],
      hdr->addr2[2],
      hdr->addr2[3],
      hdr->addr2[4],
      hdr->addr2[5]);

    // Convert SSID to string format
    // TODO GET LENGTH !
    char ssid_str[ssid_len];
    memcpy(ssid_str, pkt->payload + sizeof(wifi_ieee80211_packet_t), ssid_len);
    ssid_str[ssid_len] = '\0';

    printf("%s - %s - %u\n", mac_addr_str, ssid_str, ssid_len);
  }
}


void setup() {
  Serial.begin(9600);    // Start serial communication with the Arduino IDE
  mySerial.begin(9600);  // Start serial communication with the external serial module

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

  wifi_promiscuous_filter_t wifi_filter;
  // wifi_filter.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA_MPDU;
  // wifi_filter.filter_id = WIFI_PROMIS_FILTER_MASK_MGMT;

  esp_wifi_set_promiscuous_filter(&wifi_filter);
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
