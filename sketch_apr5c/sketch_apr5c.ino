#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>
// #include <ArduinoSTL.h>

#include <stdio.h>
#include <stdint.h>
#include <locale.h>
#include <stdlib.h>
// #include <string.h>
#include <wchar.h>

#include "FS.h"
#include "SD.h"

typedef struct {
  unsigned frame_ctrl : 16;
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

static esp_err_t event_handler(void *ctx, system_event_t *event);
static void wifi_sniffer_init(void);
// static void wifi_sniffer_set_channel(uint8_t channel);
static const char *wifi_sniffer_packet_type2str(wifi_promiscuous_pkt_type_t type);
static void wifi_sniffer_packet_handler(void *buff, wifi_promiscuous_pkt_type_t type);


HardwareSerial SerialGPS(1);
TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  delay(100);

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(wifi_sniffer_packet_handler);

  if (!SD.begin()) {
    printf("SD card initialization failed!");
    return;
  }

  File file = SD.open("/example.txt", FILE_APPEND);
  if (file) {
    printf("File created successfully!");
  } else {
    printf("Error creating file!");
  }
}

void loop() {
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }
  // Serial.println(gps.satellites.value());
  if (gps.time.isUpdated() && gps.satellites.isUpdated()) {



    // snprintf_P(
    //   currentDate,
    //   sizeof(currentDate),
    //   PSTR(""
    //        "%04.0f."
    //        "%02.0f."
    //        "%02.0f;"
    //        "%02.0f:"
    //        "%02.0f:"
    //        "%02.0f;"
    //        // "%03.5f;"
    //        // "%03.5f;"
    //        "%s"),
    //   gps.date.year(),
    //   gps.date.month(),
    //   gps.date.day(),
    //   gps.time.hour(),
    //   gps.time.minute(),
    //   gps.time.second(),
    //   // gps.location.lat(),
    //   // gps.location.lng(),
    //   "\0");

    // currentDate = String::format(
    //   ""
    //   "%04d."
    //   "%02d."
    //   "%02d;"
    //   "%02d:"
    //   "%02d:"
    //   "%02d;"
    //   // "%03.5f;"
    //   // "%03.5f;"
    //   "%s",
    //   gps.date.year(),
    //   gps.date.month(),
    //   gps.date.day(),
    //   gps.time.hour(),
    //   gps.time.minute(),
    //   gps.time.second(),
    //   // gps.location.lat(),
    //   // gps.location.lng(),
    //   "\0");

    // printf(currentDate);
  }
}


const char *wifi_sniffer_packet_type2str(wifi_promiscuous_pkt_type_t type) {
  switch (type) {
    case WIFI_PKT_MGMT: return "MGMT";
    case WIFI_PKT_DATA: return "DATA";
    default:
    case WIFI_PKT_MISC: return "MISC";
  }
}

void wifi_sniffer_packet_handler(void *buff, wifi_promiscuous_pkt_type_t type) {

  if (type != WIFI_PKT_MGMT)
    return;

  const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buff;
  const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
  const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

  uint8_t *ssid = NULL;
  uint8_t ssid_len = 0;
  uint8_t *payload = (uint8_t *)ppkt->payload;
  uint8_t payload_len = ppkt->rx_ctrl.sig_len - sizeof(wifi_ieee80211_mac_hdr_t);

  while (payload_len > 0) {
    uint8_t ie_type = payload[0];
    uint8_t ie_len = payload[1];

    if (ie_type == 0) {  // SSID parameter set
      ssid = &payload[2];
      ssid_len = ie_len;
      break;
    }

    payload += ie_len + 2;
    payload_len -= ie_len + 2;
  }

  if (ssid_len > 0) {
    // Serial.print(ssid_len);
    int arraySize = sizeof(ssid) / sizeof(ssid[0]);
    // Serial.print(ssid, ssid_len);
    for (int i = 0; i < arraySize; i++) {
      Serial.print(ssid[i]);
      Serial.print(" ");
    }

    File myFile = SD.open("myFile.bin", FILE_APPEND );
    myFile.write(ssid, ssid_len);
    myFile.close();
    
    Serial.println();
  } else {
    // Serial.write(hdr->addr1, sizeof(hdr->addr1));
    // Serial.println();
  }

  // if (ssid_len > 0) {

  //   printf(
  //     // " %s"
  //     " %s"
  //     " TYPE=%s,"
  //     " CHAN=%02d,"
  //     " RSSI=%02d,"
  //     " ADDR1=%02x:%02x:%02x:%02x:%02x:%02x,"
  //     " ADDR2=%02x:%02x:%02x:%02x:%02x:%02x,"
  //     " ADDR3=%02x:%02x:%02x:%02x:%02x:%02x,"
  //     //      " LEN=%02x,"
  //     // " SSID=%s,"
  //     "%s\n",
  //     // currentDate,
  //     wifi_sniffer_packet_type2str(type),
  //     ppkt->rx_ctrl.channel,
  //     ppkt->rx_ctrl.rssi,
  //     /* ADDR1 */
  //     hdr->addr1[0], hdr->addr1[1], hdr->addr1[2],
  //     hdr->addr1[3], hdr->addr1[4], hdr->addr1[5],
  //     /* ADDR2 */
  //     hdr->addr2[0], hdr->addr2[1], hdr->addr2[2],
  //     hdr->addr2[3], hdr->addr2[4], hdr->addr2[5],
  //     /* ADDR3 */
  //     hdr->addr3[0], hdr->addr3[1], hdr->addr3[2],
  //     hdr->addr3[3], hdr->addr3[4], hdr->addr3[5],
  //     //      ssid_len,
  //     // ssid[0]
  //     ";"
  //   );

  // }
}
