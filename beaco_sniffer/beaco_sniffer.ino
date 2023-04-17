// ESP32 WiFi beacon sniffer

#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "FS.h"
#include "SD.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>


#define I2C_SCL 4
#define I2C_SDA 5

#define WIFI_CHANNEL_SWITCH_INTERVAL (500)
#define WIFI_CHANNEL_MAX (13)

File dataFile;  // File object to handle SD card file

SoftwareSerial SerialGPS(34, -1);  // RX pin: 34, TX pin: -1 (not used)
TinyGPSPlus gps;

char full4dPlace[36];

char year[2];
char month[2];
char day[2];

char hour[2];
char minute[2];
char second[2];

char lat[8];
char lng[8];

uint8_t level = 0, channel = 1;

static wifi_country_t wifi_country = { .cc = "CN", .schan = 1, .nchan = 13 };  // Most recent esp32 library struct

typedef struct {
  unsigned protocol : 2;
  unsigned type : 2;
  unsigned subtype : 4;
  unsigned to_ds : 1;
  unsigned from_ds : 1;
  unsigned more_frag : 1;
  unsigned retry : 1;
  unsigned pwr_mgmt : 1;
  unsigned more_data : 1;
  unsigned wep : 1;
  unsigned strict : 1;
} wifi_header_frame_control_t;

// https://carvesystems.com/news/writing-a-simple-esp8266-based-sniffer/
typedef struct {
  wifi_header_frame_control_t frame_ctrl;
  unsigned duration_id : 16;
  uint8_t addr1[6]; /* receiver MAC address */
  uint8_t addr2[6]; /* sender MAC address */
  uint8_t addr3[6]; /* BSSID filtering address */
  unsigned sequence_ctrl : 16;
  uint8_t addr4[6]; /* optional */
} wifi_ieee80211_mac_hdr_t;

typedef struct {
  wifi_ieee80211_mac_hdr_t hdr;
  uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
} wifi_ieee80211_packet_t;


typedef struct {
  unsigned interval : 16;
  unsigned capability : 16;
  unsigned tag_number : 8;
  unsigned tag_length : 8;
  char ssid[0];
  uint8_t rates[1];
} wifi_beacon_hdr;

typedef struct {
  uint8_t mac[6];
} __attribute__((packed)) mac_addr;


typedef enum {
  ASSOCIATION_REQ,
  ASSOCIATION_RES,
  REASSOCIATION_REQ,
  REASSOCIATION_RES,
  PROBE_REQ,
  PROBE_RES,
  NU1, /* ......................*/
  NU2, /* 0110, 0111 not used */
  BEACON,
  ATIM,
  DISASSOCIATION,
  AUTHENTICATION,
  DEAUTHENTICATION,
  ACTION,
  ACTION_NACK,
} wifi_mgmt_subtypes_t;

static esp_err_t event_handler(void *ctx, system_event_t *event);
static void wifi_sniffer_init(void);
static void wifi_sniffer_set_channel(uint8_t channel);
static const char *wifi_sniffer_packet_type2str(wifi_promiscuous_pkt_type_t type);
static void wifi_sniffer_packet_handler(void *buff, wifi_promiscuous_pkt_type_t type);

esp_err_t event_handler(void *ctx, system_event_t *event) {
  return ESP_OK;
}

void wifi_sniffer_init(void) {
  nvs_flash_init();
  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_country(&wifi_country)); /* set country for channel range [1, 13] */
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
  ESP_ERROR_CHECK(esp_wifi_start());
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&wifi_sniffer_packet_handler);
}

void wifi_sniffer_set_channel(uint8_t channel) {
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
}

const char *wifi_sniffer_packet_type2str(wifi_promiscuous_pkt_type_t type) {
  switch (type) {
    case WIFI_PKT_MGMT: return "MGMT";
    case WIFI_PKT_DATA: return "DATA";
    default:
    case WIFI_PKT_MISC: return "MISC";
  }
}
const char *wifi_sniffer_packet_subtype2str(wifi_mgmt_subtypes_t type) {
  switch (type) {
    case BEACON: return "BEACON";
    case PROBE_REQ: return "PROBE_REQ";
    default: return "MISC";
  }
}

static void get_ssid(unsigned char *data, char ssid[32], uint8_t ssid_len) {
  int i, j;

  for (i = 26, j = 0; i < 26 + ssid_len; i++, j++) {
    ssid[j] = data[i];
  }

  ssid[j] = '\0';
}

void wifi_sniffer_packet_handler(void *buff, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT)
    return;

  // https://blog.podkalicki.com/wp-content/uploads/2017/01/esp32_promiscuous_pkt_structure.jpeg
  const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buff;
  const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
  const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

  wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buff;

  // From https://github.com/SHA2017-badge/bpp/blob/master/esp32-recv/main/bpp_sniffer.c
  // https://github.com/n0w/esp8266-simple-sniffer/blob/master/src/main.cpp
  char ssid[32] = { 0 };

  const wifi_header_frame_control_t *fctl = (wifi_header_frame_control_t *)&hdr->frame_ctrl;

  // Details about beacon frames: https://mrncciew.com/2014/10/08/802-11-mgmt-beacon-frame/
  if (fctl->subtype == BEACON) {  //beacon
    wifi_beacon_hdr *beacon = (wifi_beacon_hdr *)ipkt->payload;

    if (beacon->tag_length >= 32) {
      strncpy(ssid, beacon->ssid, 31);
    } else {
      strncpy(ssid, beacon->ssid, beacon->tag_length);
    }
    // Serial.printf("Beacon %s\n",ssid);
    // addBeacon(ssid, ppkt->rx_ctrl.channel, ppkt->rx_ctrl.rssi);
  }

  if (fctl->subtype == PROBE_REQ) {
    uint8_t ssid_len;
    ssid_len = pkt->payload[25];
    if (ssid_len > 0)
      get_ssid(pkt->payload, ssid, ssid_len);
  }

  printf(""

         " P_TYPE=%s(%d),"
         " P_S_TYPE=%s(%d),"
         " CHAN=%02d,"
         " RSSI=%02d,"
         " ADDR1=%02x:%02x:%02x:%02x:%02x:%02x,"
         " ADDR2=%02x:%02x:%02x:%02x:%02x:%02x,"
         " ADDR3=%02x:%02x:%02x:%02x:%02x:%02x"
         " SSID=%s,"
         " 4DPLACE=%s"

         "\n",
         wifi_sniffer_packet_type2str(type),
         type,
         wifi_sniffer_packet_subtype2str((wifi_mgmt_subtypes_t)fctl->subtype),
         fctl->subtype,
         ppkt->rx_ctrl.channel,
         ppkt->rx_ctrl.rssi,
         // ADDR1
         hdr->addr1[0], hdr->addr1[1], hdr->addr1[2],
         hdr->addr1[3], hdr->addr1[4], hdr->addr1[5],
         // ADDR2
         hdr->addr2[0], hdr->addr2[1], hdr->addr2[2],
         hdr->addr2[3], hdr->addr2[4], hdr->addr2[5],
         // ADDR3
         hdr->addr3[0], hdr->addr3[1], hdr->addr3[2],
         hdr->addr3[3], hdr->addr3[4], hdr->addr3[5],
         ssid,
         full4dPlace);
}
// ************************************
// DEBUG()
//
// ************************************
#define __DEBUG__

void DEBUG(String message) {
#ifdef __DEBUG__
  Serial.println(message);
#endif
}


#include <LinkedList.h>

class WiFiBeacon {
public:
  char name[32];
  int rssi;
  uint8_t channel;
  char mac[6];
  uint8_t lastseen;
};

LinkedList<WiFiBeacon *> myBeacons = LinkedList<WiFiBeacon *>();

hw_timer_t *timer = NULL;
bool timerChannel = false;

void IRAM_ATTR onTimer() {
  timerChannel = true;
}

// the setup function runs once when you press reset or power the board
void setup() {


  Serial.begin(115200);
  SerialGPS.begin(9600);

  delay(5000);
  wifi_sniffer_init();
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000, true);
  timerAlarmEnable(timer);

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
}

// the loop function runs over and over again forever
void loop() {
  if (timerChannel) {
    // Set channel
    wifi_sniffer_set_channel(channel);
    channel = (channel % WIFI_CHANNEL_MAX) + 1;
    timerChannel = false;
  }
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }
  ///
  gps.date.year();
  snprintf(
    full4dPlace,
    36,
    "%04d.%02d.%02d %02d:%02d:%02d %3.4f %3.4f",
    gps.date.year(),
    gps.date.month(),
    gps.date.day(),

    gps.time.hour(),
    gps.time.minute(),
    gps.time.second(),

    gps.location.lat(),
    gps.location.lng()

  );
  // sprintf(year, '%d', gps.date.year());
  // sprintf(month, '%d', gps.date.month());
  // sprintf(date, '%d', gps.date.day());
  // sprintf(hour, '%d', gps.time.hour());
  // sprintf(minute, '%d', gps.time.minute());
  // sprintf(second, '%d', gps.time.second());
  // sprintf(lat, '%d', gps.location.lat());
  // sprintf(lng, '%d', gps.location.lng());
}