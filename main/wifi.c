/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ESP deck firmware
 *
 * Copyright (C) 2022 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "wifi.h"

#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>
#include "esp_netif.h"
#include "esp_mac.h"

#include "com.h"
#include "spi_transport.h"
#define BLINK_GPIO 4

static esp_routable_packet_t rxp;
static esp_routable_packet_t txp;

#define MAX_SSID_SIZE (50)
#define MAX_PASSWD_SIZE (50)

static char ssid[MAX_SSID_SIZE];
static char key[MAX_SSID_SIZE];

static const int WIFI_CONNECTED_BIT = BIT0;
static const int WIFI_SOCKET_DISCONNECTED = BIT1;
static const int WIFI_PACKET_WAIT_SEND = BIT2;
static const int WIFI_PACKET_SENDING = BIT3;
static EventGroupHandle_t s_wifi_event_group;

static const int START_UP_MAIN_TASK = BIT0;
static const int START_UP_RX_TASK = BIT1;
static const int START_UP_TX_TASK = BIT2;
static const int START_UP_CTRL_TASK = BIT3;
static EventGroupHandle_t startUpEventGroup;


#define NO_CONNECTION -1
// 2 was too tight (any TCP jitter immediately backpressured the SPI/GAP8
// route), but every slot is ~1 KB of a ~40 KB heap - and heap headroom is
// what keeps the WiFi driver alive under load (see wifi_status_task).
#define WIFI_HOST_QUEUE_LENGTH (4)

// How many consecutive 1 s send timeouts before the client is declared dead
// and the socket is closed. This bounds a send stall to a few seconds instead
// of blocking the TX task (and, through the queues, the SPI/GAP8 route)
// forever.
#define WIFI_SEND_TIMEOUT_MAX (5)

// How often the TCP control loop re-asserts CLIENT_CONNECTED to the GAP8 while
// a client is connected. The one accept-time notification crosses the
// unacknowledged ESP<->GAP8 SPI link and can be dropped there; a TCP client
// (unlike a UDP one, whose periodic FER re-announce drives a re-signal) sends
// nothing after connecting, so the ESP must retransmit the notification itself
// or the GAP8 can gate on it forever and never emit a first frame. The GAP8
// makes a repeat idempotent, so this doubles as a cheap keepalive once the
// first one lands. ~1 s bounds recovery from a dropped notification to about
// one interval while costing only two 2-byte control packets per second.
#define WIFI_CLIENT_CONNECTED_REASSERT_MS (1000)

static QueueHandle_t wifiRxQueue;
static QueueHandle_t wifiTxQueue;
static volatile uint32_t wifiTxEnqueued;
static volatile uint32_t wifiTxDequeued;
static volatile uint32_t wifiSendCalls;
static volatile uint32_t wifiBytesWritten;

/* Serializes client-socket close between the RX and TX tasks */
static SemaphoreHandle_t socketCloseLock;

/* Log printout tag */
static const char *TAG = "WIFI";

/* Socket for receiving WiFi connections */
static int serverSock = -1;
/* Accepted WiFi connection */
static int clientConnection = NO_CONNECTION;

enum {
  WIFI_CTRL_SET_SSID                = 0x10,
  WIFI_CTRL_SET_KEY                 = 0x11,

  WIFI_CTRL_WIFI_CONNECT            = 0x20,
  WIFI_CTRL_SET_TRANSPORT           = 0x21,

  WIFI_CTRL_STATUS_WIFI_CONNECTED   = 0x31,
  WIFI_CTRL_STATUS_CLIENT_CONNECTED = 0x32,
};

// Host link transport, chosen at runtime by the GAP8 (WIFI_CTRL_SET_TRANSPORT)
// before it asks to connect. TCP is framed and lossless (integrity transport);
// UDP is connectionless and drop-tolerant (a stalled host cannot back-pressure
// the SPI/GAP8 route, at the cost of dropped datagrams). Default TCP so any app
// that never sends SET_TRANSPORT behaves exactly as before.
enum {
  WIFI_TRANSPORT_TCP = 0,
  WIFI_TRANSPORT_UDP = 1,
};
static volatile uint8_t transportMode = WIFI_TRANSPORT_TCP;

// UDP host address, learned from the source of the "FER" magic datagram. Guarded
// by udpTargetLock because the RX task writes it and the TX task reads it.
static struct sockaddr_in udpDestAddr;
static volatile bool udpClientKnown = false;
static SemaphoreHandle_t udpTargetLock;

// Released by the WIFI_CTRL_WIFI_CONNECT handler once the transport is final and
// WiFi is up, so wifi_task binds the right kind of socket.
static SemaphoreHandle_t serveReadySem;

// Tell the GAP8 (and STM32) whether a host is connected. Used from the UDP RX
// task on the first FER datagram; the TCP path signals the same from wifi_task.
static void wifi_signal_client_connected(uint8_t connected) {
  static esp_routable_packet_t p;
  cpxInitRoute(CPX_T_ESP32, CPX_T_GAP8, CPX_F_WIFI_CTRL, &p.route);
  p.data[0] = WIFI_CTRL_STATUS_CLIENT_CONNECTED;
  p.data[1] = connected;
  p.dataLength = 2;
  espAppSendToRouterBlocking(&p);
  p.route.destination = CPX_T_STM32;
  espAppSendToRouterBlocking(&p);
}

/* WiFi event handler */
static void event_handler(void* handlerArg, esp_event_base_t eventBase, int32_t eventId, void* eventData)
{
  if (eventBase == WIFI_EVENT) {
    switch(eventId) {
      case WIFI_EVENT_STA_START:
        ESP_ERROR_CHECK(esp_wifi_connect());
        break;
      case WIFI_EVENT_STA_DISCONNECTED:
        ESP_ERROR_CHECK(esp_wifi_connect());
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG,"Disconnected from access point");
        break;
      case WIFI_EVENT_AP_STACONNECTED:
        {
          wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*)eventData;
          ESP_LOGI(TAG, "station:"MACSTR" join, AID=%d",
                    MAC2STR(event->mac),
                    event->aid);
        }
        break;
      case WIFI_EVENT_AP_STADISCONNECTED:
        {
          wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*)eventData;
          ESP_LOGI(TAG, "station:"MACSTR"leave, AID=%d",
                    MAC2STR(event->mac),
                    event->aid);
        }
        break;
      default:
        // Fall through
        break;
    }
  }

  if (eventBase == IP_EVENT) {
    switch (eventId) {
      case IP_EVENT_STA_GOT_IP:
        {
          ip_event_got_ip_t* event = (ip_event_got_ip_t*)eventData;
          ESP_LOGI(TAG, "got ip: " IPSTR, IP2STR(&event->ip_info.ip));

          wifi_ap_record_t ap_info;
          ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&ap_info));
          ESP_LOGD(TAG, "BSAP MAC is %x:%x:%x:%x:%x:%x",
              ap_info.bssid[0], ap_info.bssid[1], ap_info.bssid[2],
              ap_info.bssid[3], ap_info.bssid[4], ap_info.bssid[5]);
          ESP_LOGI(TAG, "country: %s", ap_info.country.cc);
          ESP_LOGI(TAG, "rssi: %d", ap_info.rssi);
          ESP_LOGI(TAG, "11b: %d, 11g: %d, 11n: %d, lr: %d",
            ap_info.phy_11b, ap_info.phy_11g, ap_info.phy_11n, ap_info.phy_lr);

          cpxInitRoute(CPX_T_ESP32, CPX_T_GAP8, CPX_F_WIFI_CTRL, &txp.route);
          txp.data[0] = WIFI_CTRL_STATUS_WIFI_CONNECTED;
          memcpy(&txp.data[1], &event->ip_info.ip.addr, sizeof(uint32_t));
          txp.dataLength = 1 + sizeof(uint32_t);

          // TODO: We should probably not block here...
          espAppSendToRouterBlocking(&txp);

          txp.route.destination = CPX_T_STM32;
          espAppSendToRouterBlocking(&txp);

          xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        }
        break;
      default:
        // Fall through
        break;
    }
  }
}

/* Initialize WiFi as AP */
static void wifi_init_softap(const char *ssid, const char* key)
{
  esp_netif_t* ap_netif = esp_netif_create_default_wifi_ap();
  assert(ap_netif);

  wifi_config_t wifi_config = {
      .ap = {
          .ssid_len = strlen(ssid),
          // Default was channel 1, the most congested band in this lab; RF
          // retry pressure deepens the driver TX queues and helps trip the
          // low-heap radio failure under streaming load.
          .channel = 6,
          .max_connection = 1,
          .authmode = WIFI_AUTH_OPEN},
  };
  strncpy((char *)wifi_config.ap.ssid, ssid, strlen(ssid));
  if (strlen(key) > 0) {
    strncpy((char *)wifi_config.ap.password, key, strlen(key));
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
  }

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "wifi_init_softap finished");
}

static void wifi_init_sta(const char * ssid, const char * key)
{
  esp_netif_t* sta_netif = esp_netif_create_default_wifi_sta();
  assert(sta_netif);

  wifi_config_t wifi_config;
  memset((void *)&wifi_config, 0, sizeof(wifi_config_t));
  strncpy((char *)wifi_config.sta.ssid, ssid, strlen(ssid));
  ESP_LOGD(TAG, "SSID is %u chars", strlen(ssid));
  strncpy((char *)wifi_config.sta.password, key, strlen(key));
  ESP_LOGD(TAG, "KEY is %u chars", strlen(key));

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
  ESP_ERROR_CHECK(esp_wifi_start() );

  ESP_LOGI(TAG, "wifi_init_sta finished.");

}

static void wifi_ctrl(void* _param) {
  xEventGroupSetBits(startUpEventGroup, START_UP_CTRL_TASK);
  while (1) {
    com_receive_wifi_ctrl_blocking(&rxp);

    switch (rxp.data[0]) {
      case WIFI_CTRL_SET_SSID:
        ESP_LOGD("WIFI", "Should set SSID");
        memcpy(ssid, &rxp.data[1], rxp.dataLength - 1);
        ssid[rxp.dataLength - 1 + 1] = 0;
        ESP_LOGD(TAG, "SSID: %s", ssid);
        // Save to NVS?
        break;
      case WIFI_CTRL_SET_KEY:
        ESP_LOGD("WIFI", "Should set password");
        memcpy(key, &rxp.data[1], rxp.dataLength - 1);
        key[rxp.dataLength - 1 + 1] = 0;
        ESP_LOGD(TAG, "KEY: %s", key);
        // Save to NVS?
        break;
      case WIFI_CTRL_SET_TRANSPORT:
        transportMode = (rxp.data[1] == WIFI_TRANSPORT_UDP) ? WIFI_TRANSPORT_UDP
                                                            : WIFI_TRANSPORT_TCP;
        ESP_LOGI(TAG, "Transport set to %s",
                 transportMode == WIFI_TRANSPORT_UDP ? "UDP" : "TCP");
        break;
      case WIFI_CTRL_WIFI_CONNECT:
        ESP_LOGD("WIFI", "Should connect");

        if (strlen(ssid) > 0) {
          if (rxp.data[1] == 0) {
            wifi_init_sta(ssid, key);
          } else {
            if (0 < strlen(key) && strlen(key) < 8) {
              ESP_LOGW(TAG, "Password too short, cannot initialize AP");
            } else {
              wifi_init_softap(ssid, key);
            }
          }
          // Transport is now final and WiFi is coming up: let wifi_task bind the
          // matching (TCP stream vs UDP datagram) server socket and start serving.
          xSemaphoreGive(serveReadySem);
        } else {
          ESP_LOGW(TAG, "No SSID set, cannot start wifi");
        }

        break;
    }
  }
}

static void close_client_socket()
{
    // Both the RX and TX tasks call this on error; take the fd atomically so
    // the socket is only closed once, and shutdown() first so a task still
    // blocked in recv()/send() on it returns instead of holding a dead fd.
    xSemaphoreTake(socketCloseLock, portMAX_DELAY);
    int connection = clientConnection;
    clientConnection = NO_CONNECTION;
    xSemaphoreGive(socketCloseLock);

    if (connection != NO_CONNECTION) {
        shutdown(connection, SHUT_RDWR);
        close(connection);
        xEventGroupSetBits(s_wifi_event_group, WIFI_SOCKET_DISCONNECTED);
    }
}

void wifi_bind_socket() {
  const bool udp = (transportMode == WIFI_TRANSPORT_UDP);
  struct sockaddr_in destAddr;
  destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  destAddr.sin_family = AF_INET;
  destAddr.sin_port = htons(5000);

  serverSock = socket(AF_INET, udp ? SOCK_DGRAM : SOCK_STREAM,
                      udp ? IPPROTO_UDP : IPPROTO_IP);
  if (serverSock < 0) {
    ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    return;
  }
  ESP_LOGD(TAG, "Socket created");

  int err = bind(serverSock, (struct sockaddr *)&destAddr, sizeof(destAddr));
  if (err != 0) {
    ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
    return;
  }
  ESP_LOGD(TAG, "Socket binded");

  if (!udp) {
    err = listen(serverSock, 1);
    if (err != 0) {
      ESP_LOGE(TAG, "Error occured during listen: errno %d", errno);
    }
    ESP_LOGD(TAG, "Socket listening");
  }
  ESP_LOGI(TAG, "%s socket bound to :5000", udp ? "UDP" : "TCP");
}

void wifi_wait_for_socket_connected() {
  ESP_LOGI(TAG, "Waiting for connection");
  struct sockaddr sourceAddr;
  socklen_t addrLen = sizeof(sourceAddr);
  clientConnection = accept(serverSock, (struct sockaddr *)&sourceAddr, &addrLen);
  if (clientConnection < 0) {
    ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
  } else {
    // Bound every send and detect a vanished peer. An unbounded blocking
    // send() on this socket is what stalled the whole GAP8->WiFi route (see
    // wifi_send_packet); keepalive catches a host that disappears without a
    // FIN, and NODELAY stops Nagle from delaying the length-prefixed stream.
    const struct timeval sendTimeout = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt(clientConnection, SOL_SOCKET, SO_SNDTIMEO, &sendTimeout, sizeof(sendTimeout));
    const int enable = 1;
    setsockopt(clientConnection, IPPROTO_TCP, TCP_NODELAY, &enable, sizeof(enable));
    setsockopt(clientConnection, SOL_SOCKET, SO_KEEPALIVE, &enable, sizeof(enable));
    const int keepIdleS = 5;
    const int keepIntvlS = 2;
    const int keepCntS = 3;
    setsockopt(clientConnection, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdleS, sizeof(keepIdleS));
    setsockopt(clientConnection, IPPROTO_TCP, TCP_KEEPINTVL, &keepIntvlS, sizeof(keepIntvlS));
    setsockopt(clientConnection, IPPROTO_TCP, TCP_KEEPCNT, &keepCntS, sizeof(keepCntS));
    ESP_LOGI(TAG, "Connection accepted");
  }
}

// Wait up to `timeout` ticks for the client socket to drop. Returns true if it
// disconnected (the bit was set and is cleared on exit), false if the wait timed
// out with the client still connected - which the control loop uses to re-assert
// the connected notification without blocking indefinitely.
bool wifi_wait_for_disconnect(TickType_t timeout) {
  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_SOCKET_DISCONNECTED,
                                         pdTRUE, pdFALSE, timeout);
  return (bits & WIFI_SOCKET_DISCONNECTED) != 0;
}

static void wifi_task(void *pvParameters) {

  s_wifi_event_group = xEventGroupCreate();

  esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, event_handler, NULL, NULL);
  esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, event_handler, NULL, NULL);

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  uint8_t mac[6];
  ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_AP, mac));
  ESP_LOGD(TAG, "AP MAC is %x:%x:%x:%x:%x:%x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));
  ESP_LOGD(TAG, "STA MAC is %x:%x:%x:%x:%x:%x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  xEventGroupSetBits(startUpEventGroup, START_UP_MAIN_TASK);

  // Wait until the GAP8 has chosen the transport and asked to connect, so the
  // server socket is the right type. Binding after esp_wifi_start() is fine.
  xSemaphoreTake(serveReadySem, portMAX_DELAY);
  wifi_bind_socket();

  if (transportMode == WIFI_TRANSPORT_UDP) {
    // Connectionless: there is nothing to accept and no disconnect to wait on.
    // The RX task learns the host address from the FER magic datagram and signals
    // the GAP8; this task has no further work.
    ESP_LOGI(TAG, "UDP transport: waiting for host FER datagram");
    while (1) {
      vTaskDelay(portMAX_DELAY);
    }
  }

  while (1) {
    //blink_period_ms = 500;
    wifi_wait_for_socket_connected();
    ESP_LOGI(TAG, "Client connected");

    //blink_period_ms = 100;

    // Tell the GAP8 (and STM32) a client is connected so the GAP8 leaves its
    // pre-client gate and starts streaming, then re-assert on a fixed interval
    // until the client drops. The single accept-time packet can be lost on the
    // unacknowledged ESP<->GAP8 SPI link, and a TCP client sends nothing after
    // connecting (so there is no host-driven re-signal like UDP's FER); without
    // the periodic re-assert a dropped notification wedges the GAP8 forever with
    // no first frame. The GAP8 makes repeats idempotent - they neither restart
    // its settle timer nor republish an event - so the re-assert is a free
    // no-op once the first one lands. Routing through wifi_signal_client_connected()
    // also keeps this off the shared, non-thread-safe txp packet.
    wifi_signal_client_connected(1);
    while (!wifi_wait_for_disconnect(pdMS_TO_TICKS(WIFI_CLIENT_CONNECTED_REASSERT_MS))) {
      wifi_signal_client_connected(1);
    }
    ESP_LOGI(TAG, "Client disconnected");

    wifi_signal_client_connected(0);
  }
}

static void wifi_status_task(void *pvParameters)
{
  // Heap telemetry over the (radio-visible) console: the WiFi blob's radio TX
  // can die under sustained load while the app runs on - this shows whether
  // memory exhaustion precedes the death.
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(5000));
    uint32_t spiTransactions = 0, spiTxPackets = 0, spiRxPackets = 0;
    int gapRtt = 0, spiArmed = 0;
    spi_transport_debug(&spiTransactions, &spiTxPackets, &spiRxPackets, &gapRtt, &spiArmed);
    const char *clientState = (transportMode == WIFI_TRANSPORT_UDP)
                                  ? (udpClientKnown ? "yes" : "no")
                                  : (clientConnection == NO_CONNECTION ? "no" : "yes");
    ESP_LOGI(TAG, "status: %s heap %u (min %u), client %s, txq %u enq %u deq %u send %u bytes %u",
             transportMode == WIFI_TRANSPORT_UDP ? "UDP" : "TCP",
             (unsigned)esp_get_free_heap_size(), (unsigned)esp_get_minimum_free_heap_size(),
             clientState,
             (unsigned)uxQueueMessagesWaiting(wifiTxQueue),
             (unsigned)wifiTxEnqueued, (unsigned)wifiTxDequeued,
             (unsigned)wifiSendCalls, (unsigned)wifiBytesWritten);
    ESP_LOGI(TAG, "spi: txn %u txp %u rxp %u gap_rtt %d armed %d",
             (unsigned)spiTransactions, (unsigned)spiTxPackets,
             (unsigned)spiRxPackets, gapRtt, spiArmed);
  }
}

void wifi_led_task(void *pvParameters)
{
  int ledstate = 0;
  while(1) {
    if(clientConnection == NO_CONNECTION){
      gpio_set_level(BLINK_GPIO, !ledstate);
      ledstate = !ledstate;
      vTaskDelay(pdMS_TO_TICKS(500));
    } else {
      EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_PACKET_SENDING | WIFI_PACKET_WAIT_SEND  , pdFALSE,pdFALSE,portMAX_DELAY);
      if (bits & WIFI_PACKET_SENDING) {
        gpio_set_level(BLINK_GPIO,1);
        xEventGroupClearBits(s_wifi_event_group, WIFI_PACKET_SENDING);
      }
      if (bits & WIFI_PACKET_WAIT_SEND ) {
        gpio_set_level(BLINK_GPIO,0);
        xEventGroupClearBits(s_wifi_event_group, WIFI_PACKET_WAIT_SEND);
      }
    }
  }
}

void wifi_send_packet(const char * buffer, size_t size) {
  if (transportMode == WIFI_TRANSPORT_UDP) {
    // Fail-fast, drop-on-error datagram send. Dropping (rather than blocking to
    // retry, as TCP does) is exactly what keeps a stalled host from starving the
    // WiFi driver's heap and killing the radio - the reason UDP survives raw
    // streaming. Never blocks the GAP8 route.
    if (serverSock < 0 || !udpClientKnown) {
      return;
    }
    wifiSendCalls++;
    struct sockaddr_in target;
    xSemaphoreTake(udpTargetLock, portMAX_DELAY);
    target = udpDestAddr;
    xSemaphoreGive(udpTargetLock);
    int sent = sendto(serverSock, buffer, size, 0,
                      (struct sockaddr *)&target, sizeof(target));
    if (sent > 0) {
      wifiBytesWritten += sent;
    } else {
      ESP_LOGD(TAG, "UDP send dropped: errno %d", errno);
    }
    return;
  }

  if (clientConnection == NO_CONNECTION) {
    return;
  }
  ESP_LOGD(TAG, "Sending WiFi packet of size %u", size);
  wifiSendCalls++;
  xEventGroupSetBits(s_wifi_event_group, WIFI_PACKET_SENDING);

  // send() may write only part of the buffer (SO_SNDTIMEO is set); the old
  // code ignored the byte count, silently truncating packets and desyncing
  // the host's length-prefixed stream. Loop until fully written, and give up
  // on a peer that stops draining for WIFI_SEND_TIMEOUT_MAX seconds.
  size_t written = 0;
  int timeouts = 0;
  while (written < size && clientConnection != NO_CONNECTION) {
    int sent = send(clientConnection, buffer + written, size - written, 0);
    if (sent > 0) {
      written += sent;
      wifiBytesWritten += sent;
      timeouts = 0;
    } else if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      timeouts++;
      if (timeouts == 1) {
        ESP_LOGW(TAG, "Send timeout (errno %d), heap %u (min %u)", errno,
                 (unsigned)esp_get_free_heap_size(), (unsigned)esp_get_minimum_free_heap_size());
      }
      if (timeouts >= WIFI_SEND_TIMEOUT_MAX) {
        ESP_LOGE(TAG, "Send stalled for %d s, closing connection", WIFI_SEND_TIMEOUT_MAX);
        close_client_socket();
      }
    } else {
      ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
      close_client_socket();
    }
  }
  xEventGroupSetBits(s_wifi_event_group, WIFI_PACKET_WAIT_SEND);
}

static void wifi_sending_task(void *pvParameters) {
  static WifiTransportPacket_t txp_wifi;
  static CPXRoutablePacket_t qPacket;

  xEventGroupSetBits(startUpEventGroup, START_UP_TX_TASK);
  while (1) {
    xQueueReceive(wifiTxQueue, &qPacket, portMAX_DELAY);
    wifiTxDequeued++;

    txp_wifi.payloadLength = qPacket.dataLength + CPX_ROUTING_PACKED_SIZE;

    cpxRouteToPacked(&qPacket.route, &txp_wifi.routablePayload.route);

    memcpy(txp_wifi.routablePayload.data, qPacket.data, qPacket.dataLength);

    wifi_send_packet((const char *)&txp_wifi, txp_wifi.payloadLength + 2);
  }
}

static void wifi_receiving_task(void *pvParameters) {
  static WifiTransportPacket_t rxp_wifi;
  int len = 0;

  xEventGroupSetBits(startUpEventGroup, START_UP_RX_TASK);
  while (1) {
    if (transportMode == WIFI_TRANSPORT_UDP) {
      if (serverSock < 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }
      struct sockaddr_in src;
      socklen_t srcLen = sizeof(src);
      int len = recvfrom(serverSock, &rxp_wifi, sizeof(rxp_wifi), 0,
                         (struct sockaddr *)&src, &srcLen);
      if (len <= 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }

      // "FER" magic: the host announcing its address (and, on the host's read
      // timeout, re-announcing). Lock onto its source addr:port and repeat the
      // idempotent GAP8 notification as well. A control packet can be lost in
      // the shallow CPX queues; suppressing later notifications then leaves the
      // ESP with a client while the GAP8 waits forever and emits no first frame.
      if (len == 3 && memcmp(&rxp_wifi, "FER", 3) == 0) {
        xSemaphoreTake(udpTargetLock, portMAX_DELAY);
        udpDestAddr = src;
        xSemaphoreGive(udpTargetLock);
        bool wasKnown = udpClientKnown;
        udpClientKnown = true;
        if (!wasKnown) {
          uint32_t a = src.sin_addr.s_addr;
          ESP_LOGI(TAG, "UDP client %u.%u.%u.%u:%u",
                   (unsigned)(a & 0xff), (unsigned)((a >> 8) & 0xff),
                   (unsigned)((a >> 16) & 0xff), (unsigned)((a >> 24) & 0xff),
                   (unsigned)ntohs(src.sin_port));
        }
        wifi_signal_client_connected(1);
        continue;
      }

      // Genuine host->deck CPX packet: 2-byte length prefix + payload, one whole
      // packet per datagram. Validate strictly against the datagram bounds - the
      // LARICS fork trusted the wire length blindly, so a short/garbage datagram
      // underflowed dataLength in wifi_transport_receive and memcpy'd ~64 KB.
      if (len < 2) {
        continue;
      }
      if (rxp_wifi.payloadLength != (uint16_t)(len - 2) ||
          rxp_wifi.payloadLength < CPX_ROUTING_PACKED_SIZE ||
          rxp_wifi.payloadLength > WIFI_TRANSPORT_MTU) {
        ESP_LOGD(TAG, "Dropping malformed UDP packet: len %d, wire %u",
                 len, rxp_wifi.payloadLength);
        continue;
      }
      xQueueSend(wifiRxQueue, &rxp_wifi, portMAX_DELAY);
      continue;
    }

    if (clientConnection == NO_CONNECTION) {
      vTaskDelay(10);
      continue;
    }

    // Read the exact 2-byte length header; a short read here (or an error in
    // the payload loop below, which the old code never checked) permanently
    // desynced the host->deck stream.
    int headerLen = 0;
    while (headerLen < 2) {
      len = recv(clientConnection, ((uint8_t*)&rxp_wifi) + headerLen, 2 - headerLen, 0);
      if (len <= 0) {
        break;
      }
      headerLen += len;
    }
    if (headerLen < 2) {
      if (len == 0) {
        close_client_socket();  //Reading 0 bytes most often means the client has disconnected.
      } else {
        vTaskDelay(10);
      }
      continue;
    }

    if (rxp_wifi.payloadLength < CPX_ROUTING_PACKED_SIZE ||
        rxp_wifi.payloadLength > WIFI_TRANSPORT_MTU) {
      ESP_LOGE(TAG, "Bad wire length %i, closing connection", rxp_wifi.payloadLength);
      close_client_socket();
      continue;
    }

    ESP_LOGD(TAG, "Wire data length %i", rxp_wifi.payloadLength);
    int totalRxLen = 0;
    while (totalRxLen < rxp_wifi.payloadLength) {
      len = recv(clientConnection, &rxp_wifi.payload[totalRxLen], rxp_wifi.payloadLength - totalRxLen, 0);
      if (len <= 0) {
        break;
      }
      ESP_LOGD(TAG, "Read %i bytes", len);
      totalRxLen += len;
    }
    if (totalRxLen < rxp_wifi.payloadLength) {
      // Mid-packet EOF or error: the stream cannot be re-synced, drop the client.
      close_client_socket();
      continue;
    }

    ESP_LOG_BUFFER_HEX_LEVEL(TAG, &rxp_wifi, 10, ESP_LOG_DEBUG);
    xQueueSend(wifiRxQueue, &rxp_wifi, portMAX_DELAY);
  }
}

void wifi_transport_send(const CPXRoutablePacket_t* packet) {
  assert(packet->dataLength <= WIFI_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);
  xQueueSend(wifiTxQueue, packet, portMAX_DELAY);
  wifiTxEnqueued++;
}

void wifi_transport_receive(CPXRoutablePacket_t* packet) {
  // Not reentrant safe. Assuming only one task is popping the queue
  static WifiTransportPacket_t qPacket;
  xQueueReceive(wifiRxQueue, &qPacket, portMAX_DELAY);

  // Clamp before subtracting CPX_ROUTING_PACKED_SIZE: an out-of-range wire length
  // would otherwise underflow dataLength (uint16) to ~64 KB and overrun the
  // memcpy below. The RX tasks already reject these, but keep the guard here so
  // a bad enqueue can never corrupt memory.
  uint16_t payloadLength = qPacket.payloadLength;
  if (payloadLength < CPX_ROUTING_PACKED_SIZE) {
    payloadLength = CPX_ROUTING_PACKED_SIZE;
  } else if (payloadLength > WIFI_TRANSPORT_MTU) {
    payloadLength = WIFI_TRANSPORT_MTU;
  }
  packet->dataLength = payloadLength - CPX_ROUTING_PACKED_SIZE;

  cpxPackedToRoute(&qPacket.routablePayload.route, &packet->route);

  memcpy(packet->data, qPacket.routablePayload.data, packet->dataLength);
}

void wifi_init() {
  esp_netif_init();

  s_wifi_event_group = xEventGroupCreate();

  wifiRxQueue = xQueueCreate(WIFI_HOST_QUEUE_LENGTH, sizeof(WifiTransportPacket_t));
  wifiTxQueue = xQueueCreate(WIFI_HOST_QUEUE_LENGTH, sizeof(CPXRoutablePacket_t));

  socketCloseLock = xSemaphoreCreateMutex();
  udpTargetLock = xSemaphoreCreateMutex();
  serveReadySem = xSemaphoreCreateBinary();

  startUpEventGroup = xEventGroupCreate();
  xEventGroupClearBits(startUpEventGroup, START_UP_MAIN_TASK | START_UP_RX_TASK | START_UP_TX_TASK | START_UP_CTRL_TASK);
  xTaskCreate(wifi_task, "WiFi TASK", 5000, NULL, 1, NULL);
  xTaskCreate(wifi_sending_task, "WiFi TX", 5000, NULL, 1, NULL);
  xTaskCreate(wifi_receiving_task, "WiFi RX", 5000, NULL, 1, NULL);
  xTaskCreate(wifi_led_task, "WiFi LED", 5000, NULL, 1, NULL);
  xTaskCreate(wifi_status_task, "WiFi STATUS", 3000, NULL, 1, NULL);
  ESP_LOGI(TAG, "Waiting for main, RX and TX tasks to start");
  xEventGroupWaitBits(startUpEventGroup,
                      START_UP_MAIN_TASK | START_UP_RX_TASK | START_UP_TX_TASK,
                      pdTRUE, // Clear bits before returning
                      pdTRUE, // Wait for all bits
                      portMAX_DELAY);

  xTaskCreate(wifi_ctrl, "WiFi CTRL", 5000, NULL, 1, NULL);
  ESP_LOGI(TAG, "Waiting for CTRL task to start");
  xEventGroupWaitBits(startUpEventGroup,
                      START_UP_CTRL_TASK,
                      pdTRUE, // Clear bits before returning
                      pdTRUE, // Wait for all bits
                      portMAX_DELAY);

  ESP_LOGI("WIFI", "Wifi initialized");
}
