/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <cstdio>
#include <cstring>
#include <cstdlib>

#include "neopixel.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi station";

static int s_retry_num = 0;

extern "C" void event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    if (s_retry_num < CONFIG_WIFI_MAXIMUM_RETRY) {
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGI(TAG, "retry to connect to the AP");
    } else {
      xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
    ESP_LOGI(TAG,"connect to the AP fail");
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
    ESP_LOGI(TAG, "got ip:%s",
	     ip4addr_ntoa(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

void wifi_init_sta()
{
  s_wifi_event_group = xEventGroupCreate();

  tcpip_adapter_init();

  ESP_ERROR_CHECK(esp_event_loop_create_default());

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
  wifi_config_t wifi_config;
  memset(&wifi_config, 0, sizeof(wifi_config));
  strcpy((char *)wifi_config.sta.ssid, CONFIG_WIFI_SSID);
  strcpy((char *)wifi_config.sta.password, CONFIG_WIFI_PASSWORD);
  wifi_config.sta.pmf_cfg.capable = true;
  wifi_config.sta.pmf_cfg.required = false;
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
  ESP_ERROR_CHECK(esp_wifi_start() );

  ESP_LOGI(TAG, "wifi_init_sta finished.");

  /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
   * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
					 WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
					 pdFALSE,
					 pdFALSE,
					 portMAX_DELAY);

  /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
   * happened. */
  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
	     CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);
  } else if (bits & WIFI_FAIL_BIT) {
    ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
	     CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);
  } else {
    ESP_LOGE(TAG, "UNEXPECTED EVENT");
  }

  ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
  ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
  vEventGroupDelete(s_wifi_event_group);
}

// ATOM LED

#define I2C_MASTER_SCL_IO               (gpio_num_t)21
#define I2C_MASTER_SDA_IO               (gpio_num_t)25
#define I2C_MASTER_NUM                  (i2c_port_t)I2C_NUM_0
#define I2C_MASTER_TX_BUF_DISABLE       0
#define I2C_MASTER_RX_BUF_DISABLE       0
#define I2C_MASTER_FREQ_HZ              400000

#define I2C_PORT                        I2C_MASTER_NUM
#define WRITE_BIT                       I2C_MASTER_WRITE
#define READ_BIT                        I2C_MASTER_READ
#define ACK_CHECK_EN                    0x1
#define ACK_CHECK_DIS                   0x0
#define ACK_VAL                         (i2c_ack_type_t)0x0
#define NACK_VAL                        (i2c_ack_type_t)0x1

void _I2CInit(void)
{
    i2c_port_t i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE,
                       0);
}

esp_err_t _I2CReadN(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1)|WRITE_BIT,
			  ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1)|READ_BIT,
			  ACK_CHECK_EN);
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, buf + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t _I2CWrite1(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t d = val;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1)|WRITE_BIT,
			  ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write(cmd, &d, 1, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Button GPIO
#define BUTTON_PIN GPIO_NUM_39

// NEOPIXEL
#define NEOPIXEL_PORT					GPIO_NUM_27
#define NR_LED							GPIO_NUM_25
#define NEOPIXEL_RMT_CHANNEL            RMT_CHANNEL_0

void led_task(void *arg)
{
    pixel_settings_t px;
    uint32_t pixels[NR_LED];
    int i;
    int rc;

    rc = neopixel_init(NEOPIXEL_PORT, NEOPIXEL_RMT_CHANNEL);
    printf("neopixel_init rc = %d", rc);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    for (i = 0; i < NR_LED; i++) {
        pixels[i] = 0;
    }
    px.pixels = (uint8_t *)pixels;
    px.pixel_count = NR_LED;
    strcpy(px.color_order, "GRB");

    memset(&px.timings, 0, sizeof(px.timings));
    px.timings.mark.level0 = 1;
    px.timings.space.level0 = 1;
    px.timings.mark.duration0 = 12;
    px.nbits = 24;
    px.timings.mark.duration1 = 14;
    px.timings.space.duration0 = 7;
    px.timings.space.duration1 = 16;
    px.timings.reset.duration0 = 600;
    px.timings.reset.duration1 = 600;

    px.brightness = 0x10;
    np_show(&px, NEOPIXEL_RMT_CHANNEL);
    
    int fact = 1;
    while (true) {
        vTaskDelay(10 / portTICK_PERIOD_MS);

        // tirangle wave
        for (int j = 0; j < NR_LED; j++) {
            np_set_pixel_rgbw(&px, j, i, i, i, 0);
        }
        np_show(&px, NEOPIXEL_RMT_CHANNEL);
        if (fact > 0) {
            i += 1;
        } else {
            i -= 1;
        }
        if (i == 255) {
            fact = -1;
        } else if ( i == 0 ) {
            fact = 1;
        }
    }
}

// For SPI connected HUB75E driver

#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK  GPIO_NUM_19
#define PIN_NUM_CS   GPIO_NUM_22

spi_device_handle_t spi_led;

void spi_init(void)
{
    esp_err_t ret;
    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.miso_io_num=-1;
    buscfg.mosi_io_num=PIN_NUM_MOSI;
    buscfg.sclk_io_num=PIN_NUM_CLK;
    buscfg.quadwp_io_num=-1;
    buscfg.quadhd_io_num=-1;
    buscfg.max_transfer_sz=0;

    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.command_bits=0;
    devcfg.address_bits=0;
    devcfg.dummy_bits=0;
    devcfg.clock_speed_hz=4000000;	//Clock out at 4MHz
    devcfg.duty_cycle_pos=128;
    devcfg.mode=0;                  //SPI mode 0
    devcfg.cs_ena_posttrans=1;
    devcfg.spics_io_num=-1;         //no hardware CS pin
    devcfg.queue_size=1;
    devcfg.flags=0;

    ret=spi_bus_initialize(VSPI_HOST, &buscfg, 0);
    assert(ret==ESP_OK);
    //Attach the slave devices to the SPI bus
    ret=spi_bus_add_device(VSPI_HOST, &devcfg, &spi_led);
    assert(ret==ESP_OK);
}

esp_err_t led_writen(uint8_t *buf, size_t len)
{
  esp_err_t ret;
  static spi_transaction_t trans;
  memset(&trans, 0, sizeof(spi_transaction_t));
  trans.length = 8*len;
  trans.tx_buffer = buf;
  //printf("do transfer\n");
  ret = spi_device_transmit(spi_led, &trans);
  return ret;
}

uint8_t ledbuf[64*64*2];

#define LED_RGB_565 2
#define LED_RGB_565_UPDATE 3
#define MAX_RGBBUF_SIZE 1280
struct rpacket {
    uint8_t header;
    uint8_t count;
    uint16_t offset;
    uint16_t length;
    uint16_t rgbbuf[MAX_RGBBUF_SIZE/2];
} rpkt;

uint16_t RGB555(uint16_t rgb565)
{
    uint16_t h = (rgb565 & 0x1f) | ((rgb565 >> 1) & ~0x1f);
    return (h >> 8) | (h << 8);
}

void set_ledbuf(struct rpacket *p)
{
    //printf("pckt offset %d length %d\n", p->offset, p->length);
    for (int i = 0; i < p->length; i++) {
        int idx = p->offset + i;
        if (idx >= 2048) {
            idx &= (2048-1);
            ((uint16_t *)ledbuf)[idx << 1] = RGB555(p->rgbbuf[i]);
        } else {
            ((uint16_t *)ledbuf)[(idx << 1) + 1] = RGB555(p->rgbbuf[i]);
        }
    }
}

#define SPI_LENGTH (8*4)

void udp_task(void *arg)
{
  wifi_init_sta();

  struct sockaddr_in caddr;
  struct sockaddr_in saddr;
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    printf("UDP socket can't be opened.\n");
    vTaskDelete(NULL);
  }

  saddr.sin_family = AF_INET;
  saddr.sin_addr.s_addr = htonl(INADDR_ANY);
  saddr.sin_port = htons(CONFIG_UDP_PORT);
  if (bind (sock, (struct sockaddr *)&saddr, sizeof(saddr)) < 0) {
    printf("Failed to bind UDP socket.\n");
    vTaskDelete(NULL);
  }

  while(true) {
    socklen_t clen;
    ssize_t n = recvfrom(sock, (char *)&rpkt, sizeof(rpkt), 0,
			 (struct sockaddr *)&caddr, &clen);
    if (n < 0) {
      printf("Failed to recv UDP socket.\n");
    }
    if (n < 6) {
      continue;
    }
    // handle packet header 0x02/0x03
    if (rpkt.header != LED_RGB_565 && rpkt.header != LED_RGB_565_UPDATE) {
      // ignore this packet
      printf("bad packet (size %d) start with %02x\n", n, rpkt.header);
      continue;
    }
    // TODO check count

    // Set ledbuf
    set_ledbuf(&rpkt);
    if (rpkt.header == LED_RGB_565_UPDATE) {
        //printf("end packet (idx %d)\n", rpkt.count);
        // end packet
        gpio_set_level(PIN_NUM_CS, 0);
        for (int i = 0; i < sizeof(ledbuf); i += SPI_LENGTH) {
            led_writen(&ledbuf[i], SPI_LENGTH);
        }
        gpio_set_level(PIN_NUM_CS, 1);
    } else {
        ;//printf("detect packet (count %d)\n", rpkt.count);
    }
  }
}

extern "C" void app_main()
{
    //Initialize NVS
    nvs_flash_init();

    gpio_reset_pin(PIN_NUM_CS);
    gpio_set_level(PIN_NUM_CS, 1);
    gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);

    spi_init();

    memset(ledbuf, 0, sizeof(ledbuf));

    
    xTaskCreate(led_task, "led_task", 8192, NULL, 2, NULL);
    xTaskCreate(udp_task, "udp_task", 8192, NULL, 3, NULL);

    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    while (true) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
