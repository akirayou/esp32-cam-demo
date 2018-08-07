// Copyright 2015-2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "driver/gpio.h"
#include "camera.h"
#include "bitmap.h"
#include "http_server.h"
#include "move_detect.h"
#define USE_SLEEP 0
/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_MODE_AP CONFIG_ESP_WIFI_MODE_AP  // TRUE:AP FALSE:STA
#define EXAMPLE_ESP_WIFI_SSID CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_MAX_STA_CONN CONFIG_MAX_STA_CONN
#define CAMERA_LED_GPIO 16

#if EXAMPLE_ESP_WIFI_MODE_AP
static void wifi_init_softap(void);
#else
static void wifi_init_sta(void);
#endif 
static void cameraStop();
static void cameraStart();

static void handle_jpg(http_context_t http_ctx, void* ctx);
static void handle_reset(http_context_t http_ctx, void* ctx);
static void handle_jpg_stream(http_context_t http_ctx, void* ctx);
static esp_err_t event_handler(void *ctx, system_event_t *event);


static const char* TAG = "camera_demo";

static const char* STREAM_CONTENT_TYPE =
        "multipart/x-mixed-replace; boundary=123456789000000000000987654321";

static const char* STREAM_BOUNDARY = "--123456789000000000000987654321";

static EventGroupHandle_t s_wifi_event_group;
const int CONNECTED_BIT = BIT0;
static ip4_addr_t s_ip_addr;
static camera_pixelformat_t s_pixel_format;

#define CAMERA_PIXEL_FORMAT CAMERA_PF_GRAYSCALE
#define CAMERA_FRAME_SIZE CAMERA_FS_SVGA
static xSemaphoreHandle _cameraLock =NULL;
void cameraLockInit(){
    vSemaphoreCreateBinary(_cameraLock);
}
void cameraLock(){
    xSemaphoreTake(_cameraLock,portMAX_DELAY);
}
void cameraUnlock(){
    xSemaphoreGive(_cameraLock);
}





#include "esp_http_client.h"
#define MAX_HTTP_RECV_BUFFER 100

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (!esp_http_client_is_chunked_response(evt->client)) {
                // Write out data
                 printf("%.*s", evt->data_len, (char*)evt->data);
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
    }
    return ESP_OK;
}



void http_post(uint8_t *data,size_t len,short max_s){
    
     esp_http_client_config_t config = {
        .url = CONFIG_UPLOAD_URL ,
        .event_handler = _http_event_handler,
        .method = HTTP_METHOD_POST,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    char strBuf[10];
    sprintf(strBuf,"%d",max_s);
    esp_http_client_set_header(client, "Content-Type", "application/octet-stream");
    esp_http_client_set_header(client, "X-MaxS", strBuf);
    esp_http_client_set_post_field(client, (char*)data, len);
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
}

bool main_loop(void){
    static int count=0;
    static TickType_t lastTick=0;
    TickType_t tickFromLast=xTaskGetTickCount()-lastTick;
    lastTick=xTaskGetTickCount();
    TickType_t waitTick=100/portTICK_PERIOD_MS-tickFromLast;
    if(waitTick<1000/portTICK_PERIOD_MS)vTaskDelay(waitTick);
    count++;
    if(count>60*5){
        count=0;
        cameraStop();
        cameraStart();
    }

    uint8_t isDetect=0;
    cameraLock();
#if USE_SLEEP
    camera_sleep(0);
    for(int i=0;i<2;i++)wait_vsync();//skip some frames to get sable frame
#endif
    esp_err_t err = camera_run();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera capture failed with error = %d", err);
        esp_restart();
#if USE_SLEEP
        camera_sleep(1);
#endif
    }else{ 
#if USE_SLEEP
        camera_sleep(1);
#endif
        uint8_t *jpgData = camera_get_fb();
        size_t jpgSize = camera_get_data_size();
        unsigned short max_s;
        max_s=detect_move(jpgData,jpgSize);
        isDetect = max_s>CONFIG_MAX_S_THRESH; 
        ESP_LOGI(TAG,"Detect: max_s %d",max_s);
        if(isDetect){
            ESP_LOGI(TAG,"Move !\n");
            http_post(jpgData,jpgSize,max_s);
        }
    }
    cameraUnlock();

    return true;
}
static void cameraStop(){
    cameraLock();
    camera_deinit();
}
static void cameraStart(){
    esp_err_t err;
    camera_config_t camera_config = {
        .ledc_channel = LEDC_CHANNEL_0,
        .ledc_timer = LEDC_TIMER_0,
        .pin_d0 = CONFIG_D0,
        .pin_d1 = CONFIG_D1,
        .pin_d2 = CONFIG_D2,
        .pin_d3 = CONFIG_D3,
        .pin_d4 = CONFIG_D4,
        .pin_d5 = CONFIG_D5,
        .pin_d6 = CONFIG_D6,
        .pin_d7 = CONFIG_D7,
        .pin_xclk = CONFIG_XCLK,
        .pin_pclk = CONFIG_PCLK,
        .pin_vsync = CONFIG_VSYNC,
        .pin_href = CONFIG_HREF,
        .pin_sscb_sda = CONFIG_SDA,
        .pin_sscb_scl = CONFIG_SCL,
        .pin_reset = CONFIG_RESET,
        .xclk_freq_hz = CONFIG_XCLK_FREQ,
    };

    camera_model_t camera_model;
    err = camera_probe(&camera_config, &camera_model);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera probe failed with error 0x%x", err);
        return;
    }

    if (camera_model == CAMERA_OV7725) {
        s_pixel_format = CAMERA_PIXEL_FORMAT;
        camera_config.frame_size = CAMERA_FRAME_SIZE;
        ESP_LOGI(TAG, "Detected OV7725 camera, using %s bitmap format",
                CAMERA_PIXEL_FORMAT == CAMERA_PF_GRAYSCALE ?
                        "grayscale" : "RGB565");
    } else if (camera_model == CAMERA_OV2640) {
        ESP_LOGI(TAG, "Detected OV2640 camera, using JPEG format");
        s_pixel_format = CAMERA_PF_JPEG;
        camera_config.frame_size = CAMERA_FRAME_SIZE;
        camera_config.jpeg_quality = 15;
    } else {
        ESP_LOGE(TAG, "Camera not supported");
        return;
    }
   
    ESP_LOGI(TAG, "Free heap: %u  and lagest is %d", xPortGetFreeHeapSize(),heap_caps_get_largest_free_block(MALLOC_CAP_32BIT ) );
    camera_config.pixel_format = s_pixel_format;
    err = camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return;
    }
    assert(s_pixel_format == CAMERA_PF_JPEG);
    for(int i=0;i<10;i++)wait_vsync();//skip soe frames to get sable frame
#if USE_SLEEP
    camera_sleep(1); 
#endif
    cameraUnlock();
}

void app_main()
{
    esp_log_level_set("wifi", ESP_LOG_WARN);
    esp_log_level_set("gpio", ESP_LOG_WARN);
    esp_err_t err;
    err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ESP_ERROR_CHECK( nvs_flash_init() );
    }
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    gpio_set_direction(CAMERA_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CAMERA_LED_GPIO, 1);
    cameraLockInit();
    cameraStart();

#if EXAMPLE_ESP_WIFI_MODE_AP
    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    wifi_init_softap();
#else
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
#endif
    http_server_t server;
    http_server_options_t http_options = HTTP_SERVER_OPTIONS_DEFAULT();
    http_options.task_priority=10;// handling camera data must be high priority
    http_options.task_stack_size=2048;
    ESP_ERROR_CHECK( http_server_start(&http_options, &server) );

    ESP_ERROR_CHECK( http_register_handler(server, "/jpg", HTTP_GET, HTTP_HANDLE_RESPONSE, &handle_jpg, NULL) );
    ESP_LOGI(TAG, "Open http://" IPSTR "/jpg for single image/jpg image", IP2STR(&s_ip_addr));
    ESP_ERROR_CHECK( http_register_handler(server, "/reset", HTTP_GET, HTTP_HANDLE_RESPONSE, &handle_reset, NULL) );
    ESP_LOGI(TAG, "Open http://" IPSTR "/reset for reset camera", IP2STR(&s_ip_addr));
    ESP_ERROR_CHECK( http_register_handler(server, "/jpg_stream", HTTP_GET, HTTP_HANDLE_RESPONSE, &handle_jpg_stream, NULL) );
    ESP_LOGI(TAG, "Open http://" IPSTR "/jpg_stream for multipart/x-mixed-replace stream of JPEGs", IP2STR(&s_ip_addr));
    ESP_ERROR_CHECK( http_register_handler(server, "/", HTTP_GET, HTTP_HANDLE_RESPONSE, &handle_jpg_stream, NULL) );
    ESP_LOGI(TAG, "Camera demo ready");
    gpio_set_level(CAMERA_LED_GPIO, 0);

    
    detect_move_init();
    vTaskPrioritySet(0,20);
    while(main_loop());

}




static esp_err_t write_frame(http_context_t http_ctx)
{
    http_buffer_t fb_data = {
            .data = camera_get_fb(),
            .size = camera_get_data_size(),
            .data_is_persistent = true
    };
    return http_response_write(http_ctx, &fb_data);
}
static void handle_reset(http_context_t http_ctx, void* ctx)
{
    cameraStop();
    cameraStart();
        http_buffer_t fb_data = {
            .data = "reset done",
            .size = 10,
            .data_is_persistent = true
    };
    http_response_write(http_ctx, &fb_data);
}
static void handle_jpg(http_context_t http_ctx, void* ctx)
{
    cameraLock();
#if USE_SLEEP
    camera_sleep(0);
    for(int i=0;i<1;i++)wait_vsync();//skip soe frames to get sable frame
#endif
    esp_err_t err = camera_run();
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "Camera capture failed with error = %d", err);
#if USE_SLEEP
        camera_sleep(1);
#endif
    }else{ 
#if USE_SLEEP
        camera_sleep(1);
#endif
        http_response_begin(http_ctx, 200, "image/jpeg", camera_get_data_size());
        http_response_set_header(http_ctx, "Content-disposition", "inline; filename=capture.jpg");
        write_frame(http_ctx);
        http_response_end(http_ctx);
    }
    cameraUnlock();

}

static void handle_jpg_stream(http_context_t http_ctx, void* ctx)
{
    cameraLock();
    //gpio_set_level(CAMERA_LED_GPIO, 1);
    http_response_begin(http_ctx, 200, STREAM_CONTENT_TYPE, HTTP_RESPONSE_SIZE_UNKNOWN);
#if USE_SLEEP
    camera_sleep(0);
#endif
    while (true) {
        esp_err_t err = camera_run();
        if (err != ESP_OK) {
            ESP_LOGD(TAG, "Camera capture failed with error = %d", err);
#if USE_SLEEP
            camera_sleep(1);
#endif
            break;
        }
        err = http_response_begin_multipart(http_ctx, "image/jpg",
                camera_get_data_size());
        if (err != ESP_OK) {
            break;
        }
        err = write_frame(http_ctx);
        if (err != ESP_OK) {
            break;
        }
        err = http_response_end_multipart(http_ctx, STREAM_BOUNDARY);
        if (err != ESP_OK) {
            break;
        }
    }
    http_response_end(http_ctx);
#if USE_SLEEP
    camera_sleep(1);
#endif
    cameraUnlock();
    //gpio_set_level(CAMERA_LED_GPIO, 0);
}


// /* FreeRTOS event group to signal when we are connected*/
// static EventGroupHandle_t s_wifi_event_group;

// /* The event group allows multiple bits for each event,
//    but we only care about one event - are we connected
//    to the AP with an IP? */
// const int WIFI_CONNECTED_BIT = BIT0;

static esp_err_t event_handler(void* ctx, system_event_t* event) 
{
  switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
      esp_wifi_connect();
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      ESP_LOGI(TAG, "got ip:%s", ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
      s_ip_addr = event->event_info.got_ip.ip_info.ip;
      xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
      break;
    case SYSTEM_EVENT_AP_STACONNECTED:
      ESP_LOGI(TAG, "station:" MACSTR " join, AID=%d", MAC2STR(event->event_info.sta_connected.mac),
               event->event_info.sta_connected.aid);
#if EXAMPLE_ESP_WIFI_MODE_AP
      xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
#endif
      break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
      ESP_LOGI(TAG, "station:" MACSTR "leave, AID=%d", MAC2STR(event->event_info.sta_disconnected.mac),
               event->event_info.sta_disconnected.aid);
#if EXAMPLE_ESP_WIFI_MODE_AP
      xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
#endif
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      esp_wifi_connect();
      xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
      break;
    default:
      break;
  }
  return ESP_OK;
}

#if EXAMPLE_ESP_WIFI_MODE_AP

static void wifi_init_softap() 
{
  s_wifi_event_group = xEventGroupCreate();

  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  wifi_config_t wifi_config = {
      .ap = {.ssid = EXAMPLE_ESP_WIFI_SSID,
             .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
             .password = EXAMPLE_ESP_WIFI_PASS,
             .max_connection = EXAMPLE_MAX_STA_CONN,
             .authmode = WIFI_AUTH_WPA_WPA2_PSK},
  };
  if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;
  }

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  uint8_t addr[4] = {192, 168, 4, 1};
  s_ip_addr = *(ip4_addr_t*)&addr;

  ESP_LOGI(TAG, "wifi_init_softap finished.SSID:%s password:%s",
           EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}

#else

static void wifi_init_sta() 
{
    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .sta = {.ssid = EXAMPLE_ESP_WIFI_SSID, .password = EXAMPLE_ESP_WIFI_PASS},
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s", EXAMPLE_ESP_WIFI_SSID,
            EXAMPLE_ESP_WIFI_PASS);
    
    xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
}
#endif