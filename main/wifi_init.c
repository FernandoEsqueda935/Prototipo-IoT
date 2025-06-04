#include "wifi_init.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "freeRTOS/FreeRTOS.h"
#include "freeRTOS/event_groups.h"
#include "nvs_flash.h"
#include "string.h"

#define TAG_WIFI "WIFI"

static const int CONNECTED_BIT = BIT0;
static EventGroupHandle_t wifi_event_group;



static void event_handler_wifi(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG_WIFI, "Intentando reconectar");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG_WIFI, "IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
    }
}

void wifi_init (const char * ssid, const char * password) {
    
    
    
    wifi_event_group = xEventGroupCreate();
    
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip; 
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler_wifi, NULL, &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler_wifi, NULL, &instance_got_ip);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "",
            .password = "",
        },
    };

    strcpy((char *) wifi_config.sta.ssid, ssid);
    strcpy((char *) wifi_config.sta.password, password);

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG_WIFI, "Conectando a %s", ssid);

    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, true, true, portMAX_DELAY);
}