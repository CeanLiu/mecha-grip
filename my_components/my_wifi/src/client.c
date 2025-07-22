#include "client.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "freertos/semphr.h"
#include "esp_http_client.h"

static const char *TAG = "IMU_CLIENT";

// Semaphore handle
static SemaphoreHandle_t wifi_semaphore = NULL;

// Event handler for Wi-Fi and IP events
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
    {
        ESP_LOGI(TAG, "Wi-Fi Connected");
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGW(TAG, "Wi-Fi Disconnected. Reconnecting...");
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        // Give the semaphore once we get the IP
        xSemaphoreGive(wifi_semaphore);
    }
}

// Initialize Wi-Fi with the given SSID and password
esp_err_t init_wifi_sta()
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init));

    // Create the semaphore to synchronize Wi-Fi connection
    wifi_semaphore = xSemaphoreCreateBinary();
    if (wifi_semaphore == NULL)
    {
        ESP_LOGE(TAG, "Failed to create semaphore");
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &wifi_event_handler,
        NULL,
        NULL));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &wifi_event_handler,
        NULL,
        NULL));

    wifi_config_t wifi_config = {};
    snprintf((char *)wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid), "%s",  WIFI_SSID);
    snprintf((char *)wifi_config.sta.password, sizeof(wifi_config.sta.password), "%s",  WIFI_PASS);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_connect();

    ESP_LOGI(TAG, "Connecting to SSID: %s",  WIFI_SSID);

    // Wait for the Wi-Fi connection and IP address
    if (xSemaphoreTake(wifi_semaphore, portMAX_DELAY) == pdTRUE)
    {
        ESP_LOGI(TAG, "Wi-Fi connected and got IP, returning from init_wifi_sta");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to connect to Wi-Fi");
        return ESP_FAIL;
    }

    // Clean up
    vSemaphoreDelete(wifi_semaphore);

    return ESP_OK;
}
esp_err_t post_angles(float p1, float p2, float p3, float p4) {
    char body[64];
    snprintf(body, sizeof body, "p1:%.1f,p2:%.1f,p3:%.1f,p4:%.1f", p1, p2, p3, p4);
    esp_http_client_config_t c = {
        .url = SERVER_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 1000
    };
    esp_http_client_handle_t h = esp_http_client_init(&c);
    esp_http_client_set_header(h, "Content-Type", "text/plain");
    esp_http_client_set_post_field(h, body, strlen(body));
    esp_err_t err = esp_http_client_perform(h);
    esp_http_client_cleanup(h);
    return err;
}