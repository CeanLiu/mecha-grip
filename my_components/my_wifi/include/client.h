#ifndef CLIENT_H
#define CLIENT_H

#include "esp_err.h"

#define WIFI_SSID "ServoESP"
#define WIFI_PASS "12345678"
#define SERVER_URL "http://192.168.4.1/update"

// Function to initialize Wi-Fi with the given SSID and password
esp_err_t init_wifi_sta();
esp_err_t post_angles(const float *angles, size_t len);
 
#endif // CLIENT_H
