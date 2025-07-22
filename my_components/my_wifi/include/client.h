#ifndef CLIENT_H
#define CLIENT_H

#include "esp_err.h"

#define WIFI_SSID "ServoESP"
#define WIFI_PASS "12345678"
#define SERVER_URL "http://192.168.4.1/update"

// Function to initialize Wi-Fi with the given SSID and password
esp_err_t init_wifi_sta();
esp_err_t post_angles(float p1, float p2, float p3, float p4);
 
#endif // CLIENT_H
