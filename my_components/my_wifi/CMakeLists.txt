idf_component_register(
        SRCS "src/client.c" "src/server.c"
        INCLUDE_DIRS "include"
		REQUIRES esp_wifi esp_http_client        
)