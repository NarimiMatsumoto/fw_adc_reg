idf_component_register(SRCS "main.c" "tcp_client.c"
                    PRIV_REQUIRES spi_flash
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_gpio esp_driver_uart esp_driver_spi esp_driver_isp esp_driver_cam esp_driver_i2c 
                    esp_event protocol_examples_common nvs_flash esp_netif esp_http_server)
