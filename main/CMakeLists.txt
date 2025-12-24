idf_component_register(SRCS "./src/main.c" "./src/tcp_client.c" "./src/func_pwm.c" "./src/func_pcnt.c" "./src/ktd2052.c"
                    PRIV_REQUIRES spi_flash
                    INCLUDE_DIRS "./include"
                    REQUIRES esp_driver_gpio esp_driver_uart esp_driver_spi esp_driver_isp esp_driver_cam esp_driver_i2c 
                    esp_driver_ledc esp_driver_pcnt
                    esp_event protocol_examples_common nvs_flash esp_netif esp_http_server)
