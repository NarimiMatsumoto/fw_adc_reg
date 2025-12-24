
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "esp_event.h"
#include "esp_log.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include <sys/socket.h>
#include "esp_cache.h"

void start_websocket_server(void);

extern int ak5816_start_up_demo(uint8_t *p_reg_page);  // 関数の外部宣言
extern uint8_t reg_page;  // reg_page の外部変数宣言

#define U_STATIC_IP_ADDR        "192.168.2.202"
#define U_STATIC_NETMASK_ADDR   "255.255.255.0"
#define U_STATIC_GW_ADDR        "192.168.2.200"
#define U_MAIN_DNS_SERVER       U_STATIC_GW_ADDR
#define U_BACKUP_DNS_SERVER     "0.0.0.0"
#define U_HOST_IP_ADDR          "192.168.2.201"
#define U_PORT                  40001

#define U_RMII_CLK_GPIO 44
#define U_RMII_RXD1_GPIO 47
#define U_RMII_RXD0_GPIO 46
#define U_RMII_RXDV_GPIO 45
#define U_RMII_TXD1_GPIO 42
#define U_RMII_TXD0_GPIO 41
#define U_RMII_TXEN_GPIO 40


static esp_netif_t *u_eth_start(void);
static esp_err_t u_set_dns_server(esp_netif_t *netif, uint32_t addr, esp_netif_dns_type_t type);
static void u_set_static_ip(esp_netif_t *netif);
static void eth_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
void u_tcp_client(void);

static const char *TAG = "u_tcp";
static esp_eth_handle_t eth_handle = NULL;
static esp_eth_mac_t *u_mac = NULL;
static esp_eth_phy_t *u_phy = NULL;
static esp_eth_netif_glue_handle_t s_eth_glue = NULL;
int tcp_sock;

volatile static uint8_t *tcp_status;
extern void *u_st_buf;
extern size_t u_st_buf_len;

void tcp_start(void)
{
    tcp_status = (uint8_t *)u_st_buf;
    *tcp_status = 0;
    esp_cache_msync(u_st_buf, u_st_buf_len, ESP_CACHE_MSYNC_FLAG_DIR_C2M);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    u_eth_start();

    ESP_LOGI(TAG, "wait!!!");
    vTaskDelay(500);

    //u_tcp_client();
    start_websocket_server();
}

static esp_netif_t *u_eth_start(void)
{
    esp_netif_inherent_config_t netif_cfg = ESP_NETIF_INHERENT_DEFAULT_ETH();

    netif_cfg.if_desc = EXAMPLE_NETIF_DESC_ETH;
    netif_cfg.route_prio = 64;
    esp_netif_config_t netif_cfg2 = {
        .base = &netif_cfg,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH
    };
    esp_netif_t *netif = esp_netif_new(&netif_cfg2);
    assert(netif);

    eth_mac_config_t eth_mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    eth_mac_cfg.rx_task_stack_size = CONFIG_EXAMPLE_ETHERNET_EMAC_TASK_STACK_SIZE;
    eth_phy_config_t eth_phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    eth_phy_cfg.phy_addr = CONFIG_EXAMPLE_ETH_PHY_ADDR;
    eth_phy_cfg.reset_gpio_num = CONFIG_EXAMPLE_ETH_PHY_RST_GPIO;

    eth_esp32_emac_config_t esp32_emac_cfg = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    esp32_emac_cfg.smi_gpio.mdc_num = CONFIG_EXAMPLE_ETH_MDC_GPIO;
    esp32_emac_cfg.smi_gpio.mdio_num = CONFIG_EXAMPLE_ETH_MDIO_GPIO;

    // specify GPIO pins
    esp32_emac_cfg.clock_config.rmii.clock_gpio = (emac_rmii_clock_gpio_t) U_RMII_CLK_GPIO;
    esp32_emac_cfg.emac_dataif_gpio.rmii.tx_en_num = U_RMII_TXEN_GPIO;
    esp32_emac_cfg.emac_dataif_gpio.rmii.txd0_num = U_RMII_TXD0_GPIO;
    esp32_emac_cfg.emac_dataif_gpio.rmii.txd1_num = U_RMII_TXD1_GPIO;
    esp32_emac_cfg.emac_dataif_gpio.rmii.crs_dv_num = U_RMII_RXDV_GPIO;
    esp32_emac_cfg.emac_dataif_gpio.rmii.rxd0_num = U_RMII_RXD0_GPIO;
    esp32_emac_cfg.emac_dataif_gpio.rmii.rxd1_num = U_RMII_RXD1_GPIO;

    u_mac = esp_eth_mac_new_esp32(&esp32_emac_cfg, &eth_mac_cfg);
    //u_phy = esp_eth_phy_new_ip101(&eth_phy_cfg);
    u_phy = esp_eth_phy_new_generic(&eth_phy_cfg);

    esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(u_mac, u_phy);
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_cfg, &eth_handle));

    s_eth_glue = esp_eth_new_netif_glue(eth_handle);
    esp_netif_attach(netif, s_eth_glue);

    esp_event_handler_instance_t inst_any_id;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(ETH_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &eth_event_handler,
                                                        netif,
                                                        &inst_any_id));

    esp_eth_start(eth_handle);
    
    return netif;
}

IRAM_ATTR static esp_err_t u_set_dns_server(esp_netif_t *netif, uint32_t addr, esp_netif_dns_type_t type)
{
    if (addr && (addr != (uint32_t)0xffffffffUL)) {
        esp_netif_dns_info_t dns_inf;
        dns_inf.ip.u_addr.ip4.addr = addr;
        dns_inf.ip.type = 0;
        ESP_ERROR_CHECK(esp_netif_set_dns_info(netif, type, &dns_inf));
    }
    return ESP_OK;
}

IRAM_ATTR static void u_set_static_ip(esp_netif_t *netif)
{
    if (esp_netif_dhcpc_stop(netif) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop dhcp");
        return;
    }
    esp_netif_ip_info_t ip_inf;
    memset(&ip_inf, 0 , sizeof(esp_netif_ip_info_t));
    ip_inf.ip.addr = ipaddr_addr(U_STATIC_IP_ADDR);
    ip_inf.netmask.addr = ipaddr_addr(U_STATIC_NETMASK_ADDR);
    ip_inf.gw.addr = ipaddr_addr(U_STATIC_GW_ADDR);
    if (esp_netif_set_ip_info(netif, &ip_inf) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to esp_netif_set_ip_info");
        return;
    }
    ESP_LOGD(TAG, "Success to set static ip: %s, netmask: %s, gw: %s", U_STATIC_IP_ADDR, U_STATIC_NETMASK_ADDR, U_STATIC_GW_ADDR);
    ESP_ERROR_CHECK(u_set_dns_server(netif, ipaddr_addr(U_MAIN_DNS_SERVER), ESP_NETIF_DNS_MAIN));
    ESP_ERROR_CHECK(u_set_dns_server(netif, ipaddr_addr(U_BACKUP_DNS_SERVER), ESP_NETIF_DNS_BACKUP));
    *tcp_status = 1;
    esp_cache_msync(u_st_buf, u_st_buf_len, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
}

IRAM_ATTR static void eth_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == ETH_EVENT && event_id == ETHERNET_EVENT_CONNECTED)
    {
        u_set_static_ip(arg);
        ESP_LOGI(TAG, "set_static_ip.");
    }
    else if (event_base == ETH_EVENT && event_id == ETHERNET_EVENT_DISCONNECTED)
    {
        ESP_LOGI(TAG, "Ether is disconnected.");
        *tcp_status = 0;
        esp_cache_msync(u_st_buf, u_st_buf_len, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    }
}





