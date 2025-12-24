#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "soc/gpio_sig_map.h"
#include "driver/uart.h"
#include "soc/uart_reg.h"
#include "driver/i2c_master.h"
#include "soc/spi_periph.h"
#include "hal/spi_types.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "soc/cam_periph.h"
#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_log.h"
#include "esp_cache.h"
#include "driver/isp.h"
#include "esp_ldo_regulator.h"
#include "esp_sntp.h"
#include <sys/socket.h>
#include "esp_http_server.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_rom_sys.h"

#include "ktd2052.h"
#include "func_pwm.h"
#include "func_common.h"

#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#define U_USB_EN

//#define AD_SAMPLE_NUM 256 // ADC sampling point value
//#define AD_CHIRP_MULT 1   // Chirp cycles (The 1 mean 16 chirps)

httpd_handle_t server = NULL;
int client_fd = -1;

SemaphoreHandle_t start_sem;
volatile bool stop_requested = false;

//int sample_mode = 256;
int frame_limit = 0;
int frame_count = 0;

int ad_sample_num = 256;//AD_SAMPLE_NUM;
int ad_chirp_mult = 1;//AD_CHIRP_MULT;

typedef enum {
    v53Msps = 0,
    v20Msps = 1
} adc_fs_t;

adc_fs_t ad_fs = v53Msps;
bool ad_dfilbyps = false;

SemaphoreHandle_t param_sem;

static bool flg_pwmini = false;


static bool u_new_trans(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data);
static bool u_trans_finished(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data);
void delay(int delay);
void gpio_init(void);
void uart_init(void);
void i2c_init(void);
void spi_init(void);
void spi_send(uint16_t data);
void spi_sendread(uint16_t data, uint8_t* val);
void ak5816_write(uint8_t *p_reg_page, uint8_t page, uint8_t reg_addr, uint8_t data);
void ak5816_read(uint8_t *p_reg_page, uint8_t page, uint8_t reg_addr, uint8_t *buf);

extern void ak5816_write(uint8_t *p_reg_page, uint8_t page, uint8_t reg_addr, uint8_t data);
extern void ak5816_read(uint8_t *p_reg_page, uint8_t page, uint8_t reg_addr, uint8_t *buf);

#define CMD_UART_NUM UART_NUM_0
#define CMD_BUF_SIZE 128

#define BLOCK_CHUNK 16

//#define BIT(n) (1U << (n))

int ak5816_run_mrxg(uint8_t *p_reg_page);
int ak5816_start_up_demo(uint8_t *p_reg_page);
int ak5816_start_up_fftdemo(uint8_t *p_reg_page);
int ak5816_go_standby_demo(uint8_t *p_reg_page);
int ak5816_correction_cal_demo(uint8_t *p_reg_page);
int ak5816_go_trx_demo(uint8_t *p_reg_page);
int ak5816_go_slp_demo(uint8_t *p_reg_page);
void ak5816_error_read_demo(uint8_t *p_reg_page);
void send_data(uint8_t *send_buffp, uint32_t time, int mode);
void usb_init(void);
void usb_rx_callback(int itf, cdcacm_event_t *event);
void mes_send(void);
int u_read_bytes(uart_port_t uart_num, void *buf, uint32_t length, TickType_t ticks_to_wait);
int u_write_bytes(uart_port_t uart_num, const void *src, size_t size);
extern void tcp_start(void);
extern void u_tcp_client(void);
extern int tcp_sock;

static bool exec_mode = true;
static bool is_mrxg_enabled = false;
float trig_period_sub_ms = 1;
float trig_period_sub_ms_old = 1;
int trig_sub_times = 1;
float trig_period_ms = 200;

int64_t time_us2, time_us3, time_us4;
uint32_t time_min = 0xFFFFFFFF;
uint32_t time_max = 0;
int64_t time_sum = 0;
uint32_t time_num = 0;
uint32_t time_calc;
struct timeval u_tv;
int64_t time_us_int1;
struct timeval u_tv_int1;
uint8_t uarttxbuf[1024];
uint16_t uarttxbufmax = 1024;
uint16_t uarttxsize = 0;
i2c_master_dev_handle_t i2c_handle;
spi_device_handle_t spi_handle;
spi_transaction_t spi_transaction;
uint8_t val;
uint8_t ret = 0;
uint8_t reg_page = 0;
void *m_buffer = NULL;
size_t m_buffer_len = 0;
volatile int cntr1 = 0;
volatile int cntr2 = 0;
int cntr2_max = 1000000000;
int cntr2_pre = 0;
//uint16_t mipibuf[16384];
////#define MIPIBUF_MAX (256 * 16 * 80)
////static uint16_t mipibuf[MIPIBUF_MAX];
static uint16_t *mipibuf = NULL;
//uint8_t txdatabuf[24576+12];

//#define TXDATABUF_MAX (49152 + 512)
static size_t txdatabuf_size = 0;
static size_t mipibuf_len = 0;

uint8_t *txdatabuf = NULL; 
int txdatabufp = 0;
int error_flg = 0;
int test_cntr1 = 0;
void *fifo_buf = NULL;
size_t fifo_buf_len = 0;
volatile uint32_t fifo_inp = 0;
volatile uint32_t fifo_outp = 0;
volatile uint32_t timebuf[5120];
uint16_t timebuf_max = 5120; // 30MB/6144
volatile uint16_t timebuf_inp = 0;
volatile uint16_t timebuf_outp = 0;
uint16_t frame_cntr = 0;
void *u_st_buf = NULL;
size_t u_st_buf_len = 0;
uint8_t error_reg_buf[3] = {0,0,0};

typedef enum { MODE_ADC = 1, MODE_FFT = 2 } run_mode_t;
volatile run_mode_t active_mode = MODE_ADC;  // 初期はADCモード

#ifdef U_USB_EN
static uint8_t usb_rx_buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];
#endif
volatile uint8_t rxbuf[1024];
volatile uint16_t rxbufinp = 0;
volatile uint16_t rxbufoutp = 0;
uint16_t rxbufmask = 0x3FF;
uint8_t usb_error = 0;
bool usb_act = false;
uint8_t u_txbuf[1024];
uint16_t u_txbufmax = 1024;
uint16_t u_txsize = 0;

static void serial_cmd_task(void* arg) {
    uint8_t buf[128];
    //uint8_t current;
    int delay_ms;

    // int ad_sample_num = AD_SAMPLE_NUM;
    // int ad_chirp_mult = AD_CHIRP_MULT;

    while (1) {
        int len = u_read_bytes(CMD_UART_NUM, buf, sizeof(buf)-1, pdMS_TO_TICKS(100));
        if (len > 0) {

            buf[len] = '\0'; // Add null-terminator to treat buffer as a C string
            uint8_t page, reg, val;
            uint8_t start, count;
            float peri_sub, peri;
            int times;

            // PDN H/L
            if (strncmp((char*)buf, "PDN ", 4) == 0) {
                if (buf[4] == 'H') gpio_set_level(PDN, 1);
                else {
                    gpio_set_level(PDN, 0);
                    exec_mode = true;
                }
                u_write_bytes(CMD_UART_NUM, "PDN_OK\n", 7);
            }
            // RSTN H/L
            else if (strncmp((char*)buf, "RSTN ", 5) == 0) {
                if (buf[5] == 'H') gpio_set_level(RSTN, 1);
                else               gpio_set_level(RSTN, 0);
                u_write_bytes(CMD_UART_NUM, "RSTN_OK\n", 8);
            }
            // EXEC H/L
            else if (strncmp((char*)buf, "EXEC ", 5) == 0) {
                if (buf[5] == 'H') gpio_set_level(EXEC, 1);
                else               gpio_set_level(EXEC, 0);
                u_write_bytes(CMD_UART_NUM, "EXEC_OK\n", 8);
            }
            // Exec mode 1/0
            else if (strncmp((char*)buf, "EXEC_MODE ", 10) == 0) {
                if (buf[10] == '1') exec_mode = true;
                else               exec_mode = false;
                u_write_bytes(CMD_UART_NUM, "EXEC_MODE_OK\n", 13);
            }
            // TRIG Period (for sub-frame)
            else if (sscanf((char*)buf, "TRIG_SET_MS %f %d %f", &peri_sub, &times, &peri) == 3) {
                trig_period_sub_ms = peri_sub;
                uint32_t freq_hz = (uint32_t)(1000.0f / trig_period_sub_ms);
                if(flg_pwmini & (trig_period_sub_ms != trig_period_sub_ms_old)) {
                    PWM_SetFastFreq(freq_hz);
                    printf("[Debug]: Get Freq:%ld\n", PWM_GetFastFreq());
                    PWM_TrigLoop(2); //Dummy. Updating the frequency requires the peripheral block to be active.
                } else {
                    PWM_Initialize(CTL_PIN, freq_hz);
                    flg_pwmini = true;
                }
                trig_period_sub_ms_old = trig_period_sub_ms;
                trig_sub_times = times;
                trig_period_ms = peri;
                u_write_bytes(CMD_UART_NUM, "TRIG_SET_MS_OK\n", 15);
            }

            else if (sscanf((char*)buf, "DELAY %d", &delay_ms) == 1) {
                // delay_ms
                delay(delay_ms);
                //
                u_write_bytes(CMD_UART_NUM, "DELAY_OK\n", 9);
            }

            else if (strncmp((char*)buf, "GET_AD_SAMPLE_NUM", 17) == 0) {
                char resp[16];
                //int len = snprintf(resp, sizeof(resp), "%d\n", AD_SAMPLE_NUM);
                int len = snprintf(resp, sizeof(resp), "%d\n", ad_sample_num);
                u_write_bytes(CMD_UART_NUM, resp, len);
            }

            else if (strncmp((char*)buf, "GET_AD_CHIRP_MULT", 17) == 0) {
                char resp[16];
                //int len = snprintf(resp, sizeof(resp), "%d\n", AD_CHIRP_MULT);
                int len = snprintf(resp, sizeof(resp), "%d\n", ad_chirp_mult);
                u_write_bytes(CMD_UART_NUM, resp, len);
            }

            else if (strncmp((char*)buf, "SET_PARAM", 9) == 0) {
                int samp = 0, chirp = 0;
                if (sscanf((char*)buf, "SET_PARAM %d %d", &samp, &chirp) == 2) {
                    ad_sample_num = samp;
                    ad_chirp_mult = chirp;

                    u_write_bytes(CMD_UART_NUM, "SET_PARAM_OK\n", strlen("SET_PARAM_OK\n"));

                    xSemaphoreGive(param_sem);
                } else {
                    u_write_bytes(CMD_UART_NUM, "ERR:BAD_PARAM\n", strlen("ERR:BAD_PARAM\n"));
                }
            }           

            // 書き込みコマンド: "W pp rr vv"
            else if (sscanf((char*)buf, "W %2hhx %2hhx %2hhx", &page, &reg, &val) == 3) {
                // 1) 現在のページを把握
                uint8_t current;
                ak5816_read(&reg_page, 0, 0x02, &current);
                uint8_t current_page = current & 0x0F;

                // 2) ページ変更が必要か判定
                if (current_page != (page & 0x0F)) {
                    // (a) 上位ビットをゼロにして、下位4bitに page
                    uint8_t newcore = page & 0x0F;
                    ak5816_write(&reg_page, 0, 0x02, newcore);
                }

                // 3) 目的のレジスタに書き込み
                ak5816_write(&reg_page, page, reg, val);

                u_write_bytes(CMD_UART_NUM, (const char*)"OK\n", 3);
            }

            // 読み出しコマンド: "R pp rr"
            else if (sscanf((char*)buf, "R %2hhx %2hhx", &page, &reg) == 2) {
                // 1) 現在のページを把握
                uint8_t current;
                ak5816_read(&reg_page, 0, 0x02, &current);
                uint8_t current_page = current & 0x0F;

                // 2) ページ変更が必要か判定
                if (current_page != (page & 0x0F)) {
                    uint8_t newcore = page & 0x0F;
                    ak5816_write(&reg_page, 0, 0x02, newcore);
                }

                // 3) 目的のレジスタを読み出し
                uint8_t out;
                ak5816_read(&reg_page, page, reg, &out);
                char resp[8];
                int n = snprintf(resp, sizeof(resp), "%02X\n", out);
                u_write_bytes(CMD_UART_NUM, resp, n);
            }

            // ■ ブロック読み出し： "PR pp ss cc"
            else if (sscanf((char*)buf, "PR %2hhx %2hhx %2hhx", &page, &start, &count) == 3) {

                int remaining = count;
                int offset = 0;

                while (remaining > 0) {
                    int sub = (remaining > BLOCK_CHUNK) ? BLOCK_CHUNK : remaining;

                    char resp[3 * BLOCK_CHUNK + 2];
                    int pos = 0;
                    for (int i = 0; i < sub; i++) {
                        uint8_t out;
                        ak5816_read(&reg_page, page, start + offset + i, &out);
                        pos += snprintf(resp + pos, sizeof(resp) - pos, "%02X ", out);
                    }

                    if (pos > 0) resp[pos - 1] = '\n';
                    else          resp[0] = '\n';

                    u_write_bytes(CMD_UART_NUM, (const char*)resp, pos);

                    offset    += sub;
                    remaining -= sub;
                }
            }
            else if (strncmp((char*)buf, "BOOT", 4) == 0) {
                //u_write_bytes(CMD_UART_NUM, "BOOT_ESP32\n", 9);
                esp_restart();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        // ハンドシェイク成功時
        client_fd = httpd_req_to_sockfd(req);
        ESP_LOGI("ws_handler", "WebSocket connected: fd=%d", client_fd);
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    ws_pkt.payload = NULL;
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK || ws_pkt.len == 0) {
        return ret;
    }
    ws_pkt.payload = malloc(ws_pkt.len + 1);
    if (!ws_pkt.payload) return ESP_ERR_NO_MEM;
    ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
    if (ret == ESP_OK) {
        ((char*)ws_pkt.payload)[ws_pkt.len] = '\0';
        ESP_LOGI("ws_handler", "📩 受信: %s", (char*)ws_pkt.payload);

        // Transient Data Send Start Requested
        if (strncmp((char*)ws_pkt.payload, "START", 5) == 0) {
            // Parse samples and frames value from command
            int samples = 0;
            int frames = 0;
            sscanf((char*)ws_pkt.payload + 6, "%d %d", &samples, &frames);
            if (frames >= 0) {
                frame_limit = frames;
            }

            frame_count    = 0;
            stop_requested = false;
            active_mode    = MODE_ADC;        // ADCモードで開始
            frame_cntr     = 0;              // フレームカウンタもリセット
            fifo_outp      = fifo_inp;
            timebuf_outp   = timebuf_inp;
            xSemaphoreGive(start_sem);
        }
        else if (strncmp((char*)ws_pkt.payload, "FFTSTART", 8) == 0) {
            int frames = 0;
            sscanf((char*)ws_pkt.payload + 9, "%d", &frames);  // "FFTSTART "の後の数字を取得
            if (frames >= 0) {
                frame_limit = frames;
            }
            frame_count    = 0;
            stop_requested = false;
            active_mode    = MODE_FFT;       // FFTモードで開始
            frame_cntr     = 0;              // フレームカウンタリセット
            fifo_outp      = fifo_inp;
            timebuf_outp   = timebuf_inp;
            xSemaphoreGive(start_sem);
        }
        else if (strncmp((char*)ws_pkt.payload, "SET_PARAM", 9) == 0) {
            sscanf((char*)ws_pkt.payload + 10, "%d %d", &ad_sample_num, &ad_chirp_mult);
            ESP_LOGI("ws_handler", "ad_sample_num %d, ad_chirp_mult %d", ad_sample_num, ad_chirp_mult);
        }
        else if (strncmp((char*)ws_pkt.payload, "STOP", 4) == 0) {
            stop_requested = true;
        }
    }

    free(ws_pkt.payload);
    return ret;
}

void start_websocket_server(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 8765;
    config.uri_match_fn = httpd_uri_match_wildcard;

    ESP_ERROR_CHECK(httpd_start(&server, &config));

    httpd_uri_t ws_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = ws_handler,
        .user_ctx = NULL,
        .is_websocket = true
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &ws_uri));

    ESP_LOGI("websocket", "WebSocket server bootup: port number = %d", config.server_port);
}

void ak5816_update_reg_bits(uint8_t *p_page, uint8_t page, uint8_t addr,
                            uint8_t clear_mask, uint8_t set_mask)
{
    uint8_t val;
    ak5816_read(p_page, page, addr, &val);
    val = (val & ~clear_mask) | set_mask;
    ak5816_write(p_page, page, addr, val);
}

void ak5816_read_burst(uint8_t *reg_page, uint8_t page, uint8_t start_reg, uint8_t *buf, int len) {
    if (*reg_page != page) {
        spi_send((AK5816_CORE_PAGE_SETTING << 8) | (page & 0x0F));
        *reg_page = page;
    }

    // TX: アドレス(READ) + ダミー × lenバイト
    uint8_t txdata[16] = {0};  // 最大 15バイト程度まで対応可能
    uint8_t rxdata[16] = {0};
    txdata[0] = start_reg | 0x80;  // MSB=1でRead

    spi_transaction_t trans = {
        .flags = 0,
        .length = (len + 1) * 8,       // ビット長
        .tx_buffer = txdata,
        .rx_buffer = rxdata
    };

    esp_err_t err = spi_device_polling_transmit(spi_handle, &trans);
    if (err != ESP_OK) {
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"spi burst read error: %d\n", err);
        mes_send();
        return;
    }

    memcpy(buf, rxdata + 1, len);  // 最初の1バイトはダミー応答
}

/*
void ak5816_read_burst(uint8_t *reg_page, uint8_t page, uint8_t start_reg, uint8_t *buf, int len) {
    if (*reg_page != page) {
        spi_send((AK5816_CORE_PAGE_SETTING << 8) | (page & 0x0F));
        *reg_page = page;
    }

    spi_transaction_t trans = {
        .flags = 0,
        .length = (len + 1) * 8,
        .tx_buffer = NULL,
        .rx_buffer = buf
    };
    uint8_t txdata[7] = { start_reg | 0x80 };  // MSB=1でRead
    memset(txdata + 1, 0, len);
    uint8_t rxdata[7] = {0};

    trans.tx_buffer = txdata;
    trans.rx_buffer = rxdata;
    trans.length = (len + 1) * 8;

    //spi_device_polling_transmit(spi_handle, &trans);

    //memcpy(buf, rxdata + 1, len);  // 最初のバイトはダミー

    spi_device_queue_trans(spi_handle, &trans_bin, portMAX_DELAY);  // bin設定
    spi_device_queue_trans(spi_handle, &trans_read, portMAX_DELAY); // 6バイト読み出し

    spi_transaction_t *rtrans;
    for (int i = 0; i < 2; i++) {
        spi_device_get_trans_result(spi_handle, &rtrans, portMAX_DELAY);
        if (rtrans == &trans_read) {
            memcpy(iqbuf, rxbuf + 1, 6); // 最初はダミー
        }
    }    

}*/

void app_main(void)
{
    int i, j, k, m, n;
    esp_err_t err;
    i=0;j=0;k=0;m=0;n=0;
    i=i&j&k&m&n;

    uint32_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    printf("🧠 PSRAM available: %lu bytes\n", free_psram);    

    /*if (txdatabuf) {
        free(txdatabuf);
        txdatabuf = NULL;
    }

    txdatabuf = heap_caps_malloc(TXDATABUF_MAX, MALLOC_CAP_SPIRAM);
    if (!txdatabuf) {
        printf("❌ txdatabuf allocation failed\n");
        error_flg = 1;
        return;
    } else {
        printf("✅ txdatabuf = %p\n", txdatabuf);
    }*/

    // int ad_sample_num = AD_SAMPLE_NUM;
    // int ad_chirp_mult = AD_CHIRP_MULT;

    start_sem = xSemaphoreCreateBinary();

    param_sem = xSemaphoreCreateBinary();

    esp_log_level_set("u_tcp",       ESP_LOG_NONE);
    esp_log_level_set("esp_eth.netif.netif_glue", ESP_LOG_NONE);
    esp_log_level_set("esp_netif_handlers",       ESP_LOG_NONE);

    esp_log_level_set("set_static_ip.",       ESP_LOG_NONE);
    esp_log_level_set("Ether is disconnected.",       ESP_LOG_NONE);
    esp_log_level_set("Socket created, connecting to %s:%d",       ESP_LOG_NONE);
    esp_log_level_set("Socket unable to connect: errno %d",       ESP_LOG_NONE);
    esp_log_level_set("Successfully connected",       ESP_LOG_NONE);
    esp_log_level_set("wait!!!",       ESP_LOG_NONE);

    gpio_init();
    uart_init();
    i2c_init();
    spi_init();
    PCNT_Initialize();
#ifdef U_USB_EN
    usb_init();
#endif
    delay(500);


    uint8_t buf2b[2] = {0x00, 0x00}; 
    uint8_t buffer[2];

    for (int i=0; i<=15; i++) {
        buf2b[0] = i;
        ESP_ERROR_CHECK(i2c_master_transmit_receive(i2c_handle, buf2b, 1, buffer, 1, -1));
        uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"KTD2052 Addr%2d = %2x\n",i,buffer[0]);
        u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);
    }

    // KTD2052 LED
    KTD2052_EN_MODE ktd2052_lev = Night;
    ktd2052_set_global_level(ktd2052_lev); 

    KTD2052_PG_FADE_RATE ktd2052_rate0 = vFade32ms;
    KTD2052_PG_FADE_RATE ktd2052_rate1 = vFade32ms;
    ktd2052_set_fade_rate(ktd2052_rate0, ktd2052_rate1);

    KTD2052_PG_MODE ktd2052_pg_mode = v6Pat;
    ktd2052_set_slot_count(ktd2052_pg_mode);

    KTD2052_PG_TIME ktd2052_pg_time = vSlot500ms;
    ktd2052_set_slot_duration(ktd2052_pg_time);

    uint8_t ktd_tmp[8] = {1,1,1, 1,1,1, 0,0};
    ktd2052_set_slot_fade(ktd_tmp);

    for (int i=0; i<8; i++) ktd_tmp[i] = 1;
    ktd2052_set_pattern(4, ktd_tmp); // always on
    ktd2052_set_pattern(3, ktd_tmp); // always on
    ktd2052_set_pattern(2, ktd_tmp); // always on

    // ktd_tmp[0] = 1;
    // ktd_tmp[1] = 1;
    // ktd_tmp[2] = 1;
    // ktd_tmp[3] = 0;
    // ktd_tmp[4] = 0;
    // ktd_tmp[5] = 0;
    // ktd2052_set_pattern(3, ktd_tmp); // blink

    ktd2052_set_timeout(255); // Infinite Loop

    // LED ALL OFF
    ktd2052_set_colour(1, 0x000000);
    ktd2052_set_colour(2, 0x000000);
    ktd2052_set_colour(3, 0x000000);
    ktd2052_set_colour(4, 0x000000);

    ktd2052_rate0 = vFade500ms;
    ktd2052_rate1 = vFade1000ms;
    ktd2052_set_fade_rate(ktd2052_rate0, ktd2052_rate1);

    // LED 4 ON
    ktd2052_set_colour(4, 0x202020);  

    //fifo_buf_len = 30 * 1024 * 1024;    // fifo_buf_len mod 24576 = 0.
    fifo_buf_len = 12 * 1024 * 1024;    // fifo_buf_len mod 24576 = 0.
    fifo_buf = heap_caps_aligned_alloc(0x80, fifo_buf_len, MALLOC_CAP_SPIRAM);
    if (fifo_buf == NULL) {  // NULLチェックを追加
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"❌ fifo_buf allocation failed\n");
        mes_send();
        error_flg = 1;
        return;  // メモリ確保に失敗したので処理中断
    }
    memset(fifo_buf, 0xFF, fifo_buf_len);
    esp_cache_msync((void *)fifo_buf, fifo_buf_len, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"\nfifo_buf = %p\n",fifo_buf);
    u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);

#ifdef U_USB_EN
    // USB CDC-ACM
    uarttxbuf[0] = '\n';   // Write dummy data to USB
    tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, uarttxbuf, 1);

    // Check if USB is active
    if(tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0,1000) != ESP_OK)
    {
        usb_act = false;
        uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"usb test send is false.\n");
        u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);
    }
    else
    {
        usb_act = true;
        // uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"usb test send is ok.\n");
        // u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);
    }
#endif

    xTaskCreate(serial_cmd_task, "serial_cmd", 4096, NULL, tskIDLE_PRIORITY+1, NULL);

    u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"🔄 Waiting SET_PARAM...\n");
    mes_send();
    xSemaphoreTake(param_sem, portMAX_DELAY);
    u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"✅ Get SET_PARAM: sample=%d, chirp=%d\n", ad_sample_num, ad_chirp_mult);
    mes_send();

    // ... パラメータを受信し ad_sample_num, ad_chirp_mult が設定された直後 ...
    if (txdatabuf) {  // 以前確保したバッファがあれば解放
        free(txdatabuf);
        txdatabuf = NULL;
    }
    // ① すでに確保済みなら解放（※重要：再SETUP時のリーク防止）
    if (mipibuf) {
        free(mipibuf);
        mipibuf = NULL;
    }

    size_t tx_size;
    // buffer for tcp status
    u_st_buf_len = 64;
    u_st_buf = heap_caps_aligned_alloc(0x80, u_st_buf_len, MALLOC_CAP_DMA);
    uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"u_st_buf_len, u_st_buf, %d, %p",(int)u_st_buf_len,u_st_buf);
    u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);


    uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"\n-----------------start.-----------------\n");
    u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);

    //xTaskCreate(serial_cmd_task, "serial_cmd", 4096, NULL, tskIDLE_PRIORITY+1, NULL);


  #ifdef U_LAN_EN
    tcp_start();

  #endif    
    //mipi ldo
    esp_ldo_channel_handle_t u_ldo_mipi_phy = NULL;
    esp_ldo_channel_config_t u_ldo_mipi_phy_config = {
        .chan_id = 3,
        .voltage_mv = 2500,
    };
    err = esp_ldo_acquire_channel(&u_ldo_mipi_phy_config, &u_ldo_mipi_phy);
    uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"esp_ldo_acquire_channel, err = %d\n",err);
    u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);

// config will be overwritten later
    esp_cam_ctlr_csi_config_t csi_config = {
        .ctlr_id = 0,
        .h_res = ad_sample_num,
        .v_res = 16 * ad_chirp_mult,
        .lane_bit_rate_mbps = 320,
        .input_data_color_type = CAM_CTLR_COLOR_RAW12,
        .output_data_color_type = CAM_CTLR_COLOR_RAW12,
        .data_lane_num = 2,
        .byte_swap_en = false,
        .bk_buffer_dis = true,
        .queue_items = 1,
    };

    esp_isp_processor_cfg_t isp_config = {
        .clk_hz = 80 * 1000 * 1000,
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RAW8,
        .output_data_color_type = ISP_COLOR_RAW8,
        .has_line_start_packet = false,
        .has_line_end_packet = false,
        .h_res = ad_sample_num + (ad_sample_num >> 1),
        .v_res = 16 * ad_chirp_mult,
        .flags = {
            .bypass_isp = true,
        }
    };

    esp_cam_ctlr_handle_t mipi_handle = NULL;

    esp_cam_ctlr_trans_t esp_cam_trans;

    esp_cam_ctlr_evt_cbs_t cbs;
    cbs.on_get_new_trans = u_new_trans;
    cbs.on_trans_finished = u_trans_finished;

    isp_proc_handle_t isp_proc = NULL;


    gpio_set_level(PDN,0);
    delay(5);
    gpio_set_level(RSTN,0);
    delay(5);
    gpio_set_level(PDN,1);
    delay(24);
    gpio_set_level(RSTN,1);
    delay(5);

    TickType_t xPreviousWakeTime;
    xPreviousWakeTime = xTaskGetTickCount();
    TickType_t xNextWakeTime = xPreviousWakeTime;
    bool isFirstWaitDone = false;

    // Start of loop ---------------------------------------
    while(1)
    {
        //delay(100);
        //err = esp_isp_enable(isp_proc);
        //uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"esp_isp_enable, err = %d\n",err);
        //u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);

        /*gpio_set_level(PDN,0);
        delay(5);
        gpio_set_level(RSTN,0);
        delay(5);
        gpio_set_level(PDN,1);
        delay(24);
        gpio_set_level(RSTN,1);
        delay(5);*/     

        if(ad_sample_num == 32){
            ak5816_write(&reg_page, AK5816_PAGE0, 0x0F, 0x00); //Sampling data setting(ADC): 32 (Not auto)
        }else if(ad_sample_num == 64){
            ak5816_write(&reg_page, AK5816_PAGE0, 0x0F, 0x11); //Sampling data setting(ADC): 64 (Not auto)
        }else if(ad_sample_num == 128){
            ak5816_write(&reg_page, AK5816_PAGE0, 0x0F, 0x22); //Sampling data setting(ADC): 128 (Not auto)
        }else if(ad_sample_num == 256){
            ak5816_write(&reg_page, AK5816_PAGE0, 0x0F, 0x33); //Sampling data setting(ADC): 256 (Not auto)
        }else if(ad_sample_num == 512){
            ak5816_write(&reg_page, AK5816_PAGE0, 0x0F, 0x44); //Sampling data setting(ADC): 512 (Not auto)
        }else if(ad_sample_num == 1024){
            ak5816_write(&reg_page, AK5816_PAGE0, 0x0F, 0x55); //Sampling data setting(ADC): 1024 (Not auto)
        }

        ak5816_write(&reg_page, AK5816_PAGE0, 0x35, (ad_chirp_mult - 1));
        ak5816_write(&reg_page, AK5816_PAGE0, 0x36, (ad_chirp_mult - 1));

        //waiting xSemaphoreGive()...
        ESP_LOGI("main", "Waiting start command from WebSocket...");
        xSemaphoreTake(start_sem, portMAX_DELAY);

        ESP_LOGI("main", "Get start command, start operation");

        // Setup trigger
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"exec_mode %d\n", exec_mode);
        mes_send();
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"trig_period_sub_ms %f\n", trig_period_sub_ms);
        mes_send();
        // if (!exec_mode) {
        //     uint32_t freq_hz = (uint32_t)(1000.0f / trig_period_sub_ms);
        //     // printf("[Debug]: Get Freq:%ld\n", PWM_GetFastFreq());
        //     if(!flg_pwmini) {
        //         PWM_Initialize(CTL_PIN, freq_hz);
        //         trig_period_sub_ms_old = trig_period_sub_ms;
        //         flg_pwmini = true;
        //     }
        // }

        
        error_flg      = 0;
        stop_requested = false;
        frame_count    = 0;
        time_sum       = 0;
        time_num       = 0;

        if (active_mode == MODE_ADC) {
            val = ak5816_start_up_demo(&reg_page);
            if(val != 0){
                error_flg = 1;
            }

            // check current samples / chirpsets
            // First check SMODCONFIG
            ak5816_read(&reg_page, AK5816_PAGE0, 0x18, &val);
            uint8_t smodconfig_val = (val >> 4) & 0x3;

            // Read CSI2_SMPLS_A/B
            ak5816_read(&reg_page, AK5816_PAGE0, 0x0F, &val);
            uint8_t csi2_smpls_a = val & 0x7;
            uint8_t csi2_smpls_b = (val >> 4) & 0x7;

            uint8_t csi2_smpls = csi2_smpls_a;
            if (smodconfig_val == 1) // B only
            {
                csi2_smpls = csi2_smpls_b;
            }

            ad_sample_num = (int) 32 << (int) csi2_smpls;

            // Read SSETCNT, SPATCNT
            ak5816_read(&reg_page, AK5816_PAGE0, 0x35, &val);
            uint8_t ssetcnta = val;

            ak5816_read(&reg_page, AK5816_PAGE0, 0x36, &val);
            uint8_t ssetcntb = val;

            ak5816_read(&reg_page, AK5816_PAGE0, 0x33, &val);
            uint8_t spatcnt = val;

            // Calculate ad_chirp_mult
            ad_chirp_mult = 0;

            if (smodconfig_val == 0) // A only
            {
                ad_chirp_mult += (int)ssetcnta + 1;
            }
            else if (smodconfig_val == 1 || smodconfig_val == 2) // B only / A+B
            {
                ad_chirp_mult += (int)ssetcntb + 1;
            }
            ad_chirp_mult *= (int)spatcnt + 1;


            // check current fs / dfilbyps setup
            ak5816_read(&reg_page, AK5816_PAGE0, 0x15, &val);
            ad_fs = val & 0x1;

            ak5816_read(&reg_page, AK5816_PAGE2, 0x12, &val);
            ad_dfilbyps = (val >> 3) & 0x1;


            if (ad_fs == v53Msps)
            {
                csi_config.input_data_color_type = CAM_CTLR_COLOR_RAW12;
                csi_config.output_data_color_type = CAM_CTLR_COLOR_RAW12;

                csi_config.lane_bit_rate_mbps = 320;
                csi_config.h_res = ad_sample_num;
                csi_config.v_res = 16 * ad_chirp_mult;

                isp_config.h_res = ad_sample_num + (ad_sample_num >> 1);
                isp_config.v_res = 16 * ad_chirp_mult;
            }
            else // 20Msps
            {
                csi_config.input_data_color_type = CAM_CTLR_COLOR_RAW8;
                csi_config.output_data_color_type = CAM_CTLR_COLOR_RAW8;

                if (ad_dfilbyps)
                {
                    csi_config.lane_bit_rate_mbps = 320;
                    csi_config.h_res = ad_sample_num * 2 * 2;
                    csi_config.v_res = 16 * ad_chirp_mult;

                    isp_config.h_res = ad_sample_num * 2 * 2;
                    isp_config.v_res = 16 * ad_chirp_mult;
                }
                else
                {
                    csi_config.lane_bit_rate_mbps = 160;
                    csi_config.h_res = ad_sample_num * 2;
                    csi_config.v_res = 16 * ad_chirp_mult;

                    isp_config.h_res = ad_sample_num * 2;
                    isp_config.v_res = 16 * ad_chirp_mult;
                }
            }

            // Dispose buffers ---------------------
            if (txdatabuf) {  // 以前確保したバッファがあれば解放
                free(txdatabuf);
                txdatabuf = NULL;
            }

            // ① すでに確保済みなら解放（※重要：再SETUP時のリーク防止）
            if (mipibuf) {
                free(mipibuf);
                mipibuf = NULL;
            }

            if (m_buffer) {
                free(m_buffer);
                m_buffer = NULL;
            }

            // Calculate buffer size ---------------
            // m_buffer (m_buffer_len): buffer used in CSI-1 Rx HW IP (1 frame of received data)
            // txdatabuf (tx_size) : Ethernet Tx buffer
            // mipibuf (mipibuf_len) : RAW12 parse buffer, for Ethernet Tx (1 frames)
            if (ad_fs == v53Msps) // 53.3Msps: 12b/sample
            {
                // 必要な送信データサイズを計算（ヘッダー5B＋圧縮データ＋トレーラー9B）
                if (exec_mode) {
                    tx_size = 5 + (ad_sample_num * 16 * ad_chirp_mult) * 12 / 8 + 9;
                }
                else {
                    tx_size = 5 + (ad_sample_num * 16 * ad_chirp_mult * trig_sub_times) * 12 / 8 + 9;
                }
                // ad_sample_num * 24 (12b x 16ch = 192b = 24bytes) * ad_chirp_mult bytes
                m_buffer_len = ((ad_sample_num << 4) + (ad_sample_num << 3)) * ad_chirp_mult;

                if (exec_mode) {
                    mipibuf_len = ad_sample_num * 16 * ad_chirp_mult;
                }
                else {
                    mipibuf_len = ad_sample_num * 16 * ad_chirp_mult * trig_sub_times;
                }
            }
            else // 20Msps: 16b/sample
            {
                if (!ad_dfilbyps)
                {
                    if (exec_mode) {
                        tx_size = 5 + (ad_sample_num * 16 * ad_chirp_mult) * 2 + 9;
                    }
                    else {
                        tx_size = 5 + (ad_sample_num * 16 * ad_chirp_mult * trig_sub_times) * 2 + 9;
                    }
                    // ad_sample_num * 32 (16b x 16ch = 256n = 32bytes) * ad_chirp_mult bytes
                    m_buffer_len = (ad_sample_num << 5) * ad_chirp_mult;

                    // not used
                    mipibuf_len = ad_sample_num * 16 * ad_chirp_mult;
                }
                else
                {
                    if (exec_mode) {
                        tx_size = 5 + (ad_sample_num * 2 * 16 * ad_chirp_mult) * 2 + 9;
                    }
                    else {
                        tx_size = 5 + (ad_sample_num * 2 * 16 * ad_chirp_mult * trig_sub_times) * 2 + 9;
                    }
                    // ad_sample_num * 2 * 32 (16b x 16ch = 256n = 32bytes) * ad_chirp_mult bytes
                    m_buffer_len = 2 * (ad_sample_num << 5) * ad_chirp_mult;

                    // not used
                    mipibuf_len = 2 * ad_sample_num * 16 * ad_chirp_mult;
                }
            }

            txdatabuf = heap_caps_malloc(tx_size, MALLOC_CAP_SPIRAM);
            if (!txdatabuf) {
                u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"❌ txdatabuf allocation failed\n");
                mes_send();
                error_flg = 1;
                return;
            }
            txdatabuf_size = tx_size;  // 確保したバッファのサイズを保存

            if (ad_fs == v53Msps)
            {
                mipibuf = heap_caps_malloc(mipibuf_len * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
                if (mipibuf == NULL) {
                    u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"❌ mipibuf allocation failed\n");
                    mes_send();
                    error_flg = 1;
                    return;  // メモリ確保失敗時はエラー処理
                }    
            }


            uint32_t avail_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
            uint32_t avail_dma = heap_caps_get_free_size(MALLOC_CAP_DMA);

            if (m_buffer_len > avail_psram)
            {
                u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"❌ m_buffer (CSI-2 Rx Buffer) of %d bytes are too large (should be <= %ld bytes)\n", m_buffer_len, avail_psram);
                mes_send();
                error_flg = 1;
                return;
            }
            else if (m_buffer_len > avail_dma)
            {
                u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"[Warning] m_buffer (CSI-2 Rx Buffer) of %d bytes are beyond DMA memory (%ld bytes). DMA disabled.\n", m_buffer_len, avail_dma);
                mes_send();
                m_buffer = heap_caps_aligned_alloc(0x80, m_buffer_len, MALLOC_CAP_SPIRAM);
                if (m_buffer == NULL) {
                    u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"❌ m_buffer allocation failed\n");
                    mes_send();
                    error_flg = 1;
                    return;
                }
            }
            else {
                m_buffer = heap_caps_aligned_alloc(0x80, m_buffer_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
                if (m_buffer == NULL) {
                    u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"❌ m_buffer allocation failed\n");
                    mes_send();
                    error_flg = 1;
                    return;
                }
            }

            // Fill m_buffer with 0xFF
            memset(m_buffer, 0xFF, m_buffer_len);
            esp_cache_msync((void *)m_buffer, m_buffer_len, ESP_CACHE_MSYNC_FLAG_DIR_C2M);

            // uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"m_buffer = %p, m_buffer_len = %d\n",m_buffer, m_buffer_len);
            // u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);

            esp_cam_trans.buffer = m_buffer;
            esp_cam_trans.buflen = m_buffer_len;
            esp_cam_trans.received_size = m_buffer_len;

            cntr1 = 0;
            cntr2 = 0;
            cntr2_pre = 0;


            if (ad_fs == v53Msps)
            {
                uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax, "[Info] Fs: 53.3Msps, %d x 16ch x %ddpl, tx_size %d, m_buffer_len %d, mipibuf_len %d\n",
                ad_sample_num, ad_chirp_mult, tx_size, m_buffer_len, mipibuf_len);
            }
            else
            {
                if (!ad_dfilbyps)
                {
                    uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax, "[Info] Fs: 20Msps, %d x 16ch x %ddpl, tx_size %d, m_buffer_len %d, mipibuf_len %d\n",
                    ad_sample_num, ad_chirp_mult, tx_size, m_buffer_len, mipibuf_len);
                }

                if (ad_dfilbyps)
                {
                    uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax, "[Info] Fs: 20Msps x2, %d x 16ch x %ddpl, tx_size %d, m_buffer_len %d, mipibuf_len %d\n",
                    ad_sample_num, ad_chirp_mult, tx_size, m_buffer_len, mipibuf_len);
                }
            }
            u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);

            // Activate CSI-2 here
            err = esp_cam_new_csi_ctlr(&csi_config, &mipi_handle);
            uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"esp_cam_new_csi_ctlr, err = %d\n",err);
            u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);

            err = esp_cam_ctlr_register_event_callbacks(mipi_handle,&cbs,&esp_cam_trans);
            uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"esp_cam_ctlr_register_event_callbacks, err = %d\n",err);
            u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);

            err = esp_isp_new_processor(&isp_config, &isp_proc);
            uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"esp_isp_new_processor, err = %d\n",err);
            u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);

            err = esp_cam_ctlr_receive(mipi_handle, &esp_cam_trans, ESP_CAM_CTLR_MAX_DELAY);
            uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"esp_cam_ctlr_receive, err = %d\n",err);
            u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);

            err = esp_cam_ctlr_enable(mipi_handle);
            uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"esp_cam_ctlr_enable, err = %d\n",err);
            u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);

            err = esp_cam_ctlr_start(mipi_handle);
            uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"esp_cam_ctlr_start, err = %d\n",err);
            u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);

        } else if (active_mode == MODE_FFT) {
            // FFTモード時: 事前に必要なレジスタ設定済みであることを想定する
            // 必要ならセンサをTRX動作状態に移行させる（EXEC使用する場合は後でHighにする）
            // （ak5816_start_up_demoは呼ばず、ユーザ設定を保持）
            //ak5816_go_trx_demo(&reg_page);  // センサをTRX状態へ（必要に応じて）
            //error_flg = 0;
            val = ak5816_start_up_fftdemo(&reg_page);
            if(val != 0){
                error_flg = 1;
            }
        }

        uint8_t adnum_flg;
        /*i = AD_SAMPLE_NUM;
        if(i == 1024)       adnum_flg = 0x57;
        else if(i == 512)   adnum_flg = 0x56;
        else if(i == 256)   adnum_flg = 0x55;
        else if(i == 128)   adnum_flg = 0x54;
        else if(i == 64)    adnum_flg = 0x53;
        else                adnum_flg = 0x52;
        */
        if(ad_sample_num == 1024)       adnum_flg = 0x57;
        else if(ad_sample_num == 512)   adnum_flg = 0x56;
        else if(ad_sample_num == 256)   adnum_flg = 0x55;
        else if(ad_sample_num == 128)   adnum_flg = 0x54;
        else if(ad_sample_num == 64)    adnum_flg = 0x53;
        else                adnum_flg = 0x52;
        txdatabufp = 0;
        txdatabuf[txdatabufp] = 0xAA;
        txdatabufp++;
        txdatabuf[txdatabufp] = 0x00;
        txdatabufp++;
        txdatabuf[txdatabufp] = 0x00;
        txdatabufp++;
        txdatabuf[txdatabufp] = adnum_flg;
        txdatabufp++;
        txdatabuf[txdatabufp] = 0x20;
        txdatabufp++;

        // Initialize timing variables and capture the start timestamp (in microseconds)
        // Used for calculating relative frame times and measuring processing duration
        time_min = 0xFFFFFFFF;
        time_max = 0;
        gettimeofday(&u_tv, NULL);

        // power on LDO
        //gpio_set_level(RF_LDO_E, 1);
        //delay(200);

        // 基準時刻を取得（フレームタイムスタンプ用）
        struct timeval start_tv;
        gettimeofday(&start_tv, NULL);
        uint32_t start_time_us = (uint32_t)(((int64_t)start_tv.tv_sec * 1000000L + start_tv.tv_usec) & 0xFFFFFFFF);

        uint32_t inp1,inp2;
        bool loopact;
        int reg_check_cntr = 0;

        if (active_mode == MODE_ADC) {

            if (exec_mode) {
                if(error_flg == 0)
                {
                    //time_stamp
                    //gettimeofday(&u_tv, NULL);
                    //int64_t t_exec_pre = (int64_t)u_tv.tv_sec * 1000000L + u_tv.tv_usec;
                    //uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax, "[LOG] EXEC前 t=%lld us\n", t_exec_pre);
                    //u_write_bytes(UART_NUM_0, uarttxbuf, uarttxsize);                

                    // dummy write
                    ak5816_write(&reg_page, AK5816_PAGE0, 0x00, 0x00);

                    u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"Set EXEC 1\n");
                    mes_send();
                    gpio_set_level(EXEC,1);

                    gettimeofday(&u_tv, NULL);
                    int64_t t_exec_post = ((int64_t)u_tv.tv_sec * 1000000L + u_tv.tv_usec) - start_time_us;
                    uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax, "[Info] [%.3f ms] EXEC = 1\n", t_exec_post / 1000.0);
                    u_write_bytes(UART_NUM_0, uarttxbuf, uarttxsize);            

                    // LED 3 ON
                    ktd2052_set_colour(3, 0x202000);
                }

                //uint32_t inp1,inp2;
                //bool loopact;
                //int reg_check_cntr = 0;

                //time_stamp
                //gettimeofday(&u_tv, NULL);
                //int64_t t_exec_post = (int64_t)u_tv.tv_sec * 1000000L + u_tv.tv_usec;
                //uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax, "[LOG] EXEC後 t=%lld us\n", t_exec_post);
                //u_write_bytes(UART_NUM_0, uarttxbuf, uarttxsize);            

                while(1)
                {
                    int64_t t_while_post = 0;

                    #ifdef U_LAN_EN
                    if (error_flg != 0 || stop_requested)
                    #else
                    if ((gpio_get_level(TEST_SW) == 0) || (error_flg != 0) || stop_requested)
                    #endif
                    {
                        if(error_flg == 1)
                        {
                            //printf("error!\n");
                            ak5816_read(&reg_page, AK5816_PAGE0, 0x02, &error_reg_buf[0]);
                            uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax,
                                "err(0x02) is 0x%02x\n", error_reg_buf[0]);
                            u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);

                            ak5816_read(&reg_page, AK5816_PAGE0, 0x06, &error_reg_buf[1]);
                            uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax,
                                "err(0x06) is 0x%02x\n", error_reg_buf[1]);
                            u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);

                            ak5816_read(&reg_page, AK5816_PAGE0, 0x07, &error_reg_buf[2]);
                            uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax,
                                "err(0x07) is 0x%02x\n", error_reg_buf[2]);
                            u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);
                        
                            for (int adr_idx = 0x0A; adr_idx < 0x22; adr_idx++) {
                                ak5816_read(&reg_page, AK5816_PAGE4, adr_idx, &val);
                                if(val != 0)
                                {
                                    uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax,
                                        "err(Page 4, 0x%02x) is 0x%02x\n", adr_idx, val);
                                    u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);
                                }
                            }

                            // Read RPU Error
                            for (int aidx = 0x0F; aidx <= 0x11; aidx++) 
                            {
                                ak5816_read(&reg_page, AK5816_PAGE2, aidx, &val);
                                if (val != 0)
                                {
                                    uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax,
                                        "err(Page 2, 0x%02x) is 0x%02x\n", aidx, val);
                                    u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);
                                }
                            }
                        }

                        if (mipi_handle != NULL)
                        {
                            err = esp_cam_ctlr_stop(mipi_handle);
                            uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"esp_cam_ctlr_stop, err = %d\n",err);
                            u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);
                        }

                        if (isp_proc != NULL)
                        {
                            err = esp_isp_del_processor(isp_proc);
                            uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"esp_isp_del_processor, err = %d\n",err);
                            u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);

                            isp_proc = NULL;
                        }

                        if (mipi_handle != NULL)
                        {
                            err = esp_cam_ctlr_disable(mipi_handle);
                            uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"esp_cam_ctlr_disable, err = %d\n",err);
                            u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);

                            err = esp_cam_ctlr_del(mipi_handle);
                            uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"esp_cam_ctlr_del, err = %d\n",err);
                            u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);

                            mipi_handle = NULL;
                        }

                        uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"time_min[us] = %d\n",(int)(time_min));
                        u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);
                        uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"time_max[us] = %d\n",(int)(time_max));
                        u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);
                        time_calc = (uint32_t)(time_sum / time_num);
                        uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"time_ave[us] = %d\n",(int)(time_calc));
                        u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);

                        if(error_flg == 1)
                        {
                            delay(100);
                            send_data(error_reg_buf, 0, 1);
                            delay(1000);
                        }

                        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"stop!\n");
                        mes_send();

                        break;
                    }

                    if (cntr2 != cntr2_pre)
                    {
                        //time_stamp
                        gettimeofday(&u_tv, NULL);
                        t_while_post = (int64_t)u_tv.tv_sec * 1000000L + u_tv.tv_usec - start_time_us;
                        // uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax, "[Info] [%.3f ms] Got frame #%d: ", t_while_post / 1000.0, cntr2);
                        // u_write_bytes(UART_NUM_0, uarttxbuf, uarttxsize);                

                        cntr2_pre = cntr2;
                    }

                    inp1 = fifo_inp;
                    //delay(1);
                    //inp2 = fifo_inp;
                    int diff = inp1 - fifo_outp;
                    if (diff < 0) diff += fifo_buf_len;
                    
                    //if(inp1 == inp2)
                    if (diff >= m_buffer_len)
                    {

                        //time_stamp
                        gettimeofday(&u_tv, NULL);
                        int64_t t_inp1inp2_post = (int64_t)u_tv.tv_sec * 1000000L + u_tv.tv_usec - start_time_us;
                        //uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax, "[LOG] inp1==inp2後 t=%lld us\n", t_inp1inp2_post);
                        // uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax, "[+%.3f ms] %d bytes in FIFO: ", 
                        //     (t_inp1inp2_post - t_while_post) / 1000.0, diff);
                        // u_write_bytes(UART_NUM_0, uarttxbuf, uarttxsize);                    

                        loopact = true;
                        while(loopact)
                        {
                            n = inp1;
                            n -= fifo_outp;
                            if(n < 0)
                                n += fifo_buf_len;
                            if(n >= m_buffer_len)
                            {

                                //time_stamp
                                gettimeofday(&u_tv, NULL);
                                int64_t t_senddata_pre = (int64_t)u_tv.tv_sec * 1000000L + u_tv.tv_usec - start_time_us;
                                //uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax, "[LOG] send_data前 t=%lld us\n", t_senddata_pre);
                                // uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax, "[+%.3f ms] Sending %d bytes ...\n", 
                                //     (t_senddata_pre - t_inp1inp2_post) / 1000.0, m_buffer_len);
                                // u_write_bytes(UART_NUM_0, uarttxbuf, uarttxsize);

                                uint8_t *data_ptr = (uint8_t *)fifo_buf + fifo_outp;
                                send_data(data_ptr, timebuf[timebuf_outp], m_buffer_len);

                                //time_stamp
                                gettimeofday(&u_tv, NULL);
                                int64_t t_senddata_post = (int64_t)u_tv.tv_sec * 1000000L + u_tv.tv_usec - start_time_us;
                                //uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax, "[LOG] send_data後 t=%lld us\n", t_senddata_post);
                                // uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax, "[Info] [%.3f ms] Sent %d bytes / %.3f ms (%.3f Mbps). \n", 
                                //     t_senddata_post / 1000.0, m_buffer_len, (t_senddata_post - t_senddata_pre) / 1000.0,
                                //     m_buffer_len * 8.0 / (t_senddata_post - t_senddata_pre));
                                // u_write_bytes(UART_NUM_0, uarttxbuf, uarttxsize);

                                frame_count++;
                                if (frame_limit > 0 && frame_count >= frame_limit) {
                                    stop_requested = true;
                                    ESP_LOGI("main", "Stop since reached frame_limit");
            
            
                                    fifo_outp += m_buffer_len;
                                    if (fifo_outp >= fifo_buf_len) fifo_outp = 0;
                                    timebuf_outp++;
                                    if (timebuf_outp >= timebuf_max) timebuf_outp = 0;
            
                                    loopact = false;
                                    break;
                                }

                                if (error_flg != 0 || stop_requested) {
                                    loopact = false;
                                    break;
                                }

                                fifo_outp += m_buffer_len;
                                if(fifo_outp >= fifo_buf_len)
                                {
                                    fifo_outp = 0;
                                }
                                timebuf_outp++;
                                if(timebuf_outp >= timebuf_max)
                                {
                                    timebuf_outp = 0;
                                }
                            }
                            else
                            {
                                loopact = false;
                            }
                        }
                    }

                    reg_check_cntr++;
                    if(reg_check_cntr >= 10)
                    {
                        reg_check_cntr = 0;
                        ak5816_read(&reg_page, AK5816_PAGE0, 0x02, &val);
                        if(val != 0)
                        {
                            u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"err(0x02) is 0x%02x\n", val);
                            mes_send();
                            error_flg = 1;
                        }
                        ak5816_read(&reg_page, AK5816_PAGE0, 0x06, &val);
                        if(val != 0)
                        {
                            u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"err(0x06) is 0x%02x\n", val);
                            mes_send();
                            error_flg = 1;
                        }
                        ak5816_read(&reg_page, AK5816_PAGE0, 0x07, &val);
                        if(val != 0)
                        {
                            u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"err(0x07) is 0x%02x\n", val);
                            mes_send();
                            error_flg = 1;
                        }

                        for (int adr_idx = 0x0A; adr_idx < 0x22; adr_idx++) {
                            ak5816_read(&reg_page, AK5816_PAGE4, adr_idx, &val);
                            if(val != 0)
                            {
                                u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"err(Page 4, 0x%02x) is 0x%02x\n", adr_idx, val);
                                mes_send();
                                error_flg = 1;
                            }
                        }

                        // Read RPU Error
                        for (int aidx = 0x0F; aidx <= 0x11; aidx++) 
                        {
                            ak5816_read(&reg_page, AK5816_PAGE2, aidx, &val);
                            if (val != 0)
                            {
                                u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"Err: Page 2 Addr 0x%02x: 0x%02x\n", aidx, val);
                                mes_send();
                            }
                        }

                    }

                }

            }

            else {

                while(1)
                {

                    int64_t t_while_post = 0;

                    #ifdef U_LAN_EN
                    if (error_flg != 0 || stop_requested)
                    #else
                    if ((gpio_get_level(TEST_SW) == 0) || (error_flg != 0) || stop_requested)
                    #endif
                    {
                        if(error_flg == 1)
                        {
                            ak5816_read(&reg_page, AK5816_PAGE0, 0x02, &error_reg_buf[0]);
                            uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax,
                                "err(0x02) is 0x%02x\n", error_reg_buf[0]);
                            u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);

                            ak5816_read(&reg_page, AK5816_PAGE0, 0x06, &error_reg_buf[1]);
                            uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax,
                                "err(0x06) is 0x%02x\n", error_reg_buf[1]);
                            u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);

                            ak5816_read(&reg_page, AK5816_PAGE0, 0x07, &error_reg_buf[2]);
                            uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax,
                                "err(0x07) is 0x%02x\n", error_reg_buf[2]);
                            u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);
                        
                            for (int adr_idx = 0x0A; adr_idx < 0x22; adr_idx++) {
                                ak5816_read(&reg_page, AK5816_PAGE4, adr_idx, &val);
                                if(val != 0)
                                {
                                    uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax,
                                        "err(Page 4, 0x%02x) is 0x%02x\n", adr_idx, val);
                                    u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);
                                }
                            }

                            // Read RPU Error
                            for (int aidx = 0x0F; aidx <= 0x11; aidx++) 
                            {
                                ak5816_read(&reg_page, AK5816_PAGE2, aidx, &val);
                                if (val != 0)
                                {
                                    uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax,
                                        "err(Page 2, 0x%02x) is 0x%02x\n", aidx, val);
                                    u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);
                                }
                            }
                        }

                        if (mipi_handle != NULL)
                        {
                            err = esp_cam_ctlr_stop(mipi_handle);
                            uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"esp_cam_ctlr_stop, err = %d\n",err);
                            u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);
                        }

                        if (isp_proc != NULL)
                        {
                            err = esp_isp_del_processor(isp_proc);
                            uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"esp_isp_del_processor, err = %d\n",err);
                            u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);

                            isp_proc = NULL;
                        }

                        if (mipi_handle != NULL)
                        {
                            err = esp_cam_ctlr_disable(mipi_handle);
                            uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"esp_cam_ctlr_disable, err = %d\n",err);
                            u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);

                            err = esp_cam_ctlr_del(mipi_handle);
                            uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"esp_cam_ctlr_del, err = %d\n",err);
                            u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);

                            mipi_handle = NULL;
                        }

                        uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"time_min[us] = %d\n",(int)(time_min));
                        u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);
                        uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"time_max[us] = %d\n",(int)(time_max));
                        u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);
                        time_calc = (uint32_t)(time_sum / time_num);
                        uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"time_ave[us] = %d\n",(int)(time_calc));
                        u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);

                        if(error_flg == 1)
                        {
                            delay(100);
                            send_data(error_reg_buf, 0, 1);
                            delay(1000);
                        }

                        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"stop!\n");
                        mes_send();

                        break;
                    }

                    //xTaskDelayUntil(&xPreviousWakeTime, pdMS_TO_TICKS(trig_period_ms));
                    // initialise xNextWakeTime
                    xNextWakeTime += pdMS_TO_TICKS(trig_period_ms);

                    if (!isFirstWaitDone) {
                        xNextWakeTime = xTaskGetTickCount();
                        isFirstWaitDone = true;
                    }

                    TickType_t xCurrentTime = xTaskGetTickCount();

                    // wait if tick is overflown
                    while (xCurrentTime < xPreviousWakeTime) {
                        vTaskDelay(1);
                        xCurrentTime = xTaskGetTickCount();
                    }

                    // wait
                    if (xCurrentTime < xNextWakeTime) {
                        vTaskDelay(xNextWakeTime - xCurrentTime);
                    }

                    xPreviousWakeTime = xTaskGetTickCount();

                    ret = ak5816_go_standby_demo(&reg_page);
                    if (ret != 0){
                        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"Fail to go standby state!\n");
                        mes_send();
                        error_flg = 1;
                    }

                    ret = ak5816_go_trx_demo(&reg_page);
                    if (ret != 0){
                        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"Fail to go TRX state!\n");
                        mes_send();
                        error_flg = 1;
                    }

                    // Apply trigger
                    gettimeofday(&u_tv, NULL);
                    int64_t tmp_t = (int64_t)u_tv.tv_sec * 1000000L + u_tv.tv_usec - start_time_us;
                    // uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax, "[Info] [%.3f ms] Trigger start: %d times, Frame period %f ms, Frame Count %d\n", 
                    //     tmp_t / 1000.0, trig_sub_times, trig_period_ms, frame_count);
                    // u_write_bytes(UART_NUM_0, uarttxbuf, uarttxsize);                

                    PWM_TrigLoop(trig_sub_times);
                    // for(int i = 0; i < trig_sub_times; i++) {
                    //     gpio_set_level(TRIG, 1);
                    //     esp_rom_delay_us(trig_period_sub_ms * 1000.0 * 0.5);
                    //     gpio_set_level(TRIG, 0);
                    //     esp_rom_delay_us(trig_period_sub_ms * 1000.0 * 0.5);
                    // }

                    // Wait for last set
                    esp_rom_delay_us(trig_period_sub_ms * 1000.0);

                    ret = ak5816_go_slp_demo(&reg_page);
                    if (ret != 0){
                        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"Fail to go to Sleep state!\n");
                        mes_send();
                        error_flg = 1;
                    }

                    ak5816_read(&reg_page, AK5816_PAGE0, 0x02, &val);
                    if(val != 0)
                    {
                        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"err(0x02) is 0x%02x\n", val);
                        mes_send();
                        error_flg = 1;
                    }
                    ak5816_read(&reg_page, AK5816_PAGE0, 0x06, &val);
                    if(val != 0)
                    {
                        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"err(0x06) is 0x%02x\n", val);
                        mes_send();
                        error_flg = 1;
                    }
                    ak5816_read(&reg_page, AK5816_PAGE0, 0x07, &val);
                    if(val != 0)
                    {
                        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"err(0x07) is 0x%02x\n", val);
                        mes_send();
                        error_flg = 1;
                    }

                    // Read RPU Error
                    for (int aidx = 0x0F; aidx <= 0x11; aidx++) 
                    {
                        ak5816_read(&reg_page, AK5816_PAGE2, aidx, &val);
                        if (val != 0)
                        {
                            u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"Err: Page 2 Addr 0x%02x: 0x%02x\n", aidx, val);
                            mes_send();
                        }
                    }
                    // Read other errors
                    for (int aidx = 0x0A; aidx <= 0x21; aidx++) 
                    {
                        ak5816_read(&reg_page, AK5816_PAGE4, aidx, &val);
                        if (val != 0)
                        {
                            u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"Err: Page 4 Addr 0x%02x: 0x%02x\n", aidx, val);
                            mes_send();
                        }
                    }

                    int cntr2_tmp = cntr2;
                    if (cntr2_tmp < cntr2_pre) {
                        cntr2_tmp += cntr2_max;
                    }
                    if(cntr2_tmp >= cntr2_pre + trig_sub_times)
                    {
                        // time stamp
                        gettimeofday(&u_tv, NULL);
                        t_while_post = (int64_t)u_tv.tv_sec * 1000000L + u_tv.tv_usec - start_time_us;
                        // uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax, "[Info] [%.3f ms] Got frame #%d: ", t_while_post / 1000.0, cntr2 / trig_sub_times);
                        // u_write_bytes(UART_NUM_0, uarttxbuf, uarttxsize);                

                        cntr2_pre += trig_sub_times;
                        if (cntr2_pre > cntr2_max) {
                            cntr2_pre -= cntr2_max;
                        }
                        //time_us2 = (int64_t)u_tv.tv_sec * 1000000L + (int64_t)u_tv.tv_usec;
                        //send_data(m_buffer, (uint32_t)(time_us2 & 0xFFFFFFFF), 0);
                    }

                    // [TODO] send here
                    inp1 = fifo_inp;
                    int diff = inp1 - fifo_outp;
                    if (diff < 0) diff += fifo_buf_len;
                    
                    //if(inp1 == inp2)
                    if (diff >= m_buffer_len * trig_sub_times)
                    {

                        //time_stamp
                        gettimeofday(&u_tv, NULL);
                        int64_t t_inp1inp2_post = (int64_t)u_tv.tv_sec * 1000000L + u_tv.tv_usec - start_time_us;
                        //uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax, "[LOG] inp1==inp2後 t=%lld us\n", t_inp1inp2_post);
                        // uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax, "[+%.3f ms] %d bytes in FIFO: ", 
                        //     (t_inp1inp2_post - t_while_post) / 1000.0, diff);
                        // u_write_bytes(UART_NUM_0, uarttxbuf, uarttxsize);                    

                        loopact = true;
                        while(loopact)
                        {
                            n = inp1;
                            n -= fifo_outp;
                            if(n < 0)
                                n += fifo_buf_len;
                            if(n >= m_buffer_len * trig_sub_times)
                            {

                                //time_stamp
                                gettimeofday(&u_tv, NULL);
                                int64_t t_senddata_pre = (int64_t)u_tv.tv_sec * 1000000L + u_tv.tv_usec - start_time_us;
                                //uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax, "[LOG] send_data前 t=%lld us\n", t_senddata_pre);
                                // uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax, "[+%.3f ms] Sending %d bytes ...\n", 
                                //     (t_senddata_pre - t_inp1inp2_post) / 1000.0, m_buffer_len * trig_sub_times);
                                // u_write_bytes(UART_NUM_0, uarttxbuf, uarttxsize);

                                uint8_t *data_ptr = (uint8_t *)fifo_buf + fifo_outp;
                                send_data(data_ptr, timebuf[timebuf_outp], m_buffer_len * trig_sub_times);

                                //time_stamp
                                gettimeofday(&u_tv, NULL);
                                int64_t t_senddata_post = (int64_t)u_tv.tv_sec * 1000000L + u_tv.tv_usec - start_time_us;
                                //uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax, "[LOG] send_data後 t=%lld us\n", t_senddata_post);
                                // uarttxsize = snprintf((char*)uarttxbuf, uarttxbufmax, "[Info] [%.3f ms] Sent %d bytes / %.3f ms (%.3f Mbps). \n", 
                                //     t_senddata_post / 1000.0, m_buffer_len * trig_sub_times, (t_senddata_post - t_senddata_pre) / 1000.0,
                                //     m_buffer_len * trig_sub_times * 8.0 / (t_senddata_post - t_senddata_pre));
                                // u_write_bytes(UART_NUM_0, uarttxbuf, uarttxsize);

                                frame_count++;
                                if (frame_limit > 0 && frame_count >= frame_limit) {
                                    stop_requested = true;
                                    ESP_LOGI("main", "Stop since reached frame_limit");
            
            
                                    fifo_outp += m_buffer_len * trig_sub_times;
                                    if (fifo_outp >= fifo_buf_len) fifo_outp = 0;
                                    timebuf_outp++;
                                    if (timebuf_outp >= timebuf_max) timebuf_outp = 0;
            
                                    loopact = false;
                                    isFirstWaitDone = false; // Required to reset start time
                                    break;
                                }

                                if (error_flg != 0 || stop_requested) {
                                    loopact = false;
                                    break;
                                }

                                fifo_outp += m_buffer_len * trig_sub_times;
                                if(fifo_outp >= fifo_buf_len)
                                {
                                    fifo_outp = 0;
                                }
                                timebuf_outp++;
                                if(timebuf_outp >= timebuf_max)
                                {
                                    timebuf_outp = 0;
                                }
                            }
                            else
                            {
                                loopact = false;
                            }
                        }
                    }


                }
            }
           
    
        } else if (active_mode == MODE_FFT) {

            uint8_t frame_id_now = 0;
            uint8_t frame_id_prev = 0; 

            // FFT結果読み出しモード用ループ
            if (error_flg == 0) {

                ak5816_read(&reg_page, AK5816_PAGE2, 0x0E, &frame_id_prev);

                u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"Set EXEC 1\n");
                mes_send();
                gpio_set_level(EXEC, 1);  // センサ動作開始（16チャープ連続測定モード）
                // LED 2 ON
                ktd2052_set_colour(2, 0x202000);
            }

            while (1) {

                int wait_counter = 0;

                // 停止要求が出た場合はループを抜ける
                if (error_flg != 0 || stop_requested) {
                    if (error_flg == 1) {
                        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"Sensor error during FFT mode\n");
                        mes_send();
                        // （必要ならエラーレジスタ読み出し処理を追加できます）
                    }
                    break;
                }

                // 1フレーム（16チャープ分）のFFTデータをセンサレジスタから読み出す
                uint8_t i_low, i_mid, i_high;
                uint8_t q_low, q_mid, q_high;
                uint32_t calcsum = 0;
                txdatabufp = 5;//0
                // ヘッダー5バイト（上で設定済み）を除いたデータ部開始位置から埋める
                // 注意: txdatabufにはすでに先頭5バイト書き込み済みなのでtxdatabufpは5となっている
                //for (uint8_t chirp = 0; chirp < 16; chirp++) {
                //for (uint8_t chirp = 0; chirp < (AD_CHIRP_MULT*16); chirp++) {
                for (uint8_t chirp = 0; chirp < (ad_chirp_mult*16); chirp++) {

                    // Chirp選択レジスタ設定 (Page3 0x1D[3:0])
                    ak5816_write(&reg_page, AK5816_PAGE3, 0x1D, chirp);
                    //if (reg_page != 3) {
                    //    spi_send((AK5816_CORE_PAGE_SETTING << 8) | 0x03);
                    //    reg_page = 3;
                    //}
                    //spi_send((0x1D << 8) | chirp);

                    
                    // binを0から上限まで読み取り
                    //uint16_t bin_count = AD_SAMPLE_NUM / 2;  // 読み出すbin数（ADサンプル数の半分）
                    uint16_t bin_count = ad_sample_num / 2;  // 読み出すbin数（ADサンプル数の半分）
                    for (uint16_t bin = 0; bin < bin_count; bin++) {
                        ak5816_write(&reg_page, AK5816_PAGE3, 0x1E, bin);
                        // I/Qデータ24bit読み取り (Page3 0x1F-0x24)
                        //uint8_t iqbuf[6];
                        //ak5816_read_burst(&reg_page, AK5816_PAGE3, 0x1F, iqbuf, 6);  // 新規実装
                        //i_low = iqbuf[0]; i_mid = iqbuf[1]; i_high = iqbuf[2];
                        //q_low = iqbuf[3]; q_mid = iqbuf[4]; q_high = iqbuf[5];
                        ak5816_read(&reg_page, AK5816_PAGE3, 0x1F, &i_low);
                        ak5816_read(&reg_page, AK5816_PAGE3, 0x20, &i_mid);
                        ak5816_read(&reg_page, AK5816_PAGE3, 0x21, &i_high);
                        ak5816_read(&reg_page, AK5816_PAGE3, 0x22, &q_low);
                        ak5816_read(&reg_page, AK5816_PAGE3, 0x23, &q_mid);
                        ak5816_read(&reg_page, AK5816_PAGE3, 0x24, &q_high);
                        // txdatabufにI,Q各3バイトを書き込む
                        txdatabuf[txdatabufp++] = i_low;
                        txdatabuf[txdatabufp++] = i_mid;
                        txdatabuf[txdatabufp++] = i_high;
                        txdatabuf[txdatabufp++] = q_low;
                        txdatabuf[txdatabufp++] = q_mid;
                        txdatabuf[txdatabufp++] = q_high;
                        // チェックサム計算（各バイト値の積算）
                        calcsum += i_low + i_mid + i_high + q_low + q_mid + q_high;
                    }
                }

                // タイムスタンプ取得（基準からの経過時間を32bitに収める）
                struct timeval now_tv;
                gettimeofday(&now_tv, NULL);
                uint32_t now_us = (uint32_t)(((int64_t)now_tv.tv_sec * 1000000L + now_tv.tv_usec) & 0xFFFFFFFF);
                uint32_t rel_time = now_us - start_time_us;

                // トレーラー9バイトの付加（Time(4) + FrameCount(2) + Sum(3)）
                txdatabuf[txdatabufp++] = (rel_time >> 24) & 0xFF;
                txdatabuf[txdatabufp++] = (rel_time >> 16) & 0xFF;
                txdatabuf[txdatabufp++] = (rel_time >> 8)  & 0xFF;
                txdatabuf[txdatabufp++] = rel_time & 0xFF;
                txdatabuf[txdatabufp++] = (frame_cntr >> 8) & 0xFF;
                txdatabuf[txdatabufp++] = frame_cntr & 0xFF;
                txdatabuf[txdatabufp++] = (calcsum >> 16) & 0xFF;
                txdatabuf[txdatabufp++] = (calcsum >> 8)  & 0xFF;
                txdatabuf[txdatabufp++] = calcsum & 0xFF;

                // WebSocket経由でバイナリフレーム送信
                extern httpd_handle_t server;
                httpd_ws_frame_t frame = {
                    .type = HTTPD_WS_TYPE_BINARY,
                    .payload = txdatabuf,
                    .len = txdatabufp
                };
                esp_err_t err = httpd_ws_send_frame_async(server, client_fd, &frame);
                if (err != ESP_OK) {
                    ESP_LOGE("ws_send", "WS送信エラー: %d", err);
                    error_flg = 2;
                    // エラー時はループを抜けて停止
                    stop_requested = true;
                } else {
                    ESP_LOGI("ws_send", "WS送信成功: %d bytes", txdatabufp);
                }

                // フレーム送信後の処理: フレーム数カウントと上限チェック
                frame_count++;
                frame_cntr++;
                if (frame_limit > 0 && frame_count >= frame_limit) {
                    stop_requested = true;
                    ESP_LOGI("main", "Stop (FFT mode) - reached frame_limit");
                }

                // 次のフレーム取得前に、必要なら小休止を挟む
                // delay(5);  // （例）5ms待機：センサのフレーム周期に同期させるため（必要に応じて調整）
                do {
                    ak5816_read(&reg_page, 2, 0x0E, &frame_id_now);
                    u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"Check the frame number (Page.2 0x0E = 0x%02X)\n", frame_id_now);
                    mes_send();

                    if (frame_id_now == frame_id_prev) {
                        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"⚠ フレーム番号が更新されるのを待ちます。 (Page.2 0x0E = 0x%02X)\n", frame_id_now);
                        mes_send();
                        vTaskDelay(pdMS_TO_TICKS(2));  // 2ms待つ（センサフレーム周期より短めでOK）
                        wait_counter++;
                        if (wait_counter > 1000) {
                            u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"⚠ フレーム番号が更新されません (Page.2 0x0E = 0x%02X)\n", frame_id_now);
                            mes_send();
                            error_flg = 1;
                            break;
                        }
                    }
                } while (frame_id_now == frame_id_prev);

                //frame_id_prev = frame_id_now;
            }
        }

        // Stop triggering
        if (exec_mode) {
            u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"Set EXEC 0\n");
            mes_send();
            gpio_set_level(EXEC,0);
        }
        else {
            // Go to LOWPOWER state
            ak5816_write(&reg_page, AK5816_PAGE0, 0x0A, 0x08);
            delay(2);

            ak5816_read(&reg_page, AK5816_PAGE0, AK5816_CORE_STATE, &ret);
            if ((ret & 0x3F) != AK5816_STATE_LP){
                u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : IC is not in LP state(0x%02x) after measurement.\n", __FUNCTION__, ret);
                mes_send();
                error_flg = 1;
            }
        }

        // LED 2/3 OFF
        ktd2052_set_colour(3, 0x000000);
        ktd2052_set_colour(2, 0x000000);

        delay(1000);//higer than 1000ms       

        // Write back init RXG
        if (is_mrxg_enabled) {
            ret = 0xFF;
            ak5816_read(&reg_page, AK5816_PAGE1, 0x0F, &ret);
            ak5816_write(&reg_page, AK5816_PAGE1, 0x0F, ret | 0x01);
        }

        //CSI-2 setting: Disable
        // Clear bit0 of register 0x11 on page 0
        // BIT(n) is defined in ESP-IDF as (1UL << n)
        ak5816_update_reg_bits(&reg_page, AK5816_PAGE0, 0x11, BIT(0), 0);

        if (active_mode == MODE_FFT) {
            // Clear ERROR_ONESHOT
            ak5816_update_reg_bits(&reg_page, AK5816_PAGE0, 0x02, 0, BIT(4));
        }
    }
}

IRAM_ATTR static bool u_new_trans(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    esp_cam_ctlr_trans_t new_trans = *(esp_cam_ctlr_trans_t *)user_data;
    trans->buffer = new_trans.buffer;
    trans->buflen = new_trans.buflen;
    cntr1++;
    return false;
}

// When CSI-2 Rx got a whole new frame
IRAM_ATTR static bool u_trans_finished(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    uint8_t *buf1p = fifo_buf;
    void *buf2p = (void *)(&buf1p[fifo_inp]);
    void *buf3p = m_buffer; // Received data

    gettimeofday(&u_tv_int1, NULL);
    time_us_int1 = (int64_t)u_tv_int1.tv_sec * 1000000L + (int64_t)u_tv_int1.tv_usec;

    // Sync Memory to Cache: m_buffer (Rx data)
    esp_cache_msync((void *)m_buffer, m_buffer_len, ESP_CACHE_MSYNC_FLAG_DIR_M2C);

    // Copy Rx data to buf2p (fifo_buf[fifo_inp])
    memcpy(buf2p,buf3p,m_buffer_len);

    // Sync Cache to Memory: fifo_buf[fifo_inp]
    esp_cache_msync((void *)(&buf1p[fifo_inp]), m_buffer_len, ESP_CACHE_MSYNC_FLAG_DIR_C2M);

    // Update fifo_inp to signify 'address to write next data', rolled at fifo_buf_len
    fifo_inp += m_buffer_len;
    if(fifo_inp >= fifo_buf_len)
        //fifo_inp = 0;
        fifo_inp -= fifo_buf_len;

    timebuf[timebuf_inp] = (uint32_t)(time_us_int1 & 0xFFFFFFFF);
    timebuf_inp++;
    if(timebuf_inp >= timebuf_max)
        timebuf_inp = 0;

    cntr2++;
    if (cntr2 > cntr2_max) {
        cntr2 = 0;
    }

    gettimeofday(&u_tv, NULL);
    time_us4 = (int64_t)u_tv.tv_sec * 1000000L + (int64_t)u_tv.tv_usec;
    time_calc = (uint32_t)(time_us4 - time_us_int1);
    time_sum += time_calc;
    time_num++;
    if(time_calc < time_min)
        time_min = time_calc;
    if(time_calc > time_max)
        time_max = time_calc;

    return false;
}

void delay(int delay)
{
    uint32_t delay2 = delay;
    vTaskDelay(delay2);
}

void gpio_init(void)
{
    gpio_config_t gpio_conf;
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_conf.mode = GPIO_MODE_INPUT;
    gpio_conf.pin_bit_mask = (1ULL<<TEST_SW);
    gpio_conf.pull_down_en = 0;
    gpio_conf.pull_up_en = 0;
    gpio_conf.hys_ctrl_mode = GPIO_HYS_SOFT_DISABLE;
    gpio_config(&gpio_conf);

    gpio_config_t gpio_conf2;
    gpio_conf2.intr_type = GPIO_INTR_DISABLE;
    gpio_conf2.mode = GPIO_MODE_OUTPUT;
    gpio_conf2.pin_bit_mask = (1ULL<<RSTN)|(1ULL<<PDN)|(1ULL<<EXEC)|(1ULL<<TRIG)|(1ULL<<DBGMON)|(1ULL<<54)|(1ULL<<RF_LDO_E);
    gpio_conf2.pull_down_en = 0;
    gpio_conf2.pull_up_en = 0;
    gpio_conf2.hys_ctrl_mode = GPIO_HYS_SOFT_DISABLE;
    gpio_config(&gpio_conf2);
    gpio_set_level(RSTN,0);
    gpio_set_level(PDN,0);
    gpio_set_level(EXEC,0);
    gpio_set_level(DBGMON,0);
    gpio_set_level(54,0);   // GPIO54 C6 EN (Reset)
    gpio_set_level(RF_LDO_E,1);

    // No use pin Pull Up
    gpio_config_t gpio_conf3;
    gpio_conf3.intr_type = GPIO_INTR_DISABLE;
    gpio_conf3.mode = GPIO_MODE_DISABLE;
    gpio_conf3.pin_bit_mask = (1ULL<<15)|(1ULL<<27)|(1ULL<<43)|(1ULL<<45)|(1ULL<<46)|(1ULL<<47)|(1ULL<<48);
    gpio_conf3.pull_down_en = 0;
    gpio_conf3.pull_up_en = 1;
    gpio_conf3.hys_ctrl_mode = GPIO_HYS_SOFT_DISABLE;
    gpio_config(&gpio_conf3);

}

void uart_init(void)
{
    uart_config_t uart_conf = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;
    uart_driver_install(UART_NUM_0,1024,0,0,NULL,intr_alloc_flags);
    uart_param_config(UART_NUM_0,&uart_conf);
    uart_set_pin(UART_NUM_0, TXD, RXD, -1, -1);
}

void i2c_init(void)
{
    i2c_master_bus_config_t i2c_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = 23,
        .sda_io_num = 22,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    i2c_new_master_bus(&i2c_config, &bus_handle);
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x75, // KTD2052D
        .scl_speed_hz = 400000,
    };
    i2c_master_bus_add_device(bus_handle, &dev_cfg, &i2c_handle);
}

void spi_init(void)
{
    esp_err_t err;

    spi_bus_config_t spi_buscfg = {
        .miso_io_num = SPI_MISO,
        .mosi_io_num = SPI_MOSI,
        .sclk_io_num = SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 16,
    };
    err = spi_bus_initialize(SPI2_HOST, &spi_buscfg, SPI_DMA_CH_AUTO);
    if(err != ESP_OK)
    {
        uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"spi_bus_initialize=%d\n",err);
        u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);
    }

    spi_device_interface_config_t spi_devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .clock_speed_hz = 20000000,//10000000
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .input_delay_ns = 0,
        .spics_io_num = SPI_CS0,
        .flags = 0,
        .queue_size = 1,
    };
    err = spi_bus_add_device(SPI2_HOST, &spi_devcfg, &spi_handle);
    if(err != ESP_OK)
    {
        uarttxsize = snprintf((char*)uarttxbuf,uarttxbufmax,"spi_bus_add_device=%d\n",err);
        u_write_bytes(UART_NUM_0,uarttxbuf,uarttxsize);
    }

    spi_transaction.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    spi_transaction.length = 16;
    spi_transaction.rxlength = 16;
}

void spi_send(uint16_t data)
{
    spi_transaction.tx_data[0] = (uint8_t)((data >> 8) & 0xFF);
    spi_transaction.tx_data[1] = (uint8_t)(data & 0xFF);
    spi_device_polling_transmit(spi_handle,&spi_transaction);
}

void spi_sendread(uint16_t data, uint8_t* val)
{
    spi_transaction.tx_data[0] = (uint8_t)((data >> 8) & 0xFF);
    spi_transaction.tx_data[1] = (uint8_t)(data & 0xFF);
    spi_device_polling_transmit(spi_handle,&spi_transaction);
    *val = spi_transaction.rx_data[1];
}

void ak5816_write(uint8_t *reg_page, uint8_t page, uint8_t reg_addr, uint8_t data)
{
    uint16_t senddata;

    if (*reg_page != page){
        senddata = (AK5816_CORE_PAGE_SETTING <<8) | (page & 0x0F);
        spi_send(senddata);
        *reg_page = page;
    }

    reg_addr = reg_addr & 0x7F; // write bit 0

    senddata =  (reg_addr << 8) | data;
    spi_send(senddata);

}

void ak5816_read(uint8_t *reg_page, uint8_t page, uint8_t reg_addr, uint8_t *buf)
{
    uint16_t senddata;
    uint16_t addr;

    if (*reg_page != page){
        senddata = (AK5816_CORE_PAGE_SETTING <<8) | (page & 0x0F);
        spi_send(senddata);
        *reg_page = page;
    }

    addr = (reg_addr | 0x80) <<8; // read bit 1
    spi_sendread(addr, buf);
}

// -------------------------------------------------------------------------------
// Run manual RXG cal 
// -------------------------------------------------------------------------------
int ak5816_run_mrxg(uint8_t *p_reg_page)
{
    int RXGCAL_CHSEL;

    int16_t rfpow_code;
    float rfpow_dBm;
    int16_t bbpow_code;
    float bbpow_dBm;
    float rxgdev_dB;

    uint8_t hpf_buf, gain_buf;

    int rxg_rmx_adj;
    int rxg_rpga_adj;


    ak5816_read(p_reg_page, AK5816_PAGE5, 0x2E, &ret); // RXGCAL_CHSEL
    RXGCAL_CHSEL = (ret >> 4) & 0x3;

    ak5816_write(p_reg_page, AK5816_PAGE0, 0x04, 0x5A); // Magic

    ak5816_read(p_reg_page, AK5816_PAGE0, 0x03, &ret);
    ak5816_write(p_reg_page, AK5816_PAGE0, 0x03, ret & 0xFD); // CTCHK_E = 0;

    ak5816_read(p_reg_page, AK5816_PAGE1, 0x3D, &ret);
    ak5816_write(p_reg_page, AK5816_PAGE1, 0x3D, ret & 0xDF); // RINTSG_RF0HPF1 = 0

    // Check R_PWMEAS_R*_NEW == 0 -------------------------
    ak5816_read(p_reg_page, AK5816_PAGE1, 0x20, &ret); // R_PWMEAS_R*_NEW
    // Clear by ACK
    if (((ret >> RXGCAL_CHSEL) & 0x1) != 0)
    {
        ak5816_write(p_reg_page, AK5816_PAGE1, 0x1E, 0x10); // PWMEAS_RX_ACK
    }
    ak5816_read(p_reg_page, AK5816_PAGE1, 0x20, &ret); // R_PWMEAS_R*_NEW
    if (((ret >> RXGCAL_CHSEL) & 0x1) != 0)
    {
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"[ERROR] MRXG: R_PWMEAS_R*_NEW not cleared by ACK!\n");
        mes_send();
        return -1;
    }

    // Select PWMEAS_R*_E
    ak5816_write(p_reg_page, AK5816_PAGE1, 0x1F, 0x01 << RXGCAL_CHSEL);

    ak5816_write(p_reg_page, AK5816_PAGE10, 0x13, 0xC4); // 
    ak5816_write(p_reg_page, AK5816_PAGE10, 0x67, 0x72); // 
    ak5816_write(p_reg_page, AK5816_PAGE11, 0x25, 0x08); // 
    ak5816_write(p_reg_page, AK5816_PAGE1, 0x3E, 0x01); // 

    // Start measurement
    ak5816_write(p_reg_page, AK5816_PAGE1, 0x1E, 0x01); // PWMEAS_RX_RUN
    delay(2); // 180 us

    // Check R_PWMEAS_R*_NEW == 1 -------------------------
    ak5816_read(p_reg_page, AK5816_PAGE1, 0x20, &ret); // R_PWMEAS_R*_NEW
    // Clear by ACK
    if (((ret >> RXGCAL_CHSEL) & 0x1) == 1)
    {
        ak5816_write(p_reg_page, AK5816_PAGE1, 0x1E, 0x10); // PWMEAS_RX_ACK
    }
    else
    {
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"[ERROR] MRXG: R_PWMEAS_R*_NEW not set after 180 us!\n");
        mes_send();
        return -1;
    }

    // Read BIST RF Power
    ak5816_read(p_reg_page, AK5816_PAGE1, 0x21 + RXGCAL_CHSEL * 2, &ret);
    rfpow_code = ret << 8;
    ak5816_read(p_reg_page, AK5816_PAGE1, 0x22 + RXGCAL_CHSEL * 2, &ret);
    rfpow_code += ret;

    // Calculate BIST RF Power
    rfpow_dBm = 10 * log10(rfpow_code) - 85.86;

    ak5816_write(p_reg_page, AK5816_PAGE10, 0x13, 0x04); // 
    ak5816_write(p_reg_page, AK5816_PAGE10, 0x67, 0x42); // 

    // BB Power Measurement ------------------------------
    // keep current value
    ak5816_read(p_reg_page, AK5816_PAGE0, 0x13, &hpf_buf);
    ak5816_read(p_reg_page, AK5816_PAGE0, 0x14, &gain_buf);
    // change HPF / Gain
    ak5816_write(p_reg_page, AK5816_PAGE0, 0x13, 0x00); // HPF fc=200kHz
    ak5816_write(p_reg_page, AK5816_PAGE1, 0x3D, 0x03); // RINTSG_F = 3 (1.67 MHz)
    ak5816_write(p_reg_page, AK5816_PAGE0, 0x12, 0x4F + (RXGCAL_CHSEL << 4)); // RX_CHSEL
    ak5816_write(p_reg_page, AK5816_PAGE0, 0x14, 0x02); // RMX Low, RPGA 0dB TODO
    ak5816_write(p_reg_page, AK5816_PAGE11, 0x25, 0x00); // 
    // Start measurement
    ak5816_write(p_reg_page, AK5816_PAGE1, 0x3E, 0x01); // RINTSG_MEAS
    delay(2); // 300 us

    // Check RINTSG_MEAS == 0 -------------------------
    ak5816_read(p_reg_page, AK5816_PAGE1, 0x3E, &ret); // RINTSG_MEAS
    if ((ret & 0x1) != 0)
    {
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"[ERROR] MRXG: RINTSG_MEAS not cleared after 300 us!\n");
        mes_send();
        return -1;
    }

    // Read BB Power
    ak5816_read(p_reg_page, AK5816_PAGE1, 0x3F, &ret); // R_RINTLV
    bbpow_code = ret << 8;
    ak5816_read(p_reg_page, AK5816_PAGE1, 0x40, &ret); // R_RINTLV
    bbpow_code += ret;

    // Calculate BB Power
    bbpow_dBm = 20 * log10(bbpow_code) - 92.35;

    // Calculate calibration codes
    rxgdev_dB = 34.0 - (bbpow_dBm - rfpow_dBm);

    rxg_rmx_adj = round(rxgdev_dB / 1.5) + 2;
    // limit to 0 ~ 4
    if (rxg_rmx_adj < 0) rxg_rmx_adj = 0;
    else if (rxg_rmx_adj > 4) rxg_rmx_adj = 4;

    rxg_rpga_adj = round( (rxgdev_dB - (rxg_rmx_adj - 2) * 1.5) / 0.3) + 17;
    // limit to 0 ~ 45
    if (rxg_rpga_adj < 0) rxg_rpga_adj = 0;
    else if (rxg_rpga_adj > 45) rxg_rpga_adj = 45;


    // Write calibration result
    ak5816_write(p_reg_page, AK5816_PAGE6, 0x60, rxg_rpga_adj & 0x3F); // RXGCAL_RPGA0_ADJ
    ak5816_write(p_reg_page, AK5816_PAGE6, 0x61, rxg_rpga_adj & 0x3F); // RXGCAL_RPGA1_ADJ
    ak5816_write(p_reg_page, AK5816_PAGE6, 0x62, rxg_rpga_adj & 0x3F); // RXGCAL_RPGA2_ADJ
    ak5816_write(p_reg_page, AK5816_PAGE6, 0x63, rxg_rpga_adj & 0x3F); // RXGCAL_RPGA3_ADJ
    ak5816_write(p_reg_page, AK5816_PAGE6, 0x65, ((rxg_rmx_adj & 0x07) << 4) + (rxg_rmx_adj & 0x07)); // RXGCAL_RMX0/1_ADJ
    ak5816_write(p_reg_page, AK5816_PAGE6, 0x66, ((rxg_rmx_adj & 0x07) << 4) + (rxg_rmx_adj & 0x07)); // RXGCAL_RMX2/3_ADJ

    u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"[INFO] MRXG: RF/BB: %.2f/%.2f dBm (Correction %.2f dB), RMX %d, PGA %d\n", rfpow_dBm, bbpow_dBm, rxgdev_dB, rxg_rmx_adj, rxg_rpga_adj);
    mes_send();

    // HPF
    ak5816_write(p_reg_page, AK5816_PAGE0, 0x13, hpf_buf);
    ak5816_write(p_reg_page, AK5816_PAGE0, 0x14, gain_buf);

    ak5816_read(p_reg_page, AK5816_PAGE0, 0x03, &ret);
    ak5816_write(p_reg_page, AK5816_PAGE0, 0x03, ret | 0x02); // CTCHK_E = 1;
    ak5816_write(p_reg_page, AK5816_PAGE0, 0x04, 0x00); // Magic

    return 0;
}

int ak5816_start_up_demo(uint8_t *p_reg_page)
{
    // int ad_sample_num = AD_SAMPLE_NUM;
    // int ad_chirp_mult = AD_CHIRP_MULT;

    uint8_t ret;
    *p_reg_page = 0;

    u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"---AK5816 IC reset & start up----\n");
    mes_send();

    // PDN High
    gpio_set_level(PDN,1);
    delay(1);

    // Chip ID check
    ret = 0xff;
    ak5816_read(p_reg_page, AK5816_PAGE0, AK5816_CORE_CHIP_INFO, &ret);
    if (ret != AK5816_CHIP_ID){
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : Chip ID is incorrect. IC is not AK5816(0x%02x).\n", __FUNCTION__, ret);
        mes_send();
        return -1;
    }

    // Error check after PDN released
    ret = 0xff;
    ak5816_read(p_reg_page, AK5816_PAGE0, 0x02, &ret);
    if ((ret & 0x70) != 0x00){
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : Error detected after PDN released. Address 0x02 is (0x%02x).\n", __FUNCTION__, ret);
        mes_send();
        return -1;
    }

    // waiting for more than 24ms
    delay(24);

    // RSTN High
    gpio_set_level(RSTN,1);
    delay(1);

    // Error check after reset released
    ret = 0xff;
    ak5816_read(p_reg_page, AK5816_PAGE0, 0x02, &ret);
    if ((ret & 0x70) != 0x00){
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : Error detected after RSTN released. Address 0x02 is (0x%02x).\n", __FUNCTION__, ret);
        mes_send();
        return -1;
    }

    // Chip state check
    ret = 0xff;
    ak5816_read(p_reg_page, AK5816_PAGE0, AK5816_CORE_STATE, &ret);
    if ((ret & 0x3F) != AK5816_STATE_LP){
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : IC is not in LP state(0x%02x).\n", __FUNCTION__, ret);
        mes_send();
        return -1;
    }

    // Check if LPCHK is already done before calling this function
    ret = 0xff;
    ak5816_read(p_reg_page, AK5816_PAGE1, 0x0A, &ret);
    if ((ret & 0x10) != 0x10){ // LPCHK is not done yet
        // Execute LPCHK
        ak5816_write(p_reg_page, AK5816_PAGE1, 0x0A, 0x03); 
        delay(1);

        // Check completion status
        ret = 0xff;
        ak5816_read(p_reg_page, AK5816_PAGE1, 0x0A, &ret);
        if ((ret & 0x10) != 0x10){
            u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : LPCAL hasn't been completed. Read-back data is (0x%02x).\n", __FUNCTION__, ret);
            mes_send();
            return -1;
        }

        // Check errors
        ret = 0xff;
        ak5816_read(p_reg_page, AK5816_PAGE0, 0x02, &ret);
        if ((ret & 0x70) != 0x00){
            u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : Error detected after LPCAL. Address 0x02 is (0x%02x).\n", __FUNCTION__, ret);
            mes_send();
            return -1;
        }
    }


    //RPU setting: Disable
    ak5816_update_reg_bits(p_reg_page, AK5816_PAGE2, 0x0A, BIT(0), 0);

    //CSI-2 setting
    ak5816_update_reg_bits(p_reg_page, AK5816_PAGE0, 0x11, BIT(0), 0);

    //ADC Sampling Freq: 53.3Msps
    //ak5816_update_reg_bits(p_reg_page, AK5816_PAGE0, 0x15, BIT(0), 0);


    //ADC data output setting: Enable
    ak5816_write(p_reg_page, AK5816_PAGE2, 0x31, 0x02);

    // CSI-2 DCLK P/N swap
    ak5816_write(p_reg_page, AK5816_PAGE0, 0x0E, 0x40);

    if(ad_sample_num == 32){
        ak5816_write(p_reg_page, AK5816_PAGE0, 0x0F, 0x00); //Sampling data setting(ADC): 32 (Not auto)
    }else if(ad_sample_num == 64){
        ak5816_write(p_reg_page, AK5816_PAGE0, 0x0F, 0x11); //Sampling data setting(ADC): 64 (Not auto)
    }else if(ad_sample_num == 128){
        ak5816_write(p_reg_page, AK5816_PAGE0, 0x0F, 0x22); //Sampling data setting(ADC): 128 (Not auto)
    }else if(ad_sample_num == 256){
        ak5816_write(p_reg_page, AK5816_PAGE0, 0x0F, 0x33); //Sampling data setting(ADC): 256 (Not auto)
    }else if(ad_sample_num == 512){
        ak5816_write(p_reg_page, AK5816_PAGE0, 0x0F, 0x44); //Sampling data setting(ADC): 512 (Not auto)
    }else if(ad_sample_num == 1024){
        ak5816_write(p_reg_page, AK5816_PAGE0, 0x0F, 0x55); //Sampling data setting(ADC): 1024 (Not auto)
        //ak5816_write(p_reg_page, AK5816_PAGE0, 0x27, 0x06); //STM1A time setting
        //ak5816_write(p_reg_page, AK5816_PAGE0, 0x28, 0x4F); //20.2us
    }

    // Trig is Reg.
    //ak5816_update_reg_bits(p_reg_page, AK5816_PAGE0, 0x0B, BIT(2), 0);

    // CLK Non-continuous
    ak5816_update_reg_bits(p_reg_page, AK5816_PAGE0, 0x10, BIT(4), 0);

    //SSETCNTA setting
    //ak5816_write(p_reg_page, AK5816_PAGE0, 0x35, (AD_CHIRP_MULT - 1));
    ak5816_write(p_reg_page, AK5816_PAGE0, 0x35, (ad_chirp_mult - 1));

    /*
    // Error check before LPCAL
    ret = 0xff;
    ak5816_read(p_reg_page, AK5816_PAGE0, 0x02, &ret);
    if ((ret & 0x70) != 0x00){
        printf("%s : Error detected before LPCAL. Address 0x02 is (0x%02x).\n", __FUNCTION__, ret);
        return -1;
    }

    //LPCAL Execution
    ak5816_update_reg_bits(p_reg_page, AK5816_PAGE1, 0x0A, 0, BIT(0));
    delay(1);

    // Check for LPCAL completion
    ret = 0xff;
    ak5816_read(p_reg_page, AK5816_PAGE1, 0x0A, &ret);
    if ((ret & 0x10) != 0x10){
        printf("%s : LPCAL hasn't been completed. Read-back data is (0x%02x).\n", __FUNCTION__, ret);
        return -1;
    }

    // Error check after LPCAL
    ret = 0xff;
    ak5816_read(p_reg_page, AK5816_PAGE0, 0x02, &ret);
    if ((ret & 0x70) != 0x00){
        printf("%s : Error detected after LPCAL. Address 0x02 is (0x%02x).\n", __FUNCTION__, ret);
        return -1;
    }
    */

    // Run initial manual RXGcal if enabled
    ret = 0xFF;
    ak5816_read(p_reg_page, AK5816_PAGE1, 0x0F, &ret);
    is_mrxg_enabled =  ret & 0x01;

    if (is_mrxg_enabled || !exec_mode) { // if CAL_INIT_RXG == 1
        ak5816_write(p_reg_page, AK5816_PAGE1, 0x0F, ret & 0xFE); // disable CAL_INIT_RXG

        if (!exec_mode) {
            //CSI-2 output enable
            // this must be done in LOWPOWER state
            ak5816_update_reg_bits(p_reg_page, AK5816_PAGE0, 0x11, 0, BIT(0));
        }
        // Go to Standby state
        ak5816_write(p_reg_page, AK5816_PAGE0, 0x0A, 0x02);
        delay(2);

        // Chip state check
        ret = 0xff;
        ak5816_read(p_reg_page, AK5816_PAGE0, AK5816_CORE_STATE, &ret);
        if ((ret & 0x3F) != AK5816_STATE_STBY){
            u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : IC is not in STB state(0x%02x).\n", __FUNCTION__, ret);
            mes_send();
            return -1;
        }

        // Error check before Initial CAL
        ret = 0xff;
        ak5816_read(p_reg_page, AK5816_PAGE0, 0x02, &ret);
        if ((ret & 0x70) != 0x00){
            u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : Error detected before Initial CAL. Address 0x02 is (0x%02x).\n", __FUNCTION__, ret);
            mes_send();
            return -1;
        }

        // Initial CAL Execution
        ak5816_update_reg_bits(p_reg_page, AK5816_PAGE1, 0x0B, 0, BIT(0));
        //ak5816_write(p_reg_page, AK5816_PAGE1, 0x0B, 0x05);
        delay(12);

        // Check for Initial CAL completion
        ret = 0xff;
        ak5816_read(p_reg_page, AK5816_PAGE1, 0x0B, &ret);
        if ((ret & 0x11) != 0x10){
            u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : Initial CAL hasn't been completed. Read-back data is (0x%02x).\n", __FUNCTION__, ret);
            mes_send();
            return -1;
        }

        // Error check after Initial CAL
        ret = 0xff;
        ak5816_read(p_reg_page, AK5816_PAGE0, 0x02, &ret);
        if ((ret & 0x70) != 0x00){
            u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : Error detected after Initial CAL. Address 0x02 is (0x%02x).\n", __FUNCTION__, ret);
            mes_send();
            return -1;
        }

        // Run Manual RXG
        if (is_mrxg_enabled) {
            ret = ak5816_run_mrxg(p_reg_page);
            if (ret != 0) {
                return ret;
            }
            // Error check
            ret = 0xff;
            ak5816_read(p_reg_page, AK5816_PAGE0, 0x02, &ret);
            if ((ret & 0x70) != 0x00){
                u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : Error detected after MRXG. Address 0x02 is (0x%02x).\n", __FUNCTION__, ret);
                mes_send();
                ak5816_error_read_demo(p_reg_page);
                return -1;
            }
        }

        // Go back to LOWPOWER for EXEC mode
        if (exec_mode) {
            // Go to LOWPOWER state
            ak5816_write(p_reg_page, AK5816_PAGE0, 0x0A, 0x08);
            delay(2);

            ak5816_read(p_reg_page, AK5816_PAGE0, AK5816_CORE_STATE, &ret);
            if ((ret & 0x3F) != AK5816_STATE_LP){
                u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : IC is not in LP state(0x%02x) after initial manual RXG cal.\n", __FUNCTION__, ret);
                mes_send();
                return -1;
            }
        }
        // Go to SLEEP for manual trigger mode
        else {
            ak5816_go_slp_demo(p_reg_page);
        }
    }

    if (exec_mode) {
        //CSI-2 output enable
        ak5816_update_reg_bits(p_reg_page, AK5816_PAGE0, 0x11, 0, BIT(0));

        // Error check after CSI-2 output enable
        ret = 0xff;
        ak5816_read(p_reg_page, AK5816_PAGE0, 0x02, &ret);
        if ((ret & 0x70) != 0x00){
            u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : Error detected after CSI-2 output enable. Address 0x02 is (0x%02x).\n", __FUNCTION__, ret);
            mes_send();
            ak5816_error_read_demo(p_reg_page);
            return -1;
        }
    }

    u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"---AK5816 IC reset & start up End----\n");
    mes_send();

    return 0;
}

int ak5816_start_up_fftdemo(uint8_t *p_reg_page)
{
    // int ad_sample_num = AD_SAMPLE_NUM;
    // int ad_chirp_mult = AD_CHIRP_MULT;

    uint8_t ret;
    *p_reg_page = 0;

    u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"---AK5816 IC reset & start up----\n");
    mes_send();

    // PDN High
    gpio_set_level(PDN,1);
    delay(1);

    // Chip ID check
    ret = 0xff;
    ak5816_read(p_reg_page, AK5816_PAGE0, AK5816_CORE_CHIP_INFO, &ret);
    if (ret != AK5816_CHIP_ID){
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : Chip ID is incorrect. IC is not AK5816(0x%02x).\n", __FUNCTION__, ret);
        mes_send();
        return -1;
    }

    // Error check after PDN released
    ret = 0xff;
    ak5816_read(p_reg_page, AK5816_PAGE0, 0x02, &ret);
    if ((ret & 0x70) != 0x00){
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : Error detected after PDN released. Address 0x02 is (0x%02x).\n", __FUNCTION__, ret);
        mes_send();
        return -1;
    }

    // waiting for more than 24ms
    delay(24);

    // RSTN High
    gpio_set_level(RSTN,1);
    delay(1);

    // Error check after reset released
    ret = 0xff;
    ak5816_read(p_reg_page, AK5816_PAGE0, 0x02, &ret);
    if ((ret & 0x70) != 0x00){
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : Error detected after RSTN released. Address 0x02 is (0x%02x).\n", __FUNCTION__, ret);
        mes_send();
        return -1;
    }

    // Chip state check
    ret = 0xff;
    ak5816_read(p_reg_page, AK5816_PAGE0, AK5816_CORE_STATE, &ret);
    if ((ret & 0x3F) != AK5816_STATE_LP){
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : IC is not in LP state(0x%02x).\n", __FUNCTION__, ret);
        mes_send();
        return -1;
    }

    //RPU setting: Disable
    ak5816_update_reg_bits(p_reg_page, AK5816_PAGE2, 0x0A, BIT(0), 0);

    //CSI-2 setting
    //ak5816_update_reg_bits(p_reg_page, AK5816_PAGE0, 0x11, (BIT(1)|BIT(0)), 0);

    //ADC Sampling Freq: 53.3Msps
    ak5816_update_reg_bits(p_reg_page, AK5816_PAGE0, 0x15, BIT(0), 0);


    //ADC data output setting: Enable
    ak5816_write(p_reg_page, AK5816_PAGE2, 0x31, 0x02);

    // CSI-2 DCLK P/N swap
    //ak5816_write(p_reg_page, AK5816_PAGE0, 0x0E, 0x40);

    if(ad_sample_num == 32){
        ak5816_write(p_reg_page, AK5816_PAGE0, 0x0F, 0x00); //Sampling data setting(ADC): 32 (Not auto)
        ak5816_update_reg_bits(p_reg_page, AK5816_PAGE2, 0x12, (BIT(2)|BIT(1)|BIT(0)), 0); //Sampling data setting(RPU): 32 (Not auto)
    }else if(ad_sample_num == 64){
        ak5816_write(p_reg_page, AK5816_PAGE0, 0x0F, 0x11); //Sampling data setting(ADC): 64 (Not auto)
        ak5816_update_reg_bits(p_reg_page, AK5816_PAGE2, 0x12, (BIT(2)|BIT(1)), BIT(0)); //Sampling data setting(RPU): 64 (Not auto)
    }else if(ad_sample_num == 128){
        ak5816_write(p_reg_page, AK5816_PAGE0, 0x0F, 0x22); //Sampling data setting(ADC): 128 (Not auto)
        ak5816_update_reg_bits(p_reg_page, AK5816_PAGE2, 0x12, (BIT(2)|BIT(0)), BIT(1)); //Sampling data setting(RPU): 128 (Not auto)
    }else if(ad_sample_num == 256){
        ak5816_write(p_reg_page, AK5816_PAGE0, 0x0F, 0x33); //Sampling data setting(ADC): 256 (Not auto)
        ak5816_update_reg_bits(p_reg_page, AK5816_PAGE2, 0x12, BIT(2), (BIT(1)|BIT(0))); //Sampling data setting(RPU): 256 (Not auto)
    }else if(ad_sample_num == 512){
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : Sampling data of RPU operation does not support 512 points.","ERROR");
        mes_send();
        return -1;
    }else if(ad_sample_num == 1024){
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : Sampling data of RPU operation does not support 1024 points.","ERROR");
        mes_send();
        return -1;
    }

    // Trig is Reg.
    //ak5816_update_reg_bits(p_reg_page, AK5816_PAGE0, 0x0B, BIT(2), 0);

    // CLK Non-continuous
    //ak5816_update_reg_bits(p_reg_page, AK5816_PAGE0, 0x10, BIT(4), 0);

    //SSETCNTA setting
    //ak5816_write(p_reg_page, AK5816_PAGE0, 0x35, (AD_CHIRP_MULT - 1));
    ak5816_write(p_reg_page, AK5816_PAGE0, 0x35, (ad_chirp_mult - 1));

    /*
    // Error check before LPCAL
    ret = 0xff;
    ak5816_read(p_reg_page, AK5816_PAGE0, 0x02, &ret);
    if ((ret & 0x70) != 0x00){
        printf("%s : Error detected before LPCAL. Address 0x02 is (0x%02x).\n", __FUNCTION__, ret);
        return -1;
    }

    //LPCAL Execution
    ak5816_update_reg_bits(p_reg_page, AK5816_PAGE1, 0x0A, 0, BIT(0));
    delay(1);

    // Check for LPCAL completion
    ret = 0xff;
    ak5816_read(p_reg_page, AK5816_PAGE1, 0x0A, &ret);
    if ((ret & 0x10) != 0x10){
        printf("%s : LPCAL hasn't been completed. Read-back data is (0x%02x).\n", __FUNCTION__, ret);
        return -1;
    }

    // Error check after LPCAL
    ret = 0xff;
    ak5816_read(p_reg_page, AK5816_PAGE0, 0x02, &ret);
    if ((ret & 0x70) != 0x00){
        printf("%s : Error detected after LPCAL. Address 0x02 is (0x%02x).\n", __FUNCTION__, ret);
        return -1;
    }
    */

    /*
    //Go Standby state
    ak5816_write(p_reg_page, AK5816_PAGE0, 0x0A, 0x02);
    delay(2);

    // Chip state check
    ret = 0xff;
    ak5816_read(p_reg_page, AK5816_PAGE0, AK5816_CORE_STATE, &ret);
    if ((ret & 0x3F) != AK5816_STATE_STBY){
        printf("%s : IC is not in STB state(0x%02x).\n", __FUNCTION__, ret);
        return -1;
    }
    */

    // Clear ERROR_ONESHOT
    ak5816_update_reg_bits(p_reg_page, AK5816_PAGE0, 0x02, 0, BIT(4));

    // Error check after Clear ERROR_ONESHOT
    ret = 0xff;
    ak5816_read(p_reg_page, AK5816_PAGE0, 0x02, &ret);
    if ((ret & 0x70) != 0x00){
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : Error detected after Clear ERROR_ONESHOT. Address 0x02 is (0x%02x).\n", __FUNCTION__, ret);
        mes_send();
        return -1;
    }


 #ifndef EXEC_EN // In case not use EXEC function

    //Go Standby state
    ak5816_write(p_reg_page, AK5816_PAGE0, 0x0A, 0x02);
    delay(2);

    // Chip state check
    ret = 0xff;
    ak5816_read(p_reg_page, AK5816_PAGE0, AK5816_CORE_STATE, &ret);
    if ((ret & 0x3F) != AK5816_STATE_STBY){
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : IC is not in STB state(0x%02x).\n", __FUNCTION__, ret);
        mes_send();
        return -1;
    }

    // Clear ERROR_ONESHOT
    ak5816_update_reg_bits(p_reg_page, AK5816_PAGE0, 0x02, 0, BIT(4));

    // Error check after Clear ERROR_ONESHOT and before Initial CAL
    ret = 0xff;
    ak5816_read(p_reg_page, AK5816_PAGE0, 0x02, &ret);
    if ((ret & 0x70) != 0x00){
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : Error detected after Clear ERROR_ONESHOT and before Initial CAL. Address 0x02 is (0x%02x).\n", __FUNCTION__, ret);
        mes_send();
        return -1;
    }    

    //Initial CAL Execution
    ak5816_update_reg_bits(p_reg_page, AK5816_PAGE1, 0x0B, 0, BIT(0));
    //ak5816_write(p_reg_page, AK5816_PAGE1, 0x0B, 0x05);
    delay(12);

    // Check for Initial CAL completion
    ret = 0xff;
    ak5816_read(p_reg_page, AK5816_PAGE1, 0x0B, &ret);
    if ((ret & 0x11) != 0x10){
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : Initial CAL hasn't been completed. Read-back data is (0x%02x).\n", __FUNCTION__, ret);
        mes_send();
        return -1;
    }

    // Error check after Initial CAL
    ret = 0xff;
    ak5816_read(p_reg_page, AK5816_PAGE0, 0x02, &ret);
    if ((ret & 0x70) != 0x00){
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : Error detected after Initial CAL. Address 0x02 is (0x%02x).\n", __FUNCTION__, ret);
        mes_send();
        return -1;
    }

 #endif // In case not use EXEC function

    //CSI-2 output enable
    //ak5816_update_reg_bits(p_reg_page, AK5816_PAGE0, 0x11, 0, BIT(0));

    //RPU setting: Enable and Stop at Datacube stored
    ak5816_update_reg_bits(p_reg_page, AK5816_PAGE2, 0x0A, 0, (BIT(4)|BIT(0)));    

    // Error check after RPU enable setting
    ret = 0xff;
    ak5816_read(p_reg_page, AK5816_PAGE0, 0x02, &ret);
    if ((ret & 0x70) != 0x00){
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : Error detected after RPU enable. Address 0x02 is (0x%02x).\n", __FUNCTION__, ret);
        mes_send();
        return -1;
    }

    u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"---AK5816 IC reset & start up End----\n");
    mes_send();

    return 0;
}


int ak5816_go_standby_demo(uint8_t *p_reg_page)
{
    // Read current state
    ak5816_read(p_reg_page, AK5816_PAGE0, AK5816_CORE_STATE, &ret);
    int prev_state = ret & 0x3F;

    // Return if already at Standby state
    if (prev_state == AK5816_STATE_STBY) {
        return 0;
    }

    // Issue GO_STBY
    ak5816_write(p_reg_page, AK5816_PAGE0, 0x0A, 0x02); //Go Standby state

    // Wait
    //delay(2);
    if (prev_state == AK5816_STATE_LP) { // from LOWPOWER
        esp_rom_delay_us(1100); // needs 1000 us
    }
    else if (prev_state == AK5816_STATE_SLP) { // from SLEEP
        esp_rom_delay_us(220); // needs 200 us
    }
    else if (prev_state == AK5816_STATE_TRX) { // from TRX
        esp_rom_delay_us(110); // needs 100 us
    }

    // Check current statae
    ret = 0xff;
    ak5816_read(p_reg_page, AK5816_PAGE0, AK5816_CORE_STATE, &ret);
    if ((ret & 0x3F) != AK5816_STATE_STBY){
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : IC is not in STB state(0x%02x).\n", __FUNCTION__, ret);
        mes_send();
        return -1;
    }

    // Read error register
    ret = 0xff;
    ak5816_read(p_reg_page, AK5816_PAGE0, 0x02, &ret);
    if ((ret & 0x70) != 0x00){
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : Error detected before Initial CAL. Address 0x02 is (0x%02x).\n", __FUNCTION__, ret);
        mes_send();
        return -1;
    }
    return 0;
}

int ak5816_correction_cal_demo(uint8_t *p_reg_page)
{
    ak5816_write(p_reg_page, AK5816_PAGE1, 0x0B, 0x07); //Correction CAL Execution
    delay(10);
    // Check for Initial CAL completion
    ret = 0xff;
    ak5816_read(p_reg_page, AK5816_PAGE1, 0x0B, &ret);
    if ((ret & 0x31) != 0x30){
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : ERROR: Correction CAL hasn't been completed. Read-back data is (0x%02x).\n", __FUNCTION__, ret);
        mes_send();
        return -1;
    }
    ret = 0xff;
    ak5816_read(p_reg_page, AK5816_PAGE0, 0x02, &ret);
    if ((ret & 0x70) != 0x00){
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : Error detected after Initial CAL. Address 0x02 is (0x%02x).\n", __FUNCTION__, ret);
        mes_send();
        return -1;
    }
    return 0;
}

int ak5816_go_trx_demo(uint8_t *p_reg_page)
{
    ak5816_write(p_reg_page, AK5816_PAGE0, 0x0A, 0x01); //Go TRX state
    //delay(2);
    esp_rom_delay_us(110); // needs 100 us
    ret = 0xff;
    ak5816_read(p_reg_page, AK5816_PAGE0, 0x02, &ret);
    if ((ret & 0x70) != 0x00){
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : Error detected after transition to TRX state. Address 0x02 is (0x%02x).\n", __FUNCTION__, ret);
        mes_send();
        return -1;
    }

    // ret = 0xff;
    // ak5816_read(p_reg_page, AK5816_PAGE0, AK5816_CORE_STATE, &ret);
    // while ((ret & 0x3F) != AK5816_STATE_TRX){
    //     ret = 0xff;
    //     ak5816_read(p_reg_page, AK5816_PAGE0, AK5816_CORE_STATE, &ret);
    //     printf("State_trx:%x\n", ret);
    //     esp_rom_delay_us(110);
    // }

    return 0;
}

int ak5816_go_slp_demo(uint8_t *p_reg_page)
{
    ak5816_write(p_reg_page, AK5816_PAGE0, 0x0A, 0x04); //Go Sleep state
    //delay(2);
    esp_rom_delay_us(110); // needs 100 us

    // ret = 0xff;
    // ak5816_read(p_reg_page, AK5816_PAGE0, AK5816_CORE_STATE, &ret);
    // while ((ret & 0x3F) != AK5816_STATE_SLP){
    //     ret = 0xff;
    //     ak5816_read(p_reg_page, AK5816_PAGE0, AK5816_CORE_STATE, &ret);
    //     printf("State_slp:%x\n", ret);
    //     esp_rom_delay_us(110);
    // }

    ret = 0xff;
    ak5816_read(p_reg_page, AK5816_PAGE0, 0x02, &ret);
    if ((ret & 0x70) != 0x00){
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : Error detected after transition to sleep state. Address 0x02 is (0x%02x).\n", __FUNCTION__, ret);
        mes_send();
        ak5816_error_read_demo(p_reg_page);
        return -1;
    }
    return 0;
}

void ak5816_error_read_demo(uint8_t *p_reg_page)
{
    uint8_t readdata1, readdata2;
    ret = 0xff;
    ak5816_read(p_reg_page, AK5816_PAGE0, 0x02, &ret);
    if ((ret & 0x20) != 0x00){
        ak5816_read(p_reg_page, AK5816_PAGE0, 0x06, &readdata1);
        u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : Core 0x06 is (0x%02x).\n", __FUNCTION__, readdata1);
        mes_send();
        if ((readdata1 & 0x01) != 0x00){
            ak5816_read(p_reg_page, AK5816_PAGE4, 0x0E, &readdata2);
            u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : PAGE4 0x0E is (0x%02x).\n", __FUNCTION__, readdata2);
            mes_send();
            ak5816_read(p_reg_page, AK5816_PAGE4, 0x0F, &readdata2);
            u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : PAGE4 0x0F is (0x%02x).\n", __FUNCTION__, readdata2);
            mes_send();
        }
        if ((readdata1 & 0x02) != 0x00){
            ak5816_read(p_reg_page, AK5816_PAGE4, 0x11, &readdata2);
            u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : PAGE4 0x11 is (0x%02x).\n", __FUNCTION__, readdata2);
            mes_send();
        }
        if ((readdata1 & 0x04) != 0x00){
            ak5816_read(p_reg_page, AK5816_PAGE4, 0x0B, &readdata2);
            u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"%s : PAGE4 0x0B is (0x%02x).\n", __FUNCTION__, readdata2);
            mes_send();
        }
    }
}

void send_data(uint8_t *send_buffp, uint32_t time, int mode)
{
    int i = 0;
    txdatabufp = 0;
    uint32_t calcsum = 0;

    uint8_t adnum_flg = 0x55;

    gettimeofday(&u_tv, NULL);
    int64_t t_send_data_start_us = (int64_t)u_tv.tv_sec * 1000000L + u_tv.tv_usec;

    // int ad_sample_num = AD_SAMPLE_NUM;
    // int ad_chirp_mult = AD_CHIRP_MULT;

    if (ad_fs == v53Msps)
    {
        if(ad_sample_num == 1024)       adnum_flg = 0x57;
        else if(ad_sample_num == 512)   adnum_flg = 0x56;
        else if(ad_sample_num == 256)   adnum_flg = 0x55;
        else if(ad_sample_num == 128)   adnum_flg = 0x54;
        else if(ad_sample_num == 64)    adnum_flg = 0x53;
        else                            adnum_flg = 0x52;    
    }
    else
    {
        if (!ad_dfilbyps)
        {
            if(ad_sample_num == 1024)       adnum_flg = 0x67;
            else if(ad_sample_num == 512)   adnum_flg = 0x66;
            else if(ad_sample_num == 256)   adnum_flg = 0x65;
            else if(ad_sample_num == 128)   adnum_flg = 0x64;
            else if(ad_sample_num == 64)    adnum_flg = 0x63;
            else                            adnum_flg = 0x62;    
        }
        else
        {
            if(ad_sample_num == 1024)       adnum_flg = 0x78;
            else if(ad_sample_num == 512)   adnum_flg = 0x77;
            else if(ad_sample_num == 256)   adnum_flg = 0x76;
            else if(ad_sample_num == 128)   adnum_flg = 0x75;
            else if(ad_sample_num == 64)    adnum_flg = 0x74;
            else                            adnum_flg = 0x73;    
        }
    }

    // ヘッダー（5バイト）
    txdatabuf[txdatabufp++] = 0xAA;
    if (exec_mode) {
        txdatabuf[txdatabufp++] = (ad_chirp_mult >> 8) & 0xFF;
        txdatabuf[txdatabufp++] = ad_chirp_mult & 0xFF;
    }
    else {
        txdatabuf[txdatabufp++] = ((ad_chirp_mult * trig_sub_times) >> 8) & 0xFF;
        txdatabuf[txdatabufp++] = (ad_chirp_mult * trig_sub_times) & 0xFF;
    }
    //uint8_t adnum_flag = 0x55;  // samples = 256 のとき
    txdatabuf[txdatabufp++] = adnum_flg;
    txdatabuf[txdatabufp++] = 0x20;  // 16ch

    // Parse input 
    // mipibuf[n] : n-th sample
    if (ad_fs == v53Msps) // RAW12
    {
        // データ本体：RAW12 → 12bitデータへ展開
        // j: number of bytes
        // i: cursor for input byte
        // n: output sample #
        int j = mode;  // modeは6144バイト = (256サンプル × 16ch × 12bit) / 8
        int n = 0;
        uint16_t indata;
        i = 0;
        while (i < j) {
            indata = send_buffp[i++];
            mipibuf[n] = indata << 4;

            indata = send_buffp[i++];
            mipibuf[n + 1] = indata << 4;

            indata = send_buffp[i++];
            mipibuf[n]   |= indata & 0x0F;
            mipibuf[n + 1] |= (indata >> 4);

            indata = send_buffp[i++];
            mipibuf[n + 2] = indata << 4;

            indata = send_buffp[i++];
            mipibuf[n + 3] = indata << 4;

            indata = send_buffp[i++];
            mipibuf[n + 2] |= indata & 0x0F;
            mipibuf[n + 3] |= (indata >> 4);

            n += 4;
        }
    }

    gettimeofday(&u_tv, NULL);
    int64_t t_parse_done_us = (int64_t)u_tv.tv_sec * 1000000L + u_tv.tv_usec;

    // map mipibuf to txdatabufp
    if (ad_fs == v53Msps)
    {
        // 2つの12bitデータを3バイトに圧縮して txdatabuf[] に格納
        // [0] 1[11:4]
        // [1] 1[3:0], 2[11:8]
        // [2] 2[7:0]
        //for (i = 0; i < (ad_sample_num * (ad_chirp_mult*16)); i += 2)
        for (i = 0; i < mipibuf_len; i += 2)
        {
            if (txdatabufp >= txdatabuf_size)
            {
                u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"❌ txdatabuf overflow at sample %d/%d! req/avail %d/%d bytes\n", i, mipibuf_len, txdatabufp, txdatabuf_size);
                mes_send();
                error_flg = 1;
                return;
            }
        
            uint16_t sampleA = mipibuf[i];
            uint16_t sampleB = mipibuf[i + 1];

            uint8_t byte1 = (sampleA >> 4) & 0xFF;
            uint8_t byte2 = ((sampleA & 0x0F) << 4) | ((sampleB >> 8) & 0x0F);
            uint8_t byte3 = sampleB & 0xFF;

            txdatabuf[txdatabufp++] = byte1;
            txdatabuf[txdatabufp++] = byte2;
            txdatabuf[txdatabufp++] = byte3;

            calcsum += byte1 + byte2 + byte3;
        }
    }
    else // 20Msps: [0] [15:8], [1] [7:0]
    {
        uint8_t byte1;
        uint8_t byte2;
        // mode: number of bytes in send_buffp
        for (i = 0; i < mode / 2; i++)
        {
            if (txdatabufp >= txdatabuf_size)
            {
                u_txsize = snprintf((char*)u_txbuf,u_txbufmax,"❌ txdatabuf overflow at sample %d/%d! req/avail %d/%d bytes\n", i, mipibuf_len, txdatabufp, txdatabuf_size);
                mes_send();
                error_flg = 1;
                return;
            }

            // Little Endian
            byte1 = send_buffp[i*2]; // LSB
            byte2 = send_buffp[i*2+1]; // MSB
        
            txdatabuf[txdatabufp++] = byte1;
            txdatabuf[txdatabufp++] = byte2;

            calcsum += byte1 + byte2;
        }

    }

    // トレーラー（9バイト）
    txdatabuf[txdatabufp++] = (time >> 24) & 0xFF;
    txdatabuf[txdatabufp++] = (time >> 16) & 0xFF;
    txdatabuf[txdatabufp++] = (time >> 8) & 0xFF;
    txdatabuf[txdatabufp++] = time & 0xFF;

    txdatabuf[txdatabufp++] = (frame_cntr >> 8) & 0xFF;
    txdatabuf[txdatabufp++] = frame_cntr & 0xFF;

    txdatabuf[txdatabufp++] = (calcsum >> 16) & 0xFF;
    txdatabuf[txdatabufp++] = (calcsum >> 8) & 0xFF;
    txdatabuf[txdatabufp++] = calcsum & 0xFF;

    frame_cntr++;

    // uarttxsize = snprintf((char *)uarttxbuf, uarttxbufmax,
    //                       "\xF0 send_data: time=%lu, mode=%d, txbytes=%d, sum=%lu\n",
    //                       time, mode, txdatabufp, calcsum);
    // u_write_bytes(UART_NUM_0, uarttxbuf, uarttxsize);

 #ifdef U_UARTSEND_EN
    i = 0;
    while (i < txdatabufp) {
        u_write_bytes(UART_NUM_0, &txdatabuf[i], 1);
        i++;
    }
 #endif

 #ifdef U_LAN_EN
    extern httpd_handle_t server;
    extern int client_fd;

    httpd_ws_frame_t frame = {
        .type = HTTPD_WS_TYPE_BINARY,
        .payload = txdatabuf,
        .len = txdatabufp
    };
    gettimeofday(&u_tv, NULL);
    int64_t t_buffer_ready_us = (int64_t)u_tv.tv_sec * 1000000L + u_tv.tv_usec;

    esp_err_t err = httpd_ws_send_frame_async(server, client_fd, &frame);

    gettimeofday(&u_tv, NULL);
    int64_t t_sent_us = (int64_t)u_tv.tv_sec * 1000000L + u_tv.tv_usec;

    if (err != ESP_OK) {
        uarttxsize = snprintf((char *)uarttxbuf, uarttxbufmax,
                              "\xF1 WS送信エラー: %d\n", err);
        u_write_bytes(UART_NUM_0, uarttxbuf, uarttxsize);
        error_flg = 2;
    } else {
        //uarttxsize = snprintf((char *)uarttxbuf, uarttxbufmax,
        //                      "\xF2 WS送信成功: %d バイト\n", txdatabufp);
        //u_write_bytes(UART_NUM_0, uarttxbuf, uarttxsize);
        // uarttxsize = snprintf((char *)uarttxbuf, uarttxbufmax,
        //     "[Info] Parse %.3f ms, Pack %.3f ms, Send %d bytes / %.3f ms (%.3f Mbps)\n", 
        //     (t_parse_done_us - t_send_data_start_us) / 1000.0,
        //     (t_buffer_ready_us - t_parse_done_us) / 1000.0,
        //     txdatabufp, (t_sent_us - t_buffer_ready_us) / 1000.0,
        //     txdatabufp * 8.0 / (t_sent_us - t_buffer_ready_us));
        // u_write_bytes(UART_NUM_0, uarttxbuf, uarttxsize);

    }


 #endif
}

#ifdef U_USB_EN
// Initialise USB CDC-ACM
void usb_init(void)
{
    const tinyusb_config_t usb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,

        .fs_configuration_descriptor = NULL,
        .hs_configuration_descriptor = NULL,
        .qualifier_descriptor = NULL,
    };
    // ESP_ERROR_CHECK(tinyusb_driver_install(&usb_cfg));
    if(tinyusb_driver_install(&usb_cfg) != ESP_OK)
      usb_error = 2; // 2: TinyUSB Install Failed

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 64,
        .callback_rx = &usb_rx_callback, // the first way to register a callback
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };
    // ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
    if(tusb_cdc_acm_init(&acm_cfg) != ESP_OK)
      usb_error = 3; // 3: CDC-ACM Init Failed
}

// USB CDC-ACM Rx Callback
void usb_rx_callback(int itf, cdcacm_event_t *event)
{
    size_t rx_size = 0;
    // Read Rx data
    esp_err_t ret = tinyusb_cdcacm_read(itf, usb_rx_buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
    if (ret == ESP_OK)
    {
        int i;
        // Copy Rx data to rxbuf from address rxbufinp, rolled at rxbufmask
        for(i=0;i<rx_size;i++)
        {
            rxbuf[rxbufinp] = usb_rx_buf[i];
            rxbufinp = (rxbufinp + 1) & rxbufmask;
        }
    }
    else
    {

    }
}
#endif

void mes_send(void)
{
    u_write_bytes(UART_NUM_0,u_txbuf,u_txsize);
}

int u_read_bytes(uart_port_t uart_num, void *buf, uint32_t length, TickType_t ticks_to_wait)
{
    int read_num;
#ifdef U_USB_EN
    read_num = 0;
    uint8_t *buf2 = (uint8_t *)buf;
    if(usb_act)
    {
        bool loop_en = true;
        while(loop_en)
        {
            if (rxbufinp != rxbufoutp)
            {
                *buf2 = rxbuf[rxbufoutp];
                buf2++;
                rxbufoutp = (rxbufoutp + 1) & rxbufmask;
                read_num++;
                if(read_num >= length)
                {
                    loop_en = false;
                }
            }
            else
            {
                loop_en = false;
            }
        }
    }
#else
    read_num = uart_read_bytes(uart_num, buf, length, ticks_to_wait);
#endif
    return read_num;
}

int u_write_bytes(uart_port_t uart_num, const void *src, size_t size)
{
    int write_num;
#ifdef U_USB_EN
    write_num = -1;
    if(usb_act)
    {
        tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, src, size);
        if(tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0,1000) != ESP_OK)
        {
            usb_error = 4; // 4: Send Failed
        }
        else
        {
            // This is required since usb_error == 4 triggers error_flg = 2,
            // and will not cleared as long as usb_error == 4
            usb_error = 0; // 0: No Error
            write_num = size;
        }
    }
#else
    write_num = uart_write_bytes(uart_num, src, size);
#endif
    return write_num;
}
