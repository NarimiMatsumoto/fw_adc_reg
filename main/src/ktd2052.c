#include <inttypes.h>
#include "driver/i2c_master.h"
#include "ktd2052.h"

// Set Colour for each Ch
void ktd2052_set_colour(uint8_t ch, uint32_t __rrggbb)
{
    uint8_t buf2b[2] = {0x00, 0x00}; 
    if (ch == 1 || ch == 2 || ch == 3 || ch == 4)
    {
        buf2b[0] = ch * 3;
        buf2b[1] = (0xFF0000 & __rrggbb) >> 16; // Red
        ESP_ERROR_CHECK(i2c_master_transmit(i2c_handle, buf2b, sizeof(buf2b), -1));

        buf2b[0] = ch * 3 + 1;
        buf2b[1] = (0xFF00 & __rrggbb) >> 8; // Green
        ESP_ERROR_CHECK(i2c_master_transmit(i2c_handle, buf2b, sizeof(buf2b), -1));

        buf2b[0] = ch * 3 + 2;
        buf2b[1] = 0xFF & __rrggbb; // Blue
        ESP_ERROR_CHECK(i2c_master_transmit(i2c_handle, buf2b, sizeof(buf2b), -1));
    }
}

// level 0: all off
//       1: night mode (1.5 mA)
//       2: normal mode (24 mA)
void ktd2052_set_global_level(KTD2052_EN_MODE level)
{
    uint8_t buf2b[2]; 
    uint8_t buffer[2];

    buf2b[0] = 0x02;
    buf2b[1] = 0x00;

    // Read current register address 0x02
    ESP_ERROR_CHECK(i2c_master_transmit_receive(i2c_handle, buf2b, 1, buffer, 1, -1));

    // replace to LLxx_xxxx
    buf2b[1] = (buffer[0] & 0x3F) + ((level & 0x3) << 6);

    // write
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_handle, buf2b, sizeof(buf2b), -1));
}

// Set Fade Rate
void ktd2052_set_fade_rate(KTD2052_PG_FADE_RATE rate0, KTD2052_PG_FADE_RATE rate1)
{
    uint8_t buf2b[2]; 
    uint8_t buffer[2];

    // Write rate0
    // Read current register address 0x02
    buf2b[0] = 0x02;
    buf2b[1] = 0x00;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(i2c_handle, buf2b, 1, buffer, 1, -1));

    // replace to xxxx_xRRR
    buf2b[1] = (buffer[0] & 0xF8) + (rate0 & 0x7);

    // write
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_handle, buf2b, sizeof(buf2b), -1));


    // Write rate1
    // Read current register address 0x0F
    buf2b[0] = 0x0F;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(i2c_handle, buf2b, 1, buffer, 1, -1));

    // replace to xxxx_xRRR
    buf2b[1] = (buffer[0] & 0xF8) + (rate1 & 0x7);

    // write
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_handle, buf2b, sizeof(buf2b), -1));
}

// Set Pattern Generator Mode (Number of Slots)
void ktd2052_set_slot_count(KTD2052_PG_MODE pg_mode)
{
    uint8_t buf2b[2]; 
    uint8_t buffer[2];

    // Read current register address 0x0F
    buf2b[0] = 0x0F;
    buf2b[1] = 0x00;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(i2c_handle, buf2b, 1, buffer, 1, -1));

    // replace to MMxx_xxxx
    buf2b[1] = (buffer[0] & 0x3F) + ((pg_mode & 0x3) << 6);

    // write
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_handle, buf2b, sizeof(buf2b), -1));

}


// Set Fade Duration for all Time Slots
void ktd2052_set_slot_duration(KTD2052_PG_TIME pg_time)
{
    uint8_t buf2b[2]; 
    uint8_t buffer[2];

    // Read current register address 0x0F
    buf2b[0] = 0x0F;
    buf2b[1] = 0x00;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(i2c_handle, buf2b, 1, buffer, 1, -1));

    // replace to xxTT_Txxx
    buf2b[1] = (buffer[0] & 0xC7) + ((pg_time & 0x7) << 3);

    // write
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_handle, buf2b, sizeof(buf2b), -1));

}

// Set Fade for each Time Slot
void ktd2052_set_slot_fade(uint8_t slot_fade[8])
{
    uint8_t buf2b[2]; 

    buf2b[0] = 0x10;
    buf2b[1] = 0x00;

    for (int i=0; i<8; i++)
    {
        buf2b[1] += (slot_fade[i] & 0x1) << i;
    }

    // write
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_handle, buf2b, sizeof(buf2b), -1));
}

// Set On/Off Pattern for each Time Slot
void ktd2052_set_pattern(uint8_t ch, uint8_t slot_pat[8])
{
    uint8_t buf2b[2] = {0x00, 0x00}; 
    if (ch == 1 || ch == 2 || ch == 3 || ch == 4)
    {
        buf2b[0] = ch + 0x10;
        for (int i=0; i<8; i++)
        {
            buf2b[1] += (slot_pat[i] & 0x1) << i;
        }

        ESP_ERROR_CHECK(i2c_master_transmit(i2c_handle, buf2b, sizeof(buf2b), -1));
    }
}

// Set Pattern Timeout Count
void ktd2052_set_timeout(uint8_t time)
{
    uint8_t buf2b[2]; 

    buf2b[0] = 0x15;
    buf2b[1] = time;

    // write
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_handle, buf2b, sizeof(buf2b), -1));
}
