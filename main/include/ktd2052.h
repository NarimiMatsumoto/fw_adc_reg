#ifndef _KTD2052_
#define _KTD2052_

extern i2c_master_dev_handle_t i2c_handle;

typedef enum
{
    Off = 0,
    Night = 1,
    Normal = 2
} KTD2052_EN_MODE;

typedef enum 
{
    vOff = 0,
    v4Pat = 1,
    v6Pat = 2,
    v8Pat = 3
} KTD2052_PG_MODE;

typedef enum 
{
    vFade32ms = 0,
    vFade63ms = 1,
    vFade125ms = 2,
    vFade250ms = 3,
    vFade500ms = 4,
    vFade1000ms = 5,
    vFade2000ms = 6,
    vFade4000ms = 7
} KTD2052_PG_FADE_RATE;

typedef enum 
{
    vSlot188ms = 0,
    vSlot250ms = 1,
    vSlot375ms = 2,
    vSlot500ms = 3,
    vSlot750ms = 4,
    vSlot1000ms = 5,
    vSlot1500ms = 6,
    vSlot2000ms = 7
} KTD2052_PG_TIME;

// Set Global Level
void ktd2052_set_global_level(KTD2052_EN_MODE level);

// Set Colour for each Ch
void ktd2052_set_colour(uint8_t ch, uint32_t __rrggbb);

// Set Fade Rate
void ktd2052_set_fade_rate(KTD2052_PG_FADE_RATE rate0, KTD2052_PG_FADE_RATE rate1);

// Set Pattern Generator Mode (Number of Slots)
void ktd2052_set_slot_count(KTD2052_PG_MODE pg_mode);

// Set Fade Duration for all Time Slots
void ktd2052_set_slot_duration(KTD2052_PG_TIME pg_time);

// Set Time for each Time Slot
void ktd2052_set_slot_fade(uint8_t slot_fade[8]);

// Set On/Off Pattern for each Time Slot
void ktd2052_set_pattern(uint8_t ch, uint8_t slot_pat[8]);

// Set Pattern Timeout Count
void ktd2052_set_timeout(uint8_t time);

#endif // _KTD2052_