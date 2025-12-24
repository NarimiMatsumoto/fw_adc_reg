
#ifndef INCLUDE_FUNC_COMMON_H
#define INCLUDE_FUNC_COMMON_H

// #define U_UARTSEND_EN
#define U_LAN_EN
#define EXEC_EN
// #define PN9_EN

#define TEST_SW 35
#define TXD 37
#define RXD 38
#define RSTN 33
#define PDN 34
#define EXEC 32
#define TRIG 26
#define SPI_CS0 27
#define SPI_SCLK 30
#define SPI_MOSI 29
#define SPI_MISO 28
#define RF_LDO_E 21
#define DBGMON 36 /* Pulse for debug - currently pulses at TRIG_REG */
#define CTL_PIN TRIG

#define AK5816_CHIP_ID 0x61
#define AK5816_STATE_PDN 0x00
#define AK5816_STATE_LP 0x01
#define AK5816_STATE_STBY 0x03
#define AK5816_STATE_TRX 0x04
#define AK5816_STATE_SLP 0x08
#define AK5816_CORE_CHIP_INFO 0x01 /* Chip ID and Chip Version ID */
#define AK5816_CORE_PAGE_SETTING 0x02 /* Register page setting */
#define AK5816_CORE_BASIC_SETTING 0x03 /* Basic settings */
#define AK5816_CORE_MAGIC 0x04 /* T_MAGIC */
#define AK5816_CORE_SPI_ERR 0x05 /* SPI Error indication */
#define AK5816_CORE_CONT_ERR 0x06 /* Continuous Error indication */
#define AK5816_CORE_ONESHOT_ERR 0x07 /* One-shot Error indication */
#define AK5816_CORE_STATE 0x08 /* State indication */
#define AK5816_CORE_RPUSTATE 0x09 /* RPU State indication */
#define AK5816_PAGE0 0x00
#define AK5816_PAGE1 0x01
#define AK5816_PAGE2 0x02
#define AK5816_PAGE3 0x03
#define AK5816_PAGE4 0x04
#define AK5816_PAGE5 0x05
#define AK5816_PAGE6 0x06
#define AK5816_PAGE7 0x07
#define AK5816_PAGE8 0x08
#define AK5816_PAGE9 0x09
#define AK5816_PAGE10 0x0A
#define AK5816_PAGE11 0x0B
#define AK5816_PAGE12 0x0C
#define AK5816_PAGE13 0x0D

#endif // INCLUDE_FUNC_COMMON_H
