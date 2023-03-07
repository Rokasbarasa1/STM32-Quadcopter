#pragma once
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
// #include "driver/spi_master.h"
// #include "../util/spi/spi.h"

// Register Map
#define CONFIG          0x00
enum t_config_settings {
    CONFIG_PWR_UP       = 0b00000010,
    CONFIG_PWR_DOWN     = 0b00000000,
    CONFIG_CRC_1byte    = 0b00000000,
    CONFIG_CRC_2byte    = 0b00000100,
    CONFIG_CRC_ENABLE   = 0b00001000,
    CONFIG_CRC_DISABLE  = 0b00000000,
    CONFIG_RX_PRX       = 0b00000001,
    CONFIG_RX_PTX       = 0b00000000,
};


#define EN_AA           0x01
enum t_auto_acknowledgement {
    DISABLE_ACK = 0b00000000,
    ACK_PIPE_0   = 0b00000001,
    ACK_PIPE_1   = 0b00000010,
    ACK_PIPE_2   = 0b00000100,
    ACK_PIPE_3   = 0b00001000,
    ACK_PIPE_4   = 0b00010000,
    ACK_PIPE_5   = 0b00100000,
};

#define EN_RXADDR       0x02
enum t_receive_pipe {
    DISABLE_RC_PIPES   = 0b00000000,
    RC_PIPE_0   = 0b00000001,
    RC_PIPE_1   = 0b00000010,
    RC_PIPE_2   = 0b00000100,
    RC_PIPE_3   = 0b00001000,
    RC_PIPE_4   = 0b00010000,
    RC_PIPE_5   = 0b00100000,
};
#define SETUP_AW        0x03
enum t_address_widths {
    ADDRESS_WIDTH_3_BYTES   = 0b00000001,
    ADDRESS_WIDTH_4_BYTES   = 0b00000010,
    ADDRESS_WIDTH_5_BYTES   = 0b00000011,
};

#define SETUP_RETR      0x04
enum t_auto_retry {
    AUTO_RETRY_250us    = 0b00000000,
    AUTO_RETRY_500us    = 0b00010000,
    AUTO_RETRY_750us    = 0b00100000,
    AUTO_RETRY_1000us   = 0b00110000,
    AUTO_RETRY_1250us   = 0b01000000,
    AUTO_RETRY_1500us   = 0b01010000,
    AUTO_RETRY_1750us   = 0b01100000,
    AUTO_RETRY_2000us   = 0b01110000,
    AUTO_RETRY_2250us   = 0b10000000,
    AUTO_RETRY_2500us   = 0b10010000,
    AUTO_RETRY_2750us   = 0b10100000,
    AUTO_RETRY_3000us   = 0b10110000,
    AUTO_RETRY_3250us   = 0b11000000,
    AUTO_RETRY_3500us   = 0b11010000,
    AUTO_RETRY_3750us   = 0b11100000,
    AUTO_RETRY_4000us   = 0b11110000,

    DISABLED_AUTO_RETRY_COUNT = 0b00000000,
    AUTO_RETRY_COUNT_1   = 0b00000001,
    AUTO_RETRY_COUNT_2   = 0b00000010,
    AUTO_RETRY_COUNT_3   = 0b00000011,
    AUTO_RETRY_COUNT_4   = 0b00000100,
    AUTO_RETRY_COUNT_5   = 0b00000101,
    AUTO_RETRY_COUNT_6   = 0b00000110,
    AUTO_RETRY_COUNT_7   = 0b00000111,
    AUTO_RETRY_COUNT_8   = 0b00001000,
    AUTO_RETRY_COUNT_9   = 0b00001001,
    AUTO_RETRY_COUNT_10  = 0b00001010,
    AUTO_RETRY_COUNT_11  = 0b00001011,
    AUTO_RETRY_COUNT_12  = 0b00001100,
    AUTO_RETRY_COUNT_13  = 0b00001101,
    AUTO_RETRY_COUNT_14  = 0b00001110,
    AUTO_RETRY_COUNT_15  = 0b00001111,
};

#define RF_CH           0x05
enum t_receive_channel {
    EMPTY_RECEIVE_CHANNEL = 0b00000000,
};

#define RF_SETUP        0x06
enum t_rf_setup {
    RF_PWR_N_18_DBM = 0b00000000,
    RF_PWR_N_12_DBM = 0b00000010,
    RF_PWR_N_6_DBM  = 0b00000100,
    RF_PWR_0_DBM    = 0b00000110,
    
    RF_CONTINUOUS_WAVE = 0b10000000,
    RF_DATA_RATE_250_KBPS = 0b00100000,
    RF_DATA_RATE_1_MBPS   = 0b00000000,
    RF_DATA_RATE_2_MBPS   = 0b00001000,
};

#define STATUS          0x07
#define OBSERVE_TX      0x08
#define CD              0x09
#define RX_ADDR_P0      0x0A
#define RX_ADDR_P1      0x0B
#define RX_ADDR_P2      0x0C
#define RX_ADDR_P3      0x0D
#define RX_ADDR_P4      0x0E
#define RX_ADDR_P5      0x0F
#define TX_ADDR         0x10
#define RX_PW_P0        0x11
#define RX_PW_P1        0x12
#define RX_PW_P2        0x13
#define RX_PW_P3        0x14
#define RX_PW_P4        0x15
#define RX_PW_P5        0x16
#define FIFO_STATUS     0x17
#define DYNPD           0x1C
#define FEATURE         0x1D

// Instruction commands (the special ones it has)
// you send them and thats it 
#define R_REGISTER      0x00
#define W_REGISTER      0x20
#define REGISTER_MASK   0x1F    
#define ACTIVATE        0x50
#define R_RX_PL_WID     0x60
#define R_RX_PAYLOAD    0x61
#define W_TX_PAYLOAD    0xA0
#define W_ACK_PAYLOAD   0xA8
#define FLUSH_TX        0xE1
#define FLUSH_RX        0xE2
#define REUSE_TX_PL     0xE3
#define NOP             0xFF

uint8_t init_nrf24(SPI_HandleTypeDef * spi_port);
void nrf24_tx_mode (uint8_t *address, uint8_t channel);
void nrf24_rx_mode(uint8_t *address, uint8_t channel);
uint8_t nrf24_data_available(int pipe_number);
uint8_t nrf24_transmit (uint8_t *data);
void nrf24_receive(uint8_t *data);
void nrf24_read_all (uint8_t *data);