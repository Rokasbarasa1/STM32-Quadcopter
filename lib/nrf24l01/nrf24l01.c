#include "./nrf24l01.h"

// This video was used to help make this:
// https://www.youtube.com/watch?v=X5XDSWQYYvU&t=784s
#define SPIBUS_READ     (0b00000000)
#define SPIBUS_WRITE    (0b00100000)

#define CONFIG          0x00
#define EN_AA           0x01
#define EN_RXADDR       0x02
#define SETUP_AW        0x03
#define SETUP_RETR      0x04
#define RF_CH           0x05
#define RF_SETUP        0x06
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

static SPI_HandleTypeDef *  device_handle;

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

// Slave deselect equivalent
static void cs_deselect(){
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
}

// Slave select equivalent
static void cs_select(){
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
}

// Ce pin is mainly used to change settings. If it is enabled you 
// cant change them(cant write to the registers using spi)
static void ce_enable(){
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}

static void ce_disable(){
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}

// Spi write one byte to specific register
static void write_register(uint8_t reg, uint8_t data) {
    // The nrf24 uses a specific bit that is added to the register address to 
    uint8_t reg_temp = reg;
    reg_temp |= SPIBUS_WRITE;
    reg_temp = (reg_temp & 0x7f);
    uint8_t buf[1];
    buf[0] = data;

    HAL_StatusTypeDef ret;
    cs_select();
    HAL_SPI_Transmit(device_handle, &reg_temp, 1, 5000);
    HAL_SPI_Transmit(device_handle, buf, 1, 5000);
    cs_deselect();
}

// Read one register and return it from function
static uint8_t read_register(uint8_t reg) {
    uint8_t buf[1] = { 0 };

    cs_select();
    HAL_SPI_Transmit(device_handle, &reg, 1, 5000);
    HAL_SPI_Receive(device_handle, buf, 1, 5000);
    cs_deselect();

    return buf[0];
}

// write multiple bytes to register
static void write_register_multiple(uint8_t reg, uint8_t * data, uint8_t size, uint8_t command) {
    if(!command){
        reg |= SPIBUS_WRITE;
        reg = (reg & 0x7f);
    }
    cs_select();
    HAL_SPI_Transmit(device_handle, &reg, 1, 5000);
    HAL_SPI_Transmit(device_handle, data, size, 5000);
    cs_deselect();
}

// send a specific command. Basically write and ignore response
static void send_command(uint8_t command){
    cs_select();
    HAL_SPI_Transmit(device_handle, &command, 1, 5000);
    cs_deselect();

}

// read multiple registers from specified registers
static void read_register_multiple(uint8_t reg, uint8_t *buf, uint16_t len) {
    cs_select();
    HAL_SPI_Transmit(device_handle, &reg, 1, 5000);
    HAL_SPI_Receive(device_handle, buf, len, 5000);
    cs_deselect();
}

// Test to see if spi setup is working for nrf24
uint8_t nrf24_test(){
    write_register(CONFIG, CONFIG_PWR_UP);
    write_register(RF_CH, 0b00000000);  // Clear this register
        
    uint8_t data[1] = {0};
    read_register_multiple(RF_CH, data, 1);

    if(data[0] != 0b00000000){ // check that it is empty
        return 0;
    }

    write_register(RF_CH, 0b01010101); // write to it 

    uint8_t data1[1] = {0};
    read_register_multiple(RF_CH, data1, 1);

    if(data1[0] != 0b01010101){ // check that it is the same value as written
        return 0;
    }

    return 1;
}

// reset some registers on the nrf24
void nrf24_reset(uint8_t REG){
	if (REG == STATUS){
		write_register(STATUS, 0x00);
	}

	else if (REG == FIFO_STATUS){
		write_register(FIFO_STATUS, 0x11);
	}else {
        write_register(CONFIG, 0x08);
        write_register(EN_AA, 0x3F);
        write_register(EN_RXADDR, 0x03);
        write_register(SETUP_AW, 0x03);
        write_register(SETUP_RETR, 0x03);
        write_register(RF_CH, 0x02);
        write_register(RF_SETUP, 0x0E);
        write_register(STATUS, 0x00);
        write_register(OBSERVE_TX, 0x00);
        write_register(CD, 0x00);
        uint8_t rx_addr_p0_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
        write_register_multiple(RX_ADDR_P0, rx_addr_p0_def, 5, 0);
        uint8_t rx_addr_p1_def[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
        write_register_multiple(RX_ADDR_P1, rx_addr_p1_def, 5, 0);
        write_register(RX_ADDR_P2, 0xC3);
        write_register(RX_ADDR_P3, 0xC4);
        write_register(RX_ADDR_P4, 0xC5);
        write_register(RX_ADDR_P5, 0xC6);
        uint8_t tx_addr_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
        write_register_multiple(TX_ADDR, tx_addr_def, 5, 0);
        write_register(RX_PW_P0, 0);
        write_register(RX_PW_P1, 0);
        write_register(RX_PW_P2, 0);
        write_register(RX_PW_P3, 0);
        write_register(RX_PW_P4, 0);
        write_register(RX_PW_P5, 0);
        write_register(FIFO_STATUS, 0x11);
        write_register(DYNPD, 0);
        write_register(FEATURE, 0);
	}
}

// Set nrf24 to transmit mode
void nrf24_tx_mode(uint8_t *address, uint8_t channel){
	// disable the chip before configuring the device
	ce_disable();

	write_register(RF_CH, channel);  // select the channel

	write_register_multiple(TX_ADDR, address, 5, 0);  // Write the TX address

    uint8_t data[5] = {0,0,0,0,0};
    read_register_multiple(TX_ADDR, data, 5);

    printf("TX_ADDR "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(data[0]));
    printf("TX_ADDR "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(data[1]));
    printf("TX_ADDR "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(data[2]));
    printf("TX_ADDR "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(data[3]));
    printf("TX_ADDR "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(data[4]));

	uint8_t config = read_register(CONFIG);
    config |= CONFIG_PWR_UP; // power up 
    config |= CONFIG_RX_PTX; // set mode transmit
	write_register(CONFIG, config);

	// Enable the chip after configuring the device
	ce_enable();
}

// set nrf24 to receive mode
void nrf24_rx_mode(uint8_t *address, uint8_t channel){
	// disable the chip before configuring the device
	ce_disable();

	nrf24_reset(STATUS);

	write_register(RF_CH, channel);  // select the channel

    // setup pipe 1
    uint8_t en_rxaddr = read_register(EN_RXADDR);
	en_rxaddr = en_rxaddr | (1<<1);
	write_register(EN_RXADDR, en_rxaddr);
    write_register_multiple(RX_ADDR_P1, address, 5, 0);
    write_register(RX_PW_P1, 32);
    
	uint8_t config = read_register(CONFIG);
    config |= CONFIG_PWR_UP; // power up 
    config |= CONFIG_RX_PRX; // set mode receive
	write_register(CONFIG, config);

	ce_enable();
}

// perform the transmission with specified data
uint8_t nrf24_transmit(char *data){
    write_register_multiple(W_TX_PAYLOAD, data, 32, 1);
	// HAL_Delay(1);

	uint8_t fifo_status = read_register(FIFO_STATUS);

	// check the fourth bit of FIFO_STATUS to know if the TX fifo is empty
	if ((fifo_status&(1<<4)) && (!(fifo_status&(1<<3)))){
		send_command(FLUSH_TX);
		nrf24_reset(FIFO_STATUS);// reset FIFO_STATUS
		return 1;
	}

	return 0;
}
// change to pipe 2
// dont look at code anymore look at why connection not working

// Check if data is available on the specified pipe
uint8_t nrf24_data_available(int pipe_number){
	uint8_t status = read_register(STATUS);

	if ((status&(1<<6))&&(status&(pipe_number<<1))){
		write_register(STATUS, (1<<6));
		return 1;
	}

	return 0;
}

// receive the data form the nrf24 into the specified array
void nrf24_receive(char *data){
	// payload command
    read_register_multiple(R_RX_PAYLOAD, data, 32);

	send_command(FLUSH_RX);
}

// read all the registers on the nrf24
void nrf24_read_all(uint8_t *data){
	for (int i=0; i<10; i++){
		*(data+i) = read_register(i);
	}

	read_register_multiple(RX_ADDR_P0, (data+10), 5);
	read_register_multiple(RX_ADDR_P1, (data+15), 5);

	*(data+20) = read_register(RX_ADDR_P2);
	*(data+21) = read_register(RX_ADDR_P3);
	*(data+22) = read_register(RX_ADDR_P4);
	*(data+23) = read_register(RX_ADDR_P5);

	read_register_multiple(RX_ADDR_P0, (data+24), 5);

	for (int i=29; i<38; i++){
		*(data+i) = read_register(i-12);
	}
}

uint8_t init_nrf24(SPI_HandleTypeDef * spi_port){

    device_handle = spi_port;

    cs_deselect();
    ce_disable(); // disable to start changing registers on nrf24

    if(!nrf24_test()){ // test if spi works
        printf("NRF24 not initialized\n");
        return 0;
    }

    nrf24_reset(0);
    
    uint8_t data = 0b00000000;


    data |= CONFIG_PWR_DOWN; // Restart it 
    write_register(CONFIG, data);

    data = DISABLE_ACK; // No ack
    write_register(EN_AA, data);

    data = DISABLE_RC_PIPES; // No receiving
    write_register(EN_RXADDR, data);

    data = ADDRESS_WIDTH_5_BYTES; // 5 bytes rx/tx address field
    write_register(SETUP_AW, data); 
    
    data = DISABLED_AUTO_RETRY_COUNT; // Disabled because ACK not used
    write_register(SETUP_RETR, data); 

    data = EMPTY_RECEIVE_CHANNEL; // Receive not listening to any channel
    write_register(RF_CH, data);

    data = 0b00000000;
    data |= RF_DATA_RATE_2_MBPS; // 2Mbps 
    data |= RF_PWR_0_DBM; // 0 dBm power, highest
    write_register(RF_SETUP, data); // 0db power and data rate 2Mbps

    ce_enable();
    printf("NRF24 initialized\n");
    return 1;
}

//Examples

// If both receive and transmit are on the same address and pipe 1 then this should work 

// Receiver

// uint8_t address[5] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
// uint8_t rx_data[32];
// bool nrf24_setup = nrf24_init(SPI3_HOST, 17, 16);
// nrf24_rx_mode(address, 10);

// printf("Transmitting: ");
// if(nrf24_transmit(tx_data)){
//     printf("TX success\n");
// }else{
//     printf("TX failed\n");
// }


// Transmitter

// uint8_t address[5] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
// uint8_t tx_data[] = "hello world!\n";
// bool nrf24_setup = nrf24_init(SPI3_HOST, 17, 16);
// nrf24_tx_mode(address, 10);

// printf("Receiving: ");
// if(nrf24_data_available(1)){
//     nrf24_receive(rx_data);
//     for(uint8_t i = 0; i < strlen((char*) rx_data); i++ ){
//         printf("%c", ((char*) rx_data)[i]);
//     }
//     printf("\n");

// }else{
//     printf("no data\n");
// }