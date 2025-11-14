#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "chardisp.h"

// Make sure to set these in main.c
extern const int SPI_DISP_SCK; extern const int SPI_DISP_CSn; extern const int SPI_DISP_TX;

/***************************************************************** */

// "chardisp" stands for character display, which can be an LCD or OLED
void init_chardisp_pins() {
    // fill in
    gpio_set_function(SPI_DISP_CSn, GPIO_FUNC_SPI);
    gpio_set_function(SPI_DISP_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SPI_DISP_TX, GPIO_FUNC_SPI);
    spi_init(spi1, 10000);
    spi_set_format(spi1, 9, 0, 0, 1);   
}

void send_spi_cmd(spi_inst_t* spi, uint16_t value) {
    // fill in
    while (spi_is_busy(spi)) {
        continue;
    }
    spi_write16_blocking(spi, &value, 1);
}

void send_spi_data(spi_inst_t* spi, uint16_t value) {
    // fill in
    send_spi_cmd(spi, (value | 0x100));
}

void cd_init() {
    // fill in
    sleep_ms(1);
    //function set
    send_spi_cmd(spi1, 0b1000000000111100);
    sleep_us(40);
    //display on/off
    send_spi_cmd(spi1, 0b1000000000001100);
    sleep_us(40);
    //clear display
    send_spi_cmd(spi1, 0b1000000000000001);
    sleep_ms(2);
    //entry mode set
    send_spi_cmd(spi1, 0b1000000000000110);
    sleep_us(40); 
}

void cd_display1(const char *str) {
    // fill in
    send_spi_cmd(spi1, 0b1000000010000000);
    for(int i = 0; i <= 15; i++)
    {
        send_spi_data(spi1, str[i]);
    }
}
void cd_display2(const char *str) {
    // fill in
    send_spi_cmd(spi1, 0b1000000011000000);
    for(int i = 0; i <= 15; i++)
    {
        send_spi_data(spi1, str[i]);
    }
}

/***************************************************************** */