#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

void send_spi_cmd(spi_inst_t* spi, uint16_t value);
void send_spi_data(spi_inst_t* spi, uint16_t value);
void cd_init();
void cd_display1(const char *str);
void cd_display2(const char *str);
void init_chardisp_pins();

void display_init_bitbang();
void display_bitbang_spi();
void display_init_spi();
void display_init_dma();
void display_print();


// MS56xx SPI commands
#define CMD_RESET     0x1E
#define CMD_ADC_READ  0x00
#define CMD_ADC_CONV  0x40
#define CMD_ADC_D1    0x00
#define CMD_ADC_D2    0x10
#define CMD_ADC_4096  0x08
#define CMD_PROM_RD   0xA0

// SPI configuration
#define SPI_PORT spi0
#define PIN_MISO  32
#define PIN_CS    33
#define PIN_SCK   34
#define PIN_MOSI  35
#define VREF_VOLTS 3.3f
#define UV_GPIO 46          
#define UV_ADC_INPUT 6

const int SPI_DISP_SCK = 10;
const int SPI_DISP_CSn = 9;
const int SPI_DISP_TX = 11;


// --- Helper Functions ---
void cs_low()  { gpio_put(PIN_CS, 0); }
void cs_high() { gpio_put(PIN_CS, 1); }

void spi_send(uint8_t data) {
    spi_write_blocking(SPI_PORT, &data, 1);
}

uint8_t spi_recv() {
    uint8_t rx;
    spi_read_blocking(SPI_PORT, 0x00, &rx, 1);
    return rx;
}

void cmd_reset() {
    cs_low();
    spi_send(CMD_RESET);
    sleep_ms(3);
    cs_high();
}

uint32_t cmd_adc(uint8_t cmd) {
    cs_low();
    spi_send(CMD_ADC_CONV + cmd);
    cs_high();

    sleep_ms(10); // max conversion time for OSR=4096

    cs_low();
    spi_send(CMD_ADC_READ);
    uint8_t b[3];
    for (int i = 0; i < 3; i++) b[i] = spi_recv();
    cs_high();

    return ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
}

uint16_t cmd_prom(uint8_t coef_num) {
    cs_low();
    spi_send(CMD_PROM_RD + coef_num * 2);
    uint8_t b1 = spi_recv();
    uint8_t b2 = spi_recv();
    cs_high();
    return (b1 << 8) | b2;
}

void init_adc() {
adc_init(); 
adc_gpio_init(46); 
adc_select_input(6); 
}

uint16_t read_adc() {
return adc_read();

}

void init_adc_freerun() {
    adc_init();
    adc_gpio_init(46);
    adc_select_input(6);
    adc_run(1);
}


// --- Main ---
int main() {
    stdio_init_all();
    init_adc_freerun();

    // SPI setup
    spi_init(SPI_PORT, 20000 * 1000); // 20 MHz
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    cs_high();
    init_chardisp_pins();
    cd_init();

    printf("MS56xx SPI Demo on RP2350B\n");

    uint16_t C[8];
    cmd_reset();
    sleep_ms(3);

    for (int i = 0; i < 8; i++) {
        C[i] = cmd_prom(i);
    }

    sleep_ms(5);
    for(;;) {
       uint16_t raw = adc_hw->result & 0x0FFF;     
    float volts = (raw * VREF_VOLTS) / 4095.0f;  
    float uvi = volts / 0.1f;                   

    // Terminal print
    printf("Raw: %4u | Voltage: %.3f V | UV Index: %.2f\n", raw, volts, uvi);

    // LCD buffers (16 chars per line)
    char str_buffer1[16];
    char str_buffer2[16];

    // Format first line: Raw and Voltage
    snprintf(str_buffer1, sizeof(str_buffer1), "Raw:%4u %.3fV", raw, volts);
    int len1 = strlen(str_buffer1);
    for(int i = len1; i < 16; i++) str_buffer1[i] = ' ';
    str_buffer1[15] = '\0';

    // Format second line: UV index
    snprintf(str_buffer2, sizeof(str_buffer2), "UV:%.2f", uvi);
    int len2 = strlen(str_buffer2);
    for(int i = len2; i < 16; i++) str_buffer2[i] = ' ';
    str_buffer2[15] = '\0';

    // Send to LCD
    cd_display1(str_buffer1);
    cd_display2(str_buffer2);

    sleep_ms(250);
    }

    while (1) {
        uint32_t D1 = cmd_adc(CMD_ADC_D1 + CMD_ADC_4096);
        uint32_t D2 = cmd_adc(CMD_ADC_D2 + CMD_ADC_4096);

        double dT = D2 - (C[5] * pow(2, 8));
        double OFF = (C[2] * pow(2, 17)) + (dT * C[4]) / pow(2, 6);
        double SENS = (C[1] * pow(2, 16)) + (dT * C[3]) / pow(2, 7);

        double T = (2000 + (dT * C[6]) / pow(2, 23)) / 100.0;
        double P = (((D1 * SENS) / pow(2, 21) - OFF) / pow(2, 15)) / 100.0;

        printf("Temp: %.2f C | Pressure: %.2f mbar\n", T, P);

       char str_buffer1[16];
char str_buffer2[16];

// Convert float to string
snprintf(str_buffer1, sizeof(str_buffer1), "%.2fC", T);
snprintf(str_buffer2, sizeof(str_buffer2), "%.2fmbar", P);

// Pad with spaces to make exactly 16 characters
int len1 = strlen(str_buffer1);
for(int i = len1; i < 16; i++) {
    str_buffer1[i] = ' ';
}
str_buffer1[15] = '\0';  // ensure null termination

int len2 = strlen(str_buffer2);
for(int i = len2; i < 16; i++) {
    str_buffer2[i] = ' ';
}
str_buffer2[15] = '\0';

// Send to display
cd_display1(str_buffer1);
cd_display2(str_buffer2);
sleep_ms(1000);

}
}
