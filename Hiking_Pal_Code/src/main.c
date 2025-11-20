#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
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

//I2C
#define I2C_PORT      i2c0
#define I2C_SDA_PIN   40  
#define I2C_SCL_PIN   41
#define GPS_ADDR     0x42  // u-blox SAM-M8Q I2C address

const int SPI_DISP_SCK = 10;
const int SPI_DISP_CSn = 9;
const int SPI_DISP_TX = 11;

// ------------------------------------------------------------
// Utility: Chip Select
// ------------------------------------------------------------
void cs_low()  { gpio_put(PIN_CS, 0); }
void cs_high() { gpio_put(PIN_CS, 1); }

// ------------------------------------------------------------
// GPIO IRQ for mode switching
// ------------------------------------------------------------
 int mode = 0;

void change_mode_isr() {
    gpio_acknowledge_irq(26, GPIO_IRQ_EDGE_RISE);
    mode = (mode + 1) % 3;
}

void gpio_init_irq() {
    gpio_init(26);
   gpio_add_raw_irq_handler_masked((1u << 26), change_mode_isr);
    gpio_set_irq_enabled(26, GPIO_IRQ_EDGE_RISE, true);
    irq_set_enabled(IO_IRQ_BANK0, true);                 
}

// ------------------------------------------------------------
// SPI send/receive
// ------------------------------------------------------------
void spi_send(uint8_t data) {
    spi_write_blocking(SPI_PORT, &data, 1);
}

uint8_t spi_recv() {
    uint8_t rx;
    spi_read_blocking(SPI_PORT, 0x00, &rx, 1);
    return rx;
}

// ------------------------------------------------------------
// Barometer Reset, ADC, and PROM
// ------------------------------------------------------------
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

    sleep_ms(10);                                    // Max conversion time (OSR=4096)

    cs_low();
    spi_send(CMD_ADC_READ);

    uint8_t b[3];
    for (int i = 0; i < 3; i++)
        b[i] = spi_recv();

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

// ------------------------------------------------------------
// SPI Setup
// ------------------------------------------------------------
void spi_setup() {
    spi_init(SPI_PORT, 20 * 1000 * 1000);            // 20 MHz
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
}

// ------------------------------------------------------------
// ADC (UV Sensor)
// ------------------------------------------------------------
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

// ------------------------------------------------------------
// I2C + GPS Support
// ------------------------------------------------------------
void initialize_i2C() {
    i2c_init(I2C_PORT, 1000 * 100);
    i2c_set_slave_mode(I2C_PORT, false, 0);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
}

static double nmea_to_decimal(const char *val, const char hemi) {
    if (val == NULL || strlen(val) < 3)
        return 0.0;

    double raw = atof(val);
    int degrees = (int)(raw / 100);
    double minutes = raw - (degrees * 100);
    double decimal = degrees + (minutes / 60.0);

    if (hemi == 'S' || hemi == 'W')
        decimal = -decimal;

    return decimal;
}

bool gps_read_sentence(char *buffer, int max_len) {
    int idx = 0;
    uint8_t c;

    // Wait for '$'
    while (true) {
        int ret = i2c_read_blocking(i2c_default, GPS_ADDR, &c, 1, false);
        if (ret < 0)
            return false;

        if (c == '$' && idx == 0) {
            buffer[idx++] = '$';
            break;
        }
    }

    // Read until newline
    while (idx < max_len - 1) {
        int ret = i2c_read_blocking(i2c_default, GPS_ADDR, &c, 1, false);
        if (ret < 0)
            return false;

        buffer[idx++] = c;

        if (c == '\n')
            break;
    }

    buffer[idx] = '\0';
    return true;
}

bool gps_parse_lat_lon(const char *sentence, double *lat, double *lon) {
    if (!strstr(sentence, "GGA") && !strstr(sentence, "RMC"))
        return false;

    char copy[120];
    strcpy(copy, sentence);

    char *parts[20];
    int i = 0;
    char *token = strtok(copy, ",");

    while (token != NULL && i < 20) {
        parts[i++] = token;
        token = strtok(NULL, ",");
    }

    const char *lat_str = NULL;
    char lat_hemi = 'N';
    const char *lon_str = NULL;
    char lon_hemi = 'E';

    if (strstr(sentence, "GGA")) {
        if (i < 6)
            return false;
        lat_str = parts[2];
        lat_hemi = parts[3][0];
        lon_str = parts[4];
        lon_hemi = parts[5][0];
    }

    if (strstr(sentence, "RMC")) {
        if (i < 7)
            return false;
        lat_str = parts[3];
        lat_hemi = parts[4][0];
        lon_str = parts[5];
        lon_hemi = parts[6][0];
    }

    *lat = nmea_to_decimal(lat_str, lat_hemi);
    *lon = nmea_to_decimal(lon_str, lon_hemi);

    return true;
}

bool gps_read(double *latitude, double *longitude) {
    char sentence[120];

    while (true) {
        if (!gps_read_sentence(sentence, sizeof(sentence)))
            return false;

        if (gps_parse_lat_lon(sentence, latitude, longitude))
            return true;
    }
}

// ------------------------------------------------------------
// Main Program
// ------------------------------------------------------------
int main() {
    stdio_init_all();
    init_adc_freerun();
    spi_setup();
    initialize_i2C();
    init_chardisp_pins();
    cd_init();
    gpio_init_irq();

    double lat, lon;
    uint16_t C[8];

    // Read barometer PROM coefficients
    cmd_reset();
    sleep_ms(3);

    for (int i = 0; i < 8; i++)
        C[i] = cmd_prom(i);

    sleep_ms(100);

    while (true) {

        // ----------------------------------------------------
        // MODE 0: GPS
        // ----------------------------------------------------
        if (mode == 0) {
            if (gps_read(&lat, &lon)) {

                char str1[16], str2[16];
                snprintf(str1, sizeof(str1), "Lat:% .5f", lat);
                snprintf(str2, sizeof(str2), "Lon:% .5f", lon);

                // Pad
                for (int i = strlen(str1); i < 16; i++) str1[i] = ' ';
                for (int i = strlen(str2); i < 16; i++) str2[i] = ' ';
                str1[15] = '\0';
                str2[15] = '\0';

                cd_display1(str1);
                cd_display2(str2);
                sleep_ms(300);
            }
        }

        // ----------------------------------------------------
        // MODE 1: UV Index
        // ----------------------------------------------------
        else if (mode == 1) {

            uint16_t raw = adc_hw->result & 0x0FFF;
            float volts = (raw * VREF_VOLTS) / 4095.0f;
            float uvi = volts / 0.1f;

            char str1[16], str2[16];
            snprintf(str1, sizeof(str1), "UV: %.1f", uvi);

            for (int i = strlen(str1); i < 16; i++) str1[i] = ' ';
            for (int i = 0; i < 15; i++) str2[i] = ' ';
            str1[15] = '\0';
            str2[15] = '\0';

            cd_display1(str1);
            cd_display2(str2);
            sleep_ms(250);
        }

        // ----------------------------------------------------
        // MODE 2: Barometer (Temp + Pressure)
        // ----------------------------------------------------
        else if (mode == 2) {

            uint32_t D1 = cmd_adc(CMD_ADC_D1 + CMD_ADC_4096);
            uint32_t D2 = cmd_adc(CMD_ADC_D2 + CMD_ADC_4096);

            double dT   = D2 - (C[5] * pow(2, 8));
            double OFF  = (C[2] * pow(2, 17)) + (dT * C[4]) / pow(2, 6);
            double SENS = (C[1] * pow(2, 16)) + (dT * C[3]) / pow(2, 7);

            double T = (2000 + (dT * C[6]) / pow(2, 23)) / 100.0;
            double P = (((D1 * SENS) / pow(2, 21) - OFF) / pow(2, 15)) / 100.0;

            char str1[16], str2[16];
            snprintf(str1, sizeof(str1), "%.2fC", T);
            snprintf(str2, sizeof(str2), "%.2fmbar", P);

            for (int i = strlen(str1); i < 16; i++) str1[i] = ' ';
            for (int i = strlen(str2); i < 16; i++) str2[i] = ' ';
            str1[15] = '\0';
            str2[15] = '\0';

            cd_display1(str1);
            cd_display2(str2);
            sleep_ms(1000);
        }
    }
}

