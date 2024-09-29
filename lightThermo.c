#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "pico/binary_info.h"

#include "include/sh1106_i2c.h"
#include "include/dht20.h"

#define PRG_VERSION             "001"
#define SENS_TYPE               "DHT20"

/*
 * Function: C2F
 * Convert Celsius to Fahrenheit scale
*/
static float C2F(float temperature) {
    return temperature * (9.0f / 5) + 32;
}

/*
 * main()
*/
int main(int argc, char *argv[])
{
    int ret = 0;
    uint8_t fb[SH1106_BUF_LEN];
    float temperature, humidity;
    uint8_t temp_format = TEMP_FORMAT_CELSIUS;
    char appo_string[10];
    // only for test
    char *status_string = "OK";
    char *id_string = "Sala";
    char *connection_string ="No";


    stdio_init_all();

    // wait to give time to hardware to be ready
    sleep_ms(2000);

#if !defined(I2C_PORT_OLED) || !defined(I2C_PORT_SENS)
#warning this programs requires a board with I2C pins
    puts("Define I2C pins were not defined");
#else

    // useful information for picotool
    bi_decl(bi_2pins_with_func(I2C_SDA_OLED, I2C_SCL_OLED, GPIO_FUNC_I2C));
    bi_decl(bi_program_description("SH1106 OLED driver I2C example for the Raspberry Pi Pico"));

    printf("Hello, SH1106 OLED display! Look at my board..\n");

    // I2C for OLED display
    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT_OLED, 400*1000);
    
    gpio_set_function(I2C_SDA_OLED, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_OLED, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_OLED);
    gpio_pull_up(I2C_SCL_OLED);

    // I2C for Sensor (Temp, humidity,etc...)
    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT_SENS, 400*1000);
    
    gpio_set_function(I2C_SDA_SENS, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_SENS, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_SENS);
    gpio_pull_up(I2C_SCL_SENS);
    // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c

    // run through the complete initialization process
    // OLED display init
    SH1106_init();

    // splash screen at boot
    SH1106_show_boot_info(fb,SH1106_BUF_LEN,PRG_VERSION,SENS_TYPE);
    SH1106_full_render(fb);
    sleep_ms(2000);

    SH1106_setup_display_layout(fb,SH1106_BUF_LEN);
    SH1106_full_render(fb);

    SH1106_send_cmd(SH1106_SET_ENTIRE_ON); // go back to following RAM for pixel state

#ifdef DEBUG_FONTS
    char *test_string = "0123456789";
    char *test_string2 = "23.95";

    WriteString(fb, 2, 0, status_string);
    WriteString(fb, 50, 0, id_string);
    WriteString(fb, 98, 0, connection_string);
#endif

    SH1106_write_string(fb,2,0,status_string,8,8);
    SH1106_write_string(fb,50,0,id_string,8,8);
    SH1106_write_string(fb,98,0,connection_string,8,8);
    SH1106_full_render(fb);


    // Sensor init
    ret = DHT20_init();
    if(ret == 0)
    {
        // init OK, go to mainloop
        do
        {
            sleep_ms(1000); // TODO cambiare con gestione timer
            temperature = 0;
            humidity = 0;
            if(DHT20_read_data(&temperature,&humidity) == 0)
            {
                memset(appo_string,0,sizeof(appo_string));
                if(temp_format == TEMP_FORMAT_CELSIUS)
                    sprintf(appo_string,"%.2f",temperature);
                else
                    sprintf(appo_string,"%.2f",C2F(temperature));
                SH1106_display_temperature(fb,appo_string,12,FONT_HIGH_16);
                memset(appo_string,0,sizeof(appo_string));
                sprintf(appo_string,"%.2f",humidity);
                SH1106_display_humidity(fb,appo_string,12,FONT_HIGH_16);
                SH1106_full_render(fb);
            }
        } while (true);
    }
    else
    {
        // init KO, sensor need to be initialized
        // TODO
        do
        {
            sleep_ms(1000);
            printf("Need calibration... :-(\n");
        } while (true);
    }
#endif // !defined(I2C_PORT_OLED) || !defined(I2C_PORT_SENS)

    return ret;
}
