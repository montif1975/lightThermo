#include <dht.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "pico/binary_info.h"

#include "include/sh1106_i2c.h"

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT_OLED           i2c0
#define I2C_SDA_OLED            8
#define I2C_SCL_OLED            9

#define I2C_PORT_SENS           i2c1
#define I2C_SDA_SENS            14
#define I2C_SCL_SENS            15

#define PRG_VERSION             "0.0.1"
#define SENS_TYPE               "DHT20"

#if 0
#include "blink.pio.h"

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
}
#endif

static float celsius_to_fahrenheit(float temperature) {
    return temperature * (9.0f / 5) + 32;
}

void SH1106_setup_display_layout(uint8_t *buf, int fb_size)
{
    int i;

    // build the entire display layout (status bar + info area + extra info area)
    // __________________
    // |_____|_____|____|   2 pages - Status Bar (status + ID + Connection state)
    // |                |   4 pages - Info Area
    // |________________|
    // |________________|   2 pages - extra info area
    //  
    memset(buf, 0, fb_size);
    // build vertical line between status and ID field
    buf[48] = 0xFF; 
    buf[96] = 0xFF;  
    // build orizontal line between Status bar and Info area
    for (i=128; i<256; i++)
        buf[i] = 0x80;
    buf[176] = 0xFF;
    buf[224] = 0xFF;
    // build orizontal line between Info area and extra info bar
    for (i=768; i<896; i++)
        buf[i] = 0x01;

    return;
}

void show_boot_info(uint8_t *buf,int fb_size)
{
    char boot_string[16];

    memset(buf, 0, fb_size);
    memset(boot_string,0,sizeof(boot_string));
    sprintf(boot_string,"Vers %s",PRG_VERSION);
    SH1106_write_string(buf,8,24,boot_string,FONT_HIGH_8,FONT_HIGH_8);
    memset(boot_string,0,sizeof(boot_string));
    sprintf(boot_string,"Sens %s",SENS_TYPE);
    SH1106_write_string(buf,8,40,boot_string,FONT_HIGH_8,FONT_HIGH_8);

}

int main()
{
    uint8_t fb[SH1106_BUF_LEN];

    stdio_init_all();

    // wait to give time to hardware to be ready
    sleep_ms(2000);

#if !defined(I2C_PORT_OLED) || !defined(I2C_SDA_OLED) || !defined(I2C_SCL_OLED)
#warning i2c / SH1106_i2c example requires a board with I2C pins
    puts("Define I2C pins were not defined");
#else

    // useful information for picotool
    bi_decl(bi_2pins_with_func(I2C_SDA_OLED, I2C_SCL_OLED, GPIO_FUNC_I2C));
    bi_decl(bi_program_description("SH1106 OLED driver I2C example for the Raspberry Pi Pico"));

    printf("Hello, SH1106 OLED display! Look at my raspberries..\n");

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

    // Initialize render area for entire frame (SH1106_WIDTH pixels by SH1106_NUM_PAGES pages)
#if 0
    struct render_area frame_area = {
        start_col: 0,
        end_col : SH1106_WIDTH - 1,
        start_page : 0,
        end_page : SH1106_NUM_PAGES - 1
        };

    calc_render_area_buflen(&frame_area);
#endif

    // splash screen at boot
    show_boot_info(fb,SH1106_BUF_LEN);
    SH1106_full_render(fb);
    sleep_ms(2000);

    SH1106_setup_display_layout(fb,SH1106_BUF_LEN);
    SH1106_full_render(fb);

    SH1106_send_cmd(SH1106_SET_ENTIRE_ON); // go back to following RAM for pixel state

    // only for test
    char *status_string = "OK";
    char *id_string = "Sala";
    char *connection_string ="No";
#if 0
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


    // Sensor init (da sistemare)
    #define DHT20_I2C_ADDRESS       0x38
    #define DHT20_STATUS_OK         0x18
    #define DHT20_READ              0x01
    #define DHT20_WRITE             0x00
    #define DHT20_START_MEASURE     0xAC
    int temperature = 0;
    int humidity = 0;
    float temp, hum;
    uint8_t buf[6] = {DHT20_READ,0,0,0,0,0};
    char appo_string[10];

    i2c_write_blocking(I2C_PORT_SENS,DHT20_I2C_ADDRESS,buf,1,false);
    memset(buf,0x0,sizeof(buf));
    sleep_ms(100);
    i2c_read_blocking(I2C_PORT_SENS,DHT20_I2C_ADDRESS,buf,1,false);
    printf("--> from sensor: buf[0] = %02x\n",buf[0]);
    if(buf[0] == DHT20_STATUS_OK)
    {
        printf("Sensor DHT20 calibrated and ready.\n");
        uint8_t loop;
        do {
            sleep_ms(1000);
            memset(buf,0x0,sizeof(buf));
            buf[0] = DHT20_START_MEASURE;
            buf[1] = 0x33;
            buf[2] = 0x00;
            i2c_write_blocking(I2C_PORT_SENS,DHT20_I2C_ADDRESS,buf,3,false);
            // modificare e leggere direttamente 6 bytes con il primo che e' lo status
            loop = 5;
            do {
                memset(buf,0x0,sizeof(buf));
                sleep_ms(80);
                i2c_read_blocking(I2C_PORT_SENS,DHT20_I2C_ADDRESS,buf,6,false);
                printf("--> loop = %d, from sensor: buf[0] = %02x\n",loop,buf[0]);
                loop--;
            } while (((buf[0] & 0x80) == 0x80) && (loop > 0));
            // measure is ready
            i2c_read_blocking(I2C_PORT_SENS,DHT20_I2C_ADDRESS,buf,6,false);
            printf("--> from sensor: buf[] = %02x,%02x,%02x,%02x,%02x\n",buf[1],buf[2],buf[3],buf[4],buf[5]);
            humidity = (buf[1]<<8) | ((buf[2]));
            humidity = ((humidity << 4) | ((buf[3] & 0xF0)>>4));
            printf("humidity = 0x%X\n",humidity);
            temperature = ((buf[3] & 0x0F)<< 16) | (buf[4]<<8) | buf[5];
            printf("temperature = 0x%X\n",temperature);
            hum = ((float)humidity / 1048576)*100;
            temp = (((float)temperature / 1048576)*200)-50;
            printf("*** TEMPERATURE = %.2f C\n",temp);
            printf("*** HUMIDITY = %.2f %%RH\n",hum);
            memset(appo_string,0,sizeof(appo_string));
            sprintf(appo_string,"%.2f",temp);
            SH1106_display_temperature(fb,appo_string,12,FONT_HIGH_16);
            memset(appo_string,0,sizeof(appo_string));
            sprintf(appo_string,"%.2f",hum);
            SH1106_display_humidity(fb,appo_string,12,FONT_HIGH_16);
            SH1106_full_render(fb);
        } while(true);
    }
    else
    {
        printf("Sensor DHT20 NOT calibrated!\n");
        /* todo */
        do
        {
            sleep_ms(1000);
            printf("Need calibration... :-(\n");
        } while (true);
        
    }

#if 0
    // intro sequence: flash the screen 3 times
    for (int i = 0; i < 3; i++) {
        SH1106_send_cmd(SH1106_SET_ALL_ON);    // Set all pixels on
        sleep_ms(500);
        SH1106_send_cmd(SH1106_SET_ENTIRE_ON); // go back to following RAM for pixel state
        sleep_ms(500);
    }
    // render 3 cute little raspberries
    struct render_area area = {
        start_page : 0,
        end_page : (IMG_HEIGHT / SH1106_PAGE_HEIGHT)  - 1
    };

restart:

    area.start_col = 0;
    area.end_col = IMG_WIDTH - 1;

    calc_render_area_buflen(&area);

    uint8_t offset = 5 + IMG_WIDTH; // 5px padding

    for (int i = 0; i < 3; i++) {
        render(raspberry26x32, &area);
        area.start_col += offset;
        area.end_col += offset;
    }

    SH1106_scroll(true);
    sleep_ms(5000);
    SH1106_scroll(false);

    char *text[] = {
        "A long time ago",
        "  on an OLED ",
        "   display",
        " far far away",
        "Lived a small",
        "red raspberry",
        "by the name of",
        "    PICO"
    };

    int y = 0;
    for (uint i = 0 ;i < count_of(text); i++) {
        WriteString(buf, 5, y, text[i]);
        y+=8;
    }
    render(buf, &frame_area);

    // Test the display invert function
    sleep_ms(3000);
    SH1106_send_cmd(SH1106_SET_INV_DISP);
    sleep_ms(3000);
    SH1106_send_cmd(SH1106_SET_NORM_DISP);

    bool pix = true;
    for (int i = 0; i < 2;i++) {
        for (int x = 0;x < SH1106_WIDTH;x++) {
            DrawLine(buf, x, 0,  SH1106_WIDTH - 1 - x, SH1106_HEIGHT - 1, pix);
            render(buf, &frame_area);
        }

        for (int y = SH1106_HEIGHT-1; y >= 0 ;y--) {
            DrawLine(buf, 0, y, SH1106_WIDTH - 1, SH1106_HEIGHT - 1 - y, pix);
            render(buf, &frame_area);
        }
        pix = false;
    }

    goto restart;
#endif // if 0

#endif

#if 0
    // PIO Blinking example
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &blink_program);
    printf("Loaded program at %d\n", offset);
    
    #ifdef PICO_DEFAULT_LED_PIN
    blink_pin_forever(pio, 0, offset, PICO_DEFAULT_LED_PIN, 3);
    #else
    blink_pin_forever(pio, 0, offset, 6, 3);
    #endif
    // For more pio examples see https://github.com/raspberrypi/pico-examples/tree/master/pio

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
    }

    dht_t dht;
    dht_init(&dht, DHT_MODEL, pio0, DATA_PIN, false /* pull_up */);
    do {
        if(dht_start_measurement(&dht) == 0)
            printf("DHT measure start...");
        
        float humidity;
        float temperature_c;
        dht_result_t result = dht_finish_measurement_blocking(&dht, &humidity, &temperature_c);
        if (result == DHT_RESULT_OK) {
            printf("%.1f C (%.1f F), %.1f%% humidity\n", temperature_c, celsius_to_fahrenheit(temperature_c), humidity);
        } else if (result == DHT_RESULT_TIMEOUT) {
            puts("DHT sensor not responding. Please check your wiring.");
        } else {
            assert(result == DHT_RESULT_BAD_CHECKSUM);
            puts("Bad checksum");
        }

        sleep_ms(2000);
    } while (true);
#endif

}
