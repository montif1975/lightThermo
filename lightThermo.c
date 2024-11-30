/**
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS
 * AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *  
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/watchdog.h"
#include "pico/binary_info.h"

#include "include/general.h"
#include "include/sh1106_i2c.h"
#include "include/dht20.h"
#include "include/lightThermo.h"


/*
 * Global variables
*/
char rx_commands[NMAX_CMD_STRING];
bool cmd_received;
bool button_pushed;

/*
 * Global functions
*/

/*
 * Function: C2F(temperature)
 * Convert Celsius to Fahrenheit scale
*/
static float C2F(float temperature) {
    return temperature * (9.0f / 5) + 32;
}

/*
 * Function: uart_rx_callback() 
 */
void uart_rx_callback(void)
{
    uint8_t ch;
    while (uart_is_readable(UART_ID))
    {
        ch = uart_getc(UART_ID);
        printf("Received character: %c\n",ch);
    }

    return;
}

/*
 * Function: lt_send_to_uart()
 * Send to UART the temperature and humidity in the requested format.
 */
void lt_send_to_uart(float temperature, float humidity, uint8_t temp_format, uint8_t out_format)
{
    char app_string[24];
    float temp;

    memset(app_string,0,sizeof(app_string));
    if(temp_format == TEMP_FORMAT_FAHRENHEIT)
        temp = C2F(temperature);
    else
        temp = temperature;

    switch(out_format)
    {
        case SER_OUTPUT_FORMAT_STRING:
            // add \r\n in order to be sure to print one measure on each line 
            // in the Windows/Unix terminal (without change its configuration)
            if(temp_format == TEMP_FORMAT_FAHRENHEIT)
                sprintf(app_string,"%.02f °F - %.02f %%RH\r\n",temp,humidity);
            else
                sprintf(app_string,"%.02f °C - %.02f %%RH\r\n",temp,humidity);
            break;

        case SER_OUTPUT_FORMAT_CSV:
            // use ";" as CSV separator
            sprintf(app_string,"%.02f;%.02f\r\n",temp,humidity);
            break;
        
        default:
            break;
    }
    // send the string to UART
    uart_puts(UART_ID, app_string);

    return;
}

/**
 * Function: lt_init_dflt()
 */
void lt_init_dflt(LT_data_t *data)
{
    memset(data,0,sizeof(LT_data_t));
    data->status = LT_STATUS_BOOT;
    data->temp_format = LT_TEMP_FORMAT_DFLT;
    data->serial_output_format = LT_SEROUT_FORMAT_DFLT;
    data->display_layout_format = LT_DISPLAY_MODE_DFLT;
    data->read_period = LT_READ_PERIOD_DFLT;
/*    
    hourly_data_t   temp_hours;
    hourly_data_t   hum_hours;
    daily_data_t    temp_day;
    daily_data_t    hum_day;
*/
    return;
}

/**
 * Function: my_gpio_callback()
 */
void my_gpio_callback(uint gpio_num,uint32_t events)
{
    if(gpio_num == GPIO_INPUT_BUTTON)
        button_pushed = true;
    return;
}

/*
 * main()
*/
int main(int argc, char *argv[])
{
    int ret = 0;
    uint8_t fb[SH1106_BUF_LEN];
    float temperature, humidity;
    char appo_string[10];
    int baudrate = 0;
    int uart_irq;
    int periodic_counter = 0;
    LT_data_t LT_data;

    stdio_init_all();

    // init working data
    lt_init_dflt(&LT_data);

    // chech watchdog status
    if (watchdog_caused_reboot())
    {
        printf("Rebooted by Watchdog!\n");
    }
    else
    {
        printf("Clean boot\n");
    }

    // wait to give time to hardware to be ready
    sleep_ms(2000);

#if !defined(I2C_PORT_OLED) || !defined(I2C_PORT_SENS)
#warning this programs requires a board with I2C pins
    puts("Define I2C pins were not defined");
#else

    // useful information for picotool
    bi_decl(bi_2pins_with_func(I2C_SDA_OLED, I2C_SCL_OLED, GPIO_FUNC_I2C));
    bi_decl(bi_program_description("lightThermo - simple temperature and humidity logger and display"));

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

    // Set up input button for change display mode
    button_pushed = false;
    gpio_init(GPIO_INPUT_BUTTON);
    gpio_set_dir(GPIO_INPUT_BUTTON,false);
    gpio_pull_up(GPIO_INPUT_BUTTON);
//    gpio_set_irq_enabled_with_callback(GPIO_INPUT_BUTTON, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &my_gpio_callback);
    gpio_set_irq_enabled_with_callback(GPIO_INPUT_BUTTON, GPIO_IRQ_EDGE_FALL, true, &my_gpio_callback);

    // Set up our UART with a basic baud rate.
    uart_init(UART_ID, 2400);

    // Set the TX and RX pins by using the function select on the GPIO
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Actually, we want a different speed
    // The call will return the actual baud rate selected, which will be as close as
    // possible to that requested
    baudrate = uart_set_baudrate(UART_ID, BAUD_RATE);
    printf("Set up UART at baudrate %d\n", baudrate);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ID, false, false);
    // Set our data format
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, false);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    uart_irq = (UART_ID == uart0) ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(uart_irq, uart_rx_callback);
    irq_set_enabled(uart_irq, true);

    // prepare buffer to receive commands
    memset(rx_commands,0,sizeof(rx_commands));
    cmd_received = false;

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);

    // initialization UART completed, send a welcome message
    uart_puts(UART_ID, "\r\nlightThermo ready to work...\r\n");
    uart_puts(UART_ID, "Read temperature and humidity every minutes\r\n");

    // run through the complete initialization process
    // OLED display init
    SH1106_init();

    // splash screen at boot
    SH1106_show_boot_info(fb,SH1106_BUF_LEN,PRG_VERSION,SENS_TYPE);
    SH1106_full_render(fb);
    sleep_ms(2000);

#ifdef DEBUG_FONTS
    char *test_string_1 = "Realtime";
    char *test_string_2 = "Last 24H T";
    char *test_string_3 = "Last 24H H";

    memset(fb,0,sizeof(fb));
    SH1106_write_string(fb, 0, 0, test_string_1,FONT_WIDTH_12,FONT_HIGH_16);
    SH1106_write_string(fb, 0, 16, test_string_2,FONT_WIDTH_12,FONT_HIGH_16);
    SH1106_write_string(fb, 0, 32, test_string_3,FONT_WIDTH_12,FONT_HIGH_16);
    SH1106_full_render(fb);
    sleep_ms(10000);
#endif

    SH1106_setup_display_layout(fb,SH1106_BUF_LEN,LT_DISPLAY_MODE_DFLT);
    SH1106_full_render(fb);
    SH1106_send_cmd(SH1106_SET_ENTIRE_ON); // go back to following RAM for pixel state
    LT_data.display_layout_format = LT_DISPLAY_MODE_DFLT;

    // Sensor init
    ret = DHT20_init();
    if(ret == 0)
    {
        // init OK, go to mainloop and start watchdog
        watchdog_enable(500, 1);

        LT_data.status = LT_STATUS_RT_MES;

        do
        {
            // wakeup every 100 ms
            sleep_ms(100);
            // update watchdog to avoid unexpected restart
            watchdog_update();

            // check input button
            if(button_pushed == true)
            {
                printf("GPIO Callback: counter=%d - GPIO=%d\n",periodic_counter,GPIO_INPUT_BUTTON);
                if(LT_data.display_layout_format == DISPLAY_MODE_RT_MES)
                {
                    SH1106_setup_display_layout(fb,SH1106_BUF_LEN,DISPLAY_MODE_SHOW_LAST_24H_T);
                    LT_data.display_layout_format = DISPLAY_MODE_SHOW_LAST_24H_T;
                }
                else if (LT_data.display_layout_format == DISPLAY_MODE_SHOW_LAST_24H_T)
                {
                    SH1106_setup_display_layout(fb,SH1106_BUF_LEN,DISPLAY_MODE_SHOW_LAST_24H_H);
                    LT_data.display_layout_format = DISPLAY_MODE_SHOW_LAST_24H_H;
                }
                else
                {
                    SH1106_setup_display_layout(fb,SH1106_BUF_LEN,DISPLAY_MODE_RT_MES);
                    SH1106_display_temperature(fb,LT_data.last_temp_read,FONT_WIDTH_12,FONT_HIGH_16);
                    SH1106_display_humidity(fb,LT_data.last_hum_read,FONT_WIDTH_12,FONT_HIGH_16);
                    LT_data.display_layout_format = DISPLAY_MODE_RT_MES;
                }
                SH1106_full_render(fb);

                button_pushed = false;
            }

            periodic_counter++;
            // NOTE: read_period is expressed in minutes
            if (periodic_counter == (int)(((float)LT_data.read_period*60*1000)/100))
            {
                temperature = 0;
                humidity = 0;
                if(DHT20_read_data(&temperature,&humidity) == 0)
                {
                    memset(appo_string,0,sizeof(appo_string));
                    if(LT_data.temp_format == TEMP_FORMAT_CELSIUS)
                        sprintf(appo_string,"%.2f",temperature);
                    else
                        sprintf(appo_string,"%.2f",C2F(temperature));
                    strcpy(LT_data.last_temp_read,appo_string);
                    if(LT_data.display_layout_format == DISPLAY_MODE_RT_MES)
                        SH1106_display_temperature(fb,appo_string,FONT_WIDTH_12,FONT_HIGH_16);
                    memset(appo_string,0,sizeof(appo_string));
                    sprintf(appo_string,"%.2f",humidity);
                    strcpy(LT_data.last_hum_read,appo_string);
                    if(LT_data.display_layout_format == DISPLAY_MODE_RT_MES)
                    {
                        SH1106_display_humidity(fb,appo_string,FONT_WIDTH_12,FONT_HIGH_16);
                        SH1106_full_render(fb);
                    }
                    lt_send_to_uart(temperature,humidity,LT_data.temp_format,LT_data.serial_output_format);
                }
                periodic_counter = 0;
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
