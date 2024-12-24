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
volatile bool timer_fired = false;
volatile int tick = 0;


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
 * Funtion: lt_init_hourly_statistic()
 */
void lt_init_hourly_statistic(hourly_data_t *d)
{
    d->max_value = 0;
    d->min_value = 9999;
    d->avg_value = 0;
    d->rd = 0;
    d->wr = 0;
    return;
}

/**
 * Function: lt_init_daily_statistic()
 */
void lt_init_daily_statistic(daily_data_t *d)
{
    d->max_value = 0;
    d->min_value = 9999;
    d->avg_value = 0;
    d->full = false;
    d->wr = 0;
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
    data->last_tick = 0;
    data->nloop = 0;

    lt_init_hourly_statistic(&(data->temp_hours));
    lt_init_hourly_statistic(&(data->hum_hours));

    lt_init_daily_statistic(&(data->temp_day));
    lt_init_daily_statistic(&(data->hum_day));

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

/**
 * Function: lt_update_statistics()
 */
void lt_update_statistics(LT_data_t *data, float temp,float hum)
{
    int i;
    int sum_avg = 0;
    int temp_cents = (int)(temp * 100);
    int hum_cents = (int)(hum * 100);

    // update data for hour statistics
    if(temp_cents > data->temp_hours.max_value)
        data->temp_hours.max_value = temp_cents;
    if(temp_cents < data->temp_hours.min_value)
        data->temp_hours.min_value = temp_cents;
    PRINT_GEN_DEBUG("Hour - save temperature %d to position %d\n",temp_cents,data->temp_hours.wr);
    data->temp_hours.value[data->temp_hours.wr] = temp_cents;
    for(i=0; i<=data->temp_hours.wr; i++)
        sum_avg += data->temp_hours.value[i];
    data->temp_hours.avg_value = (sum_avg / i);
    data->temp_hours.wr = ((data->temp_hours.wr + 1) % NVALUE_PER_HOUR);

    sum_avg = 0;
    if(hum_cents > data->hum_hours.max_value)
        data->hum_hours.max_value = hum_cents;
    if(hum_cents < data->hum_hours.min_value)
        data->hum_hours.min_value = hum_cents;
    PRINT_GEN_DEBUG("Hour - save humidity %d to position %d\n",hum_cents,data->hum_hours.wr);
    data->hum_hours.value[data->hum_hours.wr] = hum_cents;
    for(i=0; i<=data->hum_hours.wr; i++)
        sum_avg += data->hum_hours.value[i];
    data->hum_hours.avg_value = (sum_avg / i);
    data->hum_hours.wr = ((data->hum_hours.wr + 1) % NVALUE_PER_HOUR);

    // update data for daily statistics
    if((data->nloop % NLOOP_PER_HOUR) == 0)
    {
        sum_avg = 0;
        PRINT_GEN_DEBUG("Day - save temperature %d to position %d\n",data->temp_hours.avg_value,data->temp_day.wr);
        data->temp_day.value[data->temp_day.wr] = data->temp_hours.avg_value;
        if(data->temp_day.full == false)
        {
            for(i=0; i<=data->temp_day.wr; i++)
            {
                sum_avg += data->temp_day.value[i];
                if(data->temp_day.value[i] > data->temp_day.max_value)
                    data->temp_day.max_value = data->temp_day.value[i];
                if(data->temp_day.value[i] < data->temp_day.min_value)
                    data->temp_day.min_value = data->temp_day.value[i];
            }
            data->temp_day.avg_value = (sum_avg / i);
        }
        else
        {
            for(i=0; i<NVALUE_PER_DAY; i++)
            {
                sum_avg += data->temp_day.value[i];
                if(data->temp_day.value[i] > data->temp_day.max_value)
                    data->temp_day.max_value = data->temp_day.value[i];
                if(data->temp_day.value[i] < data->temp_day.min_value)
                    data->temp_day.min_value = data->temp_day.value[i];
            }
            data->temp_day.avg_value = (sum_avg / NVALUE_PER_DAY);            
        }
        if((data->temp_day.full == false) && (data->temp_day.wr == (NVALUE_PER_DAY - 1)))
        {
            data->temp_day.full = true;
        }
        data->temp_day.wr = ((data->temp_day.wr + 1) % NVALUE_PER_DAY);
        lt_init_hourly_statistic(&(data->temp_hours));
        if(data->temp_day.wr == 0)
        {
            // new day, reset max and min
            data->temp_day.max_value = 0;
            data->temp_day.min_value = 9999;
        }

        sum_avg = 0;
        PRINT_GEN_DEBUG("Day - save humidity %d to position %d\n",data->hum_hours.avg_value,data->hum_day.wr);
        data->hum_day.value[data->hum_day.wr] = data->hum_hours.avg_value;
        if(data->hum_day.full == false)
        {
            for(i=0; i<=data->hum_day.wr; i++)
            {
                sum_avg += data->hum_day.value[i];
                if(data->hum_day.value[i] > data->hum_day.max_value)
                    data->hum_day.max_value = data->hum_day.value[i];
                if(data->hum_day.value[i] < data->hum_day.min_value)
                    data->hum_day.min_value = data->hum_day.value[i];
            }
            data->hum_day.avg_value = (sum_avg / i);
        }
        else
        {
            for(i=0; i<NVALUE_PER_DAY; i++)
            {
                sum_avg += data->hum_day.value[i];
                if(data->hum_day.value[i] > data->hum_day.max_value)
                    data->hum_day.max_value = data->hum_day.value[i];
                if(data->hum_day.value[i] < data->hum_day.min_value)
                    data->hum_day.min_value = data->hum_day.value[i];
            }
            data->hum_day.avg_value = (sum_avg / NVALUE_PER_DAY);
        }
        if((data->hum_day.full == false) && (data->hum_day.wr == (NVALUE_PER_DAY - 1)))
        {
            data->hum_day.full = true;
        }
        data->hum_day.wr = ((data->hum_day.wr + 1) % NVALUE_PER_DAY);
        if(data->hum_day.wr == 0)
        {
            // new_day, reset max e min
            data->hum_day.max_value = 0;
            data->hum_day.min_value = 9999;
        }
        lt_init_hourly_statistic(&(data->hum_hours));
    }

    return;
}

/**
 * Function: lt_show_t_stats();
 */
void lt_show_t_stats(uint8_t *buf,LT_data_t *data)
{
    if(data->temp_day.full == false)
        SH1106_display_t_stats(buf,data->temp_hours.max_value,data->temp_hours.min_value,data->temp_hours.avg_value,FONT_WIDTH_12,FONT_HIGH_16);
    else
        SH1106_display_t_stats(buf,data->temp_day.max_value,data->temp_day.min_value,data->temp_day.avg_value,FONT_WIDTH_12,FONT_HIGH_16);
    return;
}

/**
 * Function: lt_show_h_stats();
 */
void lt_show_h_stats(uint8_t *buf,LT_data_t *data)
{
    if(data->hum_day.full == false)
        SH1106_display_h_stats(buf,data->hum_hours.max_value,data->hum_hours.min_value,data->hum_hours.avg_value,FONT_WIDTH_12,FONT_HIGH_16);
    else
        SH1106_display_h_stats(buf,data->hum_day.max_value,data->hum_day.min_value,data->hum_day.avg_value,FONT_WIDTH_12,FONT_HIGH_16);
    return;
}

/**
 * Function: repeat_timer_callback();
 * Timer callback
 */
bool repeat_timer_callback(struct repeating_timer *timer)
{
    timer_fired = true;
    tick++;
    return true;
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
    LT_data_t LT_data;
    volatile int64_t app_tick, prev_tick;
    volatile int64_t check_loop_pre, check_loop_after;
    struct repeating_timer timer;

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

    PRINT_GEN_DEBUG("Hello, SH1106 OLED display! Look at my board..(vers=%s)\n",PRG_VERSION);

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
    PRINT_GEN_DEBUG("Set up UART at baudrate %d\n", baudrate);

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
    char *test_string_1 = "abcdefghkji";
    char *test_string_2 = "lmnopqrstuv";
    char *test_string_3 = "wxyz";

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

        // acquire us ticks to measure the time drift
        prev_tick = time_us_64();
        add_repeating_timer_ms(-MAINLOOP_TICK,repeat_timer_callback,NULL,&timer);
        check_loop_pre = prev_tick;
        check_loop_after = 0;

        do
        {
            while(!timer_fired)
            {
                tight_loop_contents();
            }
            // timer fired, check how many ms are left
            app_tick = time_us_64();
//            PRINT_GEN_DEBUG("tick = %d - Delta [us] = %lld\n",tick,(app_tick - prev_tick));
            prev_tick = app_tick;
            timer_fired = false;

            // update watchdog to avoid unexpected restart
            watchdog_update();

            // check input button
            if(button_pushed == true)
            {
                // anti-debouncing (if poor quality button is used)
                if((LT_data.last_tick == 0) || (tick > (LT_data.last_tick + 2)))
                {
                    LT_data.last_tick = tick;
                    PRINT_GEN_DEBUG("GPIO Callback: tick=%d - GPIO=%d\n",tick,GPIO_INPUT_BUTTON);
                    if(LT_data.display_layout_format == DISPLAY_MODE_RT_MES)
                    {
                        SH1106_setup_display_layout(fb,SH1106_BUF_LEN,DISPLAY_MODE_SHOW_LAST_24H_T);
                        lt_show_t_stats(fb,&LT_data);
                        LT_data.display_layout_format = DISPLAY_MODE_SHOW_LAST_24H_T;
                    }
                    else if (LT_data.display_layout_format == DISPLAY_MODE_SHOW_LAST_24H_T)
                    {
                        SH1106_setup_display_layout(fb,SH1106_BUF_LEN,DISPLAY_MODE_SHOW_LAST_24H_H);
                        lt_show_h_stats(fb,&LT_data);
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
                }
                else
                    PRINT_GEN_DEBUG("Ignore GPIO callback too close to the previous one (anti-debouncing) - tick=%d last_tick=%d\n",tick,LT_data.last_tick);
                button_pushed = false;
            }
#if 0
            // now tick is incremented inside the timer callback()
            tick++;
#endif
            // NOTE: read_period is expressed in minutes
            if (tick == (int)(((float)LT_data.read_period*60*1000)/MAINLOOP_TICK))
            {
                // reset the tick variable before to start the elaboration to don't
                // consider the time of the sensor read and display update in the polling
                // timeout. The tick is incremented in the callback function of the hardware
                // timer therefore if this routine is longer than a single tick timer, 
                // it doesn't affect the total polling timer.
                tick = 0;
                LT_data.last_tick = 0;
                temperature = 0;
                humidity = 0;
                LT_data.nloop++;
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
                // update statistics data
                lt_update_statistics(&LT_data,temperature,humidity);
                if(LT_data.display_layout_format == DISPLAY_MODE_SHOW_LAST_24H_T)
                    lt_show_t_stats(fb,&LT_data);
                if(LT_data.display_layout_format == DISPLAY_MODE_SHOW_LAST_24H_H)
                    lt_show_h_stats(fb,&LT_data);
                SH1106_full_render(fb);

                check_loop_after = time_us_64();
                PRINT_GEN_DEBUG("Polling duration [ms]= %lld\n", ((check_loop_after - check_loop_pre)/1000));
                check_loop_pre = check_loop_after;
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
            PRINT_GEN_DEBUG("%s - Need calibration... :-(\n",__FUNCTION__);
        } while (true);
    }
#endif // !defined(I2C_PORT_OLED) || !defined(I2C_PORT_SENS)

    return ret;
}
