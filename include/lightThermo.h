#define PRG_VERSION             "006"
#define SENS_TYPE               "DHT20"

// UART PARAMETERS
#define UART_ID                 uart0
#define BAUD_RATE               115200
#define DATA_BITS               8
#define STOP_BITS               1
#define PARITY                  UART_PARITY_NONE

// Use default UART pins 0 and 1
#define UART_TX_PIN             0
#define UART_RX_PIN             1

#define NMAX_CMD_STRING         32

// GPIO for input button
#define GPIO_INPUT_BUTTON       15

// mainloop tick [ms]
#define MAINLOOP_TICK           100
#define NLOOP_PER_HOUR          NVALUE_PER_HOUR

typedef struct hourly_data {
    int     value[NVALUE_PER_HOUR];
    uint8_t rd;
    uint8_t wr;
    int     max_value;
    int     min_value;
    int     avg_value;
} hourly_data_t;

typedef struct dayly_data {
    int     value[NVALUE_PER_DAY];
    bool    full;
    uint8_t wr;
    int     max_value;
    int     min_value;
    int     avg_value;
} daily_data_t;

typedef struct LT_data {
    uint8_t         status;
    uint8_t         temp_format;
    uint8_t         serial_output_format;
    uint8_t         display_layout_format;
    uint8_t         read_period;
    char            last_temp_read[10];
    char            last_hum_read[10];
    int             last_tick;
    int             nloop;
    hourly_data_t   temp_hours;
    hourly_data_t   hum_hours;
    daily_data_t    temp_day;
    daily_data_t    hum_day;
} LT_data_t;

