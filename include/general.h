/*
 * Comment/Uncomment these directive to disable/enable 
 * the output usefull for debug purpose.
*/

#define DEBUG_GEN
//#define DEBUG_FONTS
//#define DEBUG_DISPLAY
//#define DEBUG_I2C

#ifdef DEBUG_GEN
#define PRINT_GEN_DEBUG(format, ...)   printf(format, __VA_ARGS__)
#else
#define PRINT_GEN_DEBUG(format, ...)
#endif

#ifdef DEBUG_DISPLAY
#define PRINT_SH1106_DEBUG(format, ...)   printf(format, __VA_ARGS__)
#else
#define PRINT_SH1106_DEBUG(format, ...)
#endif

#ifdef DEBUG_I2C
#define PRINT_I2C_DEBUG(format, ...)   printf(format, __VA_ARGS__)
#else
#define PRINT_I2C_DEBUG(format, ...)
#endif



#define NVALUE_PER_HOUR             60
#define NVALUE_PER_DAY              24

enum lt_status {
    LT_STATUS_BOOT,
    LT_STATUS_RT_MES,
//    LT_STATUS_SHOW_LAST_24H_T,
//    LT_STATUS_SHOW_LAST_24H_H,
    LT_STATUS_CMD_MODE,
    LT_STATUS_ERROR,
    LT_STATUS_MAX
};

enum temp_format {
    TEMP_FORMAT_CELSIUS = 1,
    TEMP_FORMAT_FAHRENHEIT,
    TEMP_FORMAT_MAX
};

enum serial_output_format {
    SER_OUTPUT_FORMAT_STRING,
    SER_OUTPUT_FORMAT_CSV,
    SER_OUTPUT_FORMAT_MAX
};

enum display_layout_format {
    DISPLAY_MODE_RT_MES,
    DISPLAY_MODE_SHOW_LAST_24H_T,
    DISPLAY_MODE_SHOW_LAST_24H_H,
    DISPLAY_MODE_MAX
};

enum display_last24h_format {
    DISPLAY_MIN_VALUE,
    DISPLAY_MAX_VALUE,
    DISPLAY_AVG_VALUE,
    DISPLAY_VALUE_MAX
};

// default parameters
#define LT_STATUS_DFTL          LT_STATUS_RT_MES
#define LT_TEMP_FORMAT_DFLT     TEMP_FORMAT_CELSIUS
#define LT_SEROUT_FORMAT_DFLT   SER_OUTPUT_FORMAT_STRING
#define LT_DISPLAY_MODE_DFLT    DISPLAY_MODE_RT_MES
#define LT_READ_PERIOD_DFLT     1
