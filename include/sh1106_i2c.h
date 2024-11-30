/* SH1106 OLED DISPLAY DRIVER VIA I2C*/
#include "hardware/i2c.h"

#define I2C_PORT_OLED               i2c0
#define I2C_SDA_OLED                8
#define I2C_SCL_OLED                9

#define SH1106_HEIGHT               64
#define SH1106_WIDTH                128 // internally is 132

#define SH1106_I2C_ADDR             _u(0x3C)

#define SH1106_I2C_CLK              400

// commands (see datasheet)
#define SH1106_SETLOWCOLUMN         _u(0x00)
#define SH1106_SETHIGHCOLUMN        _u(0x10)
#define SH1106_SET_MEM_MODE         _u(0x20)
#define SH1106_SET_COL_ADDR         _u(0x21)
//#define SH1106_SET_PAGE_ADDR        _u(0x22)
#define SH1106_SET_HORIZ_SCROLL     _u(0x26)
#define SH1106_SET_SCROLL           _u(0x2E)

#define SH1106_SET_DISP_START_LINE  _u(0x40)

#define SH1106_SET_CONTRAST         _u(0x81)
#define SH1106_SET_CHARGE_PUMP      _u(0x8D)

#define SH1106_SET_SEG_REMAP        _u(0xA0)
#define SH1106_SET_ENTIRE_ON        _u(0xA4)
#define SH1106_SET_ALL_ON           _u(0xA5)
#define SH1106_SET_NORM_DISP        _u(0xA6)
#define SH1106_SET_INV_DISP         _u(0xA7)
#define SH1106_SET_MUX_RATIO        _u(0xA8)
#define SH1106_SET_DISP_OFF         _u(0xAE)
#define SH1106_SET_DISP_ON          _u(0xAF)
#define SH1106_SET_PAGE_ADDR        _u(0xB0)
#define SH1106_SET_COM_OUT_DIR      _u(0xC0)
#define SH1106_SET_COM_OUT_DIR_FLIP _u(0xC0)

#define SH1106_SET_DISP_OFFSET      _u(0xD3)
#define SH1106_SET_DISP_CLK_DIV     _u(0xD5)
#define SH1106_SET_PRECHARGE        _u(0xD9)
#define SH1106_SET_COM_PIN_CFG      _u(0xDA)
#define SH1106_SET_VCOM_DESEL       _u(0xDB)

#define SH1106_PAGE_HEIGHT          _u(8)
#define SH1106_NUM_PAGES            (SH1106_HEIGHT / SH1106_PAGE_HEIGHT)
#define SH1106_BUF_LEN              (SH1106_NUM_PAGES * SH1106_WIDTH)

#define SH1106_WRITE_MODE           _u(0xFE)
#define SH1106_READ_MODE            _u(0xFF)

// FONTS
#define FONT_HIGH_8                 8
#define FONT_HIGH_16                16
#define FONT_HIGH_24                24

#define FONT_WIDTH_8                8
#define FONT_WIDTH_12               12
#define FONT_WIDTH_16               16
#define FONT_WIDTH_24               24

// TODO: to be change with new layout
// Definition of display areas
// build the entire display layout (status bar + info area + extra info area)
// __________________
// |_____|_____|____|   2 pages - Status Bar (status + ID + Connection state)
// |                |   4 pages - Info Area
// |________________|
// |________________|   2 pages - extra info area
//  
#define FIRST_PAGE_STATUS_BAR       1
#define NPAGE_STATUS_BAR            2
#define FIRST_PAGE_INFO_AREA        (FIRST_PAGE_STATUS_BAR + NPAGE_STATUS_BAR)
#define NPAGE_INFO_AREA             4
#define FIRST_PAGE_EXTRA_INFO_AREA  (FIRST_PAGE_INFO_AREA + NPAGE_INFO_AREA)
#define NPAGE_EXTRA_INFO_AREA       1

#define DSPLY_MODE_RT_MES_DESCR_Y   8
#define DSPLY_MODE_RT_MES_ICONT_Y   (DSPLY_MODE_RT_MES_DESCR_Y + FONT_HIGH_16)
#define DSPLY_MODE_RT_MES_ICONH_Y   (DSPLY_MODE_RT_MES_ICONT_Y + FONT_HIGH_16)

#define DSPLY_MODE_LAST24H_T_DES_Y  0
#define DSPLY_MODE_LAST24H_T_MIN_Y  (DSPLY_MODE_LAST24H_T_DES_Y + FONT_HIGH_16)
#define DSPLY_MODE_LAST24H_T_MAX_Y  (DSPLY_MODE_LAST24H_T_MIN_Y + FONT_HIGH_16)
#define DSPLY_MODE_LAST24H_T_AVG_Y  (DSPLY_MODE_LAST24H_T_MAX_Y + FONT_HIGH_16)

#define DSPLY_MODE_LAST24H_H_DES_Y  0
#define DSPLY_MODE_LAST24H_H_MIN_Y  (DSPLY_MODE_LAST24H_H_DES_Y + FONT_HIGH_16)
#define DSPLY_MODE_LAST24H_H_MAX_Y  (DSPLY_MODE_LAST24H_H_MIN_Y + FONT_HIGH_16)
#define DSPLY_MODE_LAST24H_H_AVG_Y  (DSPLY_MODE_LAST24H_H_MAX_Y + FONT_HIGH_16)


enum sh1106_area_id {
    SH1106_AREA_STATUS_BAR_ID,
    SH1106_AREA_INFO_AREA_ID,
    SH1106_AREA_EXTRA_INFO_AREA_ID,
    SH1106_AREA_ID_MAX
};

typedef struct sh1106_area_descr {
    uint8_t first_page;
    uint8_t size;
} sh1106_area_descr_t;

typedef enum SH1106_ICONS {
    SH1106_ICON_CELSIUS,
    SH1106_ICON_FAHRENHEIT,
    SH1106_ICON_PERCENT,
    SH1106_ICON_TERMOMETER,
    SH1106_ICON_DROP,
    SH1106_ICON_WIFI_CONNECTED,
    SH1106_ICON_MAX
} sh1106_icons_t;

// forward declaration
void SH1106_init();
void SH1106_send_cmd(uint8_t cmd);
void SH1106_send_cmd_list(uint8_t *buf, int num);
void SH1106_send_buf(uint8_t buf[], int buflen);
void SH1106_scroll(bool on);
void SH1106_full_render(uint8_t *buf);
int SH1106_write_string(uint8_t *buf, int16_t x, int16_t y, char *str, uint8_t font_l, uint8_t font_h);
int SH1106_display_temperature(uint8_t *buf,char *str, uint8_t font_l, uint8_t font_h);
int SH1106_display_humidity(uint8_t *buf,char *str, uint8_t font_l, uint8_t font_h);
int SH1106_write_icon(uint8_t *buf, int16_t x, int16_t y, uint8_t id_icon, uint8_t font_l, uint8_t font_h);
void SH1106_setup_display_layout(uint8_t *buf, int fb_size, uint8_t mode);
void SH1106_show_boot_info(uint8_t *buf,int fb_size,char *prg_vers,char *sens_type);

