/* 
SH1106 OLED DISPLAY DRIVER
*/
// Define the size of the display we have attached. This can vary, make sure you
// have the right size defined or the output will look rather odd!
// Code has been tested on 128x32 and 128x64 OLED displays
#define SH1106_HEIGHT               64
#define SH1106_WIDTH                128 //132

#define SH1106_I2C_ADDR             _u(0x3C)

// 400 is usual, but often these can be overclocked to improve display response.
// Tested at 1000 on both 32 and 84 pixel height devices and it worked.
#define SH1106_I2C_CLK              400

#if 0
#define SH1106_SETLOWCOLUMN             0x00
#define SH1106_SETHIGHCOLUMN            0x10
#define SH1106_MEMORYMODE               0x20
#define SH1106_COLUMNADDR               0x21
#define SH1106_PAGEADDR                 0x22
// Scrolling #defines
#define SH1106_RIGHT_HORIZONTAL_SCROLL                  0x26
#define SH1106_LEFT_HORIZONTAL_SCROLL                   0x27
#define SH1106_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL     0x29
#define SH1106_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL      0x2A
#define SH1106_DEACTIVATE_SCROLL                        0x2E
#define SH1106_ACTIVATE_SCROLL                          0x2F

#define SH1106_SETSTARTLINE             0x40
#define SH1106_SETCONTRAST              0x81
#define SH1106_CHARGEPUMP               0x8D
#define SH1106_SEGREMAP                 0xA0
#define SH1106_SET_VERTICAL_SCROLL_AREA 0xA3
#define SH1106_DISPLAYALLON_RESUME      0xA4
#define SH1106_DISPLAYALLON             0xA5
#define SH1106_NORMALDISPLAY            0xA6
#define SH1106_INVERTDISPLAY            0xA7
#define SH1106_SETMULTIPLEX             0xA8
#define SH1106_DISPLAYOFF               0xAE
#define SH1106_DISPLAYON                0xAF
#define SH1106_COMSCANINC               0xC0
#define SH1106_COMSCANDEC               0xC8
#define SH1106_SETDISPLAYOFFSET         0xD3
#define SH1106_SETDISPLAYCLOCKDIV       0xD5
#define SH1106_SETPRECHARGE             0xD9
#define SH1106_SETCOMPINS               0xDA
#define SH1106_SETVCOMDETECT            0xDB

#define SH1106_EXTERNALVCC              0x1
#define SH1106_SWITCHCAPVCC             0x2
#endif


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

// Definition of display areas
// build the entire display layout (status bar + info area + extra info area)
// __________________
// |_____|_____|____|   2 pages - Status Bar (status + ID + Connection state)
// |                |   4 pages - Info Area
// |________________|
// |________________|   2 pages - extra info area
//  
#define FIRST_PAGE_STATUS_BAR       0
#define NPAGE_STATUS_BAR            2
#define FIRST_PAGE_INFO_AREA        (FIRST_PAGE_STATUS_BAR + NPAGE_STATUS_BAR)
#define NPAGE_INFO_AREA             4
#define FIRST_PAGE_EXTRA_INFO_AREA  (FIRST_PAGE_INFO_AREA + NPAGE_INFO_AREA)
#define NPAGE_EXTRA_INFO_AREA       2

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

#if 0
struct render_area
{
    uint8_t start_col;
    uint8_t end_col;
    uint8_t start_page;
    uint8_t end_page;

    int buflen;
};
#endif

// forward declaration
//void calc_render_area_buflen(struct render_area *area);
void SH1106_init();
void SH1106_send_cmd(uint8_t cmd);
void SH1106_send_cmd_list(uint8_t *buf, int num);
void SH1106_send_buf(uint8_t buf[], int buflen);
void SH1106_scroll(bool on);
void SH1106_full_render(uint8_t *buf);
//static void SetPixel(uint8_t *buf, int x,int y, bool on);
//static void DrawLine(uint8_t *buf, int x0, int y0, int x1, int y1, bool on);
//static int GetFontIndex(uint8_t ch);
//static void WriteChar(uint8_t *buf, int16_t x, int16_t y, uint8_t ch);
void WriteString(uint8_t *buf, int16_t x, int16_t y, char *str);
int SH1106_write_string(uint8_t *buf, int16_t x, int16_t y, char *str, uint8_t font_l, uint8_t font_h);
int SH1106_display_temperature(uint8_t *buf,char *str, uint8_t font_l, uint8_t font_h);
int SH1106_display_humidity(uint8_t *buf,char *str, uint8_t font_l, uint8_t font_h);
