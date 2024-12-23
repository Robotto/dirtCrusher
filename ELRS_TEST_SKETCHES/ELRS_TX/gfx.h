//https://forum.arduino.cc/t/u8glib-and-bitmap-creation-display/148125/2

//CAR ICON:
#define car_width 20
#define car_height 15
#define car_x 0
#define car_y 0
static const unsigned char car_bits[] U8X8_PROGMEM = {
   0x3c, 0xcf, 0x03, 0xe0, 0x7f, 0x00, 0xf0, 0xff, 0x00, 0x38, 0xc0, 0x00,
   0x18, 0xc0, 0x01, 0x1f, 0x80, 0x0f, 0xfe, 0xff, 0x07, 0xfe, 0xff, 0x07,
   0xe6, 0x7f, 0x06, 0xc2, 0x3f, 0x04, 0xe6, 0x7f, 0x06, 0xfe, 0xff, 0x07,
   0xfc, 0xff, 0x03, 0x1c, 0x80, 0x03, 0x1c, 0x80, 0x03 };

//CONTROLLER ICON:
#define controller_width 20
#define controller_height 14
#define controller_x 0
#define controller_y car_height+3
static const unsigned char controller_bits[] U8X8_PROGMEM = {
   0x00, 0x02, 0x00, 0x00, 0x02, 0x00, 0x00, 0x02, 0x00, 0xf0, 0xff, 0x00,
   0x0c, 0x00, 0x03, 0x02, 0x00, 0x04, 0x01, 0x00, 0x08, 0x11, 0x40, 0x08,
   0x39, 0xe0, 0x08, 0x11, 0x40, 0x08, 0x01, 0x00, 0x08, 0x02, 0x00, 0x04,
   0xc4, 0x1f, 0x02, 0x38, 0xe0, 0x01 };

#define antenna_width 20
#define antenna_height 15
#define antenna_x 64
#define antenna_y car_height+3
static const unsigned char antenna_bits[] U8X8_PROGMEM = {
  0x08, 0x00, 0x01, 0x0C, 0x00, 0x03, 0x26, 0x40, 0x06, 0x36, 0xC0, 0x06, 
  0x33, 0xC6, 0x0C, 0x13, 0x8F, 0x0C, 0x93, 0x9F, 0x0C, 0x13, 0x8F, 0x0C, 
  0x37, 0xC6, 0x0E, 0x26, 0x46, 0x06, 0x0E, 0x06, 0x07, 0x0C, 0x06, 0x03, 
  0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x0F, 0x00};
