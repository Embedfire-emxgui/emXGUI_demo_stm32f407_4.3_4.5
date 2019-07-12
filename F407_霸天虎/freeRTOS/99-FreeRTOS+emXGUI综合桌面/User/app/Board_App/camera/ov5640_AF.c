/*************************************************************************
* FUNCTION
*   OV5640_FOCUS_AD5820_Init
*
* DESCRIPTION
*   This function is to load micro code for AF function
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
#include "./camera/ov5640_AF.h"

#define Delay(ms)  GUI_msleep(ms)

static void OV5640_FOCUS_AD5820_Check_MCU(void);

static u8 OV5640_AF_FW[] =    
{
    0x02, 0x0d, 0xf3, 0x02, 0x0a, 0x5f, 0xc2, 0x01, 0x22, 0x22, 0x00, 0x02, 0x0f, 0x31, 0x30, 0x01,
    0x03, 0x02, 0x03, 0x09, 0x30, 0x02, 0x03, 0x02, 0x03, 0x09, 0x90, 0x51, 0xa5, 0xe0, 0x78, 0xbb, 
    0xf6, 0xa3, 0xe0, 0x08, 0xf6, 0xa3, 0xe0, 0x08, 0xf6, 0xe5, 0x1f, 0x70, 0x45, 0x75, 0x1e, 0x20, 
    0xd2, 0x34, 0x12, 0x0c, 0x0c, 0x78, 0x9c, 0x12, 0x0b, 0xd2, 0x78, 0xa8, 0xa6, 0x14, 0x08, 0xa6, 
    0x15, 0x78, 0xb3, 0xa6, 0x09, 0x18, 0x76, 0x01, 0x78, 0x4c, 0xa6, 0x0a, 0x08, 0xa6, 0x0b, 0x78, 
    0x6c, 0xa6, 0x14, 0x08, 0xa6, 0x15, 0x78, 0xb3, 0xe6, 0x78, 0x8c, 0xf6, 0x75, 0x1f, 0x01, 0x78, 
    0xbb, 0xe6, 0x78, 0xb8, 0xf6, 0x78, 0xbc, 0xe6, 0x78, 0xb9, 0xf6, 0x78, 0xbd, 0xe6, 0x78, 0xba, 
    0xf6, 0x22, 0x79, 0xb8, 0xe7, 0xd3, 0x78, 0xbb, 0x96, 0x40, 0x05, 0xe7, 0x96, 0xff, 0x80, 0x08, 
    0xc3, 0x79, 0xbb, 0xe7, 0x78, 0xb8, 0x96, 0xff, 0x78, 0xa6, 0x76, 0x00, 0x08, 0xa6, 0x07, 0x79, 
    0xb9, 0xe7, 0xd3, 0x78, 0xbc, 0x96, 0x40, 0x05, 0xe7, 0x96, 0xff, 0x80, 0x08, 0xc3, 0x79, 0xbc, 
    0xe7, 0x78, 0xb9, 0x96, 0xff, 0x12, 0x0c, 0x13, 0x79, 0xba, 0xe7, 0xd3, 0x78, 0xbd, 0x96, 0x40, 
    0x05, 0xe7, 0x96, 0xff, 0x80, 0x08, 0xc3, 0x79, 0xbd, 0xe7, 0x78, 0xba, 0x96, 0xff, 0x12, 0x0c, 
    0x13, 0x78, 0xb2, 0xe6, 0x25, 0xe0, 0x24, 0x4c, 0xf8, 0xa6, 0x0a, 0x08, 0xa6, 0x0b, 0x78, 0xb2, 
    0xe6, 0x25, 0xe0, 0x24, 0x6c, 0xf8, 0xa6, 0x14, 0x08, 0xa6, 0x15, 0x78, 0xb2, 0xe6, 0x24, 0x8c, 
    0xf8, 0xa6, 0x09, 0x78, 0xb2, 0xe6, 0x24, 0x01, 0xff, 0xe4, 0x33, 0xfe, 0xd3, 0xef, 0x94, 0x0f, 
    0xee, 0x64, 0x80, 0x94, 0x80, 0x40, 0x04, 0x7f, 0x00, 0x80, 0x05, 0x78, 0xb2, 0xe6, 0x04, 0xff, 
    0x78, 0xb2, 0xa6, 0x07, 0xe5, 0x1f, 0xb4, 0x01, 0x0a, 0xe6, 0x60, 0x03, 0x02, 0x03, 0x09, 0x75, 
    0x1f, 0x02, 0x22, 0x12, 0x0c, 0x0c, 0x78, 0x9e, 0x12, 0x0b, 0xd2, 0x12, 0x0c, 0x0c, 0x78, 0xa0, 
    0x12, 0x0b, 0xff, 0x78, 0xaa, 0x12, 0x0b, 0xff, 0xff, 0x78, 0xac, 0xa6, 0x06, 0x08, 0xa6, 0x07, 
    0x78, 0x8c, 0xe6, 0x78, 0xb4, 0xf6, 0x78, 0x8c, 0xe6, 0x78, 0xb5, 0xf6, 0x7f, 0x01, 0xef, 0x25, 
    0xe0, 0x24, 0x4d, 0x78, 0x9f, 0x12, 0x0b, 0xc9, 0x50, 0x0a, 0x12, 0x0b, 0xab, 0x78, 0x9e, 0xa6, 
    0x04, 0x08, 0xa6, 0x05, 0xef, 0x25, 0xe0, 0x24, 0x6d, 0x78, 0xab, 0x12, 0x0b, 0xc9, 0x50, 0x0f, 
    0xef, 0x25, 0xe0, 0x24, 0x6c, 0x12, 0x0b, 0xb0, 0x78, 0xaa, 0xa6, 0x04, 0x08, 0xa6, 0x05, 0x74, 
    0x8c, 0x2f, 0xf9, 0x78, 0xb4, 0xe6, 0xc3, 0x97, 0x50, 0x08, 0x74, 0x8c, 0x2f, 0xf8, 0xe6, 0x78, 
    0xb4, 0xf6, 0xef, 0x25, 0xe0, 0x24, 0x4d, 0xf9, 0xd3, 0x78, 0xa1, 0x12, 0x0b, 0xcb, 0x40, 0x0a, 
    0x12, 0x0b, 0xab, 0x78, 0xa0, 0xa6, 0x04, 0x08, 0xa6, 0x05, 0xef, 0x25, 0xe0, 0x24, 0x6d, 0xf9, 
    0xd3, 0x78, 0xad, 0x12, 0x0b, 0xcb, 0x40, 0x0f, 0xef, 0x25, 0xe0, 0x24, 0x6c, 0x12, 0x0b, 0xb0, 
    0x78, 0xac, 0xa6, 0x04, 0x08, 0xa6, 0x05, 0x74, 0x8c, 0x2f, 0xf9, 0x78, 0xb5, 0xe6, 0xd3, 0x97, 
    0x40, 0x08, 0x74, 0x8c, 0x2f, 0xf8, 0xe6, 0x78, 0xb5, 0xf6, 0x0f, 0xef, 0x64, 0x10, 0x60, 0x03, 
    0x02, 0x01, 0x3e, 0xc3, 0x79, 0x9f, 0x78, 0xa1, 0x12, 0x0b, 0xf7, 0x78, 0xa2, 0xf6, 0x08, 0xa6, 
    0x07, 0xc3, 0x79, 0xab, 0x78, 0xad, 0x12, 0x0b, 0xf7, 0x78, 0xae, 0xf6, 0x08, 0xa6, 0x07, 0xc3, 
    0x79, 0xb4, 0xe7, 0x78, 0xb5, 0x96, 0x08, 0xf6, 0xd3, 0x79, 0x9f, 0xe7, 0x78, 0x9d, 0x96, 0x19, 
    0xe7, 0x18, 0x96, 0x40, 0x05, 0x09, 0xe7, 0x08, 0x80, 0x06, 0xc3, 0x79, 0x9d, 0xe7, 0x78, 0x9f, 
    0x12, 0x0b, 0xf8, 0xfe, 0x78, 0xa4, 0xa6, 0x06, 0x08, 0xa6, 0x07, 0xd3, 0x79, 0xab, 0xe7, 0x78, 
    0xa9, 0x96, 0x19, 0xe7, 0x18, 0x96, 0x40, 0x05, 0x09, 0xe7, 0x08, 0x80, 0x06, 0xc3, 0x79, 0xa9, 
    0xe7, 0x78, 0xab, 0x12, 0x0b, 0xf8, 0xfe, 0x78, 0xb0, 0xa6, 0x06, 0x08, 0xa6, 0x07, 0x79, 0xb4, 
    0xe7, 0xd3, 0x78, 0xb3, 0x96, 0x40, 0x05, 0xe7, 0x96, 0xff, 0x80, 0x08, 0xc3, 0x79, 0xb3, 0xe7, 
    0x78, 0xb4, 0x96, 0xff, 0x78, 0xb7, 0xa6, 0x07, 0xe5, 0x1f, 0x64, 0x02, 0x60, 0x03, 0x02, 0x02, 
    0xef, 0x90, 0x30, 0x24, 0x74, 0x0f, 0xf0, 0x90, 0x0e, 0x8a, 0xe4, 0x93, 0xff, 0x18, 0xe6, 0xc3, 
    0x9f, 0x40, 0x03, 0x02, 0x03, 0x09, 0x90, 0x30, 0x24, 0x74, 0x0e, 0xf0, 0x78, 0xa2, 0x12, 0x0b, 
    0xd9, 0x12, 0x0b, 0xa2, 0x90, 0x0e, 0x87, 0x12, 0x0b, 0xb7, 0x78, 0x9e, 0x12, 0x0b, 0xe8, 0x7b, 
    0x04, 0x12, 0x0b, 0x90, 0xc3, 0x12, 0x07, 0x0e, 0x50, 0x6f, 0x90, 0x0e, 0x8b, 0xe4, 0x93, 0xff, 
    0x78, 0xb7, 0xe6, 0x9f, 0x40, 0x07, 0x90, 0x30, 0x24, 0x74, 0x0a, 0x80, 0x16, 0x90, 0x0e, 0x89, 
    0xe4, 0x93, 0xff, 0xd3, 0x78, 0xa7, 0xe6, 0x9f, 0x18, 0xe6, 0x94, 0x00, 0x40, 0x09, 0x90, 0x30, 
    0x24, 0x74, 0x0b, 0xf0, 0x75, 0x1f, 0x05, 0x78, 0xae, 0x12, 0x0b, 0xd9, 0x12, 0x0b, 0xa2, 0x90, 
    0x0e, 0x88, 0x12, 0x0b, 0xb7, 0x78, 0xa8, 0x12, 0x0b, 0xe8, 0x7b, 0x40, 0x12, 0x0b, 0x90, 0xd3, 
    0x12, 0x07, 0x0e, 0x40, 0x24, 0x90, 0x30, 0x24, 0x74, 0x0c, 0xf0, 0x75, 0x1f, 0x05, 0x22, 0x90, 
    0x30, 0x24, 0x74, 0x01, 0xf0, 0xe5, 0x1f, 0xb4, 0x05, 0x0f, 0xd2, 0x01, 0xc2, 0x02, 0xe4, 0xf5, 
    0x1f, 0xf5, 0x1e, 0xd2, 0x34, 0xd2, 0x32, 0xd2, 0x35, 0x22, 0xe5, 0x1f, 0x60, 0x03, 0x02, 0x03, 
    0x93, 0xf5, 0x1e, 0xd2, 0x34, 0x75, 0x34, 0xff, 0x75, 0x35, 0x0e, 0x75, 0x36, 0x55, 0x75, 0x37, 
    0x01, 0x12, 0x0d, 0x85, 0xe4, 0xff, 0xef, 0x25, 0xe0, 0x24, 0x4c, 0xf8, 0xe4, 0xf6, 0x08, 0xf6, 
    0x0f, 0xbf, 0x34, 0xf2, 0x90, 0x0e, 0x8c, 0xe4, 0x93, 0xff, 0xe5, 0x49, 0xc3, 0x9f, 0x50, 0x04, 
    0x7f, 0x05, 0x80, 0x02, 0x7f, 0xfb, 0x78, 0xbb, 0xa6, 0x07, 0x12, 0x0e, 0xbc, 0x40, 0x04, 0x7f, 
    0x03, 0x80, 0x02, 0x7f, 0x30, 0x78, 0xba, 0xa6, 0x07, 0xe6, 0x18, 0xf6, 0x08, 0xe6, 0x78, 0xb7, 
    0xf6, 0x78, 0xba, 0xe6, 0x78, 0xb8, 0xf6, 0x78, 0xbd, 0x76, 0x33, 0xe4, 0x08, 0xf6, 0x78, 0xb6, 
    0x76, 0x01, 0x75, 0x48, 0x02, 0x78, 0xb4, 0xf6, 0x08, 0xf6, 0x74, 0xff, 0x78, 0xbf, 0xf6, 0x08, 
    0xf6, 0x75, 0x1f, 0x01, 0x78, 0xba, 0xe6, 0x75, 0xf0, 0x05, 0xa4, 0xf5, 0x49, 0x12, 0x0a, 0xfd, 
    0xc2, 0x36, 0x22, 0x78, 0xb6, 0xe6, 0xd3, 0x94, 0x00, 0x40, 0x02, 0x16, 0x22, 0xe5, 0x1f, 0xb4, 
    0x05, 0x23, 0xe4, 0xf5, 0x1f, 0xc2, 0x01, 0x78, 0xb4, 0xe6, 0xfe, 0x08, 0xe6, 0xff, 0x78, 0x4c, 
    0xa6, 0x06, 0x08, 0xa6, 0x07, 0xa2, 0x36, 0xe4, 0x33, 0xf5, 0x3c, 0x90, 0x30, 0x28, 0xf0, 0x75, 
    0x1e, 0x10, 0xd2, 0x34, 0x22, 0xe5, 0x49, 0x75, 0xf0, 0x05, 0x84, 0x78, 0xba, 0xf6, 0x90, 0x0e, 
    0x85, 0xe4, 0x93, 0xff, 0x25, 0xe0, 0x24, 0x0a, 0xf8, 0xe6, 0xfc, 0x08, 0xe6, 0xfd, 0x78, 0xba, 
    0xe6, 0x25, 0xe0, 0x24, 0x4c, 0xf8, 0xa6, 0x04, 0x08, 0xa6, 0x05, 0xef, 0x12, 0x0e, 0xc3, 0xd3, 
    0x78, 0xb5, 0x96, 0xee, 0x18, 0x96, 0x40, 0x0d, 0x78, 0xba, 0xe6, 0x78, 0xb7, 0xf6, 0x78, 0xb4, 
    0xa6, 0x06, 0x08, 0xa6, 0x07, 0x90, 0x0e, 0x85, 0xe4, 0x93, 0x12, 0x0e, 0xc3, 0xc3, 0x78, 0xc0, 
    0x96, 0xee, 0x18, 0x96, 0x50, 0x0d, 0x78, 0xba, 0xe6, 0x78, 0xb8, 0xf6, 0x78, 0xbf, 0xa6, 0x06, 
    0x08, 0xa6, 0x07, 0x78, 0xb4, 0xe6, 0xfe, 0x08, 0xe6, 0xc3, 0x78, 0xc0, 0x96, 0xff, 0xee, 0x18, 
    0x96, 0x78, 0xc1, 0xf6, 0x08, 0xa6, 0x07, 0x90, 0x0e, 0x8e, 0xe4, 0x18, 0x12, 0x0e, 0xa1, 0x40, 
    0x02, 0xd2, 0x36, 0x78, 0xba, 0xe6, 0x08, 0x26, 0x08, 0xf6, 0xe5, 0x1f, 0x64, 0x01, 0x70, 0x4a, 
    0xe6, 0xc3, 0x78, 0xbe, 0x12, 0x0e, 0x97, 0x40, 0x05, 0x12, 0x0e, 0x92, 0x40, 0x39, 0x12, 0x0e, 
    0xba, 0x40, 0x04, 0x7f, 0xfe, 0x80, 0x02, 0x7f, 0x02, 0x78, 0xbb, 0xa6, 0x07, 0x78, 0xb7, 0xe6, 
    0x24, 0x03, 0x78, 0xbd, 0xf6, 0x78, 0xb7, 0xe6, 0x24, 0xfd, 0x78, 0xbe, 0xf6, 0x12, 0x0e, 0xba, 
    0x40, 0x06, 0x78, 0xbe, 0xe6, 0xff, 0x80, 0x04, 0x78, 0xbd, 0xe6, 0xff, 0x78, 0xbc, 0xa6, 0x07, 
    0x75, 0x1f, 0x02, 0x78, 0xb6, 0x76, 0x01, 0x02, 0x05, 0x59, 0xe5, 0x1f, 0x64, 0x02, 0x60, 0x03, 
    0x02, 0x05, 0x39, 0x78, 0xbc, 0xe6, 0xff, 0xc3, 0x78, 0xbe, 0x12, 0x0e, 0x98, 0x40, 0x08, 0x12, 
    0x0e, 0x92, 0x50, 0x03, 0x02, 0x05, 0x37, 0x12, 0x0e, 0xba, 0x40, 0x04, 0x7f, 0xff, 0x80, 0x02, 
    0x7f, 0x01, 0x78, 0xbb, 0xa6, 0x07, 0x78, 0xb7, 0xe6, 0x04, 0x78, 0xbd, 0xf6, 0x78, 0xb7, 0xe6, 
    0x14, 0x78, 0xbe, 0xf6, 0x18, 0x12, 0x0e, 0xbc, 0x40, 0x04, 0xe6, 0xff, 0x80, 0x02, 0x7f, 0x00, 
    0x78, 0xbd, 0xa6, 0x07, 0xd3, 0x08, 0xe6, 0x64, 0x80, 0x94, 0x80, 0x40, 0x04, 0xe6, 0xff, 0x80, 
    0x02, 0x7f, 0x00, 0x78, 0xbe, 0xa6, 0x07, 0xc3, 0x18, 0xe6, 0x64, 0x80, 0x94, 0xb3, 0x50, 0x04, 
    0xe6, 0xff, 0x80, 0x02, 0x7f, 0x33, 0x78, 0xbd, 0xa6, 0x07, 0xc3, 0x08, 0xe6, 0x64, 0x80, 0x94, 
    0xb3, 0x50, 0x04, 0xe6, 0xff, 0x80, 0x02, 0x7f, 0x33, 0x78, 0xbe, 0xa6, 0x07, 0x12, 0x0e, 0xba, 
    0x40, 0x06, 0x78, 0xbe, 0xe6, 0xff, 0x80, 0x04, 0x78, 0xbd, 0xe6, 0xff, 0x78, 0xbc, 0xa6, 0x07, 
    0x75, 0x1f, 0x03, 0x78, 0xb6, 0x76, 0x01, 0x80, 0x20, 0xe5, 0x1f, 0x64, 0x03, 0x70, 0x26, 0x78, 
    0xbc, 0xe6, 0xff, 0xc3, 0x78, 0xbe, 0x12, 0x0e, 0x98, 0x40, 0x05, 0x12, 0x0e, 0x92, 0x40, 0x09, 
    0x78, 0xb7, 0xe6, 0x78, 0xbc, 0xf6, 0x75, 0x1f, 0x04, 0x78, 0xbc, 0xe6, 0x75, 0xf0, 0x05, 0xa4, 
    0xf5, 0x49, 0x02, 0x0a, 0xfd, 0xe5, 0x1f, 0xb4, 0x04, 0x1f, 0x90, 0x0e, 0x8d, 0xe4, 0x78, 0xc1, 
    0x12, 0x0e, 0xa1, 0x40, 0x02, 0xd2, 0x36, 0x75, 0x1f, 0x05, 0x75, 0x34, 0xff, 0x75, 0x35, 0x0e, 
    0x75, 0x36, 0x59, 0x75, 0x37, 0x01, 0x12, 0x0d, 0x85, 0x22, 0xef, 0x8d, 0xf0, 0xa4, 0xa8, 0xf0, 
    0xcf, 0x8c, 0xf0, 0xa4, 0x28, 0xce, 0x8d, 0xf0, 0xa4, 0x2e, 0xfe, 0x22, 0xbc, 0x00, 0x0b, 0xbe, 
    0x00, 0x29, 0xef, 0x8d, 0xf0, 0x84, 0xff, 0xad, 0xf0, 0x22, 0xe4, 0xcc, 0xf8, 0x75, 0xf0, 0x08, 
    0xef, 0x2f, 0xff, 0xee, 0x33, 0xfe, 0xec, 0x33, 0xfc, 0xee, 0x9d, 0xec, 0x98, 0x40, 0x05, 0xfc, 
    0xee, 0x9d, 0xfe, 0x0f, 0xd5, 0xf0, 0xe9, 0xe4, 0xce, 0xfd, 0x22, 0xed, 0xf8, 0xf5, 0xf0, 0xee, 
    0x84, 0x20, 0xd2, 0x1c, 0xfe, 0xad, 0xf0, 0x75, 0xf0, 0x08, 0xef, 0x2f, 0xff, 0xed, 0x33, 0xfd, 
    0x40, 0x07, 0x98, 0x50, 0x06, 0xd5, 0xf0, 0xf2, 0x22, 0xc3, 0x98, 0xfd, 0x0f, 0xd5, 0xf0, 0xea, 
    0x22, 0xe8, 0x8f, 0xf0, 0xa4, 0xcc, 0x8b, 0xf0, 0xa4, 0x2c, 0xfc, 0xe9, 0x8e, 0xf0, 0xa4, 0x2c, 
    0xfc, 0x8a, 0xf0, 0xed, 0xa4, 0x2c, 0xfc, 0xea, 0x8e, 0xf0, 0xa4, 0xcd, 0xa8, 0xf0, 0x8b, 0xf0, 
    0xa4, 0x2d, 0xcc, 0x38, 0x25, 0xf0, 0xfd, 0xe9, 0x8f, 0xf0, 0xa4, 0x2c, 0xcd, 0x35, 0xf0, 0xfc, 
    0xeb, 0x8e, 0xf0, 0xa4, 0xfe, 0xa9, 0xf0, 0xeb, 0x8f, 0xf0, 0xa4, 0xcf, 0xc5, 0xf0, 0x2e, 0xcd, 
    0x39, 0xfe, 0xe4, 0x3c, 0xfc, 0xea, 0xa4, 0x2d, 0xce, 0x35, 0xf0, 0xfd, 0xe4, 0x3c, 0xfc, 0x22, 
    0x75, 0xf0, 0x08, 0x75, 0x82, 0x00, 0xef, 0x2f, 0xff, 0xee, 0x33, 0xfe, 0xcd, 0x33, 0xcd, 0xcc, 
    0x33, 0xcc, 0xc5, 0x82, 0x33, 0xc5, 0x82, 0x9b, 0xed, 0x9a, 0xec, 0x99, 0xe5, 0x82, 0x98, 0x40, 
    0x0c, 0xf5, 0x82, 0xee, 0x9b, 0xfe, 0xed, 0x9a, 0xfd, 0xec, 0x99, 0xfc, 0x0f, 0xd5, 0xf0, 0xd6, 
    0xe4, 0xce, 0xfb, 0xe4, 0xcd, 0xfa, 0xe4, 0xcc, 0xf9, 0xa8, 0x82, 0x22, 0xb8, 0x00, 0xc1, 0xb9, 
    0x00, 0x59, 0xba, 0x00, 0x2d, 0xec, 0x8b, 0xf0, 0x84, 0xcf, 0xce, 0xcd, 0xfc, 0xe5, 0xf0, 0xcb, 
    0xf9, 0x78, 0x18, 0xef, 0x2f, 0xff, 0xee, 0x33, 0xfe, 0xed, 0x33, 0xfd, 0xec, 0x33, 0xfc, 0xeb, 
    0x33, 0xfb, 0x10, 0xd7, 0x03, 0x99, 0x40, 0x04, 0xeb, 0x99, 0xfb, 0x0f, 0xd8, 0xe5, 0xe4, 0xf9, 
    0xfa, 0x22, 0x78, 0x18, 0xef, 0x2f, 0xff, 0xee, 0x33, 0xfe, 0xed, 0x33, 0xfd, 0xec, 0x33, 0xfc, 
    0xc9, 0x33, 0xc9, 0x10, 0xd7, 0x05, 0x9b, 0xe9, 0x9a, 0x40, 0x07, 0xec, 0x9b, 0xfc, 0xe9, 0x9a, 
    0xf9, 0x0f, 0xd8, 0xe0, 0xe4, 0xc9, 0xfa, 0xe4, 0xcc, 0xfb, 0x22, 0x75, 0xf0, 0x10, 0xef, 0x2f, 
    0xff, 0xee, 0x33, 0xfe, 0xed, 0x33, 0xfd, 0xcc, 0x33, 0xcc, 0xc8, 0x33, 0xc8, 0x10, 0xd7, 0x07, 
    0x9b, 0xec, 0x9a, 0xe8, 0x99, 0x40, 0x0a, 0xed, 0x9b, 0xfd, 0xec, 0x9a, 0xfc, 0xe8, 0x99, 0xf8, 
    0x0f, 0xd5, 0xf0, 0xda, 0xe4, 0xcd, 0xfb, 0xe4, 0xcc, 0xfa, 0xe4, 0xc8, 0xf9, 0x22, 0xeb, 0x9f, 
    0xf5, 0xf0, 0xea, 0x9e, 0x42, 0xf0, 0xe9, 0x9d, 0x42, 0xf0, 0xe8, 0x9c, 0x45, 0xf0, 0x22, 0xe8, 
    0x60, 0x0f, 0xef, 0xc3, 0x33, 0xff, 0xee, 0x33, 0xfe, 0xed, 0x33, 0xfd, 0xec, 0x33, 0xfc, 0xd8, 
    0xf1, 0x22, 0xe4, 0x93, 0xfc, 0x74, 0x01, 0x93, 0xfd, 0x74, 0x02, 0x93, 0xfe, 0x74, 0x03, 0x93, 
    0xff, 0x22, 0xe6, 0xfb, 0x08, 0xe6, 0xf9, 0x08, 0xe6, 0xfa, 0x08, 0xe6, 0xcb, 0xf8, 0x22, 0xec, 
    0xf6, 0x08, 0xed, 0xf6, 0x08, 0xee, 0xf6, 0x08, 0xef, 0xf6, 0x22, 0xa4, 0x25, 0x82, 0xf5, 0x82, 
    0xe5, 0xf0, 0x35, 0x83, 0xf5, 0x83, 0x22, 0xd0, 0x83, 0xd0, 0x82, 0xf8, 0xe4, 0x93, 0x70, 0x12, 
    0x74, 0x01, 0x93, 0x70, 0x0d, 0xa3, 0xa3, 0x93, 0xf8, 0x74, 0x01, 0x93, 0xf5, 0x82, 0x88, 0x83, 
    0xe4, 0x73, 0x74, 0x02, 0x93, 0x68, 0x60, 0xef, 0xa3, 0xa3, 0xa3, 0x80, 0xdf, 0x90, 0x38, 0x04, 
    0x78, 0x50, 0x12, 0x0c, 0x7f, 0x90, 0x38, 0x00, 0xe0, 0xfe, 0xa3, 0xe0, 0xfd, 0xed, 0xff, 0xc3, 
    0x12, 0x0c, 0x38, 0x90, 0x38, 0x10, 0x12, 0x0c, 0x2c, 0x90, 0x38, 0x06, 0x78, 0x52, 0x12, 0x0c, 
    0x7f, 0x90, 0x38, 0x02, 0xe0, 0xfe, 0xa3, 0xe0, 0xfd, 0xed, 0xff, 0xc3, 0x12, 0x0c, 0x38, 0x90, 
    0x38, 0x12, 0x12, 0x0c, 0x2c, 0xa3, 0xe0, 0xb4, 0x31, 0x07, 0x78, 0x50, 0x79, 0x50, 0x12, 0x0c, 
    0x95, 0x90, 0x38, 0x14, 0xe0, 0xb4, 0x71, 0x15, 0x78, 0x50, 0xe6, 0xfe, 0x08, 0xe6, 0x78, 0x02, 
    0xce, 0xc3, 0x13, 0xce, 0x13, 0xd8, 0xf9, 0x79, 0x51, 0xf7, 0xee, 0x19, 0xf7, 0x90, 0x38, 0x15, 
    0xe0, 0xb4, 0x31, 0x07, 0x78, 0x52, 0x79, 0x52, 0x12, 0x0c, 0x95, 0x90, 0x38, 0x15, 0xe0, 0xb4, 
    0x71, 0x15, 0x78, 0x52, 0xe6, 0xfe, 0x08, 0xe6, 0x78, 0x02, 0xce, 0xc3, 0x13, 0xce, 0x13, 0xd8, 
    0xf9, 0x79, 0x53, 0xf7, 0xee, 0x19, 0xf7, 0x79, 0x50, 0x12, 0x0c, 0x67, 0x09, 0x12, 0x0c, 0x67, 
    0xaf, 0x45, 0x12, 0x0c, 0x1d, 0x7d, 0x50, 0x12, 0x05, 0x9c, 0x78, 0x58, 0xa6, 0x06, 0x08, 0xa6, 
    0x07, 0xaf, 0x43, 0x12, 0x0c, 0x1d, 0x7d, 0x50, 0x12, 0x05, 0x9c, 0x78, 0x54, 0xa6, 0x06, 0x08, 
    0xa6, 0x07, 0xaf, 0x46, 0x78, 0x52, 0x12, 0x0c, 0x1f, 0x7d, 0x3c, 0x12, 0x05, 0x9c, 0x78, 0x5a, 
    0xa6, 0x06, 0x08, 0xa6, 0x07, 0xaf, 0x44, 0x7e, 0x00, 0x78, 0x52, 0x12, 0x0c, 0x21, 0x7d, 0x3c, 
    0x12, 0x05, 0x9c, 0x78, 0x56, 0xa6, 0x06, 0x08, 0xa6, 0x07, 0xc3, 0x78, 0x59, 0xe6, 0x94, 0x08, 
    0x18, 0xe6, 0x94, 0x00, 0x50, 0x05, 0x76, 0x00, 0x08, 0x76, 0x08, 0xc3, 0x78, 0x5b, 0xe6, 0x94, 
    0x08, 0x18, 0xe6, 0x94, 0x00, 0x50, 0x05, 0x76, 0x00, 0x08, 0x76, 0x08, 0x78, 0x58, 0x12, 0x0c, 
    0x54, 0xff, 0xd3, 0x78, 0x55, 0xe6, 0x9f, 0x18, 0xe6, 0x9e, 0x40, 0x0e, 0x78, 0x58, 0xe6, 0x13, 
    0xfe, 0x08, 0xe6, 0x78, 0x55, 0x12, 0x0c, 0x8a, 0x80, 0x04, 0x7e, 0x00, 0x7f, 0x00, 0x78, 0x5c, 
    0x12, 0x0c, 0x4c, 0xff, 0xd3, 0x78, 0x57, 0xe6, 0x9f, 0x18, 0xe6, 0x9e, 0x40, 0x0e, 0x78, 0x5a, 
    0xe6, 0x13, 0xfe, 0x08, 0xe6, 0x78, 0x57, 0x12, 0x0c, 0x8a, 0x80, 0x04, 0x7e, 0x00, 0x7f, 0x00, 
    0xe4, 0xfc, 0xfd, 0x78, 0x60, 0x12, 0x07, 0x4f, 0x78, 0x58, 0x12, 0x0c, 0x54, 0x78, 0x55, 0x26, 
    0xff, 0xee, 0x18, 0x36, 0xfe, 0x78, 0x64, 0x12, 0x0c, 0x4c, 0x78, 0x57, 0x26, 0xff, 0xee, 0x18, 
    0x36, 0xfe, 0xe4, 0xfc, 0xfd, 0x78, 0x68, 0x12, 0x07, 0x4f, 0x12, 0x0c, 0x5c, 0x78, 0x64, 0x12, 
    0x07, 0x42, 0xd3, 0x12, 0x07, 0x0e, 0x40, 0x08, 0x12, 0x0c, 0x5c, 0x78, 0x64, 0x12, 0x07, 0x4f, 
    0x78, 0x52, 0x12, 0x0c, 0x5e, 0x78, 0x68, 0x12, 0x07, 0x42, 0xd3, 0x12, 0x07, 0x0e, 0x40, 0x0a, 
    0x78, 0x52, 0x12, 0x0c, 0x5e, 0x78, 0x68, 0x12, 0x07, 0x4f, 0xe4, 0xfd, 0x78, 0x5f, 0x12, 0x0c, 
    0x77, 0x24, 0x01, 0x12, 0x0c, 0x40, 0x78, 0x63, 0x12, 0x0c, 0x77, 0x24, 0x02, 0x12, 0x0c, 0x40, 
    0x78, 0x67, 0x12, 0x0c, 0x77, 0x24, 0x03, 0x12, 0x0c, 0x40, 0x78, 0x6b, 0x12, 0x0c, 0x77, 0x24, 
    0x04, 0x12, 0x0c, 0x40, 0x0d, 0xbd, 0x05, 0xd4, 0xc2, 0x0e, 0xc2, 0x06, 0x22, 0x85, 0x08, 0x41, 
    0x90, 0x30, 0x24, 0xe0, 0xf5, 0x3d, 0xa3, 0xe0, 0xf5, 0x3e, 0xa3, 0xe0, 0xf5, 0x3f, 0xa3, 0xe0, 
    0xf5, 0x40, 0xa3, 0xe0, 0xf5, 0x3c, 0xd2, 0x33, 0xe5, 0x41, 0x12, 0x07, 0x67, 0x09, 0xb4, 0x03, 
    0x09, 0xb8, 0x04, 0x09, 0xbe, 0x05, 0x09, 0xc1, 0x06, 0x09, 0xc4, 0x07, 0x09, 0xcd, 0x08, 0x09, 
    0xde, 0x12, 0x09, 0xe0, 0x80, 0x09, 0xe5, 0x81, 0x0a, 0x43, 0x8f, 0x0a, 0x32, 0x90, 0x0a, 0x43, 
    0x91, 0x0a, 0x43, 0x92, 0x0a, 0x43, 0x93, 0x0a, 0x43, 0x94, 0x0a, 0x43, 0x98, 0x0a, 0x40, 0x9f, 
    0x00, 0x00, 0x0a, 0x5e, 0x12, 0x0e, 0xce, 0x22, 0x12, 0x0e, 0xce, 0xd2, 0x03, 0x22, 0xd2, 0x03, 
    0x22, 0xc2, 0x03, 0x22, 0xa2, 0x36, 0xe4, 0x33, 0xf5, 0x3c, 0x02, 0x0a, 0x43, 0xc2, 0x01, 0xc2, 
    0x02, 0xc2, 0x03, 0x12, 0x0d, 0x14, 0x75, 0x1e, 0x70, 0xd2, 0x34, 0x02, 0x0a, 0x43, 0x80, 0x4d, 
    0x12, 0x0f, 0x17, 0x80, 0x5e, 0x85, 0x3d, 0x43, 0x85, 0x3e, 0x44, 0xe5, 0x45, 0xc3, 0x13, 0xff, 
    0xe5, 0x43, 0xc3, 0x9f, 0x50, 0x02, 0x8f, 0x43, 0xe5, 0x46, 0xc3, 0x13, 0xff, 0xe5, 0x44, 0xc3, 
    0x9f, 0x50, 0x02, 0x8f, 0x44, 0xe5, 0x45, 0xc3, 0x13, 0xff, 0xfd, 0xe5, 0x43, 0x90, 0x0e, 0x7f, 
    0x12, 0x0e, 0xea, 0x40, 0x04, 0xee, 0x9f, 0xf5, 0x43, 0xe5, 0x46, 0xc3, 0x13, 0xff, 0xfd, 0xe5, 
    0x44, 0x90, 0x0e, 0x80, 0x12, 0x0e, 0xea, 0x40, 0x04, 0xee, 0x9f, 0xf5, 0x44, 0x12, 0x07, 0x8d, 
    0x80, 0x11, 0x85, 0x40, 0x46, 0x85, 0x3f, 0x45, 0x85, 0x3e, 0x44, 0x85, 0x3d, 0x43, 0x80, 0x03, 
    0x02, 0x07, 0x8d, 0x90, 0x30, 0x24, 0xe5, 0x3d, 0xf0, 0xa3, 0xe5, 0x3e, 0xf0, 0xa3, 0xe5, 0x3f, 
    0xf0, 0xa3, 0xe5, 0x40, 0xf0, 0xa3, 0xe5, 0x3c, 0xf0, 0x90, 0x30, 0x23, 0xe4, 0xf0, 0x22, 0xc0, 
    0xe0, 0xc0, 0x83, 0xc0, 0x82, 0xc0, 0xd0, 0x90, 0x3f, 0x0c, 0xe0, 0xf5, 0x32, 0xe5, 0x32, 0x30, 
    0xe3, 0x4c, 0x30, 0x35, 0x3e, 0x90, 0x60, 0x19, 0xe0, 0xf5, 0x0a, 0xa3, 0xe0, 0xf5, 0x0b, 0x90, 
    0x60, 0x1d, 0xe0, 0xf5, 0x14, 0xa3, 0xe0, 0xf5, 0x15, 0x30, 0x01, 0x06, 0x30, 0x32, 0x03, 0xd3, 
    0x80, 0x01, 0xc3, 0x92, 0x09, 0x30, 0x02, 0x06, 0x30, 0x32, 0x03, 0xd3, 0x80, 0x01, 0xc3, 0x92, 
    0x0a, 0x30, 0x32, 0x0c, 0x30, 0x03, 0x09, 0x20, 0x02, 0x06, 0x20, 0x01, 0x03, 0xd3, 0x80, 0x01, 
    0xc3, 0x92, 0x0b, 0x90, 0x30, 0x01, 0xe0, 0x44, 0x40, 0xf0, 0xe0, 0x54, 0xbf, 0xf0, 0xe5, 0x32, 
    0x30, 0xe1, 0x14, 0x30, 0x33, 0x11, 0x90, 0x30, 0x22, 0xe0, 0xf5, 0x08, 0xe4, 0xf0, 0x30, 0x00, 
    0x03, 0xd3, 0x80, 0x01, 0xc3, 0x92, 0x08, 0xe5, 0x32, 0x30, 0xe5, 0x12, 0x90, 0x56, 0xa1, 0xe0, 
    0xf5, 0x09, 0x30, 0x30, 0x09, 0x30, 0x05, 0x03, 0xd3, 0x80, 0x01, 0xc3, 0x92, 0x0d, 0x90, 0x3f, 
    0x0c, 0xe5, 0x32, 0xf0, 0xd0, 0xd0, 0xd0, 0x82, 0xd0, 0x83, 0xd0, 0xe0, 0x32, 0x90, 0x0e, 0x7d, 
    0xe4, 0x93, 0xfe, 0x74, 0x01, 0x93, 0xff, 0xc3, 0x90, 0x0e, 0x7b, 0x74, 0x01, 0x93, 0x9f, 0xff, 
    0xe4, 0x93, 0x9e, 0xfe, 0xe4, 0x8f, 0x3b, 0x8e, 0x3a, 0xf5, 0x39, 0xf5, 0x38, 0xab, 0x3b, 0xaa, 
    0x3a, 0xa9, 0x39, 0xa8, 0x38, 0xaf, 0x49, 0xfc, 0xfd, 0xfe, 0x12, 0x05, 0xf1, 0x12, 0x0e, 0xfc, 
    0xe4, 0x7b, 0xff, 0xfa, 0xf9, 0xf8, 0x12, 0x06, 0x7c, 0x12, 0x0e, 0xfc, 0x90, 0x0e, 0x69, 0xe4, 
    0x12, 0x0f, 0x11, 0x12, 0x0e, 0xfc, 0xe4, 0x85, 0x48, 0x37, 0xf5, 0x36, 0xf5, 0x35, 0xf5, 0x34, 
    0xaf, 0x37, 0xae, 0x36, 0xad, 0x35, 0xac, 0x34, 0xa3, 0x12, 0x0f, 0x11, 0x8f, 0x37, 0x8e, 0x36, 
    0x8d, 0x35, 0x8c, 0x34, 0xe5, 0x3b, 0x45, 0x37, 0xf5, 0x3b, 0xe5, 0x3a, 0x45, 0x36, 0xf5, 0x3a, 
    0xe5, 0x39, 0x45, 0x35, 0xf5, 0x39, 0xe5, 0x38, 0x45, 0x34, 0xf5, 0x38, 0xe4, 0xf5, 0x22, 0xf5, 
    0x23, 0x85, 0x3b, 0x31, 0x85, 0x3a, 0x30, 0x85, 0x39, 0x2f, 0x85, 0x38, 0x2e, 0x02, 0x0d, 0xc5, 
    0xad, 0x39, 0xac, 0x38, 0xfa, 0xf9, 0xf8, 0x12, 0x05, 0xf1, 0x8f, 0x3b, 0x8e, 0x3a, 0x8d, 0x39, 
    0x8c, 0x38, 0xab, 0x37, 0xaa, 0x36, 0xa9, 0x35, 0xa8, 0x34, 0x22, 0xef, 0x25, 0xe0, 0x24, 0x4c, 
    0xf8, 0xe6, 0xfc, 0x08, 0xe6, 0xfd, 0x22, 0x93, 0xff, 0xe4, 0xfc, 0xfd, 0xfe, 0x12, 0x05, 0xf1, 
    0x8f, 0x37, 0x8e, 0x36, 0x8d, 0x35, 0x8c, 0x34, 0x22, 0xf9, 0xc3, 0xe6, 0x97, 0x18, 0xe6, 0x19, 
    0x97, 0x22, 0xff, 0xa6, 0x06, 0x08, 0xa6, 0x07, 0x22, 0xe6, 0xfe, 0x08, 0xe6, 0xff, 0xe4, 0x8f, 
    0x37, 0x8e, 0x36, 0xf5, 0x35, 0xf5, 0x34, 0x22, 0xe6, 0xfe, 0x08, 0xe6, 0xff, 0xe4, 0x8f, 0x3b, 
    0x8e, 0x3a, 0xf5, 0x39, 0xf5, 0x38, 0x22, 0xe7, 0x96, 0xff, 0x19, 0xe7, 0x18, 0x96, 0x22, 0xff, 
    0xa6, 0x06, 0x08, 0xa6, 0x07, 0x78, 0x6c, 0xe6, 0xfe, 0x08, 0xe6, 0x22, 0x78, 0x4c, 0xe6, 0xfe, 
    0x08, 0xe6, 0x22, 0x78, 0xa7, 0xef, 0x26, 0xf6, 0x18, 0xe4, 0x36, 0xf6, 0x22, 0x78, 0x50, 0x7e, 
    0x00, 0xe6, 0xfc, 0x08, 0xe6, 0xfd, 0x12, 0x05, 0x8a, 0x7c, 0x00, 0x22, 0xe0, 0xa3, 0xe0, 0x75, 
    0xf0, 0x02, 0xa4, 0xff, 0xae, 0xf0, 0xc3, 0x08, 0xe6, 0x9f, 0xf6, 0x18, 0xe6, 0x9e, 0xf6, 0x22, 
    0xff, 0xe5, 0xf0, 0x34, 0x60, 0x8f, 0x82, 0xf5, 0x83, 0xec, 0xf0, 0x22, 0xe4, 0xfc, 0xfd, 0x12, 
    0x07, 0x4f, 0x78, 0x5a, 0xe6, 0xc3, 0x13, 0xfe, 0x08, 0xe6, 0x13, 0x22, 0x78, 0x50, 0xe6, 0xfe, 
    0x08, 0xe6, 0xff, 0xe4, 0xfc, 0xfd, 0x22, 0xe7, 0xc4, 0xf8, 0x54, 0xf0, 0xc8, 0x68, 0xf7, 0x09, 
    0xe7, 0xc4, 0x54, 0x0f, 0x48, 0xf7, 0x22, 0xe6, 0xfc, 0xed, 0x75, 0xf0, 0x04, 0xa4, 0x22, 0xe0, 
    0xfe, 0xa3, 0xe0, 0xfd, 0xee, 0xf6, 0xed, 0x08, 0xf6, 0x22, 0x13, 0xff, 0xc3, 0xe6, 0x9f, 0xff, 
    0x18, 0xe6, 0x9e, 0xfe, 0x22, 0xe6, 0xc3, 0x13, 0xf7, 0x08, 0xe6, 0x13, 0x09, 0xf7, 0x22, 0x75, 
    0x89, 0x03, 0x75, 0xa8, 0x01, 0x75, 0xb8, 0x04, 0x75, 0x34, 0xff, 0x75, 0x35, 0x0e, 0x75, 0x36, 
    0x15, 0x75, 0x37, 0x0d, 0x12, 0x0d, 0x85, 0x12, 0x00, 0x09, 0x12, 0x0f, 0x17, 0x12, 0x00, 0x06, 
    0xd2, 0x00, 0xd2, 0x33, 0xd2, 0xaf, 0x75, 0x34, 0xff, 0x75, 0x35, 0x0e, 0x75, 0x36, 0x49, 0x75, 
    0x37, 0x03, 0x12, 0x0d, 0x85, 0x30, 0x08, 0x09, 0xc2, 0x33, 0x12, 0x09, 0x5d, 0xc2, 0x08, 0xd2, 
    0x33, 0x30, 0x0b, 0x09, 0xc2, 0x35, 0x12, 0x00, 0x0e, 0xc2, 0x0b, 0xd2, 0x35, 0x30, 0x09, 0x09, 
    0xc2, 0x35, 0x12, 0x03, 0x0a, 0xc2, 0x09, 0xd2, 0x35, 0x30, 0x0e, 0x03, 0x12, 0x07, 0x8d, 0x30, 
    0x34, 0xd3, 0x90, 0x30, 0x29, 0xe5, 0x1e, 0xf0, 0xb4, 0x10, 0x05, 0x90, 0x30, 0x23, 0xe4, 0xf0, 
    0xc2, 0x34, 0x80, 0xc1, 0xe4, 0xf5, 0x49, 0x90, 0x0e, 0x77, 0x93, 0xff, 0xe4, 0x8f, 0x37, 0xf5, 
    0x36, 0xf5, 0x35, 0xf5, 0x34, 0xaf, 0x37, 0xae, 0x36, 0xad, 0x35, 0xac, 0x34, 0x90, 0x0e, 0x6a, 
    0x12, 0x0f, 0x11, 0x8f, 0x37, 0x8e, 0x36, 0x8d, 0x35, 0x8c, 0x34, 0x90, 0x0e, 0x72, 0x12, 0x07, 
    0x32, 0xef, 0x45, 0x37, 0xf5, 0x37, 0xee, 0x45, 0x36, 0xf5, 0x36, 0xed, 0x45, 0x35, 0xf5, 0x35, 
    0xec, 0x45, 0x34, 0xf5, 0x34, 0xe4, 0xf5, 0x22, 0xf5, 0x23, 0x85, 0x37, 0x31, 0x85, 0x36, 0x30, 
    0x85, 0x35, 0x2f, 0x85, 0x34, 0x2e, 0x12, 0x0d, 0xc5, 0xe4, 0xf5, 0x22, 0xf5, 0x23, 0x90, 0x0e, 
    0x72, 0x12, 0x0f, 0x05, 0x12, 0x0d, 0xc5, 0xe4, 0xf5, 0x22, 0xf5, 0x23, 0x90, 0x0e, 0x6e, 0x12, 
    0x0f, 0x05, 0x02, 0x0d, 0xc5, 0xae, 0x35, 0xaf, 0x36, 0xe4, 0xfd, 0xed, 0xc3, 0x95, 0x37, 0x50, 
    0x33, 0x12, 0x0f, 0x52, 0xe4, 0x93, 0xf5, 0x38, 0x74, 0x01, 0x93, 0xf5, 0x39, 0x45, 0x38, 0x60, 
    0x23, 0x85, 0x39, 0x82, 0x85, 0x38, 0x83, 0xe0, 0xfc, 0x12, 0x0f, 0x52, 0x74, 0x03, 0x93, 0x52, 
    0x04, 0x12, 0x0f, 0x52, 0x74, 0x02, 0x93, 0x42, 0x04, 0x85, 0x39, 0x82, 0x85, 0x38, 0x83, 0xec, 
    0xf0, 0x0d, 0x80, 0xc7, 0x22, 0xa2, 0xaf, 0x92, 0x31, 0xc2, 0xaf, 0xe5, 0x23, 0x45, 0x22, 0x90, 
    0x0e, 0x5d, 0x60, 0x0b, 0x12, 0x0f, 0x47, 0xe0, 0xf5, 0x2c, 0xe0, 0xf5, 0x2d, 0x80, 0x0f, 0x12, 
    0x0f, 0x47, 0xe5, 0x30, 0xf0, 0x90, 0x0e, 0x5f, 0x12, 0x0f, 0x47, 0xe5, 0x31, 0xf0, 0xa2, 0x31, 
    0x92, 0xaf, 0x22, 0x78, 0x7f, 0xe4, 0xf6, 0xd8, 0xfd, 0x75, 0x81, 0xcb, 0x02, 0x0c, 0x9f, 0x00, 
    0x11, 0x05, 0x25, 0x16, 0x33, 0x02, 0x50, 0x72, 0x6f, 0x66, 0x69, 0x74, 0x20, 0x20, 0x14, 0x00, 
    0x10, 0x00, 0x56, 0x40, 0x1a, 0x30, 0x29, 0x7e, 0x00, 0x30, 0x04, 0x20, 0xdf, 0x30, 0x05, 0x40, 
    0xbf, 0x50, 0x03, 0x00, 0xfd, 0x50, 0x27, 0x01, 0xfe, 0x60, 0x00, 0x11, 0x00, 0x3f, 0x05, 0x30, 
    0x00, 0x3f, 0x06, 0x22, 0x00, 0x3f, 0x01, 0x2a, 0x00, 0x3f, 0x02, 0x00, 0x00, 0x36, 0x06, 0x07, 
    0x00, 0x3f, 0x0b, 0x0f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x30, 0x01, 0x40, 0xbf, 0x30, 0x01, 0x00, 
    0xbf, 0x30, 0x29, 0x70, 0x00, 0x3a, 0x00, 0x00, 0xff, 0x3a, 0x00, 0x00, 0xff, 0x36, 0x03, 0x36, 
    0x02, 0x41, 0x44, 0x58, 0x20, 0x18, 0x10, 0x0a, 0x04, 0x04, 0x00, 0x03, 0xff, 0x64, 0x00, 0x00, 
    0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x04, 0x06, 0x00, 0x03, 0x98, 0x00, 0xcc, 0x50, 
    0x3c, 0x28, 0x1e, 0x0c, 0x0c, 0x00, 0x00, 0x10, 0x0c, 0x10, 0x04, 0x0c, 0x6e, 0x06, 0x05, 0x00, 
    0xa5, 0x5a, 0x78, 0xbc, 0xe6, 0xd3, 0x08, 0xff, 0xe6, 0x64, 0x80, 0xf8, 0xef, 0x64, 0x80, 0x98, 
    0x22, 0x93, 0xff, 0x7e, 0x00, 0xe6, 0xfc, 0x08, 0xe6, 0xfd, 0x12, 0x05, 0x8a, 0x78, 0xbf, 0xe6, 
    0xfc, 0x08, 0xe6, 0xfd, 0xd3, 0xef, 0x9d, 0xee, 0x9c, 0x22, 0x78, 0xbb, 0xd3, 0xe6, 0x64, 0x80, 
    0x94, 0x80, 0x22, 0x25, 0xe0, 0x24, 0x0a, 0xf8, 0xe6, 0xfe, 0x08, 0xe6, 0xff, 0x22, 0xd2, 0x01, 
    0xc2, 0x02, 0xe4, 0xf5, 0x1f, 0xf5, 0x1e, 0xd2, 0x34, 0xd2, 0x32, 0xd2, 0x35, 0xd2, 0x01, 0xc2, 
    0x02, 0xf5, 0x1f, 0xf5, 0x1e, 0xd2, 0x34, 0xd2, 0x32, 0x22, 0x2d, 0xfd, 0xe4, 0x33, 0xfc, 0xe4, 
    0x93, 0xfe, 0xfb, 0xd3, 0xed, 0x9b, 0x74, 0x80, 0xf8, 0x6c, 0x98, 0x22, 0x8f, 0x3b, 0x8e, 0x3a, 
    0x8d, 0x39, 0x8c, 0x38, 0x22, 0x12, 0x07, 0x32, 0x8f, 0x31, 0x8e, 0x30, 0x8d, 0x2f, 0x8c, 0x2e, 
    0x22, 0x93, 0xf9, 0xf8, 0x02, 0x07, 0x1f, 0x90, 0x0e, 0x81, 0x12, 0x07, 0x32, 0x8f, 0x46, 0x8e, 
    0x45, 0x8d, 0x44, 0x8c, 0x43, 0xd2, 0x06, 0x30, 0x06, 0x03, 0xd3, 0x80, 0x01, 0xc3, 0x92, 0x0e, 
    0x22, 0xc0, 0xe0, 0xc0, 0x83, 0xc0, 0x82, 0x90, 0x3f, 0x0d, 0xe0, 0xf5, 0x33, 0xe5, 0x33, 0xf0, 
    0xd0, 0x82, 0xd0, 0x83, 0xd0, 0xe0, 0x32, 0xe4, 0x93, 0xfe, 0x74, 0x01, 0x93, 0xf5, 0x82, 0x8e, 
    0x83, 0x22, 0x8f, 0x82, 0x8e, 0x83, 0x75, 0xf0, 0x04, 0xed, 0x02, 0x07, 0x5b,  
};

void OV5640_FOCUS_AD5820_Init(void)
{
    u8  state=0x8F;
    u32 iteration = 100;
    u16 totalCnt = 0; 

    CAMERA_DEBUG("OV5640_FOCUS_AD5820_Init\n");     

    OV5640_WriteReg(0x3000, 0x20);
    totalCnt = sizeof(OV5640_AF_FW); 
    CAMERA_DEBUG("Total Count = %d\n", totalCnt);

//  写入自动对焦固件 Brust mode
    OV5640_WriteFW(OV5640_AF_FW,totalCnt);

    OV5640_WriteReg(0x3022, 0x00);
    OV5640_WriteReg(0x3023, 0x00);
    OV5640_WriteReg(0x3024, 0x00);
    OV5640_WriteReg(0x3025, 0x00);
    OV5640_WriteReg(0x3026, 0x00);
    OV5640_WriteReg(0x3027, 0x00);
    OV5640_WriteReg(0x3028, 0x00);
    OV5640_WriteReg(0x3029, 0xFF);
    OV5640_WriteReg(0x3000, 0x00);
    OV5640_WriteReg(0x3004, 0xFF);
    OV5640_WriteReg(0x0000, 0x00);
    OV5640_WriteReg(0x0000, 0x00);
    OV5640_WriteReg(0x0000, 0x00);
    OV5640_WriteReg(0x0000, 0x00);
    
//    Delay(100);
    do {
        state = (u8)OV5640_ReadReg(0x3029);
        CAMERA_DEBUG("when init af, state=0x%x\n",state);	
	 
        Delay(10);
        if (iteration-- == 0)
        {
            CAMERA_DEBUG("[OV5640]STA_FOCUS state check ERROR!!, state=0x%x\n",state);	
            break;
        }
    } 
    while(state!=0x70);
    
    OV5640_FOCUS_AD5820_Check_MCU();
    return;    
}   /*  OV5640_FOCUS_AD5820_Init  */

//set constant focus
void OV5640_FOCUS_AD5820_Constant_Focus(void)
{
    u8 state = 0x8F;
    u32 iteration = 300;

//    //send idle command to firmware
//    OV5640_WriteReg(0x3023,0x01);
//    OV5640_WriteReg(0x3022,0x08);

//    iteration = 300;
//    do {
//    	 state = (u8)OV5640_ReadReg(0x3023);
//        if (iteration-- == 0)
//        {
////            RETAILMSG(1, (TEXT("[OV5640]AD5820_Single_Focus time out !!0x%x \r\n")), state);
//            CAMERA_DEBUG("[OV5640]AD5820_Single_Focus time out !! %x\n",state);
//            return ;
//        }   
//        Delay(10);
//    } while(state!=0x00); 

    //send constant focus mode command to firmware
    OV5640_WriteReg(0x3023,0x01);
    OV5640_WriteReg(0x3022,0x04);

    iteration = 5000;
    do {
    	 state = (u8)OV5640_ReadReg(0x3023);
        if (iteration-- == 0)
        {
//            RETAILMSG(1, (TEXT("[OV5640]AD5820_Single_Focus time out II\r\n")));
            CAMERA_DEBUG("[OV5640]AD5820_Single_Focus time out !! %x\n",state);
            return ;
        }
        Delay(10);
    } while(state!=0x00);//0x0 : focused 0x01: is focusing
    return;
}
static void OV5640_FOCUS_AD5820_Single_Focus()
{
     

    u8 state = 0x8F;
    u8 state_ack = 0x8F;	
    u8 state_cmd = 0x8F;		
    u32 iteration = 300;
    CAMERA_DEBUG("OV5640_FOCUS_AD5820_Single_Focus\n");
//1.update zone
    //OV5640_FOCUS_AD5820_Update_Zone();

//2.change focus window
    //OV5640_FOCUS_AD5820_Set_AF_Window_to_IC();

//3.update zone
    //OV5640_FOCUS_AD5820_Update_Zone();

//4.send single focus mode command to firmware
    OV5640_WriteReg(0x3023,0x01);
    OV5640_WriteReg(0x3022,0x03);

    CAMERA_DEBUG("after single focus  \n");

//5.after sigle focus cmd, check the STA_FOCUS until S_FOCUSED 0x10
    iteration = 1000;  
    do{
        state = (u8)OV5640_ReadReg(0x3023);
        CAMERA_DEBUG("test,Single state = 0x%x,state_ack=0x%x,state_cmd=0x%x\n",state,state_ack,state_cmd);
        
        if(state == 0x00)
        {
//            state = (u8)OV5640_ReadReg(0x3029);
//            if(state == 0x10)
 //           {   
                CAMERA_DEBUG("single focused!\n");
                break;
 //           }
        }			
        Delay(10);
        iteration --;

    }while(iteration);
    return;

}

//static void OV5640_FOCUS_AD5820_Pause_Focus()
//{
//    u8 state = 0x8F;
//    u32 iteration = 300;

//    //send idle command to firmware
//    OV5640_WriteReg(0x3023,0x01);
//    OV5640_WriteReg(0x3022,0x06);

//    iteration = 100;  
//    do{
//        state = (u8)OV5640_ReadReg(0x3023);
//        
//        if(state == 0x00)
//        {
//            CAMERA_DEBUG("idle!\n");
//            break;
//        }			
//        Delay(10);
//        iteration --;

//    }while(iteration);

//}

//static void OV5640_FOCUS_AD5820_Cancel_Focus()
//{
//    u8 state = 0x8F;
//    u32 iteration = 300;
//    CAMERA_DEBUG("OV5640_FOCUS_AD5820_Cancel_Focus\n");
//    //send idle command to firmware
//    OV5640_WriteReg(0x3023,0x01);
//    OV5640_WriteReg(0x3022,0x08);
//	
//    iteration = 100;  
//    do{
//        state = (u8)OV5640_ReadReg(0x3023);

//        if(state == 0x00)
//    {
//            CAMERA_DEBUG("idle!\n");
//            break;
//    }
//        Delay(10);
//        iteration --;
//		
//    }while(iteration);
//			
//}

static void OV5640_FOCUS_AD5820_Check_MCU(void)
{
    int i = 0;    
    u8 check[13] = {0x00};
	//mcu on
    check[0] = OV5640_ReadReg(0x3000);
    check[1] = OV5640_ReadReg(0x3004);
	//soft reset of mcu
    check[2] = OV5640_ReadReg(0x3f00);	
	//afc on
    check[3] = OV5640_ReadReg(0x3001);
    check[4] = OV5640_ReadReg(0x3005);
	//gpio1,gpio2
    check[5] = OV5640_ReadReg(0x3018);
    check[6] = OV5640_ReadReg(0x301e);
    check[7] = OV5640_ReadReg(0x301b);
    check[8] = OV5640_ReadReg(0x3042);
	//y0
    check[9] = OV5640_ReadReg(0x3018);
    check[10] = OV5640_ReadReg(0x301e);
    check[11] = OV5640_ReadReg(0x301b);
    check[12] = OV5640_ReadReg(0x3042);


    for(i = 0; i < 13; i++)
    CAMERA_DEBUG("check[%d]=0x%x\n", i, check[i]);
	
}
uint8_t OV5640_FOCUS_AD5820_Pause_Focus(void)
{
    u8 state = 0x8F;
    u32 iteration = 300;

    //send idle command to firmware
    OV5640_WriteReg(0x3023,0x01);
    OV5640_WriteReg(0x3022,0x06);

    iteration = 100;  
    do{
        state = (u8)OV5640_ReadReg(0x3023);
        
        if(state == 0x00)
        {
            CAMERA_DEBUG("Pause Focus!\n");
            return 0;
        }			
        Delay(10);
        iteration --;

    }while(iteration);
		return 1;
}
void OV5640_AUTO_FOCUS(void)
{
   OV5640_FOCUS_AD5820_Init();
   OV5640_FOCUS_AD5820_Constant_Focus();
}


