///////////////////////////////////////////////////////////
//Additional prototype functions for the epuck camera    //
//Developed by: Christian Ahler.                         //  
///////////////////////////////////////////////////////////

#ifndef __ADD_FUNC_H__
#define __ADD_FUNC_H__

//Used for camera auto control register 1
#define AUTOMATIC_EXPOSURE 0x00
#define USE_EXPOSURE_REGISTER 0x01
#define USE_EXT_INT_LIN_GAIN 0x02
#define USE_INT_TIME_GLOBAL_GAIN 0x03
#define AUTOMATIC_WHITE_BALANCE 0x00
#define USE_WHITE_BALANCE_REGISTER 0x01

void manual_camera_calibration();

void e_po6030k_set_mirror(int vertical, int horizontal);

void e_po6030k_exposure_control(unsigned char exposure); //Auto control register 1. Page 44 in 6030k datasheet.

void e_po6030k_manual_exposure_value(unsigned long exposure_value);

void e_po6030k_white_balance(unsigned char white_balance); //Auto control register 1. Page 44 in 6030k datasheet.

void e_po6030k_automatic_digital_gain(unsigned char automatic_digital_gain);

unsigned int e_po6030k_mean_brightness();

unsigned char e_po6030k_read_register(unsigned char bank, unsigned char reg);

void e_po6030k_set_mirror(int vertical, int horizontal);

void wait_for_bluetooth();


#endif
