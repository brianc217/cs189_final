/////////////////////////////////////////////////
//Additional functions for the epuck camera    //
//Developed by: Christian Ahler.               //  
/////////////////////////////////////////////////
 
#include "camera/fast_2_timer/e_po6030k.h"
#include "camera/fast_2_timer/e_poxxxx.h"
#include "additional_functions_seas.h"
#include "motor_led/advance_one_timer/e_led.h"


void manual_camera_calibration() // Adjust the color correction matrix, the exposure level, and the while balance settings.
{

	//Exposure control: Automatic and manual ways
	//Auto control register 1. Page 44 in 6030k datasheet.
	//AUTOMATIC_EXPOSURE 0x00
	//USE_EXPOSURE_REGISTER 0x01
	//USE_EXT_INT_LIN_GAIN 0x02
	//USE_INT_TIME_GLOBAL_GAIN 0x03

	//As long as color correction and manual white balance 
	//is used you also get resonable values using 
	//automatic exposure. This might be beneficial in some cases where the lighting changes a lot.
	e_po6030k_exposure_control(USE_EXPOSURE_REGISTER);  
	//AUTOMATIC_EXPOSURE 0x00
	//USE_EXPOSURE_REGISTER 0x01
	//USE_EXT_INT_LIN_GAIN 0x02  
	//USE_INT_TIME_GLOBAL_GAIN 0x03 
	
	//If you use manual exposure you can use the below to adjust the exposure level. 
	e_po6030k_manual_exposure_value(0x3FFFF); //The default value is 0x8000. This also affect the mean brightness readout.

	//If you use automatic exposure you MUST enable the below two lines, otherwise you will experience
	//color bleeding (e.g. you will see a lot of green and red when holding a blue object infront of the camera)  
	e_po6030k_write_register(BANK_C, 0x55, 0x00); //Exposure up speed set to 0 to manually control IntTime and Global gain
	e_po6030k_write_register(BANK_C, 0x56, 0x00); //Exposure down speed set to 0 to manually control IntTime and Global gain

//	e_po6030k_write_register(BANK_C, 0x41, 10); //Set maximum exposure level. Use in auto exposure mode. Default is 0x3E (62)
												//Can be used to reduce autoexposure such that over time green is seen when looking at 
												//for example pure red.											
												//Can also be used if too low values are placed in the output buffer

	e_po6030k_white_balance(USE_WHITE_BALANCE_REGISTER); //To get useable values it is ESSENTIAL to MANUALLY control the white balance!!
	//AUTOMATIC_WHITE_BALANCE 0x00
	//USE_WHITE_BALANCE_REGISTER 0x01
	
	//Color correction matrix - necessary adjust factory set calibration values    
	e_po6030k_write_register(BANK_B, 0xA6, 2);   //Correction of red content in blue color!!! 0-10 are reasonable values.
	e_po6030k_write_register(BANK_B, 0xA9, 155); //Correction of green content in blue color!!! 150-180 are reasonable values.
	e_po6030k_write_register(BANK_B, 0xAA, 140); // Correction of blue content in red color (when red is infront)!!! 140-180 are good values.

	//White balance R/G/B gains. Adjust to affect white balance. Note that the value is written to BANK C of the camera, NOT BANK B.
	e_po6030k_write_register(BANK_C, 0xA4, 64); //Make sure white balance is set to default after power up 
	e_po6030k_write_register(BANK_C, 0xA5, 64); //Make sure white balance is set to default after power up
	e_po6030k_write_register(BANK_C, 0xA6, 64); //Make sure white balance is set to default after power up

	//Use the first function below to calibration for light intensity in the room, so colors do not saturate or
	//have too low values (low resolution)
//	e_po6030k_write_register(BANK_C, 0x6F, 0x60);   //User weight Y target offset. Controls the brightness weight balance.
	//e_po6030k_write_register(BANK_C, 0xA5, 40);   //Reduce green. Manual white balance: Use (676-678) White Balance R/G/B gain registers.
	//e_po6030k_write_register(BANK_C, 0xA6, 0x50); //Amplify Blue. Manual white balance: Use (676-678) White Balance R/G/B gain registers.

//	e_po6030k_write_register(BANK_C, 0xA1, 40); //Reduce max automatic red gain 
//	e_po6030k_write_register(BANK_C, 0xA3, 40); //Reduce max automatic blue gain  

//	e_po6030k_write_register(BANK_C, 0x84, 0xFF); //AWB R/B ratio. Helps reducing green content in red by a factor 2 
//	e_po6030k_write_register(BANK_C, 0x85, 0xFF); //AWB R/B ratio. Helps reducing green content in blue by a factor 2

//	e_po6030k_automatic_digital_gain(0); //Disable automatic gain. Has no effect when using manual white balance.
//	e_po6030k_write_register(BANK_C, 0x51, 10); //Gain

}

void e_po6030k_exposure_control(unsigned char exposure)
{
	unsigned char bc;
	bc = e_po6030k_read_register(BANK_C, 0x04); //Read present register state
	if (exposure == 0x00) 
		bc = (bc&0b11111100)|0x00; //AUTOMATIC_EXPOSURE
	else if (exposure == 0x01)
		bc = (bc&0b11111100)|0x01; //USE_EXPOSURE_REGISTER
	else if (exposure == 0x02)
		bc = (bc&0b11111100)|0x02; //USE_EXT_INT_LIN_GAIN
	else if (exposure == 0x03)
		bc = (bc&0b11111100)|0x03; //USE_INT_TIME_GLOBAL_GAIN

		e_po6030k_write_register(BANK_C, 0x04, bc);
}

void e_po6030k_manual_exposure_value(unsigned long exposure_value)
{
	e_po6030k_write_register(BANK_C, 0x2C, (exposure_value&0xFF000000)>>24); //Exposure top
	e_po6030k_write_register(BANK_C, 0x2D, (exposure_value&0x00FF0000)>>16); //Exposure high
	e_po6030k_write_register(BANK_C, 0x2E, (exposure_value&0x0000FF00)>>8);  //Exposure medium
	e_po6030k_write_register(BANK_C, 0x2F, (exposure_value&0x000000FF)>>0);  //Exposure low
}

void e_po6030k_white_balance(unsigned char white_balance)
{
	unsigned char bc;
	bc = e_po6030k_read_register(BANK_C, 0x04); //Read present register state
	if (white_balance == 0x00) 
		bc = (bc&0b11111011)|0x00; //AUTOMATIC_WHITE_BALANCE
	else if (white_balance == 0x01)
		bc = (bc&0b11111011)|0x04; //USE_WHITE_BALANCE_REGISTER

		e_po6030k_write_register(BANK_C, 0x04, bc);
}

void e_po6030k_automatic_digital_gain(unsigned char automatic_digital_gain)
{
	unsigned char bc;
	bc = e_po6030k_read_register(BANK_C, 0x05); //Read present register state
	if (automatic_digital_gain == 0x00) 
		bc = (bc&0b11111011)|0x00; //MANUAL GAIN
	else if (automatic_digital_gain == 0x01)
		bc = (bc&0b11111011)|0x04; //AUTOMATIC GAIN

		e_po6030k_write_register(BANK_C, 0x05, bc);
}

unsigned int e_po6030k_mean_brightness()
{
	return (e_po6030k_read_register(BANK_C, 0x6A)<<8 | e_po6030k_read_register(BANK_C, 0x6B));
}


void wait_for_bluetooth() //Short delay to ensure that programming is possible
{
	long static start_delay;
	char static start_blink;
	for (start_blink=0; start_blink<8; start_blink++)
	{
		for (start_delay=0; start_delay<250000; start_delay++);
			e_set_led(8, 2);
	}
}
