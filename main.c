//////////////////////////////////////////////////////
// CS189                                            //
// Lab3 - IR									    //
// Christian Ahler - 2012, mod K. Petersen 2013     //
//////////////////////////////////////////////////////

// The camera is, by default, configured with the following settings:
// - Automatic white balance control
// - Automatic exposure control
// - Automatic flicker detection ( 50Hz and 60Hz )

#include <p30f6014A.h>

#include "stdio.h"  //Necessary for sprint 
#include "string.h" //Necessary for string manipulation 
#include "stdlib.h" //Neccessary for itoa conversion 

#include "e_epuck_ports.h"
#include "e_init_port.h"
#include "motor_led/utility.h"
#include "motor_led/advance_one_timer/e_led.h"
#include "motor_led/advance_one_timer/e_motors.h"
#include "a2d/e_prox.h"
#include "uart/e_uart_char.h"
#include "bluetooth/btcom.h"
#include "e_randb.h"

#include "I2C/e_I2C_master_module.h"
#include "I2C/e_I2C_protocol.h"
#include "camera/fast_2_timer/e_poxxxx.h"
#include "camera/fast_2_timer/e_po6030k.h"
#include "additional_functions_seas.h"
#include "motor_led/advance_one_timer/e_agenda.h"
#include "ir_comm.h"

// CHANGE ME
#define LINE_OF_INTEREST 42
#define HALF_SPEED 500

unsigned char sel;					//the position of the selector switch

//Buffer for the camera image
static unsigned char buffer[300];
//Buffer for sending back messages over bluetooth
static unsigned char print_buffer[100];

unsigned char seed = 'B';
unsigned char id = 0x01;

//Eases the transmission of integers as ascii code
#define uart_send_text(msg) do { e_send_uart1_char(msg,strlen(msg)); while(e_uart1_sending()); } while(0) 

static union comm_value tx_data;

int intensityToRange(unsigned int intensity) {
	if(intensity < 338) {
		return 15;
		//old number 33->15cm  1500->5cm
	} else if(intensity >= 200 && intensity < 750) {
		return 10;
	} else {
		return 5;
	}
}

int main(void)
{	
	//Definitions of variables
	int cam_mode, cam_width, cam_heigth, cam_zoom, buffer_size;
	
	myWait(500);
	e_init_port();    //Configure port pins
	e_init_motors();
	e_init_uart1();   //Initialize UART to 115200 Kbit
	e_i2cp_init();    //I2C bus for the camera

	comm_init(seed, id);

	if(RCONbits.POR) {	//Reset if necessary
		RCONbits.POR=0;
		RESET();
	}

	sel = getselector();			//Read position of selector switch, refer to utility.c

	if(sel==0)						//During this class, sel==0 should always do nothing. This will be the programming mode.
	{
		while(1) NOP();
	}
	else if(sel==1)						//Camera stuff! :)
	{
		e_start_agendas_processing(); 	//Motor control
		
		while(1)
		{
			// TODO
		}

	// All other selector numbers
	} else {
		while(1) NOP();
	}
}
