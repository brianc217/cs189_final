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
#include "time.h"

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
unsigned char id = 0x02;

//Eases the transmission of integers as ascii code
#define uart_send_text(msg) do { e_send_uart1_char(msg,strlen(msg)); while(e_uart1_sending()); } while(0) 

static union comm_value tx_data;

typedef struct IR {
	finalDataRegister data;
	unsigned int time;
} IR;

IR robots[12];
	

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

void orientToSignal(union comm_value rx_data, double theta) {
	double bearing = e_randb_get_bearing();
	turn(bearing + theta, HALF_SPEED);
}

void printRobots() {
	int i = 0;
	for(; i<12; i++) {
		IR robot = robots[i];
		char string[80];
		sprintf(string, "Robot:%i, Range:%i, Bearing:%f, Time:%i\r\n", i, robot.data.range, robot.data.bearing, robot.time);
		btcomSendString(string);
	}
		
}

int main(void)
{	
	
	myWait(500);
	e_init_port();    //Configure port pins
	e_init_motors();
	e_init_uart1();   //Initialize UART to 115200 Kbit
	e_i2cp_init();    //I2C bus for the camera


	if(RCONbits.POR) {	//Reset if necessary
		RCONbits.POR=0;
		RESET();
	}

	sel = getselector();			//Read position of selector switch, refer to utility.c

	if(sel==0)						//During this class, sel==0 should always do nothing. This will be the programming mode.
	{
		while(1) NOP();
	
	} else if(sel==1) {	

		e_start_agendas_processing(); 	//Motor control
		comm_init(0x44, 0x02);
		
		while(1)
		{
			btcomSendString("Start");
			finalDataRegister *ir_data;
			union comm_value rx;
			comm_rx(ir_data);
			rx.value = (*ir_data).data;
			
			IR rData;
			rData.data = *ir_data;
			time_t timer = time(NULL);
			rData.time = timer;
			
			if(rx.bits.ID >= 0 && rx.bits.ID <= 12) {
				robots[rx.bits.ID] = rData;
			}
			char myString[60];
			sprintf(myString, "Saw Robot:%i\r\n", rx.bits.ID);
			btcomSendString(myString);
			//printRobots();

		}

	} else if (sel == 2) {	// Always broadcast an increasing number through IR
		comm_init(time(NULL), 0x01);
		while(1){
			myWait(500);
		}

	} else { // All other selector numbers

		while(1) NOP();
	}
}
