//////////////////////////////////////////////////////
// Stuff goes here 							    //
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


#define LINE_OF_INTEREST 290
#define MORDOR 0 			// the goal will be green
#define GONDOR 1			// the goal will be yellow
#define PENALTY_THRESH 50
#define GOAL_THRESH 8
#define DONE_THRESH 70

#define HI_SPEED 800
#define LO_SPEED 300
#define HALF_SPEED 500

// modes
#define GOTO_GOAL 0
#define ESCORT_LEADER 1
#define DEFEND_GOAL 2


char msg[80]; 				//this is some data to store screen-bound debug messages
double range = 0;

unsigned char sel;					//the position of the selector switch

unsigned int i, red, green, blue; 
int byte1, byte2;
int cam_width;

finalDataRegister data;

//Buffer for the camera image
static unsigned char buffer[300];
//Buffer for sending back messages over bluetooth
static unsigned char print_buffer[100];

//For storing pixel color values
unsigned char pixelColors[80];
unsigned char smoothedColors[80];

/* PUT ROLES IN HERE */
unsigned char id1 = 0x01;  	// FRODO
unsigned char id2 = 0x02;	// Ally 1
unsigned char id3 = 0x03;	// Ally 2
unsigned char id4 = 0x04;	// Ally 3

unsigned char id5 = 0x05;	// WITCH_KING	
unsigned char id6 = 0x06;	// Bad 1 - goaltender
unsigned char id7 = 0x07;	// Bad 2 - escort
unsigned char id8 = 0x08;	// Bad 3 - escort

unsigned char id9 = 0x09;	// Ent 1
unsigned char id10 = 0x0a;	// Ent 2


/*** CHANGE TEAM AND ID HERE ***/
unsigned int team = MORDOR;
unsigned char myID = 0x07;
/************************/

// current state of the robot 
unsigned int mode;
extern int time_counter;

// stuff related to goal-finding
unsigned int goalLost;

//Eases the transmission of integers as ascii code
#define uart_send_text(msg) do { e_send_uart1_char(msg,strlen(msg)); while(e_uart1_sending()); } while(0)


typedef struct IR {
	finalDataRegister data;
	double time;
} IR;

IR robots[12];

void orientToRobot(int id, double theta) {
	double bearing = robots[id].data.bearing;
	turn(bearing + theta, HALF_SPEED);
}

void kill(int id) {
	/*orientToRobot(id, 0);
	while(time_counter/2 - robots[id].time < 15) {
		setSpeeds(HI_SPEED, HI_SPEED);	
	}
	setSpeeds(0,0);*/

	//Opting for simpler idea - turn towards robot and go straight every 3 seconds
	int i = 0;
	for(; i<5; i++){
		orientToRobot(id, 0);
		setSpeeds(HI_SPEED, HI_SPEED);
		myWait(3000);
	}
}

void printRobots() {
	int i = 0;
	for(; i<12; i++) {
		IR robot = robots[i];
		char string[80];
		sprintf(string, "Robot:%i, Range:%i, Bearing:%f, Time:%f\r\n", i, robot.data.range, robot.data.bearing, robot.time);
		btcomSendString(string);
	}
		
}

void getColor(int pixel) {
	byte1 = buffer[pixel];
    byte2 = buffer[pixel + 1];

    red = byte1 >> 3;
    blue = byte2 & 31;
    green = (byte1 & 7) | ((byte2 & (7 << 5)) >> 5);
}

// Given a pixel on [0,cam_width), return 1 if yellow, 0 otherwise
int checkYellow() {
        return (red > 10 && green >= 2 && blue < 2);
}

// Given a pixel on [0,cam_width), return 1 if green, 0 otherwise
int checkGreen() {
        return (red < 5 && green >= 2 && blue < 2);
}

// Given a pixel on [0,cam_width), return 1 if  red, 0 otherwise
int checkRed() {
        return (red > 13 && green <= 1 && blue <= 1);
}

// for debugging
int printRGB(int pixel) {
	getColor(pixel);
	
	sprintf(msg, "red: %d; green: %d; blue: %d\r\n", red, green, blue);
	btcomSendString(msg);
}

void smoothColorLine() {
	// Smooth array of pixels
	smoothedColors[0] = (pixelColors[1] + pixelColors[0]) > 1;
	smoothedColors[cam_width - 1] = (pixelColors[79] + pixelColors[78]) > 1;
	for (i = 1; i < cam_width + 1; i++) {
		smoothedColors[i] = (pixelColors[i-1] + pixelColors[i] + pixelColors[i+1]) >= 2;
	}
}

void getPenaltyCameraLine() {
	// Store array of which pixels are the penalty box color (red)
	for (i = 0; i < cam_width; i++) {
		getColor(i*2);
		pixelColors[i] = checkRed(i*2);
	}
	smoothColorLine();		// puts values in smoothedColors (global)
}

int inPenaltyBox() {
	int sum = 0;
	// check to make sure most pixels are red
	for (i = 0; i < cam_width; i++) {
		sum += smoothedColors[i];
	}
	return (sum > PENALTY_THRESH);	
}

void getGoalCameraLine() {
	// Store array of which pixels are the goal color (yellow or green)
	for (i = 0; i < cam_width; i++) {
		getColor(i*2);
		if (team == GONDOR) 
	    	pixelColors[i] = checkYellow(i*2);
		else
			pixelColors[i] = checkGreen(i*2);
	}
	smoothColorLine();
}

// converts intensity to range in cm
int intensityToRange(unsigned int intensity) {
	if(intensity < 100) {
		return 100;
		//old number 33->15cm  1500->5cm
	} else if(intensity >= 100 && intensity < 150) {
		return 50;
	} else if(intensity >= 150 && intensity < 230) {
		return 25;
	} else if(intensity >= 230 && intensity < 475) {
		return 20;
	} else if(intensity >= 475 && intensity < 515) {
		return 15;
	} else if(intensity >= 515 && intensity < 625) {
		return 10;
	} else if(intensity >= 625 && intensity < 850) {
		return 9;
	} else if(intensity >= 850 && intensity < 1250) {
		return 8;
	} else if(intensity >= 1250 && intensity < 1500) {
		return 7;
	} else if(intensity >= 1500 && intensity < 2200) {
		return 6;
	} else if(intensity >= 2200 && intensity < 2500) {
		return 4;
	} else {
		return 0;
	}
}

int goalInView() {
	int sum = 0;
	// check to make sure the num. pixels of goal color exceeds threshold
	for (i = 0; i < cam_width; i++) {
		sum += smoothedColors[i];
	}
	return (sum > GOAL_THRESH);
}

// assumes goalInView has been called (to update smoothedColors)
int getGoalMidpoint() {
	int sum = 0;
	int total = 0;
	for (i = 0; i < cam_width; i++) {
		if (smoothedColors[i]) {
			sum += i;
			total++;
		}
	}
	return sum/total;
}

// assumes goalInView has been called (to update smoothedColors)
int atGoal() {
	int sum = 0;
	for (i = 0; i < cam_width; i++) {
		sum += smoothedColors[i];
	}
	return (sum > DONE_THRESH);		// TODO: this will need to later include "prox"
}


int main(void)
{	
	//Definitions of variables
	int cam_mode, cam_heigth, cam_zoom, buffer_size;

	myWait(500);
	e_init_port();    //Configure port pins
	e_init_motors();
	e_init_uart1();   //Initialize UART to 115200 Kbit
	e_i2cp_init();    //I2C bus for the camera

	//e_init_randb(I2C); // IR ring in I2C mode
	//e_randb_set_range(0); // Transmit IR with full power (0xff is min power)
	//e_randb_store_light_conditions(); // Calibrate background IR levels


	if(RCONbits.POR) {	//Reset if necessary
		RCONbits.POR=0;
		RESET();
	}

	sel = getselector();			//Read position of selector switch, refer to utility.c

	if(sel==0)						//During this class, sel==0 should always do nothing. This will be the programming mode.
	{
		while(1) {
			NOP();
		};
	}
	else if(sel==1)						//Camera stuff! :)
	{
		cam_mode=RGB_565_MODE; 					//Value defined in e_poxxxx.h (RGB_565_MODE, GREY_SCALE_MODE, YUV_MODE)
		cam_width=80;
		cam_heigth=1; 
		cam_zoom=8; 							//Fully zoomed out
		buffer_size=cam_width*cam_heigth*2; 	//Multiply by 1 or 2 depending on if grayscale or color is used.
		e_poxxxx_init_cam(); 					//Located in e_common.c 
		 
		//Returns 0 if setup parameters for the camera are ok. Returns -1 if not.       
		if(0 != e_poxxxx_config_cam((ARRAY_WIDTH -cam_width*cam_zoom)/2,LINE_OF_INTEREST,cam_width*cam_zoom,cam_heigth*cam_zoom,cam_zoom,cam_zoom,cam_mode))	
		{
			e_set_led(0, 1);  //Turn on center diode when robot is considered from the front if setup FAILED.
			while(1);         //And then stay passive 
		}

		manual_camera_calibration();
		e_poxxxx_write_cam_registers(); //Initialization and changes to the setup of the camera.
	
		unsigned char seed = time(NULL);
		comm_init(seed, myID);
		
		float ir_bearing;
		unsigned int ID;
		union comm_value msg_data;
		unsigned int custom_msg;
		unsigned char companion;		// Refers to an escort's fellow escort
		unsigned char leader;			// Leader of the team

		goalLost = 0;

		if(team == MORDOR){
			leader = 0x05;
			companion = (myID==0x07) ? 0x08 : 0x07;
		} else {
			leader = 0x01;
			companion = (myID==0x03) ? 0x04 : 0x03;
		}
		
		// Dynamically figure out role and strategy
		if(myID == 0x01 || myID == 0x05){
			mode = GOTO_GOAL;
		} else if(myID == 0x02 || myID == 0x06){
			mode = DEFEND_GOAL;
		} else {
			mode = ESCORT_LEADER;
		}


		while(1)
		{
			//myWait(1000);
			
			/* IR receive */
			comm_rx(&data);

			data.bearing = 57.296*(data.bearing);
			msg_data = (union comm_value) (data).data;
			ID = (unsigned int) msg_data.bits.ID;

			IR rData;
			rData.data = data;
			rData.time = time_counter / 2;
			
			if(ID >= 0 && ID <= 12) {
				robots[ID] = rData;
			}

			//kill(5);
			printRobots();
		

			/* CAMERA */
			e_poxxxx_launch_capture(&buffer[0]); 	//Start image capture    
			while(!e_poxxxx_is_img_ready());		//Wait for capture to complete

			// if in penalty box, stop all movement 
			getPenaltyCameraLine();
			if (inPenaltyBox()) {
				setSpeeds(0,0);
				sprintf(msg, "IN THE PENALTY BOX\r\n");
				btcomSendString(msg);
			}
			sprintf(msg, "Mode: %i", mode);
			btcomSendString(msg);

			switch (mode) {
				case GOTO_GOAL:
					if (goalLost >= 3) {
						goalLost = 0;
						setSpeeds(LO_SPEED, -LO_SPEED);
						break;
					}
					getGoalCameraLine();
					if (goalInView()) {
						goalLost = 0;
						if (atGoal()) {
							setSpeeds(0,0);
							sprintf(msg, "DONE\r\n");
							btcomSendString(msg);
						}
						else {
							// compute midpoint (center of gravity, really) of goal pixels
							int mid = getGoalMidpoint();
							int delta = cam_width/2 - mid;
							sprintf(msg, "delta: %d\r\n", delta);
							btcomSendString(msg);
							float p;
		
							// if roughly in center of view, move straight forward
							if (abs(delta) < 15) {
		                    	setSpeeds(HI_SPEED, HI_SPEED);
		                    }
		
		                    // if the puck is not centered, turn softly toward puck (proportional to midpoint's distance from camera center)
		                    else {
								// set proportionality constant
		                        p = 1.0 - (abs(delta) - 15.0)/50.0;
		
		                        // turn softly right
		                        if (delta > 0) { // soft turn right
		                        	setSpeeds(HI_SPEED, HI_SPEED*p);
		                        }
		
		                        // turn softly left
		                        else {
		                        	setSpeeds(HI_SPEED*p, HI_SPEED);
		                        }               
		                    }
							sprintf(msg, "SEES GOAL\r\n");
							btcomSendString(msg);
						}
					}
					else {			
						//setSpeeds(LO_SPEED, -LO_SPEED);
						goalLost++;						
					}
		
					// Debug print of camera line
					sprintf(msg, "");
					for (i = 0; i < cam_width; i++) {
						sprintf(msg, "%s%d", msg, smoothedColors[i]);
					}
					sprintf(msg, "%s\r\n", msg);
					btcomSendString(msg);

					break;

				case ESCORT_LEADER:
					myWait(500);
					while(robots[(int)leader].data.bearing == 0) NOP();
					orientToRobot(leader, 0);
					int range = robots[(int)leader].data.range;
					double p = (range - 10) / 50;
					setSpeeds(HI_SPEED+p, HI_SPEED+p);
						
					break;
				case DEFEND_GOAL:
					break;
			}
			
		}
	}

	/*
	else if (sel == 2) {	// Always broadcast an increasing number through IR
		tx_data.value = 0xfe;
		init_timer3();		// Start broadcasting the contents of tx_data every 200 ms
		while (1) { 
	  		
			//Choose specific channel
			e_randb_send_all_data(0xFE);
			//Send all stored data
		} ;

	}
	else if (sel == 3) {	// Wait until receiving a packet through IR
		union comm_value rx_data;
		double ir_bearing;
		double ir_range;
		while (1)
		{
			range = e_randb_get_range();
		
			//e_randb_uart_store_data(10,range);
			sprintf(msg, "%i\r\n", range);
			btcomSendString(msg);
		}
	}
	*/
	else
	{
		while(1) NOP();
	}


	
}

/*
static void init_timer3(void)
{
    T3CON = 0x0;            // init config register
    TMR3 = 0;               // clear timer counter
    PR3 = 5760;             // period = 200 ms (* 28.8 KHz) -> 5760
    T3CONbits.TCKPS = 0x3;  // prescaler = 256
                            // @ clock 7.3728 MHz -> tick @ 28.8 KHz
    T3CONbits.TCS = 0;      // internal clock source
    IFS0bits.T3IF = 0;      // clear interrupt flag
    IEC0bits.T3IE = 1;      // enable timer interrupt
    T3CONbits.TON = 1;      // start timer

}
*/

/* Every 200 ms send the value of tx_data over IR in all directions */
/*void __attribute__((interrupt, auto_psv))
_T3Interrupt(void) {
    IFS0bits.T3IF = 0;

    TMR3 = 0;

    e_randb_send_all_data(tx_data.value);
}
*/