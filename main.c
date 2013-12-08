//////////////////////////////////////////////////////
// NOTES:
// 2180 has a fantastic camera
// 2110 has pretty good IR							    
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
#include "camera_helpers.h"


/* global vars */
double range = 0;
unsigned char sel;					//the position of the selector switch

// camera init
#define LINE_OF_INTEREST 290

#define HI_SPEED 800
#define LO_SPEED 300
#define HALF_SPEED 500

// current state of the robot 
unsigned int mode;
// debug messages
char msg[80]; 				

/* PUT ROLES IN HERE */
// Ox01 = FRODO, 0x02-0x04 allies; 0x04 = WITCH-KING, 0x05-0x07 allies
unsigned char id1 = 0x01;
unsigned char id2 = 0x02;
unsigned char id3 = 0x03;
unsigned char id4 = 0x04;

unsigned char id5 = 0x05;
unsigned char id6 = 0x06;
unsigned char id7 = 0x07;
unsigned char id8 = 0x08;
// TODO: decide what to do with this

/* PUT ROBOT IDS HERE */
int robot0 = 2046; 		// Artisan
int robot1 = 2137; 		// Evaporation
int robot2 = 2110; 		// Reverence
int robot3 = 2180; 		// Flux

/*** CHANGE TEAM HERE ***/
unsigned int team = GONDOR;
/************************/

// Camera
unsigned char pixelColors[80];
unsigned char smoothedColors[80];
int cam_width;
unsigned int i, red, green, blue; 
int byte1, byte2;
int cam_width;
unsigned int goalLost;
unsigned int spinMode;
//Buffer for the camera image
unsigned char buffer[300];

// IR-related globals
finalDataRegister data;
unsigned int ir_range;
float ir_bearing;
unsigned char ir_sensor;
unsigned char receivedID;
union comm_value msg_data;
unsigned int custom_msg;

//Buffer for sending back messages over bluetooth
static unsigned char print_buffer[100];


//Eases the transmission of integers as ascii code
#define uart_send_text(msg) do { e_send_uart1_char(msg,strlen(msg)); while(e_uart1_sending()); } while(0)


// for debugging
void printRGB(int pixel) {
	getColor(pixel);
	
	sprintf(msg, "red: %d; green: %d; blue: %d\r\n", red, green, blue);
	btcomSendString(msg);
}

void receiveIR() {
	comm_rx(&data);
	ir_range = data.range;
	ir_bearing = 57.296*(data.bearing);
	ir_sensor = data.max_sensor;
	msg_data = (union comm_value) data.data;
	receivedID = (unsigned char) msg_data.bits.ID;
	custom_msg = (unsigned int) msg_data.bits.data;
	sprintf(msg, "sensor: %u, range: %u, bearing: %f, ID: %u\r\n", (unsigned int) ir_sensor, ir_range, ir_bearing, receivedID);
	btcomSendString(msg);
}

int atObstacle(int robotID) {
	switch (robotID) {
		case 2046:
			switch (ir_sensor) {
				case 0: return (ir_range > 60);
				case 1: return (ir_range > 1000);
				case 2: return (ir_range > 300);
				case 3: return (ir_range > 350);
				case 9: return (ir_range > 450);
				case 10: return (ir_range > 100);
				case 11: return (ir_range > 100); 
				default: return 0;
			}
			break;
		case 2180:
			switch (ir_sensor) {
				case 0: return (ir_range > 1100);
				case 1: return (ir_range > 1500);
				case 2: return (ir_range > 1400); // dummy value -- no sensor 2 readings
				case 3: return (ir_range > 1300);
				case 9: return (ir_range > 1400);
				case 10: return (ir_range > 1400);
				case 11: return (ir_range > 1300); 
				default: return 0;
			}
			break;
		case 2110:
			switch (ir_sensor) {
				case 0: return (ir_range > 600);
				case 1: return (ir_range > 1000);
				case 2: return (ir_range > 1500);
				case 3: return (ir_range > 1700);
				case 9: return (ir_range > 1700);
				case 10: return (ir_range > 1500);
				case 11: return (ir_range > 1000); 
				default: return 0;
			}
			break;
		default:
			return 0;
			break;
	}
}


// this isn't really being used
int atGoal(robotID) {
	int sum = 0;
	for (i = 0; i < cam_width; i++) {
		sum += smoothedColors[i];
	}
	/*
	return (sum > DONE_THRESH 
			&& ((ir_bearing < 90 && ir_bearing > 70) || (ir_bearing > -90 && ir_bearing < -70))
			&& atObstacle(robotID));
	*/
	return 0;
}

int nearGoal(robotID) {
	int sum = 0;
	for (i = 0; i < cam_width; i++) {
		sum += smoothedColors[i];
	}
	return (sum > 60);

}

void robot_init() {
	//Definitions of variables
	int cam_mode, cam_heigth, cam_zoom, buffer_size;

	myWait(500);
	e_init_port();    //Configure port pins
	e_init_motors();
	e_init_uart1();   //Initialize UART to 115200 Kbit
	e_i2cp_init();    //I2C bus for the camera

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
}

// yeah not working
void wallFollow(int robotID, int sendID) {
	int delta;
	if (receivedID == sendID) {
		sprintf(msg, "ANGLE ADJUSTMENT: sensor %u, range %u, bearing %f\r\n", (unsigned int) ir_sensor, ir_range, ir_bearing);
		btcomSendString(msg);
		if (ir_bearing > 0) { // left wall follow
			delta = ir_bearing - 90;
		}
		else { 	// right wall follow
			delta = ir_bearing + 90;
		}
		if (abs(delta) > 30) {
			if (delta > 0)
				setSpeeds(3*HI_SPEED/4, HI_SPEED); // soft left
			else
				setSpeeds(HI_SPEED, 3*HI_SPEED/4); // soft right
		}
		else {
			setSpeeds(HI_SPEED, HI_SPEED);
		}
		
	}
}

int avoidObstacle(int robotID, int sendID) {
	if (receivedID == sendID) {
		if (atObstacle(robotID)) {
			//sprintf(msg, "OBSTACLE: sensor %u, range %u, bearing %f\r\n", (unsigned int) ir_sensor, ir_range, ir_bearing);
			//btcomSendString(msg);

			// soft turn left
			if (ir_bearing < -30) {
				setSpeeds(HI_SPEED/2, HI_SPEED);
				//setSpeeds(-LO_SPEED, LO_SPEED);
			}
			// soft turn right
			else if (ir_bearing > 30) {
				setSpeeds(HI_SPEED, HI_SPEED/2);
				//setSpeeds(LO_SPEED, -LO_SPEED);
			}
			// hard turn based on sign of bearing
			else {
				// hard turn right
				if (ir_bearing > 0) 
					setSpeeds(HI_SPEED, -HI_SPEED);
				else
					setSpeeds(-HI_SPEED, HI_SPEED);
			}
			return 1;
		}
	}
	return 0;
}

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
	for(i=0; i<5; i++){
		orientToRobot(id, 0);
		setSpeeds(HI_SPEED, HI_SPEED);
		myWait(300);
		receiveIR();
	}
}


int closeToRobot(int robotID) {
	switch (robotID) {
		case 2180:
			switch (ir_sensor) {
				case 0: return (ir_range > 890);
				case 1: return (ir_range > 690);
				case 2: return (ir_range > 1000); // dummy value -- no sensor 2 readings
				case 3: return (ir_range > 500);
				case 4: return (ir_range > 1070);			
				case 5: return (ir_range > 1030);
				case 6: return (ir_range > 880);
				case 7: return (ir_range > 1000);
				case 8: return (ir_range > 1200);
				case 9: return (ir_range > 1150);
				case 10: return (ir_range > 900);
				case 11: return (ir_range > 1100); 
				default: return 0;
			}
			break;
		case 2110:
			switch (ir_sensor) {
				case 0: return (ir_range > 800);
				case 1: return (ir_range > 800);
				case 2: return (ir_range > 1100); // dummy value -- no sensor 2 readings
				case 3: return (ir_range > 1100);
				case 4: return (ir_range > 1170);			
				case 5: return (ir_range > 900);
				case 6: return (ir_range > 1000);
				case 7: return (ir_range > 1100);
				case 8: return (ir_range > 1150);
				case 9: return (ir_range > 1100);
				case 10: return (ir_range > 1000);
				case 11: return (ir_range > 800); 
				default: return 0;
			}
			break;
		default:
			switch (ir_sensor) {
				case 0: return (ir_range > 890);
				case 1: return (ir_range > 690);
				case 2: return (ir_range > 1000); // dummy value -- no sensor 2 readings
				case 3: return (ir_range > 500);
				case 4: return (ir_range > 1070);			
				case 5: return (ir_range > 1030);
				case 6: return (ir_range > 880);
				case 7: return (ir_range > 1000);
				case 8: return (ir_range > 1200);
				case 9: return (ir_range > 1150);
				case 10: return (ir_range > 900);
				case 11: return (ir_range > 1100); 
				default: return 0;
			}
			break;
	}
}

int avoidRobot(int robotID, int sendID) {
	if (receivedID != sendID) {
		if (closeToRobot(robotID)) {
			// front
			if (ir_bearing > -45 && ir_bearing < 45) {
				setSpeeds(0, HI_SPEED);
			}
			// right
			else if (ir_bearing < -45 && ir_bearing > -135) {
				setSpeeds(0, HI_SPEED);
			}
			// left
			else if (ir_bearing > 45 && ir_bearing < 135) {
				setSpeeds(HI_SPEED, 0);
			}
			// back
			else {
				setSpeeds(HI_SPEED, HI_SPEED);
			}
		}
		return 1;
	}
	return 0;
}


void printCameraLine() {
	// Debug print of camera line
	sprintf(msg, "");
	for (i = 0; i < cam_width; i++) {
		sprintf(msg, "%s%d", msg, smoothedColors[i]);
	}
	sprintf(msg, "%s\r\n", msg);
	btcomSendString(msg); 
}

void beelineToGoal(int robotID) {
	if (goalLost >= 3) {
		if (spinMode < 18) {
			setSpeeds(HI_SPEED, HI_SPEED);
		}
		else { // spin
			setSpeeds(LO_SPEED, -LO_SPEED);
		}
		spinMode = (spinMode + 1) % 36;
	}
	getGoalCameraLine(robotID, team);
	//printCameraLine();

	if (goalInView()) {
		goalLost = 0;
		if (atGoal(robotID)) {  // TODO: change this
			setSpeeds(0,0);
			myWait(2000);
		}
		else {
			//sprintf(msg, "SEES GOAL\r\n");
			//btcomSendString(msg);
			// compute midpoint (center of gravity, really) of goal pixels
			int mid = getGoalMidpoint();
			int delta = cam_width/2 - mid;
			//sprintf(msg, "delta: %d\r\n", delta);
			//btcomSendString(msg);
			float p;

			// if roughly in center of view, move straight forward
			if (abs(delta) < 12) {
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
		}
	}
	else {			
		goalLost++;					
	}
}


int main(void)
{	
	if(RCONbits.POR) {	//Reset if necessary
		RCONbits.POR=0;
		RESET();
	}

	sel = getselector();			//Read position of selector switch, refer to utility.c
	if (sel != 0) {
		robot_init();
	} 
	else {
		while(1) { NOP(); }
	}
	
	/* Each selector corresponds to a robot */

	if (sel == 1)
	{
		int robotID = 2180;
		unsigned char sendID = 0x01;
		
		goalLost = 0;
		spinMode = 0;
		mode = 0;

		unsigned char seed = time(NULL);
		comm_init(seed, sendID); 

		while(1)
		{
			/* CAMERA */
			e_poxxxx_launch_capture(&buffer[0]); 	//Start image capture    
			while(!e_poxxxx_is_img_ready());		//Wait for capture to complete

			/* IR */
			receiveIR();
			if (!nearGoal(robotID)) {
				if (!avoidRobot(robotID, (int) sendID) && !avoidObstacle(robotID, (int) sendID)) {
					beelineToGoal(robotID);		
				}
			}
			else {
				beelineToGoal(robotID);
			} 
		}
	}
	else if (sel == 2) {
		int robotID = 2110;
		unsigned char sendID = 0x02;
		
		goalLost = 0;
		spinMode = 0;
		mode = 0;

		unsigned char seed = time(NULL);
		comm_init(seed, sendID); // need to factor out a way 

		while(1)
		{
			/* CAMERA */
			e_poxxxx_launch_capture(&buffer[0]); 	//Start image capture    
			while(!e_poxxxx_is_img_ready());		//Wait for capture to complete

			receiveIR();
			if (!nearGoal(robotID)) {
				if (!avoidRobot(robotID, sendID) && !avoidObstacle(robotID, sendID)) {
					beelineToGoal(robotID);		
				}
			}
			else {
				beelineToGoal(robotID);
			} 
		}
	}
	else if (sel == 3) {
		int robotID = 2046;
		unsigned char sendID = 0x03;
		
		goalLost = 0;
		spinMode = 0;
		mode = 0;

		unsigned char seed = time(NULL);
		comm_init(seed, sendID); // need to factor out a way 

		while(1)
		{
			/* CAMERA */
			e_poxxxx_launch_capture(&buffer[0]); 	//Start image capture    
			while(!e_poxxxx_is_img_ready());		//Wait for capture to complete

			receiveIR();
			if (!nearGoal(robotID)) {
				if (!avoidObstacle(robotID, sendID)) {
					beelineToGoal(robotID);		
				}
			}
			else {
				beelineToGoal(robotID);
			} 
		}
	}
	else if (sel == 4) {
		/* calibration stuff save for later */
		while (1) {
			e_poxxxx_launch_capture(&buffer[0]); 	//Start image capture    
			while(!e_poxxxx_is_img_ready());
		
			printRGB(80);
			myWait(500);
		}
	}
	else
	{
		while(1) NOP();
	}
}


/* calibration stuff save for later */
//printRGB(80);
//myWait(500);

//getGoalCameraLine(robotID);
//printCameraLine();
//myWait(500);


// if in penalty box, stop all movement 
/*getPenaltyCameraLine(robotID);
if (inPenaltyBox()) {
	setSpeeds(0,0);
	sprintf(msg, "IN THE PENALTY BOX\r\n");
	btcomSendString(msg);
}*/