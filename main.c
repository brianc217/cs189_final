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

// thresholds
#define PENALTY_THRESH 50
#define GOAL_THRESH 8
#define DONE_THRESH 70

// speeds
#define HI_SPEED 800
#define LO_SPEED 300

// modes
#define GOAL_MODE 0
#define WALL_FOLLOW 1


/* global vars */

char msg[80]; 				//this is some data to store screen-bound debug messages
double range = 0;

unsigned char sel;					//the position of the selector switch

unsigned int i, red, green, blue; 
int byte1, byte2;
int cam_width;

// IR-related globals
finalDataRegister data;
unsigned int ir_range;
float ir_bearing;
unsigned char ir_sensor;
unsigned int receivedID;
union comm_value msg_data;
unsigned int custom_msg;

//Buffer for the camera image
static unsigned char buffer[300];
//Buffer for sending back messages over bluetooth
static unsigned char print_buffer[100];

//For storing pixel color values
unsigned char pixelColors[80];
unsigned char smoothedColors[80];

/* PUT ROLES IN HERE */
// Ox00 = FRODO, 0x01-0x03 allies; 0x04 = WITCH-KING, 0x05-0x07 allies
unsigned char id0 = 0x00; 
unsigned char id1 = 0x01;
unsigned char id2 = 0x02;
unsigned char id3 = 0x03;

unsigned char id4 = 0x04;
unsigned char id5 = 0x05;
unsigned char id6 = 0x06;
unsigned char id7 = 0x07;
// TODO: decide what to do with this

/*** CHANGE TEAM HERE ***/
unsigned int team = MORDOR;
/************************/

// current state of the robot 
unsigned int mode;

// stuff related to goal-finding
unsigned int goalLost;
unsigned int spinMode;

//Eases the transmission of integers as ascii code
#define uart_send_text(msg) do { e_send_uart1_char(msg,strlen(msg)); while(e_uart1_sending()); } while(0)

// Extracts RGB values for a given pixel from the cam buffer
void getColor(int pixel) {
	byte1 = buffer[pixel];
    byte2 = buffer[pixel + 1];

    red = byte1 >> 3;
    blue = byte2 & 31;
    green = (byte1 & 7) | ((byte2 & (7 << 5)) >> 5);
}

// Given a pixel on [0,cam_width), return 1 if yellow, 0 otherwise
int checkYellow(int robotID) {
	int red_thresh, green_thresh, blue_thresh;

	switch (robotID) {
		case 2046:
			red_thresh = 10;
			green_thresh = 2;
			blue_thresh = 2;
			break;
		case 2180:
			red_thresh = 12;
			green_thresh = 2;
			blue_thresh = 1;
			break;
		
		default:
			red_thresh = 10;
			green_thresh = 2;
			blue_thresh = 2;
			break;
	}
    return (red > red_thresh && green >= green_thresh && blue < blue_thresh);
}

// Given a pixel on [0,cam_width), return 1 if green, 0 otherwise
int checkGreen(int robotID) {
	int red_thresh, green_thresh, blue_thresh;

	switch (robotID) {
		case 2046:
			red_thresh = 5;
			green_thresh = 2;
			blue_thresh = 2;
			break;
		case 2180:
			red_thresh = 8;
			green_thresh = 2;
			blue_thresh = 1;
			break;
		default:
			red_thresh = 5;
			green_thresh = 2;
			blue_thresh = 2;
			break;
	}
    return (red < red_thresh && green >= green_thresh && blue < blue_thresh);
}

// Given a pixel on [0,cam_width), return 1 if  red, 0 otherwise
int checkRed(int robotID) {
	int red_thresh, green_thresh, blue_thresh;

	switch (robotID) {
		case 2046:
			red_thresh = 13;
			green_thresh = 1;
			blue_thresh = 1;
			break;
		default:
			red_thresh = 13;
			green_thresh = 1;
			blue_thresh = 1;
			break;
	}
    return (red > red_thresh && green <= green_thresh && blue <= blue_thresh);
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

void getPenaltyCameraLine(int robotID) {
	// Store array of which pixels are the penalty box color (red)
	for (i = 0; i < cam_width; i++) {
		getColor(i*2);
		pixelColors[i] = checkRed(robotID);
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

void getGoalCameraLine(int robotID) {
	// Store array of which pixels are the goal color (yellow or green)
	for (i = 0; i < cam_width; i++) {
		getColor(i*2);
		if (team == GONDOR) 
	    	pixelColors[i] = checkYellow(robotID);
		else
			pixelColors[i] = checkGreen(robotID);
	}
	smoothColorLine();
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

int atObstacle(int robotID) {
	switch (robotID) {
		case 2046:
			switch (ir_sensor) {
				case 0: return (ir_range > 30);
				case 1: return (ir_range > 1000);
				case 2: return (ir_range > 300);
				case 3: return (ir_range > 350);
				case 9: return (ir_range > 450);
				case 10: return (ir_range > 100);
				case 11: return (ir_range > 100); 
				default: return 0;
			}
			break;
		default:
			return 0;
			break;
	}
}

// assumes goalInView has been called (to update smoothedColors)
int atGoal(robotID) {
	int sum = 0;
	for (i = 0; i < cam_width; i++) {
		sum += smoothedColors[i];
	}
	return (sum > DONE_THRESH 
			&& ((ir_bearing < 90 && ir_bearing > 70) || (ir_bearing > -90 && ir_bearing < -70))
			&& atObstacle(robotID));
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

void receiveIR() {
	comm_rx(&data);
	ir_range = (data).range;
	ir_bearing = 57.296*((data).bearing);
	ir_sensor = (data).max_sensor;
	msg_data = (union comm_value) (data).data;
	receivedID = (unsigned int) msg_data.bits.ID;
	custom_msg = (unsigned int) msg_data.bits.data;
	//sprintf(msg, "sensor: %u, range: %u, bearing: %f, ID: %u\r\n", (unsigned int) ir_sensor, ir_range, ir_bearing, receivedID);
	//btcomSendString(msg);
}

void beelineToGoal(int robotID) {
	if (goalLost >= 3) {
		goalLost = 0;
		if (spinMode > 5) {
			setSpeeds(HI_SPEED, HI_SPEED);
		}
		else { // spin
			setSpeeds(LO_SPEED, -LO_SPEED);
		}
		spinMode = (spinMode + 1) % 11;
	}
	getGoalCameraLine(robotID);
	printCameraLine();

	if (goalInView()) {
		goalLost = 0;
		if (atGoal(robotID)) {
			setSpeeds(0,0);
			sprintf(msg, "DONE\r\n");
			btcomSendString(msg);
			myWait(2000);
		}
		else {
			sprintf(msg, "SEES GOAL\r\n");
			btcomSendString(msg);
			// compute midpoint (center of gravity, really) of goal pixels
			int mid = getGoalMidpoint();
			int delta = cam_width/2 - mid;
			//sprintf(msg, "delta: %d\r\n", delta);
			//btcomSendString(msg);
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
		}
	}
	else {			
		goalLost++;						
	}
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

	if (sel == 1)		// ARTISAN 2046
	{
		int robotID = 2180;
		int sendID = id0;
		
		goalLost = 0;
		spinMode = 0;
		mode = 0;

		unsigned char seed = time(NULL);
		comm_init(seed, sendID); // need to factor out a way 
	
		//setSpeeds(HI_SPEED, HI_SPEED);

		while(1)
		{
			/* CAMERA */
			e_poxxxx_launch_capture(&buffer[0]); 	//Start image capture    
			while(!e_poxxxx_is_img_ready());		//Wait for capture to complete


			
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

			/* IR */
			//receiveIR();
			
					
			switch (mode) {
				case 0:
					//if (!avoidObstacle(robotID, sendID)) {
						beelineToGoal(robotID);
					//}
					break;
				case 1:
					//wallFollow(robotID, sendID);
					break;	
				default:
					//wallFollow(robotID, sendID);
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