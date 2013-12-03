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

<<<<<<< HEAD
#define HI_SPEED 800
#define LO_SPEED 300
#define HALF_SPEED 500

// modes
#define GOTO_GOAL 0
#define ESCORT_LEADER 1
#define DEFEND_GOAL 2
=======
#include "ir_comm.h"
#include "camera_helpers.h"
>>>>>>> 1327d371741702d8ea6c5cab7d76f2f3ae22550a


/* global vars */
double range = 0;
unsigned char sel;					//the position of the selector switch

// camera init
#define LINE_OF_INTEREST 290

#define HI_SPEED 800
#define LO_SPEED 300

// current state of the robot 
unsigned int mode;
// debug messages
char msg[80]; 				

/* PUT ROLES IN HERE */
<<<<<<< HEAD
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
=======
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
>>>>>>> 1327d371741702d8ea6c5cab7d76f2f3ae22550a
unsigned int goalLost;
unsigned int spinMode;
//Buffer for the camera image
unsigned char buffer[300];

<<<<<<< HEAD
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
=======
// IR-related globals
finalDataRegister data;
unsigned int ir_range;
float ir_bearing;
unsigned char ir_sensor;
unsigned int receivedID;
union comm_value msg_data;
unsigned int custom_msg;
>>>>>>> 1327d371741702d8ea6c5cab7d76f2f3ae22550a

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
				case 0: return (ir_range > 700);
				case 1: return (ir_range > 1100);
				case 2: return (ir_range > 1500);
				case 3: return (ir_range > 1700);
				case 9: return (ir_range > 1700);
				case 10: return (ir_range > 1500);
				case 11: return (ir_range > 1100); 
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

<<<<<<< HEAD
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
=======
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
		
>>>>>>> 1327d371741702d8ea6c5cab7d76f2f3ae22550a
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
	sprintf(msg, "sensor: %u, range: %u, bearing: %f, ID: %u\r\n", (unsigned int) ir_sensor, ir_range, ir_bearing, receivedID);
	btcomSendString(msg);
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
	
<<<<<<< HEAD
		unsigned char seed = time(NULL);
		comm_init(seed, myID);
		
		float ir_bearing;
		unsigned int ID;
		union comm_value msg_data;
		unsigned int custom_msg;
		unsigned char companion;		// Refers to an escort's fellow escort
		unsigned char leader;			// Leader of the team
=======
	/* Each selector corresponds to a robot */
>>>>>>> 1327d371741702d8ea6c5cab7d76f2f3ae22550a

	if (sel == 1)		// ARTISAN 2046
	{
		int robotID = robot0;
		int sendID = (team == GONDOR ) ? id0 : id4;
		
		goalLost = 0;
		spinMode = 0;
		mode = 0;

		unsigned char seed = time(NULL);
		comm_init(seed, sendID); // need to factor out a way 
	
		//setSpeeds(HI_SPEED, HI_SPEED);

<<<<<<< HEAD
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
		

=======
		while(1)
		{
>>>>>>> 1327d371741702d8ea6c5cab7d76f2f3ae22550a
			/* CAMERA */
			e_poxxxx_launch_capture(&buffer[0]); 	//Start image capture    
			while(!e_poxxxx_is_img_ready());		//Wait for capture to complete


			/* testing stuff */
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

			/* IR */
			receiveIR();
			/*
			if (!nearGoal(robotID)) {
				if (!avoidObstacle(robotID, sendID)) {
					beelineToGoal(robotID);		
				}
			}
<<<<<<< HEAD
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
			
=======
			else {
				beelineToGoal(robotID);
			} */
>>>>>>> 1327d371741702d8ea6c5cab7d76f2f3ae22550a
		}
	}
	else if (sel == 3) {
		int robotID = robot2;
		int sendID = (team == GONDOR ) ? id1 : id5;

		unsigned char seed = time(NULL);
		comm_init(seed, sendID); 

		receiveIR();

	}
	else
	{
		while(1) NOP();
	}
}
