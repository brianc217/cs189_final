//////////////////////////////////////////////////////
// CALIBRATED ROBOTS:
// *2180 (flux) -- GREAT camera; good IR
// *2110 (reverence) --  pretty good IR; bad green on camera
// *2117 (cosmetic) -- decent camera but better yellow than green; great IR
// 2046 (artisan) -- average camera; pretty bad IR
// 2137 (eve) -- CAMERAS NEEDS RECALIBRATION; IR readings (for bounce-back) very good, except sensor 7 is too strong
// 2099 (surrender) -- great camera; lousy IR		
// 2087 (bathtub) -- bad camera; IR is meh (can't sense from 3 IR receivers)
//
// *: potentially good for leader
//
// Not fully calibrated:
// 2151 (hayley) -- really bad camera; IR not tested
// 2028 (ballast) -- bad camera; IR not tested				    
//////////////////////////////////////////////////////

/*
ROBOT ROLE PSEUDOCODE

1. LEADER
IF (x+3 seconds) have passed OR [GUARD] has not been detected nearby for > 10 seconds OR enemy detected nearby:
	{state 1} beelineToGoal()
ELSE:
	{state 0} stay still


2. GUARD
IF (x seconds) have passed:
	become a [SLAYER]
ELSE:
	IF [LEADER] has been out of range (25 cm?) for >= 3 seconds:
		move back toward [LEADER]
	ELSE IF enemy robot within y cm: 
		attack that robot
	ELSE:
		circle around in front of goal??


3/4. SLAYERS (one slightly staggered to prevent immediate collision)
IF enemy leader in range:
	attack enemy leader
ELSE IF other enemy robot in range:
	pick nearest one
	attack that robot
ELSE:
	beelineToGoal()


*PERVASIVE (all robots do this):
- Stop if in penalty box
- Avoid robots (that aren't your target, if you're a killer)
- Avoid obstacles
*/


#include <p30f6014A.h>

#include "stdio.h"  
#include "string.h" 
#include "stdlib.h" 
#include "time.h"

#include "e_epuck_ports.h"
#include "e_init_port.h"
#include "motor_led/utility.h"
#include "motor_led/advance_one_timer/e_led.h"
#include "motor_led/advance_one_timer/e_motors.h"
#include "a2d/e_prox.h"
#include "uart/e_uart_char.h"
#include "bluetooth/btcom.h"

#include "I2C/e_I2C_master_module.h"
#include "I2C/e_I2C_protocol.h"
#include "camera/fast_2_timer/e_poxxxx.h"
#include "camera/fast_2_timer/e_po6030k.h"
#include "additional_functions_seas.h"
#include "motor_led/advance_one_timer/e_agenda.h"

#include "camera_helpers.h"
#include "ir_helpers.h"


/* global vars */
double range = 0;
unsigned char sel;					//the position of the selector switch
int time_counter;

// camera init
#define LINE_OF_INTEREST 290


unsigned int mode;		// current state of the robot 
char msg[80]; 			// debug messages				


// Ox01 = FRODO, 0x02-0x04 allies; 0x04 = WITCH-KING, 0x05-0x07 allies
unsigned char id1 = 0x01;
unsigned char id2 = 0x02;
unsigned char id3 = 0x03;
unsigned char id4 = 0x04;

unsigned char id5 = 0x05;
unsigned char id6 = 0x06;
unsigned char id7 = 0x07;
unsigned char id8 = 0x08;


/* PUT ROBOT IDS HERE */
int robot0 = 2046; 		// Artisan
int robot1 = 2137; 		// Evaporation
int robot2 = 2110; 		// Reverence
int robot3 = 2180; 		// Flux


unsigned int team;

// Camera
unsigned char pixelColors[80];
unsigned char smoothedColors[80];
int cam_width;
unsigned int i, red, green, blue; 
int byte1, byte2;
int cam_width;
unsigned int goalLost;
unsigned int spinMode;
unsigned char buffer[300];			//Buffer for the camera image

// IR-related globals
finalDataRegister data;
unsigned int ir_range;
float ir_bearing;
unsigned char ir_sensor;
unsigned char receivedID;
union comm_value msg_data;
unsigned int custom_msg;
IR robots[12];

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

void printRobots() {
	int i = 0;
	for(; i<12; i++) {
		IR robot = robots[i];
		char string[80];
		sprintf(string, "Robot:%i, Range:%i, Bearing:%f, Time:%f\r\n", i, robot.data.range, robot.data.bearing, robot.time);
		btcomSendString(string);
	}
		
}

void printReadings() {
	sprintf(msg, "sensor: %u, range: %u, bearing: %f, ID: %u\r\n", (unsigned int) ir_sensor, ir_range, ir_bearing, receivedID);
	btcomSendString(msg);
}


int nearGoal(robotID) {
	int sum = 0;
	for (i = 0; i < cam_width; i++) {
		sum += smoothedColors[i];
	}
	return (sum > 65);

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

	time_counter = 0;
}


void orientToRobot(int id, double theta) {
	double bearing = robots[id].data.bearing;
	turn(bearing + theta, HALF_SPEED);
}

void kill(int robotID, int sendID, int killID) {
	/*orientToRobot(id, 0);
	while(time_counter/2 - robots[id].time < 15) {
	setSpeeds(HI_SPEED, HI_SPEED);
	}
	setSpeeds(0,0);*/
	
	//Opting for simpler idea - turn towards robot and go straight every 3 seconds
	double prevBearing = 0.0;

	if (!avoidRobot(robotID, sendID, killID) && !avoidObstacle(robotID, sendID)) {
		while(time_counter/2 - robots[killID].time < 15) {
			//avoidRobot(robotID,sendID,killID);
			//avoidObstacle(robotID,sendID);
			//stopIfInPenaltyBox(robotID);
	
			if(prevBearing != robots[killID].data.bearing) {
				orientToRobot(killID, 0);
				prevBearing = robots[killID].data.bearing;
			}		
			setSpeeds(HI_SPEED, HI_SPEED);
			receiveIR();
			//sprintf(msg,"trying to kill: %i \r\n oriented: %f\r\n",killID,prevBearing);
			//btcomSendString(msg);
		}
	}
}

// not currently being used for anything
void avoidFriends() {
	int init = (team == GONDOR) ? 1 : 5;
	int i = init;
	
	// get the your id
	int id = getselector();

	//if a friend is close turn 90 away from them and move until they are no longer close
	for(;i < init + 4; i++){
		if(time_counter/2 - robots[i].time < 8 && robots[i].time != 0.0 && robots[i].data.range > 1111 && i!= id) {
			orientToRobot(i,90);
			while(time_counter/2 - robots[i].time < 8 && robots[i].time != 0.0 && robots[i].data.range > 1111) {
				setSpeeds(HI_SPEED, HI_SPEED);
				receiveIR();
				flow_led();
			}
			setSpeeds(0,0);
			e_led_clear();
		}			
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

// NOW INCLUDES OBSTACLE AVOIDANCE
void beelineToGoal(int robotID, int sendID) {
	if (!nearGoal(robotID)) {
		if (!avoidObstacle(robotID, sendID) && !avoidRobot(robotID, sendID, 0)) 
			moveToGoal(robotID);		
	}
	else {
		moveToGoal(robotID);
	} 
}

void moveToGoal(robotID) {
	// wandering...
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
	else {			
		goalLost++;					
	}
}

int bestEnemyIfExists(){
	int init = (team == GONDOR) ? 5 : 1;
	int i = init;
	int closest = 0;

	for(;i < init + 4; i++){
		if(time_counter/2 - robots[i].time < 30 && robots[i].time != 0.0) {
			if(i== init){
				return i;
			}
			else if(robots[i].data.range > closest){
				closest = i;
			}
		}			
	}
	return closest;
}

// if in penalty box, stop all movement 
int stopIfInPenaltyBox(int robotID) {
	getPenaltyCameraLine(robotID);
	btcomSendString("Checking Penalty\r\n");
	if (inPenaltyBox()) {
		setSpeeds(0,0);
		sprintf(msg, "IN THE PENALTY BOX\r\n");
		btcomSendString(msg);
		return 1;
	}
	return 0;
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
	if (sel == 1) { // LEADER
		int robotID = 2117;
		unsigned char sendID = 0x01;
		team = GONDOR;
		
		goalLost = 0;
		spinMode = 0;
		mode = 0;

		unsigned char seed = time(NULL);
		comm_init(seed, sendID); 
		comm_store_tx(0);
		double lastTime = 0.0;
		int wallDirection = 0;
		double prevBearing = 0.0;

		while(1)
		{
			/* receive camera and IR readings */
			e_poxxxx_launch_capture(&buffer[0]); 	//Start image capture    
			while(!e_poxxxx_is_img_ready());		//Wait for capture to complete
			receiveIR();
			stopIfInPenaltyBox(robotID);

			if(robots[1].time != lastTime){
				lastTime = robots[1].time;
				if(!wallDirection) {
					prevBearing = robots[1].data.bearing;
					if(prevBearing < 0){
						wallDirection = -1;
					} else {
						wallDirection = 1;
					}
				}

				if(wallDirection > 0){ // Keep wall on left side
					double diff = robots[1].data.bearing - 90 - 10;
					if(abs(diff) < 80){
						turn(diff, HALF_SPEED);
						setSpeeds(HALF_SPEED, HALF_SPEED);
					}
				} else {
					double diff = robots[1].data.bearing + 90 - 10;
					if(abs(diff) < 80){
						turn(diff, HALF_SPEED);
						setSpeeds(HALF_SPEED, HALF_SPEED);
					}
				}
			}
			/*switch (mode) {
				case 0:
					setSpeeds(0,0);
					if (time_counter/2 > 30) {
						 mode = 1;
					}
					break;
				case 1:
					beelineToGoal(robotID, sendID);
					break;
			}*/
		}
	}
	else if (sel == 2) { // GUARD
		int robotID = 2046;
		unsigned char sendID = 0x02;
		team = GONDOR;
		
		int death_count = 0;
		int start_time = time_counter/2;
		int in_box = 0;

		int left_box = 0;
		int move_flag = 1;
		
		goalLost = 0;
		spinMode = 0;

		unsigned char seed = time(NULL);
		comm_init(seed, sendID); 
		comm_store_tx(0);

		while(1)
		{
			/* CAMERA */
			e_poxxxx_launch_capture(&buffer[0]); 	//Start image capture    
			while(!e_poxxxx_is_img_ready());		//Wait for capture to complete
			receiveIR();

			sprintf(msg,"deathcount: %i\r\n",death_count);
			btcomSendString(msg);

			if(stopIfInPenaltyBox(robotID)){
				in_box++;
			}
			else {
				if (in_box > 10) {
					left_box++;
				}
				if (left_box > 10) {
					death_count++;
					in_box = 0;
					left_box = 0;
					//move back into position
					moveForward(200, HI_SPEED);
					turn(90,HI_SPEED);
				}

				int bestEnemy = bestEnemyIfExists();
				if(bestEnemy || time_counter/2 < start_time + 300){
					
					if(bestEnemy && closeEnoughToKill(robotID,robots[bestEnemy].data.range)) {
						kill(robotID, sendID, bestEnemy);
					}
					else {
						// wait
						setSpeeds(0,0);
					}
				} 
	
				//if killed 2 times go for goal
				else{
					btcomSendString("I'm going for the goal!\r\n");
					beelineToGoal(robotID, sendID);
				}	
			}
		}
	}
	else if (sel == 3) { // SLAYER #1
		int robotID = 2117; 
		unsigned char sendID = 0x03;
		team = GONDOR;
		int death_count = 0;
		int start_time = time_counter/2;
		int in_box = 0;

		int left_box = 0;
		int move_flag = 1;
		
		goalLost = 0;
		spinMode = 0;
		mode = 0;

		unsigned char seed = time(NULL);
		comm_init(seed, sendID); 
		comm_store_tx(0);
		
		//move forward for 3sec to clear the goal
		while (time_counter/2 <= start_time + 3){
			setSpeeds(HI_SPEED, HI_SPEED);
			btcomSendString("starting move\r\n");
		}

		while(1)
		{
			/* CAMERA */
			e_poxxxx_launch_capture(&buffer[0]); 	//Start image capture    
			while(!e_poxxxx_is_img_ready());		//Wait for capture to complete
			receiveIR();
						
			//sprintf(msg,"deathcount: %i\r\n",death_count);
			//btcomSendString(msg);

			if(stopIfInPenaltyBox(robotID)){
				in_box++;
			}
			else {
				if (in_box > 10) {
					left_box++;
				}
				if (left_box > 10) {
					death_count++;
					in_box = 0;
					left_box = 0;
				}

				if(death_count < 3){
					int bestEnemy = bestEnemyIfExists();
					if(bestEnemy) {
						kill(robotID, sendID, bestEnemy);
					}
					else {
						// if no targets found, go towards goal
						beelineToGoal(robotID, sendID);
					}
				} 
	
				//if killed 2 times, go for goal
				else{
					btcomSendString("I'm going for the goal!\r\n");
					beelineToGoal(robotID, sendID); 
				}
			}
		}
	}
	else if (sel == 4) { 	// SLAYER #2
		int robotID = 2110; 
		unsigned char sendID = 0x04;
		team = GONDOR;
		int death_count = 0;
		int start_time = time_counter/2;
		int in_box = 0;

		int left_box = 0;
		int move_flag = 1;
		
		goalLost = 0;
		spinMode = 0;
		mode = 0;

		unsigned char seed = time(NULL);
		comm_init(seed, sendID); 
		comm_store_tx(0);
		
		//move forward for 3sec to clear the goal
		while (time_counter/2 <= start_time + 3){
			setSpeeds(HI_SPEED, HI_SPEED);
			btcomSendString("starting move\r\n");
		}

		while(1)
		{
			/* CAMERA */
			e_poxxxx_launch_capture(&buffer[0]); 	//Start image capture    
			while(!e_poxxxx_is_img_ready());		//Wait for capture to complete
			receiveIR();
						
			//sprintf(msg,"deathcount: %i\r\n",death_count);
			//btcomSendString(msg);

			if(stopIfInPenaltyBox(robotID)){
				in_box++;
			}
			else {
				if (in_box > 10) {
					left_box++;
				}
				if (left_box > 10) {
					death_count++;
					in_box = 0;
					left_box = 0;
				}

				if(death_count < 3){
					int bestEnemy = bestEnemyIfExists();
					if(bestEnemy) {
						kill(robotID, sendID, bestEnemy);
					}
					else {
						// if no targets found, go towards goal
						beelineToGoal(robotID, sendID);
					}
				} 
	
				//if killed 2 times, go for goal
				else{
					btcomSendString("I'm going for the goal!\r\n");
					beelineToGoal(robotID, sendID); 
				}
			}
		}
	}

	else if (sel == 5) { // MORDOR LEADER
		int robotID = 2137;
		unsigned char sendID = 0x05;
		team = MORDOR;
		
		goalLost = 0;
		spinMode = 0;
		mode = 0;

		unsigned char seed = time(NULL);
		comm_init(seed, sendID); 
		comm_store_tx(0);

		while(1)
		{
			/* receive camera and IR readings */
			e_poxxxx_launch_capture(&buffer[0]); 	//Start image capture    
			while(!e_poxxxx_is_img_ready());		//Wait for capture to complete
			receiveIR();
			stopIfInPenaltyBox(robotID);

			switch (mode) {
				case 0:
					setSpeeds(0,0);
					if (time_counter/2 > 30) {
						 mode = 1;
					}
					break;
				case 1:
					beelineToGoal(robotID, sendID);
					break;
			}
		}
	}
	else if (sel == 6) { // GUARD
		int robotID = 2046;
		unsigned char sendID = 0x06;
		team = MORDOR;
		
		int death_count = 0;
		int start_time = time_counter/2;
		int in_box = 0;

		int left_box = 0;
		int move_flag = 1;
		
		goalLost = 0;
		spinMode = 0;

		unsigned char seed = time(NULL);
		comm_init(seed, sendID); 
		comm_store_tx(0);

		while(1)
		{
			/* CAMERA */
			e_poxxxx_launch_capture(&buffer[0]); 	//Start image capture    
			while(!e_poxxxx_is_img_ready());		//Wait for capture to complete
			receiveIR();

			sprintf(msg,"deathcount: %i\r\n",death_count);
			btcomSendString(msg);

			if(stopIfInPenaltyBox(robotID)){
				in_box++;
			}
			else {
				if (in_box > 10) {
					left_box++;
				}
				if (left_box > 10) {
					death_count++;
					in_box = 0;
					left_box = 0;
					//move back into position
					moveForward(200, HI_SPEED);
					turn(90,HI_SPEED);
				}

				int bestEnemy = bestEnemyIfExists();
				if(bestEnemy || time_counter/2 < start_time + 300){
					
					if(bestEnemy && closeEnoughToKill(robotID,robots[bestEnemy].data.range)) {
						kill(robotID, sendID, bestEnemy);
					}
					else {
						// wait
						setSpeeds(0,0);
					}
				} 
	
				//if killed 2 times go for goal
				else{
					btcomSendString("I'm going for the goal!\r\n");
					beelineToGoal(robotID, sendID);
				}	
			}
		}
	} 
	else if (sel == 7) {
		int robotID = 2117; 
		unsigned char sendID = 0x07;
		team = MORDOR;
		int death_count = 0;
		int start_time = time_counter/2;
		int in_box = 0;

		int left_box = 0;
		int move_flag = 1;
		
		goalLost = 0;
		spinMode = 0;
		mode = 0;

		unsigned char seed = time(NULL);
		comm_init(seed, sendID); 
		comm_store_tx(0);
		
		//move forward for 3sec to clear the goal
		while (time_counter/2 <= start_time + 3){
			setSpeeds(HI_SPEED, HI_SPEED);
			btcomSendString("starting move\r\n");
		}

		while(1)
		{
			/* CAMERA */
			e_poxxxx_launch_capture(&buffer[0]); 	//Start image capture    
			while(!e_poxxxx_is_img_ready());		//Wait for capture to complete
			receiveIR();
						
			//sprintf(msg,"deathcount: %i\r\n",death_count);
			//btcomSendString(msg);

			if(stopIfInPenaltyBox(robotID)){
				in_box++;
			}
			else {
				if (in_box > 10) {
					left_box++;
				}
				if (left_box > 10) {
					death_count++;
					in_box = 0;
					left_box = 0;
				}

				if(death_count < 3){
					int bestEnemy = bestEnemyIfExists();
					if(bestEnemy) {
						kill(robotID, sendID, bestEnemy);
					}
					else {
						// if no targets found, go towards goal
						beelineToGoal(robotID, sendID);
					}
				} 
	
				//if killed 2 times, go for goal
				else{
					btcomSendString("I'm going for the goal!\r\n");
					beelineToGoal(robotID, sendID); 
				}
			}
		}
	} 
	else if (sel == 8) {
		int robotID = 2137; 
		unsigned char sendID = 0x08;
		team = MORDOR;
		int death_count = 0;
		int start_time = time_counter/2;
		int in_box = 0;

		int left_box = 0;
		int move_flag = 1;
		
		goalLost = 0;
		spinMode = 0;
		mode = 0;

		unsigned char seed = time(NULL);
		comm_init(seed, sendID); 
		comm_store_tx(0);
		
		//move forward for 3sec to clear the goal
		while (time_counter/2 <= start_time + 3){
			setSpeeds(HI_SPEED, HI_SPEED);
			btcomSendString("starting move\r\n");
		}

		while(1)
		{
			/* CAMERA */
			e_poxxxx_launch_capture(&buffer[0]); 	//Start image capture    
			while(!e_poxxxx_is_img_ready());		//Wait for capture to complete
			receiveIR();
						
			//sprintf(msg,"deathcount: %i\r\n",death_count);
			//btcomSendString(msg);

			if(stopIfInPenaltyBox(robotID)){
				in_box++;
			}
			else {
				if (in_box > 10) {
					left_box++;
				}
				if (left_box > 10) {
					death_count++;
					in_box = 0;
					left_box = 0;
				}

				if(death_count < 3){
					int bestEnemy = bestEnemyIfExists();
					if(bestEnemy) {
						kill(robotID, sendID, bestEnemy);
					}
					else {
						// if no targets found, go towards goal
						beelineToGoal(robotID, sendID);
					}
				} 
	
				//if killed 2 times, go for goal
				else{
					btcomSendString("I'm going for the goal!\r\n");
					beelineToGoal(robotID, sendID); 
				}
			}
		}
	} 

	/* some stuff for calibration */
	else if (sel == 9) {		// test: for calibrating camera
		/* calibration stuff save for later */
		while(1)
		{
			/* CAMERA */
			e_poxxxx_launch_capture(&buffer[0]); 	//Start image capture    
			while(!e_poxxxx_is_img_ready());
		
			printRGB(80);
			myWait(500);

			//getGoalCameraLine(robotID);
			//printCameraLine();
			//myWait(500);
		}
	}
	else if (sel == 10) {		// test: for calibrating IR
		unsigned char sendID = 0x03;
		unsigned char seed = time(NULL);
		comm_init(seed, sendID); 
		comm_store_tx(0);

		while (1) {
			receiveIR();
			printReadings();
			setSpeeds(0,0);
		}

	}
	else if (sel == 11) {		// test: for sending IR only
		unsigned char sendID = 0x09;
		unsigned char seed = time(NULL);
		comm_init(seed, sendID); 
		comm_store_tx(0);

		while (1) {
			receiveIR();
			setSpeeds(0,0);
		}
	}
	else {
		while(1) NOP();
	}
}



/*
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
*/