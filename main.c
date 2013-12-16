//////////////////////////////////////////////////////
// The Battle of Pelennor Fields
// Johnathan Budd, Brian Connolly, 
// Lakshmi Parthasarathy, Vanessa Tan
//
// See also ir_helpers.c, ir_helpers.h, 
// camera_helpers.h and camera_helpers.c.
//
// CALIBRATED ROBOTS:
// 2180 (flux)
// 2117 (cosmetic)
// 2110 (reverence) 
// 2137 (eve)
// 2028 (ballast)
// 2046 (artisan)
// 2099 (surrender)		
// 2087 (bathtub) 
//
// Not fully calibrated:
// 2151 (hayley) -- IR not tested
////////////////////////////////////////////////////// 


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


#define LINE_OF_INTEREST 290

/* Control-related globals */
unsigned char sel;		//the position of the selector switch
int time_counter;
int turnSwitch = 0;
unsigned int mode;		// current state of the robot 
char msg[80]; 			// debug messages				
unsigned int team;

// Camera-related globals
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

/* Some debugging helpers */
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

void printCameraLine() {
	// Debug print of camera line
	sprintf(msg, "");
	for (i = 0; i < cam_width; i++) {
		sprintf(msg, "%s%d", msg, smoothedColors[i]);
	}
	sprintf(msg, "%s\r\n", msg);
	btcomSendString(msg); 
}



/* Returns whether the robot is "near" the goal, based on camera line.
 * Used to turn off obstacle avoidance when entering goal. */
int nearGoal(robotID) {
	int sum = 0;
	for (i = 0; i < cam_width; i++) {
		sum += smoothedColors[i];
	}
	return (sum > 65);

}

/* Given a robot ID and a theta offset, turns by that robot's offset 
 * plus theta degrees. */
void orientToRobot(int id, double theta) {
	double bearing = robots[id].data.bearing;
	turn(bearing + theta, HALF_SPEED);
}


/* Make a beeline to the goal. Combines obstacle/robot avoidance with 
 * moveToGoal(). */
void beelineToGoal(int robotID, int sendID) {
	if (!nearGoal(robotID)) {
		if (!avoidObstacle(robotID, sendID) && !avoidRobot(robotID, sendID, 0)) 
			moveToGoal(robotID);		
	}
	else {
		moveToGoal(robotID);
	} 
}

/* Given robot ID, takes one step towards the goal. 
 * When goal not seen, alternates between "wandering" (moving forward) 
 * and "scanning," where scanning alternates turn directions. 
 * If goal is seen, adjusts angle based on the midpoint of the goal pixels. */
void moveToGoal(robotID) {
	// wandering...
	if (goalLost >= 3) {
		if (spinMode == 18) {
			turnSwitch = (turnSwitch + 1) % 2;
		}
		if (spinMode < 18) {
			setSpeeds(HI_SPEED, HI_SPEED);
		}
		else { // spin
			if (turnSwitch) 
				setSpeeds(LO_SPEED, -LO_SPEED);
			else
				setSpeeds(-LO_SPEED, LO_SPEED);
		}
		spinMode = (spinMode + 1) % 36;
	}
	getGoalCameraLine(robotID, team);

	if (goalInView()) {
		goalLost = 0;
		// compute midpoint (center of gravity, really) of goal pixels
		int mid = getGoalMidpoint();
		int delta = cam_width/2 - mid;
		float p;

		// if roughly in center of view, move straight forward
		if (abs(delta) < 12) {
      setSpeeds(HI_SPEED, HI_SPEED);
    }

    // if the puck is not centered, turn softly toward puck 
    // (proportional to midpoint's distance from camera center)
    else {
		   // set proportionality constant
       p = 1.0 - (abs(delta) - 15.0)/50.0;
 
       // turn softly right
       if (delta > 0)  
        	setSpeeds(HI_SPEED, HI_SPEED*p);

       // turn softly left
       else 
         	setSpeeds(HI_SPEED*p, HI_SPEED);
   	}	
	}
	else {			
		goalLost++;					
	}
}

/* Returns the ID of the "best" enemy to target, if a robot exists in range.
 * Prioritizes the enemy's leader robot, then the closest enemy. */
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

/* Given a robot ID (e.g. 2046), and the IR IDs (e.g. 0x04) of self
 * and kill target, continuously follows the kill target. */
void kill(int robotID, int sendID, int killID) {
	// Turn towards robot and go straight every 3 seconds
	double prevBearing = 0.0;

	if (!avoidRobot(robotID, sendID, killID) && !avoidObstacle(robotID, sendID)) {
		while(time_counter/2 - robots[killID].time < 15) {
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

/* If in penalty box, stop all movement and return true */ 
int stopIfInPenaltyBox(int robotID) {
	getPenaltyCameraLine(robotID);
	if (inPenaltyBox()) {
		setSpeeds(0,0);
		sprintf(msg, "IN THE PENALTY BOX\r\n");
		btcomSendString(msg);
		return 1;
	}
	return 0;
}

/* Initialize the robot. */
void robot_init() {
	int cam_mode, cam_heigth, cam_zoom, buffer_size;

	myWait(500);
	e_init_port();    //Configure port pins
	e_init_motors();
	e_init_uart1();   //Initialize UART to 115200 Kbit
	e_i2cp_init();    //I2C bus for the camera

	cam_mode=RGB_565_MODE; 				
	cam_width=80;
	cam_heigth=1; 
	cam_zoom=8; 							
	buffer_size=cam_width*cam_heigth*2; 	
	e_poxxxx_init_cam(); 					// Located in e_common.c 
	 
	//Returns 0 if setup parameters for the camera are ok, -1 if not.       
	if(0 != e_poxxxx_config_cam((ARRAY_WIDTH -cam_width*cam_zoom)/2,LINE_OF_INTEREST,cam_width*cam_zoom,cam_heigth*cam_zoom,cam_zoom,cam_zoom,cam_mode))	
	{
		e_set_led(0, 1);  //Turn on center diode when robot is considered from the front if setup FAILED.
		while(1);         //And then stay passive 
	}

	manual_camera_calibration();
	e_poxxxx_write_cam_registers(); //Initialization and changes to the setup of the camera.

	time_counter = 0;
}


int main(void)
{	
	if(RCONbits.POR) {	
		RCONbits.POR=0;
		RESET();
	}

	sel = getselector();		
	if (sel != 0) {
		robot_init();
	} 
	else {
		while(1) { NOP(); }
	}
	
	/* Each selector corresponds to a robot */
	if (sel == 1) { // "GOOD" LEADER
		int robotID = 2110;
		unsigned char sendID = 0x01;
		team = GONDOR;
		
		goalLost = 0;
		spinMode = 0;
		mode = 0;

		unsigned char seed = time(NULL);
		comm_init(seed, sendID); 
		comm_store_tx(0);

		while(1)
		{
			/* receive camera and IR readings */
			e_poxxxx_launch_capture(&buffer[0]);     
			while(!e_poxxxx_is_img_ready());		
			receiveIR();
			stopIfInPenaltyBox(robotID);

			switch (mode) {
				case 0: // Remain still for 30 seconds
					setSpeeds(0,0);
					if (time_counter/2 > 30) {
						 mode = 1;
					}
					break;
				case 1: // Move forward for seconds 31-55
					if (!avoidRobot(robotID, sendID, 0) && !avoidObstacle(robotID, sendID)) {
						setSpeeds(HALF_SPEED, HALF_SPEED);
					}

					if(time_counter/2 > 55) {
						mode = 2;
					}
					break;
				case 2: // Beeline to goal
					beelineToGoal(robotID, sendID);
					break;
			}
		}
	}
	else if (sel == 2) { // "GOOD" GUARD
		int robotID = 2137; // always use eve
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
			/* Camera and IR  */
			e_poxxxx_launch_capture(&buffer[0]); 	//Start image capture    
			while(!e_poxxxx_is_img_ready());		//Wait for capture to complete
			receiveIR();

      /* Check whether the robot is in the box and keep track of deaths */
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

					// after leaving penalty box, move back into position in front of goal
					moveForward(755, HI_SPEED);
					turn(90,HI_SPEED);
				}

        // Get the enemy within range and target if exists
				int bestEnemy = bestEnemyIfExists();
				if(bestEnemy || time_counter/2 < start_time + 600){
					int close = 0;
					close = closeEnoughToKill(robotID,robots[bestEnemy].data.range);
					sprintf(msg,"I am detecting robot: %i\r\n\r\n close: %i\r\n", bestEnemy, close);
					btcomSendString(msg);
					if(bestEnemy > 0 && closeEnoughToKill(robotID,robots[bestEnemy].data.range)!= 0) {
						btcomSendString("I'm going to kill them\r\n");
						kill(robotID, sendID, bestEnemy);
					}
					else {
						// If no enemy nearby, stay still near goal
						setSpeeds(0,0);
						btcomSendString("waiting\r\n");
					}
				} 
	
				// If 5 minutes have passed, guard may head for goal
				else{
					beelineToGoal(robotID, sendID);
				}	
			}
		}
	}
	else if (sel == 3) { // "GOOD" SLAYER #1
		int robotID = 2028; 
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
		
		//move forward for 3 sec to clear the goal
		while (time_counter/2 <= start_time + 3){
			setSpeeds(HI_SPEED, HI_SPEED);
		}

		while(1)
		{
			/* Camera and IR */
			e_poxxxx_launch_capture(&buffer[0]); 	//Start image capture    
			while(!e_poxxxx_is_img_ready());		//Wait for capture to complete
			receiveIR();
						
      
      /* Check whether the robot is in the box and keep track of deaths */
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

        // If killed less than twice, look for enemy
				if(death_count < 3){
					int bestEnemy = bestEnemyIfExists();  
          // If best enemy exists, kill it
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
					beelineToGoal(robotID, sendID); 
				}
			}
		}
	}
	else if (sel == 4) { 	// "GOOD" SLAYER #2
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
			/* CAMERA AND IR */
			e_poxxxx_launch_capture(&buffer[0]); 	//Start image capture    
			while(!e_poxxxx_is_img_ready());		//Wait for capture to complete
			receiveIR();
						
      /* Check whether the robot is in the box and keep track of deaths */
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
        
        // If died less than twice, check for enemy
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

  /* "EVIL" SELECTORS. Equivalent to above. See sel 1-4 for more detailed comments. */
	else if (sel == 5) { // "EVIL" LEADER
		int robotID = 2110;
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
					if (!avoidRobot(robotID, sendID, 0) && !avoidObstacle(robotID, sendID)) {
						setSpeeds(HALF_SPEED, HALF_SPEED);
					}

					if(time_counter/2 > 55) {
						mode = 2;
					}
					break;
				case 2:
					beelineToGoal(robotID, sendID);
					break;
			}
		}
	}
	else if (sel == 6) { // "EVIL" GUARD
		int robotID = 2137;
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
					moveForward(755, HI_SPEED);
					turn(90,HI_SPEED);
				}

				int bestEnemy = bestEnemyIfExists();
				if(bestEnemy || time_counter/2 < start_time + 600){
					int close = 0;
					close = closeEnoughToKill(robotID,robots[bestEnemy].data.range);
					sprintf(msg,"I am detecting robot: %i\r\n\r\n close: %i\r\n", bestEnemy, close);
					btcomSendString(msg);
					if(bestEnemy > 0 && closeEnoughToKill(robotID,robots[bestEnemy].data.range)!= 0) {
						btcomSendString("I'm going to kill them\r\n");
						kill(robotID, sendID, bestEnemy);
					}
					else {
						// wait
						setSpeeds(0,0);
						btcomSendString("waiting\r\n");
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
	else if (sel == 7) { // "EVIL" SLAYER #1
		int robotID = 2110; 
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
	else if (sel == 8) { // "EVIL" SLAYER #2
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
	else {
		while(1) NOP();
	}
}


