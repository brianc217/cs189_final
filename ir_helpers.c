#include "ir_helpers.h"


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
		case 2099: 		// surrender
			switch (ir_sensor) {
				case 0: return (ir_range > 1);
				case 1: return (ir_range > 20);
				case 2: return (ir_range > 50);
				case 3: return (ir_range > 180);
				case 9: return (ir_range > 10);
				case 10: return (ir_range > 1);
				case 11: return (ir_range > 1); 
				default: return 0;
			}
			break;
		case 2117: 		// cosmetic
			switch (ir_sensor) {
				case 0: return (ir_range > 900);
				case 1: return (ir_range > 900);
				case 2: return (ir_range > 1000);
				case 3: return (ir_range > 1400);
				case 9: return (ir_range > 1100);
				case 10: return (ir_range > 1000);
				case 11: return (ir_range > 900); 
				default: return 0;
			}
			break;
		case 2087: 		// bathtub
			switch (ir_sensor) {
				case 0: return (ir_range > 40);
				case 1: return (ir_range > 30);
				case 2: return (ir_range > 140);
				case 3: return (ir_range > 50);
				case 9: return (ir_range > 300);
				case 10: return (ir_range > 20);
				case 11: return (ir_range > 10); // dummy value -- doesn't receive from sensor 11
				default: return 0;
			}
			break;
		case 2137: 		// eve
			switch (ir_sensor) {
				case 0: return (ir_range > 1300);
				case 1: return (ir_range > 3200);
				case 2: return (ir_range > 2600);
				case 3: return (ir_range > 1000); // dummy value -- doesn't receive from 3
				case 9: return (ir_range > 1400);
				case 10: return (ir_range > 1400);
				case 11: return (ir_range > 1400); 
				default: return 0;
			}
			break;
		default:
			return 0;
			break;
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


void receiveIR() {
	//btcomSendString("started receive IR");
	comm_rx(&data);
	ir_range = data.range;
	ir_bearing = 57.296*(data.bearing);
	data.bearing = 57.296*(data.bearing);
	ir_sensor = data.max_sensor;
	msg_data = (union comm_value) data.data;
	//btcomSendString("received comm");
	receivedID = (unsigned char) msg_data.bits.ID;
	custom_msg = (unsigned int) msg_data.bits.data;
	
	IR rData;
	rData.data = data;
	rData.time = time_counter / 2;
	
	if((int) receivedID >= 0 && (int) receivedID <= 12) {
		robots[receivedID] = rData;
	}
	
	//printRobots();
}

int avoidObstacle(int robotID, int sendID) {
	if (receivedID == sendID) {
		if (atObstacle(robotID)) {
			sprintf(msg, "OBSTACLE: sensor %u, range %u, bearing %f\r\n", (unsigned int) ir_sensor, ir_range, ir_bearing);
			btcomSendString(msg);

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

int avoidRobot(int robotID, int sendID, int excludeID) {
	if (receivedID != sendID && receivedID != excludeID) {
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

