#include "camera_helpers.h"

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
		case 2046:	// artisan
			red_thresh = 11;
			green_thresh = 2;
			blue_thresh = 2;
			break;
		case 2180:	// flux
			red_thresh = 12;
			green_thresh = 2;
			blue_thresh = 1;
			break;
		case 2110:
			red_thresh = 15;
			green_thresh = 3;
			blue_thresh = 2;
			break;
		case 2137: //eve
			red_thresh = 13;
			green_thresh = 2;
			blue_thresh = 1;
			break;

		case 2099: // surrender
			red_thresh = 12;
			green_thresh = 2;
			blue_thresh = 1;
			break;
		case 2117: 	// cosmetic
			red_thresh = 12;
			green_thresh = 3;
			blue_thresh = 1;
			break;
		case 2087: 	// bathtub
			red_thresh = 12;
			green_thresh = 2;
			blue_thresh = 2;
			break;
		case 2028: 	// ballast
			red_thresh = 12;
			green_thresh = 3;
			blue_thresh = 4;
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
		case 2046:	// artisan
			red_thresh = 5;
			green_thresh = 2;
			blue_thresh = 2;
			break;
		case 2180: // flux
			red_thresh = 8;
			green_thresh = 2;
			blue_thresh = 1;
			break;
		case 2110: // reverence. shitty green 
			red_thresh = 3;
			green_thresh = 2;
			blue_thresh = 6;
			break;
		case 2137: //evaporation
			red_thresh= 3;
			green_thresh= 2;
			blue_thresh = 3;
			break;
		case 2099: // surrender
			red_thresh = 8;
			green_thresh = 3;
			blue_thresh = 1;
			break;
		case 2117: // cosmetic
			red_thresh = 8;
			green_thresh = 3;
			blue_thresh = 1;
			break;
		case 2087: 	// bathtub
			red_thresh = 8;
			green_thresh = 2;
			blue_thresh = 2;
			break;
		case 2028: // ballast
			red_thresh = 5;
			green_thresh = 5;
			blue_thresh = 6;
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
			red_thresh = 10;
			green_thresh = 1;
			blue_thresh = 1;
			break;
		default: // works for most robots
			red_thresh = 8;
			green_thresh = 1;
			blue_thresh = 1;
			break;
	}
    return (red > red_thresh && green <= green_thresh && blue <= blue_thresh);
}

/* Smooths out array of ones and zeroes from pixelColors, putting results in
 * smoothedColors array. */
void smoothColorLine() {
	int i;
	// Smooth array of pixels
	smoothedColors[0] = (pixelColors[1] + pixelColors[0]) > 1;
	smoothedColors[cam_width - 1] = (pixelColors[79] + pixelColors[78]) > 1;
	for (i = 1; i < cam_width + 1; i++) {
		smoothedColors[i] = (pixelColors[i-1] + pixelColors[i] + pixelColors[i+1]) >= 2;
	}
}

/* Checks the pixels in the most recent camera line for red. */
void getPenaltyCameraLine(int robotID) {
	int i;
	// Store array of which pixels are the penalty box color (red)
	for (i = 0; i < cam_width; i++) {
		getColor(i*2);
		pixelColors[i] = checkRed(robotID);
	}
	smoothColorLine();		// puts values in smoothedColors (global)
}

/* Checks the pixels in the most recent camera line for the goal color */
void getGoalCameraLine(int robotID, int team) {
	int i;
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

/* Returns true (1) if the number of red pixels in camera line exceeds 
 * threshold, false otherwise */
int goalInView() {
	int sum = 0;
	int i;
	// check to make sure the num. pixels of goal color exceeds threshold
	for (i = 0; i < cam_width; i++) {
		sum += smoothedColors[i];
	}
	return (sum > GOAL_THRESH);
}

/* Finds the center of mass of the filtered and smoothed camera line, to find
 * the "midpoint" of the goal in the robot's view */
int getGoalMidpoint() {
	int sum = 0;
	int total = 0;
	int i;
	for (i = 0; i < cam_width; i++) {
		if (smoothedColors[i]) {
			sum += i;
			total++;
		}
	}
	return sum/total;
}

/* Returns true (1) if the amount of red in camera line exceeds threshold, 
 * false otherwise */
int inPenaltyBox() {
	int sum = 0;
	int i;
	// check to make sure most pixels are red
	for (i = 0; i < cam_width; i++) {
		sum += smoothedColors[i];
	}
	return (sum > PENALTY_THRESH);	
}
