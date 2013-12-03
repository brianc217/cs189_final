// stuff related to goal-finding thresholds
#define PENALTY_THRESH 50
#define GOAL_THRESH 8
#define DONE_THRESH 70

#define MORDOR 0 			// the goal will be green
#define GONDOR 1			// the goal will be yellow

// globals for camera line
extern unsigned int i, red, green, blue; 
extern int byte1, byte2;
extern int cam_width;

extern unsigned int goalLost;
extern unsigned int spinMode;

//For storing pixel color values
extern unsigned char pixelColors[80];
extern unsigned char smoothedColors[80];

//Buffer for the camera image
extern unsigned char buffer[300];

void getColor(int pixel);
int checkYellow(int robotID);
int checkGreen(int robotID);
int checkRed(int robotID);
void smoothColorLine();
void getPenaltyCameraLine(int robotID);
void getGoalCameraLine(int robotID, int team);
int goalInView();
int getGoalMidpoint();
int inPenaltyBox();