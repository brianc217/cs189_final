#include "e_randb.h"
#include "ir_comm.h"

// IR-related globals
extern finalDataRegister data;
extern unsigned int ir_range;
#define HI_SPEED 800
#define LO_SPEED 300
#define HALF_SPEED 500

extern float ir_bearing;
extern unsigned char ir_sensor;
extern unsigned char receivedID;
extern union comm_value msg_data;
extern unsigned int custom_msg;

// Array for storing IR data
typedef struct IR {
	finalDataRegister data;
	double time;
} IR;

extern IR robots[12];

extern char msg[80]; 
extern int time_counter;

void receiveIR();
int atObstacle(int robotID);
int atGoal(robotID);
int avoidObstacle(int robotID, int sendID);
int closeToRobot(int robotID);

// Avoid robots except the one specified by excludeID
int avoidRobot(int robotID, int sendID, int excludeID);

