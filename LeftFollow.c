/*
* File:          my_controller.c
* Date:          
* Description:   
* Author:        
* Modifications:  39
*/

/*
* You may need to add include files like <webots/distance_sensor.h> or
* <webots/differential_wheels.h>, etc.
*/
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>

#include <stdio.h>
#include <time.h>
#include <stdlib.h>

/*
* You may want to add macros here.
*/
#define TIME_STEP 64
#define WARNING_THRESHOLD 80
#define SPEED_MAGNITUDE 1 //set to make epuck faster in general. DO NOT SET HIGHER THAN 10
#define MAX_SPEED 1000
#define BUFFER_SIZE 5
#define THRESHOLD_NO_WALL 150
#define THRESHOLD_FAR_OK 200
#define THRESHOLD_FAR 250
#define THRESHOLD_CLOSE 500
#define THRESHOLD_COLLISION 1000 

#define SENSOR_RFRONT 0
#define SENSOR_RDIAG 1
#define SENSOR_RIGHT 2
#define SENSOR_RREAR 3
#define SENSOR_LREAR 4
#define SENSOR_LEFT 5
#define SENSOR_LDIAG 6
#define SENSOR_LFRONT 7
typedef enum { ST_UNDEFINED, ST_OK, ST_TOO_FAR, ST_TOO_CLOSE, ST_RIGHT_TURN, ST_LEFT_TURN_START, ST_LEFT_TURN, ST_NUM_STATES } tState  ;
char* gStateNames [] = {"ST_UNDEFINED", "ST_OK", "ST_TOO_FAR", "ST_TOO_CLOSE", "ST_RIGHT_TURN", "ST_LEFT_TURN_START", "ST_LEFT_TURN", "ST_NUM_STATES" } ;

#define NUM_SENSORS 8

#define ST_OK_SPD_L 100
#define ST_OK_SPD_R 100

#define ST_TOO_FAR_SPD_L 20
#define ST_TOO_FAR_SPD_R 100

#define ST_TOO_CLOSE_SPD_L 100
#define ST_TOO_CLOSE_SPD_R 20

#define ST_LEFT_TURN_START_SPD_L 100
#define ST_LEFT_TURN_START_SPD_R 100

#define ST_LEFT_TURN_SPD_L 40
#define ST_LEFT_TURN_SPD_R 100

#define ST_RIGHT_TURN_SPD_L 100
#define ST_RIGHT_TURN_SPD_R -100

/*
* This is the main program.
* The arguments of the main function can be specified by the
* "controllerArgs" field of the Robot node
*/

float filter_val(int sensor_num, float val)
{
    int i = 0;
    static int index[NUM_SENSORS];

    static float buffer[NUM_SENSORS][BUFFER_SIZE] ;

    buffer[sensor_num][index[sensor_num]++] = val ;

    index[sensor_num] = index[sensor_num] % BUFFER_SIZE ;

    float sum = 0 ;
    for(i = 0 ; i < BUFFER_SIZE ; i++)
    {
        sum += buffer[sensor_num][i] ;
    }
    return sum / (float)BUFFER_SIZE ;
}

void speed(FILE *file, int left, int right)
{
    if((abs((SPEED_MAGNITUDE * left)) - MAX_SPEED > 0) || (abs((SPEED_MAGNITUDE * right)) - MAX_SPEED > 0))
    {
        fprintf(file, "WARNING, SPEED SET TOO HIGH") ;
        fflush(file) ;
        left = left < 0 ? -MAX_SPEED : MAX_SPEED ;
        right = right < 0 ? -MAX_SPEED : MAX_SPEED ;
    }
    else
    {
        left = left * SPEED_MAGNITUDE ;
        right = right * SPEED_MAGNITUDE ;
    }
    wb_differential_wheels_set_speed(left,right) ;
}

int main(int argc, char **argv)
{
    /* 
    * Create a log file
    */
    FILE *file ;
    //char filename[1000] = "C:\\Users\\BJ\\Webots\\Dropbox\\Project\\Logs\\" ;
    char filename[] = "C:\\Logs\\WebotsLog.txt" ;
    file = fopen(filename, "w") ;
    fprintf(file, "   RFRONT   RDIAG     RIGHT     RREAR     LREAR     LEFT     LDIAG     LFRONT     State\n") ;
    printf("File written to %s\n",filename) ;
    fflush(file) ;
    /*
    * Initialize variables
    */
    /* state variable
    * use int instead of bool to prevent more than one being true at a time
    * 0 = OK, Following wall, relatively straight path
    * 1 = DRIFT AWAY, epuck is drifting AWAY from the wall. Detected when 5 AND 6 is below THRESHOLD_FAR. 0 and 7 below THRESHOLD_NO_WALL 
    * 2 = DRIFT_TOWARDS, epuck is drifting towards the wall. Detected when 5 AND 6 is above THRESHOLD_CLOSE and below THRESHOLD_COLLISION. 0 and 7 below THRESHOLD_NO_WALL 
    * 3 = CORNER, epuck has reached a corner/dead end and must turn right. Detected when 0 and 7 above THRESHOLD_COLLISSION and 5 and 6 above THRESHOLD_NO_WALL
    * 4 = TURN, epuck is about to turn left. Detected when 5 is above THRESHOLD_NO_WALL and 6 is below THRESHOLD_NO_WALL. 0 and 7 state doesn't matter
    */
    
    int i = 0 ;  
    tState state = ST_OK ; // Start at OK state, 0
    float ir_values[NUM_SENSORS] ;
    
    /* necessary to initialize webots stuff */
    wb_robot_init() ;
    
    /*
    * You should declare here WbDeviceTag variables for storing
    * robot devices like this:
    *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
    *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
    */
    
    WbDeviceTag ps[NUM_SENSORS];
    char ps_names[NUM_SENSORS][4] = {
        "ps0", "ps1", "ps2", "ps3",
        "ps4", "ps5", "ps6", "ps7"
    };

    for (i = 0; i < NUM_SENSORS; i++)
    {
        ps[i] = wb_robot_get_device(ps_names[i]) ;
        wb_distance_sensor_enable(ps[i], TIME_STEP) ;
    }

    // read sensors until the filtering is stable...
    int j = 0 ;
    for (j = 0 ; j < (2 * BUFFER_SIZE) ; j++)
    {
        wb_robot_step(TIME_STEP) ; 
        for(i=0; i<NUM_SENSORS; i++)
        {
            ir_values[i] = filter_val(i, wb_distance_sensor_get_value(ps[i]));
        }
    }
    
    /* main loop
    * Perform simulation steps of TIME_STEP milliseconds
    * and leave the loop when the simulation is over
    */
    while (wb_robot_step(TIME_STEP) != -1) 
    {
       /* 
        * Read the sensors :
        * Enter here functions to read sensor data, like:
        *  double val = wb_distance_sensor_get_value(my_sensor);
        */
        for(i=0; i<NUM_SENSORS; i++)
        {
            ir_values[i] = filter_val(i, wb_distance_sensor_get_value(ps[i]));
        }
        fprintf(file,"%8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %s \n",ir_values[0], ir_values[1], ir_values[2], ir_values[3], ir_values[4], ir_values[5], ir_values[6], ir_values[7], gStateNames[state]) ;
        fflush(file) ;
        
        /* Process sensor data here */
        switch(state)
        {
            case ST_OK :
            
                // Is something in my way?
                if((ir_values[SENSOR_RFRONT] > THRESHOLD_COLLISION) || 
                   (ir_values[SENSOR_LFRONT]  > THRESHOLD_COLLISION) || 
                   (ir_values[SENSOR_RDIAG] > THRESHOLD_COLLISION) ||
                   (ir_values[SENSOR_LDIAG] > THRESHOLD_COLLISION))
                {
                    printf ("Something in the way\n") ;
                    printf ("going to right turn state\n") ;
                    //Left is blocked, will treat as Corner
                    state = ST_RIGHT_TURN ;
                }
                // Am I too close to the left wall?
                //else if((ir_values[SENSOR_LEFT] > THRESHOLD_CLOSE) || (ir_values[SENSOR_LDIAG] > THRESHOLD_CLOSE))
                else if(ir_values[SENSOR_LDIAG] > THRESHOLD_CLOSE)
                {
                    printf ("Too close to left wall\n") ;
                    //Drifting towards the wall
                    state = ST_TOO_CLOSE ;
                }
                // Am I too far from the left wall?
                else if(ir_values[SENSOR_LDIAG] < THRESHOLD_FAR)
                {
                    if (ir_values[SENSOR_LDIAG] < THRESHOLD_NO_WALL)
                    {
                      printf ("LOST left wall\n") ;
                      //Drifting away from wall
                      state = ST_LEFT_TURN_START ;
                    }
                    else
                    {
                      printf ("too far from left wall\n") ;
                      //Drifting away from wall
                      state = ST_TOO_FAR ;
                    }
                }
                else
                {
                    // My positioning is good!  Forward!
                    speed(file, ST_OK_SPD_L,ST_OK_SPD_R) ;
                }
                break ;
            case ST_TOO_FAR :
                //Too far from left wall, turn back towards wall.
                
                // turn to the left...
                speed (file, ST_TOO_FAR_SPD_L, ST_TOO_FAR_SPD_R) ;
                // Did I turn enough?
                if (ir_values[SENSOR_LDIAG] > THRESHOLD_FAR_OK)  
                    state = ST_OK ; // Yes- go back to "OK"
                else if (ir_values[SENSOR_LDIAG] < THRESHOLD_NO_WALL)  
                    state = ST_LEFT_TURN_START ; // Yes- go back to "OK"
                    
                break ;
            case ST_TOO_CLOSE :
                // Too close to left wall, turn away from wall
                
                // turn to the right...
                speed (file, ST_TOO_CLOSE_SPD_L, ST_TOO_CLOSE_SPD_R) ;
                // Did I turn enough?
                if (ir_values[SENSOR_LDIAG] < THRESHOLD_CLOSE)  
                    state = ST_OK ; // Yes- go back to "OK"
                
                break ;
            case ST_LEFT_TURN_START :
                //lost left wall, Make sweeping left turn.
                
                // turn to the left...
                speed (file, ST_LEFT_TURN_START_SPD_L, ST_LEFT_TURN_START_SPD_R) ;
                // Did I turn enough?
                if (ir_values[SENSOR_LEFT] < THRESHOLD_NO_WALL)  
                    state = ST_LEFT_TURN ; // Yes- go back to "OK"
                break;
            case ST_LEFT_TURN :
                //lost left wall, Make sweeping left turn.
                
                // turn to the left...
                speed (file, ST_LEFT_TURN_SPD_L, ST_LEFT_TURN_SPD_R) ;
                // Did I turn enough?
                if (ir_values[SENSOR_LDIAG] > THRESHOLD_FAR)  
                    state = ST_OK ; // Yes- go back to "OK"
                break;
            case ST_RIGHT_TURN :
                //Something wrong, spin right
                
                // turn to the left...
                speed (file, ST_RIGHT_TURN_SPD_L, ST_RIGHT_TURN_SPD_R) ;
                // Did I turn enough?
                if ((ir_values[SENSOR_LFRONT] < THRESHOLD_NO_WALL) && (ir_values[SENSOR_RFRONT] < THRESHOLD_NO_WALL)) 
                    state = ST_OK ; // Yes- go back to "OK"
                break;
            default :
                printf("ERROR. State not set.") ;
                speed(file, 0,0) ;
                fprintf(file, "ERROR STATE NOT SET (%d)", state);
                break ;
        }
    };

    /* Enter your cleanup code here */
    printf("Closing file: %s", filename) ;
    fclose(file) ;
    /* This is necessary to cleanup webots resources */
    wb_robot_cleanup() ;

    return 0 ;
}
