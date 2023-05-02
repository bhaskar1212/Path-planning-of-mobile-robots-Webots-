#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <webots/distance_sensor.h>
#include "e-puck-move_to_destination_location.h"
#include "robot_controller.h"
#include "cartesian.h"
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/display.h>
// tangensial/linear speed in m/s. 
// Tangensial speed = angular speed * wheel radius 
// Tangensial speed = 6.28 rad * 2.05 cm = 0.12874 m/s
#define TANGENSIAL_SPEED 0.12874
#define MAX_SPEED 6.28
#define TIME_STEP 64
// Speed of robot to spinning in place (in cycles per second)
// 1 cycle = 360 degrees. 
// Robot rotational speed = tangensial speed / (phi * axle length) 
// note: axle length is distance between wheels
// Robot rotational speed = 0.12874 / (phi*0.052) = 0.787744755
#define ROBOT_ROTATIONAL_SPEED 0.772881647

// Speed of robot to spinning in place (in degrees per second)
// Robot angular speed in degrees = robot rotational speed * 360 
// Robot angular speed in degrees = 0.787744755*360 = 283.588111888 
#define ROBOT_ANGULAR_SPEED_IN_DEGREES 278.237392796


//void wb_display_fill_rectangle(WbDeviceTag tag, int 4, int 5, float 0.5, float 0.5);
int time_step;

int getTimeStep()
{
    static int time_step = -1;
    if (time_step == -1)
        time_step = (int)wb_robot_get_basic_time_step();
    return time_step;
}

void step()
{
    if (wb_robot_step(time_step) == -1)
    {
        wb_robot_cleanup();
        exit(EXIT_SUCCESS);
    }
}

static void init()
{    
    time_step = getTimeStep();
    
    robotControllerInit(time_step);
    
    step();
}

void rootate(const double thetaDotToDestination){
    if (!isThetaEqual(thetaDotToDestination, 0))
    {
        // if the destination is on the left, robot will rotate to left
        if (thetaDotToDestination > 0)
        {
            // set robot motor to rotate left
            motorRotateLeft();
        }
        // if the destination is on the right, robot will rotate to right
        else if (thetaDotToDestination < 0)
        {
            // set robot motor to rotate right
            motorRotateRight();
        }

        // the duration needed for the robot to rotate the body to face the destination
        double duration = abs(thetaDotToDestination) / ROBOT_ANGULAR_SPEED_IN_DEGREES;
        printf("duration to face the destination: %.5f\n", duration);
        /*
        if (duration<0.1){
        motorStop();
                wb_robot_cleanup();
                      
        }
        */

        // run the simulator
        double start_time = wb_robot_get_time();
        do
        {
            step();
        }
        while (wb_robot_get_time() < start_time + duration);
    }   

}
//-------------------------------------------------
static float calculate_rotation_time(float degrees)
{
    return abs(degrees) / ROBOT_ANGULAR_SPEED_IN_DEGREES;
}

void motor_rotate_left_in_degrees(float degrees) {
    motorRotateLeft();;
    
    float duration = calculate_rotation_time(degrees);
    float start_time = wb_robot_get_time();
    do
    {
        wb_robot_step(TIME_STEP);
    } while (wb_robot_get_time() < start_time + duration);
    
    motorStop();
}

void motor_rotate_right_in_degrees(float degrees) {
    motorRotateRight();
    
    float duration = calculate_rotation_time(degrees);
    float start_time = wb_robot_get_time();
    do
    {
        wb_robot_step(TIME_STEP);
    } while (wb_robot_get_time() < start_time + duration);
    
    motorStop();
    
    
}
void motor_move(float degrees) {
    motorMoveForward();
    WbDeviceTag ps[8];
    double ps_values[8];
    char ps_names[8][4] = {
        "ps0", "ps1", "ps2", "ps3",
        "ps4", "ps5", "ps6", "ps7"
    };
    for (int i = 0; i < 8 ; i++) {
        ps[i] = wb_robot_get_device(ps_names[i]);
        wb_distance_sensor_enable(ps[i], TIME_STEP);
    }
    
    float duration = calculate_rotation_time(degrees);
    float start_time = wb_robot_get_time();
    do
    {
        wb_robot_step(TIME_STEP);
        for (int i = 0; i < 8 ; i++)
                    ps_values[i] = wb_distance_sensor_get_value(ps[i]);
                    printf("pos0: %.5f\n", ps_values[0]);
                    printf("pos1: %.5f\n", ps_values[1]);
                    printf("pos2: %.5f\n", ps_values[2]);
                    printf("pos3: %.5f\n", ps_values[3]);
                    printf("pos4: %.5f\n", ps_values[4]);
                    printf("pos5: %.5f\n", ps_values[5]);
                    printf("pos6: %.5f\n", ps_values[6]);
                    printf("pos7: %.5f\n", ps_values[7]);
                    
         if (ps_values[0]>80 || ps_values[1]>80  ) {
                        // turn left
                        
                        motorStop();
                        motor_rotate_left_in_degrees(10);
                        motorMoveForward();
                        //rootate(thetaDotToDestination);
                 
                      }
         else if ( ps_values[6]>80 || ps_values[7]>80 ) {
                        // turn left
                        
                        motorStop();
                        motor_rotate_right_in_degrees(10);
                        motorMoveForward();
                        //rootate(thetaDotToDestination);
                 
                      }
        
    } while (wb_robot_get_time() < start_time + duration);
    
    motorStop();
    
    
}

void rotate_front(float degrees) {
        motorRotateRight();
    
    float duration = calculate_rotation_time(degrees);
    float start_time = wb_robot_get_time();
    do
    {
        
        wb_robot_step(TIME_STEP);
        
    } while (wb_robot_get_time() < start_time + duration);
    
    motorStop();
    
    motorMoveForward();
    
    float duration1 = calculate_rotation_time(400);
    float start_time1 = wb_robot_get_time();
    do
    {
        wb_robot_step(TIME_STEP);
    } while (wb_robot_get_time() < start_time1+ duration1);
    
    motorStop();
}

//-------------------------------------------------
double calcDistanceToDestination(const double destinationCoordinate[2])
{
    const double *currentCoordinate = robotControllerGetRobotCoordinate();
    return calcDistance(currentCoordinate, destinationCoordinate);
}

double calcThetaDotToDestination(const double destinationCoordinate[2])
{
    const double *currentCoordinate = robotControllerGetRobotCoordinate();
    double robotHeading = robotControllerGetRobotHeading();
    double destinationTheta = calcDestinationThetaInDegrees(currentCoordinate, destinationCoordinate);
    return calcThetaDot(robotHeading, destinationTheta);
}

void back(){
        WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
        WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
        wb_motor_set_position(left_motor, INFINITY);
        wb_motor_set_position(right_motor, INFINITY);
        wb_motor_set_velocity(left_motor, 0.0);
     wb_motor_set_velocity(right_motor, 0.0);
        wb_motor_set_velocity(left_motor, -0.5*MAX_SPEED);
        wb_motor_set_velocity(right_motor, -0.5*MAX_SPEED);
}

int main(int argc, char **argv)
{
    wb_robot_init();
    int i;
    WbDeviceTag ps[8];
    char ps_names[8][4] = {
        "ps0", "ps1", "ps2", "ps3",
        "ps4", "ps5", "ps6", "ps7"
    };

  // initialize devices
    for (i = 0; i < 8 ; i++) {
        ps[i] = wb_robot_get_device(ps_names[i]);
        wb_distance_sensor_enable(ps[i], TIME_STEP);
    }



    init();
    
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

    const double destinationCoordinate[2] = {0.04, 0.25};
    
    //robotControllerMoveToDestination(destinationCoordinate);
    while (wb_robot_step(TIME_STEP) != -1){
        double ps_values[8];
        wb_supervisor_set_label( 0, "GOAL", 0.45, 0.343, 0.1, 0xff0000, 0, "Arial");
        //wb_supervisor_set_label( 0, "START", 2, 2, 0.1, 0xff0000, 0, "Arial");
        //WbDeviceTag Arena = wb_robot_get_device("rectangle arena");
        //void wb_display_draw_rectangle(, int 0.5, int 0.5, int 1, int 1);
        //double left_speed  = 0.5 * MAX_SPEED;
        //double right_speed = 0.5 * MAX_SPEED;
        // detect obstacles
        
        double thetaDotToDestination = calcThetaDotToDestination(destinationCoordinate);
   // double * currentCoordinate = robotControllerGetRobotCoordinate();
    double distanceToDestination = calcDistanceToDestination(destinationCoordinate);
    double duration = distanceToDestination / TANGENSIAL_SPEED;
/*
        printf("pos0: %.5f\n", ps_values[0]);
        printf("pos1: %.5f\n", ps_values[1]);
        printf("pos2: %.5f\n", ps_values[2]);
        printf("pos3: %.5f\n", ps_values[3]);
        printf("pos4: %.5f\n", ps_values[4]);
        printf("pos5: %.5f\n", ps_values[5]);
        printf("pos6: %.5f\n", ps_values[6]);
        printf("pos7: %.5f\n", ps_values[7]);
*/
        rootate(thetaDotToDestination);
        motorMoveForward();
            //rotate_front(180);
            
                      
             
                
                
        double start_time = wb_robot_get_time();
        
        do
        {
            
            for (i = 0; i < 8 ; i++)
                    ps_values[i] = wb_distance_sensor_get_value(ps[i]);
                    printf("pos0: %.5f\n", ps_values[0]);
                    printf("pos1: %.5f\n", ps_values[1]);
                    printf("pos2: %.5f\n", ps_values[2]);
                    printf("pos3: %.5f\n", ps_values[3]);
                    printf("pos4: %.5f\n", ps_values[4]);
                    printf("pos5: %.5f\n", ps_values[5]);
                    printf("pos6: %.5f\n", ps_values[6]);
                    printf("pos7: %.5f\n", ps_values[7]);
        
             
             if (ps_values[0]>80 && (ps_values[6]>80) ){
                        motorStop();
                        //motor_rotate_right_in_degrees(60);
                        motor_rotate_right_in_degrees(110);
                        
                        motor_move(500);
                        motor_rotate_left_in_degrees(45);
                        motor_move(200);
                        motorMoveForward();
                        //rotate_front(180);
                        //back();
                        
                      }
                      else if (ps_values[1]>80 && (ps_values[7]>80)){
                        motorStop();
                        //motor_rotate_right_in_degrees(60);
                        motor_rotate_left_in_degrees(110);
                        motor_move(500);
                        //motorStop();
                        motor_rotate_right_in_degrees(45);
                        motor_move(200);
                        motorMoveForward();
                      }
                      else if ((ps_values[1]>80 || ps_values[2]>80) && (ps_values[5]>80 ||ps_values[6]>80 )){
                        motorStop();
                        motor_rotate_right_in_degrees(180);
                        motorMoveForward();
                      }
                      
                    else if (ps_values[0]>80 || ps_values[1]>80  ) {
                        // turn left
                        
                        motorStop();
                        motor_rotate_left_in_degrees(10);
                        motorMoveForward();
                        //rootate(thetaDotToDestination);
                 
                      }
                      else if ( ps_values[6]>80 || ps_values[7]>80 ) {
                        // turn left
                        
                        motorStop();
                        motor_rotate_right_in_degrees(10);
                        motorMoveForward();
                        //rootate(thetaDotToDestination);
                 
                      }
                      else
                      
                      step();
        }
        while (wb_robot_get_time() < start_time + duration-1.5);
    
        // stop the motor
        motorStop();
    step();
    

    }
    wb_robot_cleanup();
    return EXIT_SUCCESS;
}