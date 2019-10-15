
/*******about the rays and distance*******/

//               front sonar beams
//                      3 4            +/-10 degree
//                   2   ^   5         +/-30 degree
//                 1     |     6       +/-50 degree
//             ___0___center____7____  +/-90 degree
//             |         |          |
//             |         |          |
//             |         |          |
//             |      P3_ROBOT      |
//             |         |          |
//             |         |          |
//             |         |          |
//             |         |          |
//             |         |          |
//             |_________|__________|

// distance offset(mm):
//rays no.: 0   1   2   3   4   5   6   7
//offset  :160 220 240 240 240 240 220 160
/*******about the rays and distance*******/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>   //for sonar data
#include "geometry_msgs/Twist.h"
#include <math.h>
#include <iostream>


using geometry_msgs::Twist;
using namespace std;

Twist vel;

#define SONAR_NUM 8
const float PI = 3.1416;

const int OFFSET[SONAR_NUM] ={160, 220, 240, 240, 240, 240, 220, 160};
//angles of rays to heading
const float ANGLE_RAY[SONAR_NUM] = {(-1.0/2)*PI, (-5.0/18)*PI, (-1.0/6)*PI, (-1.0/18)*PI, (1.0/18)*PI, (1.0/6)*PI, (5.0/18)*PI, (1.0/2)*PI};


//distance to the obstacle in front of the sonar ray
int distToObstace[SONAR_NUM];

//desired safe distance to object (mm)
const int DIST_SAFTY = 300; 
//the obstacle within this distance(mm) is recognized as a object
const int DIST_DESIRED = 400;

const float HEADING_VEL_MAX = 0.50;
const float HEADING_VEL_MIN = -0.300;
const float ANGULAR_VEL_MAX = PI/4.0;
const float ANGULAR_VEL_MIN = PI/(-4.0);

//The time desired to catch up with the object
const float TIME_DESIRED = 1.2;
//show that if any object is detected
bool ifObjectDetected = false;

//the heading velocity to follow the object
float heading_vel = 0.0; 
//the angular velocity to follow the object
float angular_vel = 0.0;

//the angle between the object and the robot's heading
float angle_error_to_head = 0;
//the object's direction
//not used
//int objection_direction = 0;

//which ray detectes the object with te minimum distane
int ray_seq_min_dist = 3;
//the minimum distane to the object detected
int dist_to_obj_min = 0;


bool CheckIfObjectDetected()
{
    int ray_seq = 1;
    
    while(ray_seq <= 6)
    {
         if (!(distToObstace[ray_seq] > DIST_SAFTY))
         {//if one ray detects an object in effective area
             return true;
         }
         else
         {
             ray_seq++;
         }
    }
    //if no object is in effective area
    return false;

}


/***********************************
Function   : CalculateDistaneAndHeading
Description:
             Calculate the distace between the robot
              and the object, and determine if the object
              is on the left/right or in front the robot
***********************************/
void CalculateDistaneAndHeading()
{
    dist_to_obj_min = distToObstace[1];

    /*get the direction and minimum distance*/
    for (int ray_seq=2; ray_seq<=6; ray_seq++)
    {//discard ray_0 and ray_6
        if (distToObstace[ray_seq] < dist_to_obj_min)
        {
            dist_to_obj_min = distToObstace[ray_seq];
            ray_seq_min_dist = ray_seq;
        }
    }
#ifdef DEBUG_PRINT
    printf("dist_to_obj_min is %d\n", dist_to_obj_min);
#endif
    /*get the angular between HEADING AND OBJECT*/
    if (ray_seq_min_dist ==3 || ray_seq_min_dist ==4)
    {//the object is just ahead        
        angle_error_to_head = 0;
    }
    else if (ray_seq_min_dist ==1 || ray_seq_min_dist ==2)
    {//the object is on the left        
        angle_error_to_head = (ANGLE_RAY[1] + ANGLE_RAY[2])/2;
    }
    else if (ray_seq_min_dist ==5 || ray_seq_min_dist ==6)
    {//the object is on the right        
        angle_error_to_head = (ANGLE_RAY[5] + ANGLE_RAY[6])/2;
    }
    else
    {
        //we donot consider the object is verticle to heading,
        //which means
        //(ray_seq_min_dist ==0 || ray_seq_min_dist ==8)
    }
#ifdef DEBUG_PRINT
    printf("Angle_error_to_head is %f\n", angle_error_to_head);
#endif
}

/***********************************
Function   : Heading_Vel_Determination
Description:
             Determine the heading velocity
***********************************/
void Heading_Vel_Determination()
{
    heading_vel =((dist_to_obj_min - DIST_DESIRED)*1.0/1000)/TIME_DESIRED;
    if (heading_vel > 0)
        printf("Safe again!\n");
    else if (heading_vel < 0)
        printf("Too close!\n");

    printf("heading_vel = %f\n", heading_vel);
}


/***********************************
Function   : Angular_Vel_Determination
Description:
             Determine the angular velocity
***********************************/
void Angular_Vel_Determination()
{
    if (angle_error_to_head < 0)
	angular_vel = -1.0*(PI/2-angle_error_to_head) / TIME_DESIRED;
    else if(angle_error_to_head > 0)
	angular_vel = 1.0*(PI/2-angle_error_to_head) / TIME_DESIRED;
    else if(angle_error_to_head == 0)
	angular_vel = PI/2 / TIME_DESIRED;

    printf("angular_vel = %f\n", angular_vel);

}


void AvoidingObject()
{
    ifObjectDetected = CheckIfObjectDetected();
   
    if (ifObjectDetected)
    {//if object detected
#ifdef DEBUG_PRINT
        printf("Object found!\n");
#endif

        CalculateDistaneAndHeading();
        Heading_Vel_Determination();
        Angular_Vel_Determination();
  
        /*Make sure the heading vel is between 
          HEADING_VEL_MIN and HEADING_VEL_MAX*/
        if (heading_vel > HEADING_VEL_MAX)
        {
            heading_vel =  HEADING_VEL_MAX;
        }
        else if (heading_vel <  HEADING_VEL_MIN)
        {
            heading_vel =  HEADING_VEL_MIN;  
        }

        /*Make sure the angular vel is between 
          ANGULAR_VEL_MIN and ANGULAR_VEL_MAX*/
        if (angular_vel > ANGULAR_VEL_MAX)
        {
            angular_vel = ANGULAR_VEL_MAX;
        }
        else if (angular_vel < ANGULAR_VEL_MIN)
        {
            angular_vel = ANGULAR_VEL_MIN;
        }
 
        /*for publishing the vel*/
        vel.linear.x = heading_vel;
        vel.angular.z  = angular_vel;
    }
    else
    {//if object disappears, stop
        vel.linear.x = 0;
        vel.angular.z  = 0;
#ifdef DEBUG_PRINT
        printf("The object disappears!\n");
#endif
    }
#ifdef DEBUG_PRINT
    printf("vel.linear.x = %f, vel.angular.z = %f\n",vel.linear.x, vel.angular.z);
#endif
}



void Get_sonarData_Callback(const sensor_msgs::PointCloud::ConstPtr &sonarScanedData)
{
    //sensor_msgs::PointCloud sonarData;
    int seq = 0;    
    float tmpX = 0.0, tmpY=0.0;
    seq = sonarScanedData->header.seq;

    printf("seq of sonar beam  and  distance measured-->\n");
    printf("Frame[%d] :  \n", seq);

    for (int i=0; i<SONAR_NUM; i++)    
    { 
        printf("%d\t",i);
    }
    printf("\n");

    for (int i=0; i<SONAR_NUM; i++)    
    {
        tmpX= sonarScanedData->points[i].x; //x coordinate
        tmpY= sonarScanedData->points[i].y; //y coordinate
        distToObstace[i] = int(sqrt(tmpX*tmpX+tmpY*tmpY)*1000-OFFSET[i]);

        printf("%d\t",distToObstace[i]);
    }
    printf("\n\n");

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "read_sonar");
    ros::NodeHandle n;

    ros::Publisher vel_pub;
    vel_pub = n.advertise<Twist>("/RosAria/cmd_vel", 1);

    ros::Subscriber sonar = n.subscribe<sensor_msgs::PointCloud>("/RosAria/sonar", 100, Get_sonarData_Callback);
    printf("\n********** Sonar Readings: **********\n");

    while (ros::ok())
    {
      ros::Duration(0.4).sleep();          
      ros::spinOnce();

      //execute avoiding the object
      AvoidingObject();
      vel_pub.publish(vel);
    }




    return 0;
}
