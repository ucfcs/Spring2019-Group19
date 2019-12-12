#include <stdio.h>
#include <stdlib.h>
#include <cstdio>
#include <stack>
#include <thread>
#include <chrono>
#include <pthread.h>
#include <iostream>
#include <fstream>
#include <string>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header
#include "/home/odroid/mybot_ws/src/LiDARMapping/LiDARMapping/Helper.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Vector3.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

#define PI 3.14159265
#define SPEED 1
#define MINIMUM_DISTANCE_TO_BE_CONSIDERED_THERE 250

using namespace rp::standalone::rplidar;

std::stack <destination> stack;
double robotsPosition[3];
double robotsOrientation;
int scansSinceRobotWasBlocked;
bool doneScanning = false;

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;


    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

double distanceToTarget(double point[3]) {
	double ret = distance(point, robotsPosition);
	//robotsPosition[0] += cos(robotsOrientation * PI / 180.0) * SPEED;
	//robotsPosition[2] += sin(robotsOrientation * PI / 180.0) * SPEED;
	return ret;
}

bool robotIsBlocked() {
	return scansSinceRobotWasBlocked < 500;
}

void pushCurrentLocation() {
	addReturnPositionToStack(robotsPosition, robotsOrientation, stack);
}


#include <signal.h>
#include <csignal>
bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

void *verticalLiDARFunction(void *vargp) {
	std::ofstream pointCloud("/home/odroid/mybot_ws/src/LiDARMapping/LiDARMapping/point_cloud.xyz");

	const char * opt_com_path = NULL;
  	_u32         baudrateArray[2] = {115200, 256000};
  	_u32         opt_com_baudrate = 0;
   	u_result     op_result;

    	bool useArgcBaudrate = false;

	
	opt_com_path = "/dev/ttyUSB1";



    	// create the driver instance
	RPlidarDriver * drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    	if (!drv) {
        	fprintf(stderr, "insufficent memory, exit\n");
        	exit(-2);
    	}

    	rplidar_response_device_info_t devinfo;
    	bool connectSuccess = false;
    	// make connection...
    	if(useArgcBaudrate)
    	{
        	if(!drv)
            		drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
        	if (IS_OK(drv->connect(opt_com_path, opt_com_baudrate)))
        	{
            		op_result = drv->getDeviceInfo(devinfo);

            		if (IS_OK(op_result))
            		{
                		connectSuccess = true;
            		}
            		else
            		{
                		delete drv;
                		drv = NULL;
            		}
        	}
    	}
    	else
    	{
        	size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));
        	for(size_t i = 0; i < baudRateArraySize; ++i)
        	{
            		if(!drv)
                	drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
           	if(IS_OK(drv->connect(opt_com_path, baudrateArray[i])))
            	{
                	op_result = drv->getDeviceInfo(devinfo);

                	if (IS_OK(op_result))
                	{
                    		connectSuccess = true;
                    		break;
                	}
                	else
                	{
                    		delete drv;
                   		drv = NULL;
                	}
            	}
        	}
    	}
    	if (!connectSuccess) {

        	fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            		, opt_com_path);
        	//goto on_finished;
		return 0;
    	}

    	// print out the device serial number, firmware and hardware version number..
    	printf("RPLIDAR S/N: ");
    	for (int pos = 0; pos < 16 ;++pos) {
        	printf("%02X", devinfo.serialnum[pos]);
    	}

    	printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);



    	// check health...
    	if (!checkRPLIDARHealth(drv)) {
        	//goto on_finished;
			return 0;
    	}

    	signal(SIGINT, ctrlc);

	drv->startMotor();
    	// start scan...
    	drv->startScan(0,1);
	
	int pointCount = 0;

    	// fetech result and print it out...
    	while (!doneScanning) {
        	rplidar_response_measurement_node_t nodes[8192];
	
        	size_t   count = _countof(nodes);

        	op_result = drv->grabScanData(nodes, count);

        	if (IS_OK(op_result)) {
            		drv->ascendScanData(nodes, count);
            		for (int pos = 0; pos < (int)count ; ++pos) {
				if(nodes[pos].sync_quality == 0)
					continue;
			
				if (nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT != 0) {
					double position[3] = { 0 };

					getRealLocation(position, robotsPosition[0], robotsPosition[1], robotsPosition[2], robotsOrientation,
						nodes[pos].distance_q2/4.0f, (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
					char point[50];
					sprintf(point, "%.3f %.3f %.3f", position[0], position[1], position[2]);
					pointCloud << point << std::endl;
					pointCount++;
				}
            		}
        	}

        	if (ctrl_c_pressed){
            		break;
        	}
    	}
	
    	drv->stop();
    	drv->stopMotor();
	RPlidarDriver::DisposeDriver(drv);
    	drv = NULL;
	pointCloud.close();
}

int main(int argc, char * argv[]) {
    	//std::this_thread::sleep_for(std::chrono::seconds(10));
	pthread_t verticalLiDARThreadID;	
	pthread_create(&verticalLiDARThreadID, NULL, verticalLiDARFunction, NULL);

    	const char * opt_com_path = NULL;
  	_u32         baudrateArray[2] = {115200, 256000};
  	_u32         opt_com_baudrate = 0;
   	u_result     op_result;

    	bool useArgcBaudrate = false;

	
	opt_com_path = "/dev/ttyUSB0";



    // create the driver instance
	RPlidarDriver * drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    	if (!drv) {
        	fprintf(stderr, "insufficent memory, exit\n");
        	exit(-2);
    	}

    	rplidar_response_device_info_t devinfo;
    	bool connectSuccess = false;
    	// make connection...
    	if(useArgcBaudrate)
    	{
        	if(!drv)
            		drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
        	if (IS_OK(drv->connect(opt_com_path, opt_com_baudrate)))
        	{
            		op_result = drv->getDeviceInfo(devinfo);

            		if (IS_OK(op_result))
            		{
                		connectSuccess = true;
            		}
            		else
            		{
                		delete drv;
                		drv = NULL;
            		}
        	}
    	}
    else
    {
        size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));
        for(size_t i = 0; i < baudRateArraySize; ++i)
        {
            if(!drv)
                drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
            if(IS_OK(drv->connect(opt_com_path, baudrateArray[i])))
            {
                op_result = drv->getDeviceInfo(devinfo);

                if (IS_OK(op_result))
                {
                    connectSuccess = true;
                    break;
                }
                else
                {
                    delete drv;
                    drv = NULL;
                }
            }
        }
    }
    if (!connectSuccess) {

        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , opt_com_path);
        //goto on_finished;
		return 0;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);



    // check health...
    if (!checkRPLIDARHealth(drv)) {
        //goto on_finished;
		return 0;
    }

    signal(SIGINT, ctrlc);

	
	/*ros::init(argc, argv, "LiDARMapping");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<geometry_msgs::Vector3>("localization", 10);*/

	/*wall walls[5000];
	int wallCnt = 0;
	for (int i = 0; i < 3; i++)
		robotsPosition[i] = 0;
	robotsOrientation = 0;
	scansSinceRobotWasBlocked = 0;
	double targetLocation[3];
	for (int i = 0; i < 3; i++)
		targetLocation[i] = 0;
	bool goForward = false;
	double distance = 0;
	double targetOrientation = 0;*/
	bool firstScan = true;
	double firstScanDistance = 0;
	double oldPosition[3] = {0};
	bool localized = false;
	int scanCount = 0;
	for (int i = 0; i < 3; i++)
		robotsPosition[i] = 0;
	robotsOrientation = 0;	

    drv->startMotor();
    // start scan...
    drv->startScan(0,1);

    // fetech result and print it out...
    while (true) {
        rplidar_response_measurement_node_t nodes[8192];
	
	/*geometry_msgs::Vector3 msg;
	msg.x = distance;
	msg.y = targetOrientation - robotsOrientation;
	msg.z = 10.0;
	if (!goForward)
		msg.z = -msg.z;
	pub.publish(msg);*/
	
        size_t   count = _countof(nodes);

        op_result = drv->grabScanData(nodes, count);

        if (IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);
	    bool localizedThisScan = false;
            for (int pos = 0; pos < (int)count ; ++pos) {
			if(nodes[pos].sync_quality == 0)
				continue;

			//Handle Localization
			if(!localizedThisScan){
				localizedThisScan = true;
				double thisAngle = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
				double distanceToWallInFront = nodes[pos].distance_q2 / 4.0f * cos(thisAngle * PI / 180.0);
				if(!localized){
					localized = true;
					firstScanDistance = distanceToWallInFront;
					for (int i = 0; i < 3; i++)
						oldPosition[i] = robotsPosition[i];
				}
				else{
					double differenceBetweenNewAndOld = firstScanDistance - distanceToWallInFront;
					robotsPosition[0] = oldPosition[0] + differenceBetweenNewAndOld * cos(robotsOrientation * PI / 180.0);
					robotsPosition[2] = oldPosition[2] + differenceBetweenNewAndOld * sin(robotsOrientation * PI / 180.0);
				}
			}
			
			/*double position[3] = { 0 };

			getRealLocation(position, robotsPosition[0], robotsPosition[1], robotsPosition[2], robotsOrientation,
				nodes[pos].distance_q2/4.0f, (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
			if (nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT != 0) {

				if (processPointForMapping(nodes[pos].distance_q2/4.0f, (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f,
					position, robotsPosition, robotsOrientation, walls, wallCnt, stack)) {
						scansSinceRobotWasBlocked++;
						if (scansSinceRobotWasBlocked > 1000)
							scansSinceRobotWasBlocked = 1000;
					}
					else {
						scansSinceRobotWasBlocked = 0;
					}

					distance = distanceToTarget(targetLocation);
					
					while ((firstScan || distance < MINIMUM_DISTANCE_TO_BE_CONSIDERED_THERE) && !stack.empty()) {
						destination nextDestination = stack.top();
						stack.pop();
						if (!nextDestination.isAWall && !nextDestination.isAReturnPoint && (*nextDestination.Wall).visited) {
							stack.pop();
							stack.pop();
						}
						else if (nextDestination.isAWall) {
							wall targetWall = *nextDestination.Wall;
							targetLocation[0] = (targetWall.xMin + targetWall.xMax) / 2;
							targetLocation[1] = (targetWall.yMin + targetWall.yMax) / 2;
							targetLocation[2] = (targetWall.zMin + targetWall.zMax) / 2;
							targetOrientation = (180 + atan2(robotsPosition[2] - targetLocation[2], robotsPosition[0] - targetLocation[0]) * 180 / PI);
							if(targetOrientation >= 360)
								targetOrientation -= 360;
							goForward = nextDestination.goForward;
						}
						else {
							targetOrientation = nextDestination.orientation;
							for (int i = 0; i < 3; i++)
								targetLocation[i] = nextDestination.location[i];
							goForward = nextDestination.goForward;
						}
						distance = distanceToTarget(targetLocation);
						if(distance >= MINIMUM_DISTANCE_TO_BE_CONSIDERED_THERE){
							firstScan = false;
						}
					}
					
				}*/
            }
        }

        if (ctrl_c_pressed){
            break;
        }
    }
	doneScanning = true;
    	drv->stop();
    	drv->stopMotor();

	/*while (!stack.empty()) {
		destination dest = stack.top();
		stack.pop();
		if (!dest.isAWall && !dest.isAReturnPoint && (*dest.Wall).visited) {
			stack.pop();
			stack.pop();
		}
		else if (dest.isAWall) {
			wall wall = *dest.Wall;
			targetLocation[0] = (wall.xMin + wall.xMax) / 2;
			targetLocation[1] = (wall.yMin + wall.yMax) / 2;
			targetLocation[2] = (wall.zMin + wall.zMax) / 2;
			targetOrientation = (180 + atan2(robotsPosition[2] - targetLocation[2], robotsPosition[0] - targetLocation[0]) * 180 / PI);
			if (targetOrientation < 0)
				targetOrientation += 360;
			printf("Turn robot to face %.3f degrees and move to wall at point (%.3f, %.3f)\n", targetOrientation, targetLocation[0],targetLocation[2]);
		}
		else {
			printf("Move backwards to point (%.3f, %.3f) and turn robot to face %.3f degrees\n", dest.location[0], dest.location[2], dest.orientation);
		}
	}*/

	//scanf("%d",&i);

    // done!
	//on_finished:
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
    return 0;
}
