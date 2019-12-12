#include <stdio.h>
#include <stdlib.h>
#include "Helper.h"


void getRealLocation(double pos[3], double robotX, double robotY, double robotZ, double robotOrientation, double distance, double theta) {
	theta += robotOrientation;
	
	double x = distance * cos(theta * PI / 180.0) + robotX;
	double y = robotY;
	double z = distance * sin(theta * PI / 180.0) + robotZ;
	//if(theta>320 ||theta<50)
	//	printf("%.3f, %.3f, %.3f, %.3f\n",x,z,distance,theta);
	static double ret[3] = { x, y, z };
	pos[0] = x;
	pos[1] = y;
	pos[2] = z;
}

double min(double a, double b) {
	if (a <= b)
		return a;
	return b;
}

double max(double a, double b) {
	if (a >= b)
		return a;
	return b;
}

double distance(double point1[3], double point2[3]) {
	return sqrt((point1[0] - point2[0]) * (point1[0] - point2[0]) + (point1[1] - point2[1]) * (point1[1] - point2[1]) + (point1[2] - point2[2]) * (point1[2] - point2[2]));
}

double distanceFromPointToWall(double point[3], wall wall) {
	double top = std::abs(wall.A * point[0] + wall.B * point[2] + wall.C);
	double bottom = sqrt(wall.A * wall.A + wall.B * wall.B);
	return top / bottom;
}

bool checkWall2D(wall wall, double point[3]) {
	if (wall.pointCnt == 1)
		return distance(point, wall.firstPoint) < minDist;
	
	if (distanceFromPointToWall(point, wall) > minDist)
		return false;

	double rise = point[2] - wall.firstPoint[2];
	double run = point[0] - wall.firstPoint[0];
	double thisSlope = rise / run;
	//printf("%.3f, %.3f\n", thisSlope, wall.slope);
	//return std::abs(thisSlope - wall.slope) / ((thisSlope + wall.slope) / 2) < .2;
	//if(std::abs(thisSlope - wall.slope) < 0.2)
		//printf("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",thisSlope,wall.slope,thisSlope - wall.slope,sqrt(point[0]*point[0]+point[2]*point[2]),point[0],point[2]);
	return (std::abs(thisSlope - wall.slope) < 0.2) || (std::abs(point[0] - wall.firstPoint[0]) < 3) || (std::abs(point[2] - wall.firstPoint[2]) < 3);
	//return abs(thisSlope - oldSlope) < difference;
}

bool checkWall(wall wall, double point[3]) {
	double t = 0;
	/*if (abs(wall.xMax - wall.xMin) > ep) {
		t = (point[0] - wall.xMin) / (wall.xMax - wall.xMin);
	}
	else if (abs(wall.yMax - wall.yMin) > ep) {
		t = (point[1] - wall.yMin) / (wall.yMax - wall.yMin);
	}
	else if (abs(wall.zMax - wall.zMin) > ep) {
		t = (point[2] - wall.zMin) / (wall.zMax - wall.zMin);
	}*/
	if (wall.pointCnt != 1) {
		t = (point[0] - wall.firstPoint[0]) / (wall.secondPoint[0] - wall.firstPoint[0]);
	}
	else {
		double wallPoints[3] = {wall.xMin,wall.yMin,wall.zMin};
		return distance(point, wallPoints) <= minDist;
	}

	//Check if the point is on the wall's plane
	bool ret = abs(wall.firstPoint[0] + (wall.secondPoint[0] - wall.firstPoint[0]) * t - point[0]) < difference;
	ret &= abs(wall.firstPoint[1] + (wall.secondPoint[1] - wall.firstPoint[1]) * t - point[1]) < difference;
	ret &= abs(wall.firstPoint[2] + (wall.secondPoint[2] - wall.firstPoint[2]) * t - point[2]) < difference;

	//Check if the point is close enough to the wall
	ret &= point[0] >= wall.xMin - minDist && point[0] <= wall.xMax + minDist;
	ret &= point[1] >= wall.yMin - minDist && point[1] <= wall.yMax + minDist;
	ret &= point[2] >= wall.zMin - minDist && point[2] <= wall.zMax + minDist;
	return ret;
}

void updateWall(wall &wall, double* point, double oldDistance) {
	wall.xMin = min(wall.xMin, point[0]);
	wall.xMax = max(wall.xMax, point[0]);
	wall.yMin = min(wall.yMin, point[1]);
	wall.yMax = max(wall.yMax, point[1]);
	wall.zMin = min(wall.zMin, point[2]);
	wall.zMax = max(wall.zMax, point[2]);
	
	if (wall.pointCnt == 1) {
		for (int i = 0; i < 3; i++) {
			wall.secondPoint[i] = point[i];
		}

		wall.slope = (wall.secondPoint[2] - wall.firstPoint[2]) / (wall.secondPoint[0] - wall.firstPoint[0]);
		wall.A = wall.slope;
		wall.B = -1;
		wall.C = (-wall.slope) * wall.firstPoint[0] + wall.firstPoint[2];
	}
	else{
		double thisSlope1 = (point[2] - wall.firstPoint[2]) / (point[0] - wall.firstPoint[0]);
		double thisSlope2 = (point[2] - wall.secondPoint[2]) / (point[0] - wall.secondPoint[0]);
		double avgSlope = (thisSlope1 + thisSlope2) / 2;
		//wall.slope = (wall.slope * (wall.pointCnt - 1) + avgSlope) / wall.pointCnt;
		for (int i = 0; i < 3; i++) {
			wall.secondPoint[i] = point[i];
		}
	}
	wall.pointCnt++;

	if (oldDistance < distanceFromLiDARToCorner) {
		wall.visited = true;
	}
}

wall createWall(double point[3], double oldDistance) {
	wall newWall;
	newWall.xMin = point[0];
	newWall.xMax = point[0];
	newWall.yMin = point[1];
	newWall.yMax = point[1];
	newWall.zMin = point[2];
	newWall.zMax = point[2];
	newWall.pointCnt = 1;
	newWall.slope = 0;
	newWall.visited = false;
	for (int i = 0; i < 3; i++) {
		newWall.firstPoint[i] = point[i];
	}
	if (oldDistance < distanceFromLiDARToCorner) {
		newWall.visited = true;
	}
	return newWall;
}

int findWall(double point[3], wall* walls, int numWalls, double distance) {
	int i = 0;
	for (i; i < numWalls; i++) {
		if (checkWall2D(walls[i], point)) {
			//printf("%d\n", i);
			updateWall(walls[i], point, distance);
			return i;
		}
	}
	walls[numWalls] = createWall(point, distance);
	return numWalls;
}

double getDistanceToClosestWall(double point[3], wall* walls, int numWalls) {
	double distanceToClosestWall = oo;
	int i = 0;

	for (i; i < numWalls; i++) {
		distanceToClosestWall = min(distanceToClosestWall, distanceFromPointToWall(point, walls[i]));
	}

	return distanceToClosestWall;
}


bool checkIfPointBlocksRobot(double distance, double angle) {
	double x = distance * cos(angle * PI / 180.0);
	double y = distance * sin(angle * PI / 180.0);
	if (x < lengthOfRobot / 2.0 + extraSpace && x > 0 && std::abs(y) < widthOfRobot) {
		return false;
	}
	return true;
}


bool processPointForMapping(double initialDistanceToPoint, double initialAngle, double realPointLocation[3], double robotPosition[3],
	double robotOrientation, wall* walls, int &wallCount, std::stack<destination> &stack) {

	int wallIndex = findWall(realPointLocation, walls, wallCount, initialDistanceToPoint);
	//if(initialAngle>320 ||initialAngle<50)
	//	printf("%d\n",wallIndex);
	wall thisPointsWall = walls[wallIndex];
	if (thisPointsWall.pointCnt == 1) {
		walls[wallCount] = thisPointsWall;
		wallCount++;
	}
	if (thisPointsWall.pointCnt == 20){
		destination currentLocation;
		for (int i = 0; i < 3; i++)
			currentLocation.location[i] = robotPosition[i];
		currentLocation.isAWall = false;
		currentLocation.goForward = false;
		currentLocation.isAReturnPoint = true;
		currentLocation.orientation = robotOrientation;
		stack.push(currentLocation);

		destination wallDestination;
		wallDestination.isAWall = true;
		wallDestination.goForward = true;
		wallDestination.Wall = &walls[wallIndex];
		wallDestination.orientation = robotOrientation;
		stack.push(wallDestination);

		currentLocation.isAReturnPoint = false;
		currentLocation.Wall = &walls[wallIndex];
		stack.push(currentLocation);
	}
	return checkIfPointBlocksRobot(initialDistanceToPoint, initialAngle);
}

//Call this when the robot starts going down a path, after it's turned to face the right direction
void addReturnPositionToStack(double robotPosition[3], double robotOrientation, std::stack<destination>& stack) {
	destination currentLocation;
	currentLocation.isAWall = false;
	currentLocation.isAReturnPoint = false;
	for (int i = 0; i < 3; i++)
		currentLocation.location[i] = robotPosition[i];
	currentLocation.orientation = robotOrientation;
	stack.push(currentLocation);
}
