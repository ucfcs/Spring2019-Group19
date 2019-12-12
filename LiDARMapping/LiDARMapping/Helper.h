#ifndef HELPER_H
#define HELPER_H

#include <math.h>
#include <cmath>
#include <stack>

#define PI 3.14159265
#define ep 1
#define difference 500
#define minDist 500
#define oo 1e10
#define widthOfRobot 40
#define lengthOfRobot 43
#define extraSpace 0
#define distanceFromLiDARToCorner 650

typedef struct {
	double xMin, yMin, zMin;
	double xMax, yMax, zMax;
	double slope;
	double A, B, C;
	int pointCnt;
	bool visited;
	double firstPoint[3];
	double secondPoint[3];
} wall;

typedef struct {
	bool isAWall;
	bool isAReturnPoint;
	bool goForward;
	wall *Wall;
	double location[3];
	double orientation;
} destination;

double distance(double point1[3], double point2[3]);
void getRealLocation(double pos[3], double robotX, double robotY, double robotZ, double robotsOrientation, double distance, double theta);
bool checkWall(wall wall, double point[3]);
void updateWall(wall &wall, double* point, double distance);
int findWall(double point[3], wall* walls, int numWalls, double distance);
double getDistanceToClosestWall(double point[3], wall* walls, int numWalls);
bool processPointForMapping(double initialDistanceToPoint, double initialAngle, double realPointLocation[3], double robotPosition[3], 
	double robotOrientation, wall* walls, int &wallCount, std::stack<destination> &stack);
void addReturnPositionToStack(double robotPosition[3], double robotOrientation, std::stack<destination>& stack);

#endif
