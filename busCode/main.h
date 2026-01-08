#ifndef MAIN_H
#define MAIN_H

#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <set>
#include <string>
#include <string.h>
#include <vector>
#include <iterator>  
#include <algorithm>
#include <list>
#include <time.h>
#include <cmath>
#include <cfloat>
#include <iomanip>
#include <sstream>

using namespace std;

//Define the information kept on each shape
struct STOP {  
   double x;			//Longitude
   double y;			//Lattitude
   string label;		//Name of the stop
   bool required;
};

struct ADDR {
	double x;			//Longitude
	double y;			//Lattitude
	string label;		//Name of the address
	int numPass;		//Number of passengers
};

struct SOL {
	vector<vector<int> > items;			//List of stops on each route
	vector<vector<int> > W;				//Number boarding in each instance of a stop in items
	vector<vector<int> > routeOfStop;	//Gives the route number of each stop (could be blank or have multiple elements)
	vector<bool> stopUsed;				//Tells us whether the stop is being used or not
	vector<int> assignedTo;				//Tells us, for each address, the assigned bus stop (must be the closest available)
	vector<int> numBoarding;			//The total number of students who are boarding at each stop
	vector<double> routeLen;			//The total length (in seconds) of each bus's route, including dwell times
	vector<int> passInRoute;			//The total numer of students assigned to each route
	vector<bool> hasOutlier;			//True if route i has one or more outlier stop, false otherwise
	vector<vector<int> > posInRoute;	//Element i, j indicates the position of stop i in route j (set to -1 if i is not in j)
	vector<vector<bool> > commonStop;	//Element i, j is true if bus-routes i and j share a common stop, false otherwise
	double cost;						//Cost of the solution (sum of route lengths (in seconds), with weighting)
	double costWalk;					//Cost of the solution in terms of total walk time of all passengers
	int numFeasibleRoutes;				//Number of feasible routes in the solution (i.e. <= the specified maximum route length)
	int numEmptyRoutes;					//Number of empty routes in the solution (if any)
	int numUsedStops;					//Number of used stops
	int solSize;						//Total number of times all stops are used
	int numRoutesWithOutliers;			//Number of routes containing outlier stops
};

struct EVALINFO {
	bool flippedX;					//Tells us whether X section should be flipped
	bool flippedI;					//Tells us whether I section should be flipped
	double innerX;					//Total travel time of X section
	double innerI;					//Total travel time of I section
	double innerXF;					//Total travel time of X section in reverse direction
	double innerIF;					//Total travel time of I section in reverse direction
	double dwellXSection;			//Total dwell time of X section
	double dwellISection;			//Total dwell time of I section
	int passXSection;				//Total passengers in X Section
	int passISection;				//Total passengers in I Section
};

#include "input.h"
#include "optimiser.h"
#include "initsol.h"
#include "bpp.h"
#include "fns.h"
#include "setcover.h"
#include "mobj.h"

#endif //MAIN_H


