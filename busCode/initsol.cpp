#include "initsol.h"
#include "fns.h"
#include "setcover.h"

extern vector<STOP> stops;
extern vector<ADDR> addresses;
extern vector<bool> isOutlier;
extern vector<vector<double> > dDist;
extern vector<vector<double> > dTime;
extern vector<vector<double> > wDist;
extern vector<vector<double> > wTime;
extern vector<vector<int> > stopAdjList;
extern vector<vector<int> > addrAdjList;
extern vector<vector<bool> > addrStopAdj;
extern int totalPassengers, maxBusCapacity;
extern double maxWalkDist;
extern double minEligibilityDist;
extern string distUnits;
extern double dwellPerPassenger;
extern double dwellPerStop;
extern double excessWeight;
extern double maxJourneyTime;

vector<int> stopsToPack, weightOfStopsToPack;

void eliminateFromW(SOL &S, int v, int x) {
	//Eliminates x passengers from occurrences of stop v in S.W and updates S.passInRoute
	int i, r, c;
	for (i = 0; i < S.routeOfStop[v].size(); i++) {
		r = S.routeOfStop[v][i];
		c = S.posInRoute[v][r];
		if (x <= S.W[r][c]) {
			S.passInRoute[r] -= x;
			S.W[r][c] -= x;
			return;
		}
		else {
			S.passInRoute[r] -= S.W[r][c];
			x -= S.W[r][c];
			S.W[r][c] = 0;
		}
	}
	cout << "Error: should not be here at all....\n";
	exit(1);
}

void repopulateAuxiliaries(SOL &S) {
	//This procedure takes a solution defined according to the following structures
	//S.stopUsed, S.assignedTo, S.numBoarding, S.W, S.items, and S.passInRoute.
	//It then uses these to repopulate the remaining auxiliary structures (not the costs though)	
	int i, j, u, k = S.items.size();
	S.routeOfStop.clear();
	S.routeOfStop.resize(stops.size(), vector<int>());
	S.stopUsed.clear();
	S.stopUsed.resize(stops.size(), false);
	S.routeLen.clear();
	S.routeLen.resize(k, 0.0);
	S.hasOutlier.clear();
	S.hasOutlier.resize(k, false);
	S.posInRoute.clear();
	S.posInRoute.resize(stops.size(), vector<int>(k, -1));
	S.commonStop.clear();
	S.commonStop.resize(k, vector<bool>(k, false));
	S.numFeasibleRoutes = 0;
	S.numEmptyRoutes = k;
	S.numUsedStops = 0;
	S.solSize = 0;
	S.numRoutesWithOutliers = 0;
	for (i = 0; i < S.items.size(); i++) {
		//Calculate solSize and numEmptyRoutes
		S.solSize += S.items[i].size();
		for (j = 0; j < S.items[i].size(); j++) {
			u = S.items[i][j];
			//Calculate stopUsed array
			if (S.stopUsed[u] == false) {
				S.stopUsed[u] = true;
				S.numUsedStops++;
			}
			//Calculate stuff to do with outliers
			if (isOutlier[u]) S.hasOutlier[i] = true;
			//Calculate pos in route
			S.posInRoute[u][i] = j;
			S.routeOfStop[u].push_back(i);
		}
	}
	//Now populate the commonStop matrix (assuming there is more than one route)
	if (k > 1) {
		for (i = 0; i < k - 1; i++) {
			for (j = i + 1; j < k; j++) {
				if (existsCommonStop(S, i, j)) {
					S.commonStop[i][j] = true;
					S.commonStop[j][i] = true;
				}
			}
		}
	}
	//Next calculate the raw length of each route (no weightings -- just travel and dwell times) and the num routes containing outliers
	for (i = 0; i < k; i++) {
		S.routeLen[i] = calcRouteLenFromScratch(S, i);
		if (!S.items[i].empty()) S.numEmptyRoutes--;
		if (S.hasOutlier[i]) S.numRoutesWithOutliers++;
		if (S.routeLen[i] <= maxJourneyTime || S.hasOutlier[i]) S.numFeasibleRoutes++;
	}
}

void makeInitSol(SOL &S, int k, int heuristic) {
	//For the set covering algorithm a greedy algorithm is used. 
	//Heuristic: 1: choose set with most uncovered elements at each iteration
	//           2: choose any set with an uncovered element at each iteration
	//			 3: This is different, it simply takes the stops closest to each student	
	int i, j;
	
	//Initialise the arrays that define the solution 
	S.items = vector<vector<int> >(k, vector<int>());
	S.W = vector<vector<int> >(k, vector<int>());
	S.stopUsed = vector<bool>(stops.size(), false);
	S.assignedTo = vector<int>(addresses.size());
	S.numBoarding = vector<int>(stops.size(), 0);
	S.passInRoute = vector<int>(k, 0);
	
	if (heuristic != 3) {
		//Make a minimal covering for the initial set of bus stops
		generateNewCovering(S.stopUsed, -1, heuristic);
	}
	else {
		getClosestStops(S.stopUsed);
	}
	
	//Assign each address to the closest used bus stop
	for (i = 0; i < addresses.size(); i++) {
		for (j = 0; j < addrAdjList[i].size(); j++) {
			if (S.stopUsed[addrAdjList[i][j]]) {
				S.assignedTo[i] = addrAdjList[i][j];
				break;
			}
		}
	}

	//Calculate the number boarding at each stop
	for (i = 0; i < addresses.size(); i++) {
		for (j = 0; j < addrAdjList[i].size(); j++) {
			if (S.stopUsed[addrAdjList[i][j]]) {
				S.numBoarding[addrAdjList[i][j]] += addresses[i].numPass;
				break;
			}
		}
	}

	//If there are any stops with no students assigned to them, delete them from the solution
	for (i = 1; i < stops.size(); i++) {
		if (S.numBoarding[i] <= 0) {
			S.stopUsed[i] = false;
		}
	}
		
	//Use BPP style procedure to pack all the children on to k buses. First Gather together the information on 
	//each stop that is being used and the number boarding there, then pack them
	stopsToPack.clear();
	weightOfStopsToPack.clear();
	for (i = 1; i < S.stopUsed.size(); i++) {
		if (S.stopUsed[i]) {
			stopsToPack.push_back(i);
			weightOfStopsToPack.push_back(S.numBoarding[i]);
		}
	}
	binPacker(S.items, S.W, S.passInRoute, stopsToPack, weightOfStopsToPack);

	//Finally, we need to repopulate the residual structures. 
	repopulateAuxiliaries(S);

	//...and calculate the costs
	S.cost = calcSolCostFromScratch(S);
	S.costWalk = calcWalkCostFromScratch(S);
}

void rebuildSolution(SOL &S) {
	//Takes an existing solution and a new minimal covering of bus stops and adapts the solution accordingly 
	int i, j, r, c, v, x, k = S.items.size(), excess;
	//First assign each address to the closest used bus stop
	for (i = 0; i < addresses.size(); i++) {
		for (j = 0; j < addrAdjList[i].size(); j++) {
			if (S.stopUsed[addrAdjList[i][j]]) {
				S.assignedTo[i] = addrAdjList[i][j];
				break;
			}
		}
	}
	//And calculate the number boarding at each stop
	S.numBoarding.clear();
	S.numBoarding.resize(stops.size(), 0);
	for (i = 0; i < addresses.size(); i++) {
		for (j = 0; j < addrAdjList[i].size(); j++) {
			if (S.stopUsed[addrAdjList[i][j]]) {
				S.numBoarding[addrAdjList[i][j]] += addresses[i].numPass;
				break;
			}
		}
	}
	//If there are any stops with no students assigned to them, delete them from the solution
	for (i = 1; i < stops.size(); i++) {
		if (S.numBoarding[i] <= 0) {
			S.stopUsed[i] = false;
		}
	}
	//Now set up the bin packing problem by altering S.W and S.passInRoute, and building up BPP arrays
	stopsToPack.clear();
	weightOfStopsToPack.clear();
	for (v = 1; v < stops.size(); v++) {
		if (S.stopUsed[v]) {
			//Calculate number boarding stop v in the old solution (i.e. according to S.W)
			x = calcWSum(S, v);
			if (x < S.numBoarding[v]) {
				//A further (S.numBoarding[v] - x) passengers boarding at v need to packed into the solution
				stopsToPack.push_back(v);
				weightOfStopsToPack.push_back(S.numBoarding[v] - x);
			}
			else if (x > S.numBoarding[v]) {
				//some excess passengers need to be removed from instances of v in S.W (too many currently boarding v)
				excess = x - S.numBoarding[v];
				eliminateFromW(S, v, excess);
			}
		}
		else {
			//Stop v is not being used now, so set all related S[W]'s to zero and update passInRoute
			for (i = 0; i < S.routeOfStop[v].size(); i++) {
				r = S.routeOfStop[v][i];
				c = S.posInRoute[v][r];
				S.passInRoute[r] -= S.W[r][c];
				S.W[r][c] = 0;
			}
		}
	}
	//Now remove any stops in the solution for which S.W[i][j] = 0;
	for (i = 0; i < k; i++) {
		j = 0;
		while (j < S.W[i].size()) {
			if (S.W[i][j] == 0) {
				S.W[i].erase(S.W[i].begin() + j);
				S.items[i].erase(S.items[i].begin() + j);
			}
			else j++;
		}
	}
	//We can now pack the remaining stops into the solution 
	binPacker(S.items, S.W, S.passInRoute, stopsToPack, weightOfStopsToPack);

	//Finally, we need to repopulate the residual structures. 
	repopulateAuxiliaries(S);
	
	//...and calculate the costs
	S.cost = calcSolCostFromScratch(S);
	S.costWalk = calcWalkCostFromScratch(S);
}