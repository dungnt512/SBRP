#include "fns.h"

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
extern int totalPassengers, maxBusCapacity, kInit;
extern double maxWalkDist;
extern double minEligibilityDist;
extern string distUnits;
extern double dwellPerPassenger;
extern double dwellPerStop;
extern double excessWeight;
extern double maxJourneyTime;
extern bool useMinCoverings;

inline
void swapVals(double &x, double &y) {
	double z; z = x; x = y; y = z;
}

inline
void swapVals(int &x, int &y) {
	int z; z = x; x = y; y = z;
}

void randPermute(vector<int> &A) {
	//Randomly permutes the contents of an array A
	if (A.empty()) return;
	int i, r;
	for (i = A.size() - 1; i >= 0; i--) {
		r = rand() % (i + 1);
		swap(A[i], A[r]);
	}
}

double sumDouble(vector<double> &X) {
	//Sums the numbers in X
	double total = 0;
	for (int i = 0; i < X.size(); i++) total += X[i];
	return total;
}

bool approxEqual(double x, double y) {
	//Checks if 2 doubles are equal to 5 dps
	if (abs(x - y) < 0.00001) return true;
	else return false;
}

double calcDwellTime(int numPass) {
	//Uses the length of a route l to calculate its cost
	return dwellPerStop + numPass * dwellPerPassenger;
}

double calcRCost(double l) {
	//Uses the length of a route l to calculate its cost
	if (l > maxJourneyTime) return maxJourneyTime + excessWeight * (l - maxJourneyTime + 1);
	else return l;
}

double calcRouteLenFromScratch(SOL &S, int route) {
	//Calculates the total time to traverse in a particular route using only items and W
	if (S.items[route].empty()) return 0.0;
	int i;
	double total = 0.0;
	for (i = 0; i < S.items[route].size() - 1; i++) {
		total += calcDwellTime(S.W[route][i]) + dTime[S.items[route][i]][S.items[route][i + 1]];
	}
	total += calcDwellTime(S.W[route][i]) + dTime[S.items[route][i]][0];
	return total;
}

double calcSolCostFromScratch(SOL &S) {
	int i;
	double tCst = 0, cst;
	for (i = 0; i < S.items.size(); i++) {
		cst = calcRCost(calcRouteLenFromScratch(S, i));
		tCst += cst;
	}
	return tCst;
}

bool containsOutlierStop(vector<int> &R) {
	//Returns true if route R contains an outlier stop
	for (int i = 0; i < R.size(); i++) {
		if (isOutlier[R[i]]) return true;
	}
	return false;
}

double calcWalkCostFromScratch(SOL &S) {
	//Takes a solution S and calculates its walkCost (each student walking to closest used stop)
	int i;
	double total = 0;
	for (i = 0; i < addresses.size(); i++) {
		total += addresses[i].numPass * wTime[i][S.assignedTo[i]];
	}
	return total;
}

int calcWSum(SOL &S, int v) {
	//Calculates the number of people boarding stop v according to S[W] only
	int i, r, c, total = 0;
	for (i = 0; i < S.routeOfStop[v].size(); i++) {
		r = S.routeOfStop[v][i];
		c = S.posInRoute[v][r];
		total += S.W[r][c];
	}
	return total;
}

double roundUp(double x, double base) {
	//Rounds x up to the nearest multiple of base
	if (base != 0 && x != 0) {
		double sign;
		if (x > 0) sign = 1;
		else sign = -1;
		x *= sign;
		x /= base;
		int fixedPoint = (int)ceil(x);
		x = fixedPoint * base;
		x *= sign;
	}
	return x;
}

double roundDown(double x, double base) {
	//Rounds x down to the nearest multiple of base
	if (base != 0 && x != 0) {
		double sign;
		if (x > 0) sign = 1;
		else sign = -1;
		x *= sign;
		x /= base;
		int fixedPoint = (int)floor(x);
		x = fixedPoint * base;
		x *= sign;
	}
	return x;
}

//-------------- Some procedures for checking and validating a solution --------------------------------
void prettyPrintSol(SOL &S) {
	int i, j;
	double tLen = 0, len, cst, tCst = 0;
	cout << "Rte    Act.-Len   Claim-len  Act-Cost  numPass\n";
	for (i = 0; i < S.items.size(); i++) {
		len = calcRouteLenFromScratch(S, i);
		cst = calcRCost(len);
		cout << setw(3) << i << setw(12) << len << setw(12) << S.routeLen[i] << setw(10) << cst << setw(9) << S.passInRoute[i] << "\t";
		if (S.items[i].empty()) cout << "-\n";
		else {
			cout << "{ ";
			for (j = 0; j < S.items[i].size() - 1; j++) cout << S.items[i][j] << " ";
			cout << S.items[i][j] << " }\n";
			tLen += len;
			tCst += cst;
		}
	}
	cout << "Actual Total Len = " << tLen << endl << "Actual Total Cost = " << tCst << endl << endl;
}

void checkCoveringIsMinimal(SOL &S, bool &OK) {
	//This checks the covering defined by stopUsed is minimal (i.e. that the removal of any stop causes
	//at least one address to not be served).
	//First, for each address count the number of adjacent stops in the solution and put this info in Y
	vector<int> Y(addresses.size(), 0);
	int i, j;
	for (i = 1; i < S.stopUsed.size(); i++) {
		if (S.stopUsed[i]) {
			for (j = 0; j < stopAdjList[i].size(); j++) Y[stopAdjList[i][j]] += 1;
		}
	}
	//Now, for each used stop, see if removing this stop causes an address to have no suitable stop; in which 
	//case the stop is definately needed. If it is not needed, then the matching is not minimal
	for (i = 1; i < S.stopUsed.size(); i++) {
		if (S.stopUsed[i]) {
			//Check if removing row i would give us an incomplete covering
			for (j = 0; j < stopAdjList[i].size(); j++) {
				if (Y[stopAdjList[i][j]] == 1) {
					break;
				}
			}
			if (j >= stopAdjList[i].size()) {
				cout << "Error: Stop " << i << " is not needed. Covering is not minimal.\n";
				OK = false;
			}
		}
	}
}

bool existsCommonStop(SOL &S, int r1, int r2) {
	//Returns true if routes r1 and r2 have a common stop, false otherwise
	int i, j;
	for (i = 0; i < S.items[r1].size(); i++) {
		for (j = 0; j < S.items[r2].size(); j++) {
			if (S.items[r1][i] == S.items[r2][j]) return true;
		}
	}
	return false;
}

bool contains(vector<int> &A, int x) {
	//Returns true iff x is an element in A
	for (int i = 0; i < A.size(); i++)
		if (A[i] == x) return true;
	return false;
}

void checkSolutionValidity(SOL &S, bool shouldBeMinimal) {
	//Performs a number of checks on a solution to see if it is valid and its cost values match those claimed.
	int i, j, n = stops.size(), m = addresses.size(), passInRoute = 0, totalNumPass = 0, numUsedStops = 0, solSize = 0;
	double timeToClosestStop;
	bool OK = true;
	vector<bool> used(n, false);
	//ID all stops used in the solution
	for (i = 0; i < S.items.size(); i++) {
		solSize += S.items[i].size();
		for (j = 0; j < S.items[i].size(); j++) {
			if (S.items[i][j] < 1 || S.items[i][j] >= n) {
				cout << "Error: Solution conains an invalid value (Route " << i << ", element " << j << +")\n";
				OK = false;
			}
			else {
				if (used[S.items[i][j]] == false) numUsedStops++;
				used[S.items[i][j]] = true;
			}
		}
	}
	//Check that the claimed values for S.solSize and S.numUsedStops are correct
	if (S.solSize != solSize) {
		cout << "Error: S.solSize (" << S.solSize << ") and actual number (" << solSize << ") do not match\n";
		OK = false;
	}
	if (S.numUsedStops != numUsedStops) {
		cout << "Error: S.numUsedStops (" << S.numUsedStops << ") and actual number (" << numUsedStops << ") do not match\n";
		OK = false;
	}
	//Check for existence of duplicates in each route.
	vector<int> appearsInRoute(n, 0);
	for (i = 0;i < S.items.size(); i++) {
		for (j = 0; j < S.items[i].size(); j++) appearsInRoute[S.items[i][j]]++;
		for (j = 0; j < S.items[i].size(); j++) if (appearsInRoute[S.items[i][j]] > 1) {
			cout << "Error: Route " << i << " contains a duplicate stop (" << S.items[i][j] << ")\n";
			OK = false;
		}
		for (j = 0; j < S.items[i].size(); j++) appearsInRoute[S.items[i][j]] = 0;
	}
	//Check the validity of the routeOfStop array (both ways)
	for (i = 0; i < S.items.size(); i++) {
		for (j = 0; j < S.items[i].size(); j++) {
			if (!contains(S.routeOfStop[S.items[i][j]], i)) {
				cout << "Error: S.routeOfStop and S.items are inconsistent (a) for (stop " << S.items[i][j] << ")\n";
				OK = false;
			}
		}
	}
	for (i = 0; i < S.routeOfStop.size(); i++) {
		for (j = 0; j < S.routeOfStop[i].size(); j++) {
			if (!contains(S.items[S.routeOfStop[i][j]], i)) {
				cout << "Error: S.routeOfStop and S.items are inconsistent (b) for (stop " << i << ")\n";
				OK = false;
			}
		}
	}
	//Also check the validity of the commonStops matrix
	if (S.items.size() > 1) {
		for (i = 0; i < S.commonStop.size() - 1; i++) {
			for (j = i + 1; j < S.commonStop[i].size(); j++) {
				if (S.commonStop[i][j] != S.commonStop[j][i]) {
					cout << "Error: S.commonStop is not symmetric\n";
					OK = false;
				}
				else if (S.commonStop[i][j] != existsCommonStop(S, i, j)) {
					cout << "Error S.commonStop and solution are inconsistent\n";
					OK = false;
				}
			}
		}
	}
	//And check the validity of the S.posInRoute matrix
	for (i = 0; i < S.posInRoute.size(); i++) {
		for (j = 0; j < S.posInRoute[i].size(); j++) {
			if (S.posInRoute[i][j] != -1) {
				if (S.posInRoute[i][j] >= S.items[j].size()) {
					cout << "Error S.posInRoute is not consistent with S.items\n";
					OK = false;
				}
				else if ( S.items[j][S.posInRoute[i][j]] != i) {
					cout << "Error S.posInRoute is not consistent with S.items\n";
					OK = false;
				}
			}
		}
	}
	for (i = 0; i < S.items.size(); i++) {
		for (j = 0; j < S.items[i].size(); j++) {
			if (S.posInRoute[S.items[i][j]][i] != j) {
				cout << "Error S.posInRoute is not consistent with S.items\n";
				OK = false;
			}
		}
	}
	//Check that the correct routes contain the outliers (if there are any at all)
	int outlierRouteCnt = 0;
	for (i = 0; i < S.items.size(); i++) {
		if (containsOutlierStop(S.items[i]) == true) {
			outlierRouteCnt++;
			if (S.hasOutlier[i] == false) {
				cout << "Error: Route " << i << " is claimed to have an outlier stop, but doesn't\n";
				OK = false;
			}
		}
		else {
			if (S.hasOutlier[i] == true) {
				cout << "Error: Route " << i << " is claimed to not have an outlier stop, but it does\n";
				OK = false;
			}
		}
	}
	if (S.numRoutesWithOutliers != outlierRouteCnt) cout << "Error. Claimed value for S.numRoutesWithOutliers (" << S.numRoutesWithOutliers << ") not correct. It should be " << outlierRouteCnt << "\n";
	//Check that the S.items and S.W arrays match
	if (S.items.size() != S.W.size()) {
		cout << "Error: S.items and S.W contain different number of routes.\n";
		OK = false;
	}
	else {
		for (i = 0; i < S.items.size(); i++) {
			if (S.items[i].size() != S.W[i].size()) {
				cout << "Error: Rows " << i << " in S.items and S.W are different in size and therefore invalid.\n";
				OK = false;
			}
		}
	}
	//Check that S.W holds valid values
	for (i = 0; i < S.W.size(); i++) {
		for (j = 0; j < S.W[i].size(); j++) {
			if (S.W[i][j] < 0) {
				cout << "Error: S.W " << i << "," << j << " has value " << S.W[i][j] << ". Only +ve integers allowed (zero is also allowed when using EmptyX operator)\n";
				OK = false;
			}
		}
	}
	//Check validity of S.stopUsed
	for (i = 1; i < n; i++) {
		if (S.stopUsed[i] && !used[i]) {
			cout << "Error: Stop " << i << " is recorded as being used in the stopUsed array, but is not actually used\n";
			OK = false;
		}
		else if (!S.stopUsed[i] && used[i]) {
			cout << "Error: Stop " << i << " is being used in the solution, but the stopUsed array does not record this\n";
			OK = false;
		}
	}
	//If specified, check that S.stopUsed defines a minimal covering
	if (shouldBeMinimal && useMinCoverings) checkCoveringIsMinimal(S, OK);
	//Check that the number of recorded empty stops is correct
	int emptyCount = 0;
	for (i = 0; i < S.items.size(); i++) if (S.items[i].empty()) emptyCount++;
	if (emptyCount != S.numEmptyRoutes) {
		cout << "Error: There are " << emptyCount << " empty routes, but S records this as " << S.numEmptyRoutes << "\n";
		OK = false;
	}
	//Check that each address i is assigned to the closest used stop
	for (i = 0; i < m; i++) {
		timeToClosestStop = DBL_MAX;
		//Recall that the addrAdjList structure has, for each address, the adjacent stops in non-descending order of time
		for (j = 0; j < addrAdjList[i].size(); j++) {
			if (used[addrAdjList[i][j]]) {
				timeToClosestStop = wTime[i][addrAdjList[i][j]];
				break;
			}
		}
		if (j >= addrAdjList[i].size() || timeToClosestStop == DBL_MAX) {
			cout << "Error: Address " << i << " does not have a used stop within " << maxWalkDist << distUnits << "\n";
			OK = false;
		}
		else if (addrAdjList[i][j] != S.assignedTo[i] && wTime[i][S.assignedTo[i]] != timeToClosestStop) {
			cout << "Error: Address " << i << " is assigned to stop " << S.assignedTo[i] << " in the solution (" << wTime[i][S.assignedTo[i]] << "), while the closest available stop is " << addrAdjList[i][j] << "(" << timeToClosestStop << ")\n";
			OK = false;
		}
	}
	//Check that the number of students boarding at each stop is valid and adds up to the correct amount
	for (i = 0; i < S.W.size(); i++) {
		passInRoute = 0;
		for (j = 0; j < S.W[i].size(); j++) {
			passInRoute += S.W[i][j];
			totalNumPass += S.W[i][j];
		}
		if (passInRoute != S.passInRoute[i]) {
			cout << "Error: Route " << i << " S.passInRoute value does not match the number calculated manually\n";
			OK = false;
		}
		if (passInRoute > maxBusCapacity) {
			cout << "Error: Route " << i << " contains " << passInRoute << " passengers, which is more than the allowed maximum of " << maxBusCapacity << "\n";
			OK = false;
		}
	}
	if (totalNumPass != totalPassengers) {
		cout << "Error: Total passengers in S.W (" << totalNumPass << ") does not equal actual total number of passengers (" << totalPassengers << ")\n";
		OK = false;
	}
	//Check that all passengers are served in S
	vector<int> numAtStop(n, 0);
	for (i = 0; i < S.W.size(); i++) {
		for (j = 0; j < S.W[i].size(); j++) {
			numAtStop[S.items[i][j]] += S.W[i][j];
		}
	}
	for (i = 0; i < n; i++) {
		if (numAtStop[i] != S.numBoarding[i]) {
			cout << "Error: Total number of people recorded as using stop " << i << " in S.W " << numAtStop[i] << " not equal to number specified in S.numBoarding (" << S.numBoarding[i] << ")\n";
			OK = false;
		}
	}
	for (i = 1; i < n; i++) {
		if (S.numBoarding[i] < 0) {
			cout << "Error: Number recorded as boarding at stop " << i << " in S.numBoarding is invalid (" << S.numBoarding[i] << ")\n";
			OK = false;
		}
		if (S.numBoarding[i] == 0 && S.stopUsed[i]) {
			cout << "Error: No students boarding at stop " << i << ", but it is still being used in the solution.\n";
			OK = false;
		}
		if (S.numBoarding[i] > 0 && !S.stopUsed[i]) {
			cout << "Error: " << S.numBoarding[i] << " students boarding at stop " << i << ", but it is not recorded as being used in the solution.\n";
			OK = false;
		}
	}
	//Check that the claimed route lengths are correct
	double actualCost = 0.0, actualRouteLen;
	for (i = 0; i < S.items.size(); i++) {
		actualRouteLen = calcRouteLenFromScratch(S, i);
		actualCost += calcRCost(actualRouteLen);
		if (!approxEqual(S.routeLen[i], actualRouteLen)) {
			cout << "Error: Claimed length of Route " << i << " (" << S.routeLen[i] << ") does not match actual cost of " << calcRouteLenFromScratch(S, i) << "\n";
			OK = false;
		}
	}
	//NOTE: We check the S.cost variable directly here, but in reality there may be a discrpancy if the weight has changed
	if (!approxEqual(S.cost, actualCost)) {
		cout << "Error: Claimed cost of solution (" << S.cost << ") does not match actual cost of " << actualCost << "\n";
		OK = false;
	}
	double actualWalkCost = calcWalkCostFromScratch(S);
	if (!approxEqual(S.costWalk, actualWalkCost)) {
		cout << "Error: Claimed Walking cost of solution (" << S.costWalk << ") does not match actual cost of " << actualWalkCost << "\n";
		OK = false;
	}
	//If there is an error, report it and exit
	if (!OK) {
		prettyPrintSol(S);
		exit(1);
	}
}

//-------------- Some procedures for adding solutions to the slnSet (if apt) --------------------------------
void reduceNumBuses(SOL &S, int busesRequired) {
	//Removes empty buses from a solution until we have the required number
	int i = 0, j;
	while (i < S.items.size()) {
		if (S.items[i].empty()) {
			//Delete all information relating to this route
			S.items.erase(S.items.begin() + i);
			S.W.erase(S.W.begin() + i);
			S.routeLen.erase(S.routeLen.begin() + i);
			S.passInRoute.erase(S.passInRoute.begin() + i);
			S.hasOutlier.erase(S.hasOutlier.begin() + i);
			S.numEmptyRoutes--;
			S.numFeasibleRoutes--;
			for (int r = 0; r < S.posInRoute.size(); r++) {
				S.posInRoute[r].erase(S.posInRoute[r].begin() + i);
			}
			for (int r = 0; r < S.commonStop.size(); r++) {
				S.commonStop[r].erase(S.commonStop[r].begin() + i);
			}
			S.commonStop.erase(S.commonStop.begin() + i);
		}
		else i++;
		if (S.items.size() == busesRequired) break;
	}
	//Remember to update the routeOfBus array
	for (i = 0; i < S.routeOfStop.size(); i++) S.routeOfStop[i].clear();
	for (i = 0; i < S.items.size(); i++) for (j = 0; j < S.items[i].size(); j++) S.routeOfStop[S.items[i][j]].push_back(i);
}

void addEmptyRoute(SOL &S) {
	//Adds a single empty route to the solution S and updates all data structures
	int i, k = S.items.size();
	for (i = 0; i < k; i++) S.commonStop[i].push_back(false);
	S.commonStop.push_back(vector<bool>(k, false));
	for (i = 1; i < stops.size(); i++) S.posInRoute[i].push_back(-1);
	S.items.push_back(vector<int>());
	S.W.push_back(vector<int>());
	S.routeLen.push_back(0.0);
	S.passInRoute.push_back(0);
	S.hasOutlier.push_back(false);
	S.numEmptyRoutes++;
	S.numFeasibleRoutes++;
}

//--------------Some functions for calculating metrics for output----------------------------------------/
void calcMetrics(double &stopsPerAddr, double &addrPerStop) {
	//Calculate some metrics for output: First calculate numAddresses per stop and num stops per address
	int i, total1 = 0, total2 = 0;
	stopsPerAddr = addrPerStop = 0.0;
	for (i = 1; i < stopAdjList.size(); i++) total1 += stopAdjList[i].size();
	for (i = 0; i < addrAdjList.size(); i++) total2 += addrAdjList[i].size();
	addrPerStop = total1 / double(stopAdjList.size() - 1);
	stopsPerAddr = total2 / double(addrAdjList.size());
}

int calcSingletonStops(SOL &S) {
	//Calculate the number of stops that are used just once
	int i, numSingletonStops = 0;
	for (i = 1; i < stops.size(); i++) {
		if (S.routeOfStop[i].size() == 1) numSingletonStops++;
	}
	return numSingletonStops;
}

void getOutliers() {
	//Determines any stops that are compulsory and that are too far from the school 
	int i, j, stus;
	bool containsOutlier = false;
	isOutlier.resize(stops.size(), false);
	for (i = 1; i < stops.size(); i++) {
		if (dTime[i][0] > maxJourneyTime) {
			isOutlier[i] = true;
			cout << "Bus Stop " << i << " = \"" << stops[i].label << "\" is an outlier (" << ceil(dTime[i][0] / 60.0) << " mins from the school)\n";
			containsOutlier = true;
		}
		else if (stops[i].required) {
			//Calculate the smallest number of students who will be boarding stop i if it is being used (i.e. the num stus for whom i is their closest stop)
			stus = 0;
			for (j = 0; j < addrAdjList.size(); j++) {
				if (addrAdjList[j][0] == i) stus += addresses[j].numPass;
			}
			//and also check if this makes the stop an outlier
			if (dTime[i][0] + calcDwellTime(stus) > maxJourneyTime) {
				isOutlier[i] = true;
				cout << "Bus Stop " << i << " = \"" << stops[i].label << "\" is an outlier (" << ceil(dTime[i][0] / 60.0) << " mins from the school, plus at least " << stus << " students must board here)\n";
				containsOutlier = true;
			}
		}
	}
	if(containsOutlier) cout << "Routes containing outlier bus stops do not need to have lengths less than m_t to be feasible.\n\n";
}

