#include "optimiser.h"
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
extern int verbosity;

//A "local" global variable for 
vector<int> tempVec1, tempVec2, tempVec3, tempVec4;

inline
void swapVals(int &a, int &b) {
	int temp = a; a = b; b = temp;
}

inline
double minVal(double a, double b) {
	if (a < b) return a; else return b;
}

inline
int minVal(int a, int b) {
	if (a < b) return a; else return b;
}

void twoOpt(vector<int> &S, int a, int b) {
	//Does a Two-Opt move
	int i = a, j = b;
	if (i > j) swapVals(i, j);
	while (i < j) {
		swapVals(S[i], S[j]);
		i++;
		j--;
	}
}

void updateSol(SOL &S, int r) {
	//Updates various data structures in the solution after changes have been made to S.items and S.W
	int passengers = 0, i;
	//Calculate the cost of the new route and keep track on the number of "feasible routes" in S
	bool routeHasOutlier = containsOutlierStop(S.items[r]);
	double newLen = calcRouteLenFromScratch(S, r);
	if (routeHasOutlier == true && S.hasOutlier[r] == false) S.numRoutesWithOutliers++;
	else if (routeHasOutlier == false && S.hasOutlier[r] == true) S.numRoutesWithOutliers--;

	if ( (routeHasOutlier || newLen <= maxJourneyTime) && !(S.routeLen[r] <= maxJourneyTime || S.hasOutlier[r]))
		S.numFeasibleRoutes++;
	else if (!(routeHasOutlier || newLen <= maxJourneyTime) && (S.routeLen[r] <= maxJourneyTime || S.hasOutlier[r]))
		S.numFeasibleRoutes--;

	S.hasOutlier[r] = routeHasOutlier;
	S.routeLen[r] = newLen;
	//Updatethe number of passengers in this route
	for (i = 0; i < S.W[r].size(); i++) 
		passengers += S.W[r][i];
	S.passInRoute[r] = passengers;
	//Update the posInRoute array
	for (i = 0; i < S.items[r].size(); i++) {
		S.posInRoute[S.items[r][i]][r] = i;
	}
}

void resetPosInRoute(SOL &S, int r) {
	//Used when a route r is about to be changed. Here, all non -1 values in column r are switched to -1s
	int i;
	for (i = 0; i < S.items[r].size(); i++) S.posInRoute[S.items[r][i]][r] = -1;
}

void removeElement(int x, vector<int> &A) {
	//Remove an element x from a vector A. (We assume there is exatly one occurence of x)
	int i;
	for (i = 0; i < A.size(); i++) if (A[i] == x) break;
	if (i >= A.size()) { cout << "Error. Element must be present\n"; exit(1); }
	swapVals(A[i], A.back());
	A.pop_back();
}

void updateRouteOfStops(SOL &S, int x, int y1, int y2, int i, int j1, int j2) {
	//Used when two routes, x and i, are about to be changed
	int y, j;
	for (y = y1; y < y2; y++) removeElement(x, S.routeOfStop[S.items[x][y]]);
	for (j = j1; j < j2; j++) removeElement(i, S.routeOfStop[S.items[i][j]]);
	for (y = y1; y < y2; y++) S.routeOfStop[S.items[x][y]].push_back(i);
	for (j = j1; j < j2; j++) S.routeOfStop[S.items[i][j]].push_back(x);
}

void updateCommonStopMatrix(SOL &S, int r) {
	//Updates the common stops matrix based on the move that has just been made 
	int i;
	for (i = 0; i < S.items.size(); i++) {
		if (i != r) {
			if (existsCommonStop(S, r, i)) {
				S.commonStop[i][r] = true;
				S.commonStop[r][i] = true;
			}
			else {
				S.commonStop[i][r] = false;
				S.commonStop[r][i] = false;
			}
		}
	}
}

int checkForPresence(SOL &S, int item, int r) {
	//Checks if "item" occurs in route r. If so its position is returned, else -1 is returned
	return (S.posInRoute[item][r]);
}

int checkForPresence(SOL &S, int item, int r, int a, int b) {
	//Checks if "item" occurs in positions [(0,...,(a - 1)] or [b,...,(n - 1)] of route r. If so its position is returned, else -1 is returned
	int pos = S.posInRoute[item][r];
	if (pos == -1 || (pos >= a && pos < b)) return -1;
	else return pos;
}

void insertSection(SOL &S, int x, int y1, int y2, int i, int j1) {
	//Inserts section from route x into route i and eliminates duplicates
	int y, pos;
	tempVec1.clear();
	tempVec2.clear();
	//Form the section to be inserted into i
	for (y = y1; y < y2; y++) {
		pos = checkForPresence(S, S.items[x][y], i);
		if (pos == -1) {
			//The item in route x at position y is not duplicated in route i, so simply copy the stop
			tempVec1.push_back(S.items[x][y]);
			tempVec2.push_back(S.W[x][y]);
		}
		else {
			//The item in route x at position y is duplicated in route i, so we don't copy it but re-assign the affected passengers
			S.W[i][pos] += S.W[x][y];
			S.solSize--;
		}
	}
	//Now insert these sections into i and delete from x
	resetPosInRoute(S, i);
	resetPosInRoute(S, x);

	//update the routeOfStops structure (cannot use the function above for doing this as some elements may have been eliminated
	for (y = y1; y < y2; y++) removeElement(x, S.routeOfStop[S.items[x][y]]);
	for (y = 0; y < tempVec1.size(); y++) S.routeOfStop[tempVec1[y]].push_back(i);

	S.items[i].insert(S.items[i].begin() + j1, tempVec1.begin(), tempVec1.end());
	S.items[x].erase(S.items[x].begin() + y1, S.items[x].begin() + y2);
	S.W[i].insert(S.W[i].begin() + j1, tempVec2.begin(), tempVec2.end());
	S.W[x].erase(S.W[x].begin() + y1, S.W[x].begin() + y2);
}

void swapSections(SOL &S, int x, int y1, int y2, int i, int j1, int j2) {
	//Swaps sections from route x and route i and eliminates duplicates
	int y, j, pos;
	tempVec1.clear();
	tempVec2.clear();
	tempVec3.clear();
	tempVec4.clear();
	//Form the section to be inserted into route i
	for (y = y1; y < y2; y++) {
		pos = checkForPresence(S, S.items[x][y], i, j1, j2);
		if (pos == -1) {
			//The item in route x at position y is not duplicated in route i (outside of the selected section), so simply copy the stop
			tempVec1.push_back(S.items[x][y]);
			tempVec2.push_back(S.W[x][y]);
		}
		else {
			//The item in route x at position y is duplicated in route i, so we don't copy it but need to re-assign the affected passengers
			//cout << "Removing instance of stopa " << S.items[x][y] << endl;
			S.W[i][pos] += S.W[x][y];
			S.solSize--;
		}
	}
	//Form the section to be inserted into route x
	for (j = j1; j < j2; j++) {
		pos = checkForPresence(S, S.items[i][j], x, y1, y2);
		if (pos == -1) {
			//The item in route i at position j is not duplicated in route x (outside of the selected section), so simply copy the stop
			tempVec3.push_back(S.items[i][j]);
			tempVec4.push_back(S.W[i][j]);
		}
		else {
			//The item in route i at position j is duplicated in route x, so we don't copy it but need to re-assign the affected passengers
			//cout << "Removing instance of stopb " << S.items[i][j] << endl;
			S.W[x][pos] += S.W[i][j];
			S.solSize--;
		}
	}
	resetPosInRoute(S, i);
	resetPosInRoute(S, x);
	
	//update the routeOfStops structure (cannot use the function above for doing this as some elements may have been eliminated
	for (y = y1; y < y2; y++) removeElement(x, S.routeOfStop[S.items[x][y]]);
	for (j = j1; j < j2; j++) removeElement(i, S.routeOfStop[S.items[i][j]]);
	for (y = 0; y < tempVec1.size(); y++) S.routeOfStop[tempVec1[y]].push_back(i);
	for (j = 0; j < tempVec3.size(); j++) S.routeOfStop[tempVec3[j]].push_back(x);

	//Reform route i
	S.items[i].erase(S.items[i].begin() + j1, S.items[i].begin() + j2);
	S.items[i].insert(S.items[i].begin() + j1, tempVec1.begin(), tempVec1.end());
	S.W[i].erase(S.W[i].begin() + j1, S.W[i].begin() + j2);
	S.W[i].insert(S.W[i].begin() + j1, tempVec2.begin(), tempVec2.end());
	//Reform route x	
	S.items[x].erase(S.items[x].begin() + y1, S.items[x].begin() + y2);
	S.items[x].insert(S.items[x].begin() + y1, tempVec3.begin(), tempVec3.end());
	S.W[x].erase(S.W[x].begin() + y1, S.W[x].begin() + y2);
	S.W[x].insert(S.W[x].begin() + y1, tempVec4.begin(), tempVec4.end());
}

void doMove1(SOL &S, int x, int y1, int y2, int i, double newCost, bool flippedx) {
	//Insert a section from x into the empty route i
	if (flippedx) {
		twoOpt(S.items[x], y1, y2 - 1);
		twoOpt(S.W[x], y1, y2 - 1);
	}
	resetPosInRoute(S, i);
	resetPosInRoute(S, x);
	updateRouteOfStops(S, x, y1, y2, i, 0, 0);
	if (!(y1 == 0 && y2 == S.items[x].size())) S.numEmptyRoutes--;
	S.items[i].insert(S.items[i].begin(), S.items[x].begin() + y1, S.items[x].begin() + y2);
	S.items[x].erase(S.items[x].begin() + y1, S.items[x].begin() + y2);
	S.W[i].insert(S.W[i].begin(), S.W[x].begin() + y1, S.W[x].begin() + y2);
	S.W[x].erase(S.W[x].begin() + y1, S.W[x].begin() + y2);
	updateCommonStopMatrix(S, x);
	updateCommonStopMatrix(S, i);
	updateSol(S, x);
	updateSol(S, i);
	S.cost = newCost;
}

void doMove2(SOL &S, int x, int y1, int y2, int i, int j1, double newCost, bool flippedx) {
	//Insert a section from route x into the non-empty route i
	if (y1 == 0 && y2 == S.items[x].size()) S.numEmptyRoutes++;
	if (flippedx) {
		twoOpt(S.items[x], y1, y2 - 1);
		twoOpt(S.W[x], y1, y2 - 1);
	}
	if (S.commonStop[x][i] == false) {
		//No common stops in routes i and x so can make the changes very simply
		resetPosInRoute(S, i);
		resetPosInRoute(S, x);
		updateRouteOfStops(S, x, y1, y2, i, 0, 0);
		S.items[i].insert(S.items[i].begin() + j1, S.items[x].begin() + y1, S.items[x].begin() + y2);
		S.items[x].erase(S.items[x].begin() + y1, S.items[x].begin() + y2);
		S.W[i].insert(S.W[i].begin() + j1, S.W[x].begin() + y1, S.W[x].begin() + y2);
		S.W[x].erase(S.W[x].begin() + y1, S.W[x].begin() + y2);
	}
	else {
		//Routes x and i contain common stops so we need to take care to delete these if they end up in the same route
		insertSection(S, x, y1, y2, i, j1);
	}
	updateCommonStopMatrix(S, x);
	updateCommonStopMatrix(S, i);
	updateSol(S, x);
	updateSol(S, i);
	S.cost = newCost;
}

void doMove3(SOL &S, int x, int y1, int y2, int i, int j1, int j2, double newCost, bool flippedx, bool flippedi) {
	//Swap a section from route x and a section from route i
	if (flippedx) {
		twoOpt(S.items[x], y1, y2 - 1);
		twoOpt(S.W[x], y1, y2 - 1);
	}
	if (flippedi) {
		twoOpt(S.items[i], j1, j2 - 1);
		twoOpt(S.W[i], j1, j2 - 1);
	}
	if (S.commonStop[x][i] == false) {
		//No common stops in routes i and x so can make the changes very simply
		resetPosInRoute(S, i);
		resetPosInRoute(S, x);
		updateRouteOfStops(S, x, y1, y2, i, j1, j2);
		tempVec1.clear();
		tempVec1.assign(S.items[i].begin() + j1, S.items[i].begin() + j2);
		S.items[i].erase(S.items[i].begin() + j1, S.items[i].begin() + j2);
		S.items[i].insert(S.items[i].begin() + j1, S.items[x].begin() + y1, S.items[x].begin() + y2);
		S.items[x].erase(S.items[x].begin() + y1, S.items[x].begin() + y2);
		S.items[x].insert(S.items[x].begin() + y1, tempVec1.begin(), tempVec1.end());
		tempVec1.clear();
		tempVec1.assign(S.W[i].begin() + j1, S.W[i].begin() + j2);
		S.W[i].erase(S.W[i].begin() + j1, S.W[i].begin() + j2);
		S.W[i].insert(S.W[i].begin() + j1, S.W[x].begin() + y1, S.W[x].begin() + y2);
		S.W[x].erase(S.W[x].begin() + y1, S.W[x].begin() + y2);
		S.W[x].insert(S.W[x].begin() + y1, tempVec1.begin(), tempVec1.end());
	}
	else {
		//Routes i and x contain common stops so we need to take care to delete these if they end up in the same route
		swapSections(S, x, y1, y2, i, j1, j2);
	}
	if (S.items[x].empty()) S.numEmptyRoutes++;
	if (S.items[i].empty()) S.numEmptyRoutes++;
	updateCommonStopMatrix(S, x);
	updateCommonStopMatrix(S, i);
	updateSol(S, x);
	updateSol(S, i);
	S.cost = newCost;
}

void doMove4(SOL &S, int x, int y1, int y2, double newCost){
	//Swap two stops in a route x
	resetPosInRoute(S, x);
	swapVals(S.items[x][y1], S.items[x][y2]);
	swapVals(S.W[x][y1], S.W[x][y2]);
	updateSol(S, x);
	S.cost = newCost;
}

void doMove5(SOL &S, int x, int y1, int y2, double newCost){
	//Do a two-opt in a single route x
	resetPosInRoute(S, x);
	twoOpt(S.items[x], y1, y2);
	twoOpt(S.W[x], y1, y2);
	updateSol(S, x);
	S.cost = newCost;
}

void doMove6(SOL &S, int x, int y1, int y2, int z, double newCost, bool flippedx) {
	//So an Or-opt on a single route x
	if (flippedx) {
		twoOpt(S.items[x], y1, y2);
		twoOpt(S.W[x], y1, y2);
	}
	resetPosInRoute(S, x);
	tempVec1.clear();
	tempVec1.assign(S.items[x].begin() + y1, S.items[x].begin() + (y2 + 1));
	S.items[x].erase(S.items[x].begin() + y1, S.items[x].begin() + (y2 + 1));
	if (y1 > z)		S.items[x].insert(S.items[x].begin() + z, tempVec1.begin(), tempVec1.end());
	else			S.items[x].insert(S.items[x].begin() + (z - tempVec1.size()), tempVec1.begin(), tempVec1.end());
	tempVec1.clear();
	tempVec1.assign(S.W[x].begin() + y1, S.W[x].begin() + (y2 + 1));
	S.W[x].erase(S.W[x].begin() + y1, S.W[x].begin() + (y2 + 1));
	if (y1 > z)		S.W[x].insert(S.W[x].begin() + z, tempVec1.begin(), tempVec1.end());
	else			S.W[x].insert(S.W[x].begin() + (z - tempVec1.size()), tempVec1.begin(), tempVec1.end());
	updateSol(S, x);
	S.cost = newCost;
}

void doMove7(SOL &S, int x, int i, int j, double newCost) {
	int u, v = S.items[i][j], pos = S.posInRoute[v][x], bestInsertPos = -1, spareCapX = maxBusCapacity - S.passInRoute[x], toTransfer;
	double minCost, inserCost;
	if (pos == -1 && S.items[x].size() > 0) {
		//Stop v is not in nonempty route x, so we ID the best point to insert it (before stop "bestInsertPos")
		minCost = dTime[v][S.items[x][0]];
		bestInsertPos = 0;
		for (u = 1; u < S.items[x].size(); u++) {
			inserCost = dTime[S.items[x][u - 1]][v] + dTime[v][S.items[x][u]] - dTime[S.items[x][u - 1]][S.items[x][u]];
			if(inserCost < minCost){
				minCost = inserCost;
				bestInsertPos = u;
			}
		}
		inserCost = dTime[S.items[x].back()][v] + dTime[v][0] - dTime[S.items[x].back()][0];
		if (inserCost < minCost) {
			minCost = inserCost;
			bestInsertPos = S.items[x].size();
		}
	}
	//Now calculate how many passengers we will transfer from v in route i, to v in route j
	if (S.routeLen[i] < maxJourneyTime) toTransfer = 1;
	else toTransfer = minVal(S.W[i][j] - 1, spareCapX);
	//Now do the move
	if (pos != -1) {
		//We are just transferring passengers between existing multistops
		S.W[i][j] -= toTransfer;
		S.W[x][pos] += toTransfer;
		updateSol(S, i);
		updateSol(S, x);
		S.cost = newCost;
	}
	else if (S.items[x].empty()) {
		//We are inserting a copy of v into an empty route
		S.W[i][j] -= toTransfer;
		S.items[x].push_back(v);
		S.W[x].push_back(toTransfer);
		S.routeOfStop[v].push_back(x);
		updateCommonStopMatrix(S, x);
		updateCommonStopMatrix(S, i);
		updateSol(S, i);
		updateSol(S, x);
		S.cost = newCost;
		S.numEmptyRoutes--;
		S.solSize++;
	}
	else {
		//We are making a copy of v and inserting it into position "bestInsertPos"
		S.W[i][j] -= toTransfer;
		S.items[x].insert(S.items[x].begin() + bestInsertPos, v);
		S.W[x].insert(S.W[x].begin() + bestInsertPos, toTransfer);
		S.routeOfStop[v].push_back(x);
		updateCommonStopMatrix(S, x);
		updateCommonStopMatrix(S, i);
		updateSol(S, i);
		updateSol(S, x);
		S.cost = newCost;
		S.solSize++;
	}
}

void evaluateOrOpt(double &newCost, SOL &S, int route, int x, int y, int z, EVALINFO &info)
{
	//Evaluate the effect of removing sequence (x,....,y) reinserting before pos z
	int n = S.items[route].size();
	double newLen, newLenF;
	if (z >= x && z <= y + 1) {
		cout << "Error should not be here\n"; 
		exit(1);
	} 
	else if (x == 0 && z == n) {
		newLen = S.routeLen[route] - dTime[S.items[route][y]][S.items[route][y + 1]] - dTime[S.items[route][z - 1]][0] + dTime[S.items[route][z - 1]][S.items[route][x]] + dTime[S.items[route][y]][0];
		newLenF = S.routeLen[route] - dTime[S.items[route][y]][S.items[route][y + 1]] - dTime[S.items[route][z - 1]][0] + dTime[S.items[route][z - 1]][S.items[route][y]] + dTime[S.items[route][x]][0] - info.innerX + info.innerXF;
	}
	else if (x == 0) {
		newLen = S.routeLen[route] - dTime[S.items[route][y]][S.items[route][y + 1]] - dTime[S.items[route][z - 1]][S.items[route][z]] + dTime[S.items[route][z - 1]][S.items[route][x]] + dTime[S.items[route][y]][S.items[route][z]];
		newLenF = S.routeLen[route] - dTime[S.items[route][y]][S.items[route][y + 1]] - dTime[S.items[route][z - 1]][S.items[route][z]] + dTime[S.items[route][z - 1]][S.items[route][y]] + dTime[S.items[route][x]][S.items[route][z]] - info.innerX + info.innerXF;
	}
	else if (z == n) {
		newLen = S.routeLen[route] - dTime[S.items[route][x - 1]][S.items[route][x]] - dTime[S.items[route][y]][S.items[route][y + 1]] - dTime[S.items[route][z - 1]][0] + dTime[S.items[route][x - 1]][S.items[route][y + 1]] + dTime[S.items[route][z - 1]][S.items[route][x]] + dTime[S.items[route][y]][0];
		newLenF = S.routeLen[route] - dTime[S.items[route][x - 1]][S.items[route][x]] - dTime[S.items[route][y]][S.items[route][y + 1]] - dTime[S.items[route][z - 1]][0] + dTime[S.items[route][x - 1]][S.items[route][y + 1]] + dTime[S.items[route][z - 1]][S.items[route][y]] + dTime[S.items[route][x]][0] - info.innerX + info.innerXF;
	}
	else if (y == n - 1 && z == 0) {
		newLen = S.routeLen[route] - dTime[S.items[route][x - 1]][S.items[route][x]] - dTime[S.items[route][y]][0] + dTime[S.items[route][x - 1]][0] + dTime[S.items[route][y]][S.items[route][z]];
		newLenF = S.routeLen[route] - dTime[S.items[route][x - 1]][S.items[route][x]] - dTime[S.items[route][y]][0] + dTime[S.items[route][x - 1]][0] + dTime[S.items[route][x]][S.items[route][z]] - info.innerX + info.innerXF;
	}
	else if (y == n - 1) {
		newLen = S.routeLen[route] - dTime[S.items[route][x - 1]][S.items[route][x]] - dTime[S.items[route][y]][0] - dTime[S.items[route][z - 1]][S.items[route][z]] + dTime[S.items[route][x - 1]][0] + dTime[S.items[route][z - 1]][S.items[route][x]] + dTime[S.items[route][y]][S.items[route][z]];
		newLenF = S.routeLen[route] - dTime[S.items[route][x - 1]][S.items[route][x]] - dTime[S.items[route][y]][0] - dTime[S.items[route][z - 1]][S.items[route][z]] + dTime[S.items[route][x - 1]][0] + dTime[S.items[route][z - 1]][S.items[route][y]] + dTime[S.items[route][x]][S.items[route][z]] - info.innerX + info.innerXF;
	}
	else if (z == 0) {
		newLen = S.routeLen[route] - dTime[S.items[route][x - 1]][S.items[route][x]] - dTime[S.items[route][y]][S.items[route][y + 1]] + dTime[S.items[route][x - 1]][S.items[route][y + 1]] + dTime[S.items[route][y]][S.items[route][z]];
		newLenF = S.routeLen[route] - dTime[S.items[route][x - 1]][S.items[route][x]] - dTime[S.items[route][y]][S.items[route][y + 1]] + dTime[S.items[route][x - 1]][S.items[route][y + 1]] + dTime[S.items[route][x]][S.items[route][z]] - info.innerX + info.innerXF;
	}
	else {
		newLen = S.routeLen[route] - dTime[S.items[route][x - 1]][S.items[route][x]] - dTime[S.items[route][y]][S.items[route][y + 1]] - dTime[S.items[route][z - 1]][S.items[route][z]] + dTime[S.items[route][z - 1]][S.items[route][x]] + dTime[S.items[route][y]][S.items[route][z]] + dTime[S.items[route][x - 1]][S.items[route][y + 1]];
		newLenF = S.routeLen[route] - dTime[S.items[route][x - 1]][S.items[route][x]] - dTime[S.items[route][y]][S.items[route][y + 1]] - dTime[S.items[route][z - 1]][S.items[route][z]] + dTime[S.items[route][z - 1]][S.items[route][y]] + dTime[S.items[route][x]][S.items[route][z]] + dTime[S.items[route][x - 1]][S.items[route][y + 1]] - info.innerX + info.innerXF;
	}
	if (newLen <= newLenF) info.flippedX = false;
	else info.flippedX = true;
	newLen = minVal(newLen, newLenF);
	newCost = S.cost - calcRCost(S.routeLen[route]) + calcRCost(newLen);
}
	
void evaluateSwapTwoOpt(double &newCost, SOL &S, int route, int x, int y, int moveType, EVALINFO &info)
{
	if (x == y) { newCost = S.cost;	return; }
	//Given current solution and cost, calculates the new cost according to x <= y and moveType
	int xl = x - 1, xr = x + 1, yl = y - 1, yr = y + 1, n = S.items[route].size();
	double newLen;
	if (moveType == 5 || x == y - 1) {
		//Evaluating a twoOpt. NB: A two-opt with 2 consecutive locations is the same as a swap
		if (x == 0 && y == n - 1)	newLen = S.routeLen[route] - dTime[S.items[route][y]][0] + dTime[S.items[route][x]][0] - info.innerX + info.innerXF;
		else if (x == 0)			newLen = S.routeLen[route] - dTime[S.items[route][y]][S.items[route][yr]] + dTime[S.items[route][x]][S.items[route][yr]] - info.innerX + info.innerXF;
		else if (y == n - 1)		newLen = S.routeLen[route] - dTime[S.items[route][xl]][S.items[route][x]] - dTime[S.items[route][y]][0] + dTime[S.items[route][xl]][S.items[route][y]] + dTime[S.items[route][x]][0] - info.innerX + info.innerXF;
		else						newLen = S.routeLen[route] - dTime[S.items[route][xl]][S.items[route][x]] - dTime[S.items[route][y]][S.items[route][yr]] + dTime[S.items[route][xl]][S.items[route][y]] + dTime[S.items[route][x]][S.items[route][yr]] - info.innerX + info.innerXF;
	}
	else {
		//Evaluating a swap (the swapping of two consecutive elements is covered above)
		if (x == 0 && y == n - 1)	newLen = S.routeLen[route] - dTime[S.items[route][x]][S.items[route][xr]] - dTime[S.items[route][yl]][S.items[route][y]] - dTime[S.items[route][y]][0] + dTime[S.items[route][y]][S.items[route][xr]] + dTime[S.items[route][yl]][S.items[route][x]] + dTime[S.items[route][x]][0];
		else if (x == 0)			newLen = S.routeLen[route] - dTime[S.items[route][x]][S.items[route][xr]] - dTime[S.items[route][yl]][S.items[route][y]] - dTime[S.items[route][y]][S.items[route][yr]] + dTime[S.items[route][y]][S.items[route][xr]] + dTime[S.items[route][yl]][S.items[route][x]] + dTime[S.items[route][x]][S.items[route][yr]];
		else if (y == n - 1)		newLen = S.routeLen[route] - dTime[S.items[route][xl]][S.items[route][x]] - dTime[S.items[route][x]][S.items[route][xr]] - dTime[S.items[route][yl]][S.items[route][y]] - dTime[S.items[route][y]][0] + dTime[S.items[route][xl]][S.items[route][y]] + dTime[S.items[route][y]][S.items[route][xr]] + dTime[S.items[route][yl]][S.items[route][x]] + dTime[S.items[route][x]][0];
		else						newLen = S.routeLen[route] - dTime[S.items[route][xl]][S.items[route][x]] - dTime[S.items[route][x]][S.items[route][xr]] - dTime[S.items[route][yl]][S.items[route][y]] - dTime[S.items[route][y]][S.items[route][yr]] + dTime[S.items[route][xl]][S.items[route][y]] + dTime[S.items[route][y]][S.items[route][xr]] + dTime[S.items[route][yl]][S.items[route][x]] + dTime[S.items[route][x]][S.items[route][yr]];
	}
	newCost = S.cost - calcRCost(S.routeLen[route]) + calcRCost(newLen);
}

void evaluateInterEmpty(double &newCost, SOL &S, int i, int x, int y1, int y2, EVALINFO &info)
{
	//We are inserting a section from route x into the empty route i. Calculate the result of doing this
	double newLenx, newLeni, newLeniF;
	newLeni = dTime[S.items[x][y2 - 1]][0] + info.innerX + info.dwellXSection;
	newLeniF = dTime[S.items[x][y1]][0] + info.innerXF + info.dwellXSection;
	//Determine which is better and proceed with this result
	if (newLeni <= newLeniF) info.flippedX = false;
	else info.flippedX = true;
	newLeni = minVal(newLeni, newLeniF);
	//Also calculate the result of removing the section from x 
	if (y1 == 0) {
		if (y2 == S.items[x].size())		newLenx = S.routeLen[x] - dTime[S.items[x][y2 - 1]][0] - info.innerX - info.dwellXSection;
		else								newLenx = S.routeLen[x] - dTime[S.items[x][y2 - 1]][S.items[x][y2]] - info.innerX - info.dwellXSection;
	}
	else if (y2 == S.items[x].size())		newLenx = S.routeLen[x] - dTime[S.items[x][y1 - 1]][S.items[x][y1]] - dTime[S.items[x][y2 - 1]][0] + dTime[S.items[x][y1 - 1]][0] - info.innerX - info.dwellXSection;
	else									newLenx = S.routeLen[x] - dTime[S.items[x][y1 - 1]][S.items[x][y1]] - dTime[S.items[x][y2 - 1]][S.items[x][y2]] + dTime[S.items[x][y1 - 1]][S.items[x][y2]] - info.innerX - info.dwellXSection;
	newCost = S.cost - calcRCost(S.routeLen[x])	+ calcRCost(newLenx) + calcRCost(newLeni);
}

double calcRealInsertCost(SOL &S, int i, int j1, vector<int> &xSection, double internalX, EVALINFO &info) {
	//We are INSERTING xSection before position S.items[i][j1]. Calculate the result of doing this
	if (xSection.empty())
		return S.routeLen[i] + info.dwellXSection;
	else if (j1 == 0) 
		return S.routeLen[i] + dTime[xSection.back()][S.items[i][j1]] + internalX + info.dwellXSection;
	else if (j1 == S.items[i].size()) 
		return S.routeLen[i] - dTime[S.items[i][j1 - 1]][0] + dTime[S.items[i][j1 - 1]][xSection.front()] + dTime[xSection.back()][0] + internalX + info.dwellXSection;
	else 
		return S.routeLen[i] - dTime[S.items[i][j1 - 1]][S.items[i][j1]] + dTime[S.items[i][j1 - 1]][xSection.front()] + dTime[xSection.back()][S.items[i][j1]] + internalX + info.dwellXSection;
}

void evaluateInsert(double &newCost, SOL &S, int i, int j1, int x, int y1, int y2, EVALINFO &info)
{
	double newLenx, newLeni, newLeniF;
	//We are insertnig a section from route x before position j1 in route i. Calculate the result of doing this
	if (j1 == 0)						newLeni = S.routeLen[i] + dTime[S.items[x][y2 - 1]][S.items[i][j1]] + info.innerX + info.dwellXSection;
	else if (j1 == S.items[i].size())	newLeni = S.routeLen[i] - dTime[S.items[i][j1 - 1]][0] + dTime[S.items[i][j1 - 1]][S.items[x][y1]] + dTime[S.items[x][y2 - 1]][0] + info.innerX + info.dwellXSection;
	else								newLeni = S.routeLen[i] - dTime[S.items[i][j1 - 1]][S.items[i][j1]] + dTime[S.items[i][j1 - 1]][S.items[x][y1]] + dTime[S.items[x][y2 - 1]][S.items[i][j1]] + info.innerX + info.dwellXSection;
	//Calculate the result of inserting the section flipped 
	if (j1 == 0)						newLeniF = S.routeLen[i] + dTime[S.items[x][y1]][S.items[i][j1]] + info.innerXF + info.dwellXSection;
	else if (j1 == S.items[i].size())	newLeniF = S.routeLen[i] - dTime[S.items[i][j1 - 1]][0] + dTime[S.items[i][j1 - 1]][S.items[x][y2 - 1]] + dTime[S.items[x][y1]][0] + info.innerXF + info.dwellXSection;
	else								newLeniF = S.routeLen[i] - dTime[S.items[i][j1 - 1]][S.items[i][j1]] + dTime[S.items[i][j1 - 1]][S.items[x][y2 - 1]] + dTime[S.items[x][y1]][S.items[i][j1]] + info.innerXF + info.dwellXSection;
	//Determine which is better and proceed with this result
	if (newLeni <= newLeniF) info.flippedX = false;
	else info.flippedX = true;
	newLeni = minVal(newLeni, newLeniF);
	//Finally calculate the result of removing the section from route x
	if (y1 == 0) {
		if (y2 == S.items[x].size())	newLenx = S.routeLen[x] - dTime[S.items[x][y2 - 1]][0] - info.innerX - info.dwellXSection;
		else							newLenx = S.routeLen[x] - dTime[S.items[x][y2 - 1]][S.items[x][y2]] - info.innerX - info.dwellXSection;
	}
	else if (y2 == S.items[x].size())	newLenx = S.routeLen[x] - dTime[S.items[x][y1 - 1]][S.items[x][y1]] - dTime[S.items[x][y2 - 1]][0] + dTime[S.items[x][y1 - 1]][0] - info.innerX - info.dwellXSection;
	else								newLenx = S.routeLen[x] - dTime[S.items[x][y1 - 1]][S.items[x][y1]] - dTime[S.items[x][y2 - 1]][S.items[x][y2]] + dTime[S.items[x][y1 - 1]][S.items[x][y2]] - info.innerX - info.dwellXSection;
	
	if (S.commonStop[x][i]) {
		//If we are here we also need to cope with any duplicates and re-evaluate the routes with their removal
		int y, pos;
		double internalX = 0.0, dwellXSaving = 0.0;
		bool dupInXSec = false;
		tempVec1.clear();
		//First flip the x section if needed
		if (info.flippedX) { twoOpt(S.items[x], y1, y2 - 1); twoOpt(S.W[x], y1, y2 - 1); }
		//And build up the actual x section that will be inserted into route i
		for (y = y1; y < y2; y++) {
			pos = checkForPresence(S, S.items[x][y], i);
			if (pos == -1) {
				//The item in route x at position y is not duplicated in route i, so it isn't removed
				tempVec1.push_back(S.items[x][y]);
				if (tempVec1.size() > 1) internalX += dTime[tempVec1[tempVec1.size() - 2]][tempVec1.back()];
			}
			else {
				//The item in route x's section at position y is duplicated in route i, so we don't copy it and we record the associted stopping time
				dwellXSaving += calcDwellTime(0);
				dupInXSec = true;
			}
		}
		//If there is a duplicate somewhere, we recalculate the move, otherwise our previous calculation was correct
		if (dupInXSec) {
			newLeni = calcRealInsertCost(S, i, j1, tempVec1, internalX, info) - dwellXSaving;
		}
		//Flip the x section back if needed
		if (info.flippedX) { twoOpt(S.items[x], y1, y2 - 1); twoOpt(S.W[x], y1, y2 - 1); }
	}
	newCost = S.cost - calcRCost(S.routeLen[x])	- calcRCost(S.routeLen[i]) + calcRCost(newLenx) + calcRCost(newLeni);
}

void calcRealInterCost(double &newLenx, double &newLeni, SOL &S, int x, int y1, int y2, int i, int j1, int j2, 
	vector<int> &xSection, vector<int> &iSection, double internalX, double internalI, EVALINFO &info,
	double lenXSecRemoved, double lenISecRemoved) {
	//We are swapping xSection and iSection. Calculate the result of doing this with the altered sections
	if (xSection.empty())					newLeni = lenISecRemoved + info.dwellXSection;
	else {
		if (j1 == 0)
			if (j2 == S.items[i].size())	newLeni = lenISecRemoved + dTime[xSection.back()][0] + internalX + info.dwellXSection;
			else							newLeni = lenISecRemoved + dTime[xSection.back()][S.items[i][j2]] + internalX + info.dwellXSection;
		else if (j2 == S.items[i].size())	newLeni = lenISecRemoved - dTime[S.items[i][j1 - 1]][0] + dTime[S.items[i][j1 - 1]][xSection.front()] + dTime[xSection.back()][0] + internalX + info.dwellXSection;
		else								newLeni = lenISecRemoved - dTime[S.items[i][j1 - 1]][S.items[i][j2]] + dTime[S.items[i][j1 - 1]][xSection.front()] + dTime[xSection.back()][S.items[i][j2]] + internalX + info.dwellXSection;
	}
	if (iSection.empty())					newLenx = lenXSecRemoved + info.dwellISection;
	else {
		if (y1 == 0)
			if (y2 == S.items[x].size())	newLenx = lenXSecRemoved + dTime[iSection.back()][0] + internalI + info.dwellISection;
			else							newLenx = lenXSecRemoved + dTime[iSection.back()][S.items[x][y2]] + internalI + info.dwellISection;
		else if (y2 == S.items[x].size())	newLenx = lenXSecRemoved - dTime[S.items[x][y1 - 1]][0] + dTime[S.items[x][y1 - 1]][iSection.front()] + dTime[iSection.back()][0] + internalI + info.dwellISection;
		else								newLenx = lenXSecRemoved - dTime[S.items[x][y1 - 1]][S.items[x][y2]] + dTime[S.items[x][y1 - 1]][iSection.front()] + dTime[iSection.back()][S.items[x][y2]] + internalI + info.dwellISection;
	}
}

void evaluateInter(double &newCost, SOL &S, int i, int j1, int j2, int x, int y1, int y2, EVALINFO &info)
{
	double newLenx, newLeni, newLenxF, newLeniF, lenXSecRemoved, lenISecRemoved;
	//We are swapping a nonempty section from route x with a nonempty section in route i. First calculate the result of removing the two sections)
	if (y1 == 0) {
		if (y2 == S.items[x].size())	lenXSecRemoved = S.routeLen[x] - dTime[S.items[x][y2 - 1]][0] - info.innerX - info.dwellXSection;
		else							lenXSecRemoved = S.routeLen[x] - dTime[S.items[x][y2 - 1]][S.items[x][y2]] - info.innerX - info.dwellXSection;
	}
	else if (y2 == S.items[x].size())	lenXSecRemoved = S.routeLen[x] - dTime[S.items[x][y1 - 1]][S.items[x][y1]] - dTime[S.items[x][y2 - 1]][0] + dTime[S.items[x][y1 - 1]][0] - info.innerX - info.dwellXSection;
	else								lenXSecRemoved = S.routeLen[x] - dTime[S.items[x][y1 - 1]][S.items[x][y1]] - dTime[S.items[x][y2 - 1]][S.items[x][y2]] + dTime[S.items[x][y1 - 1]][S.items[x][y2]] - info.innerX - info.dwellXSection;
	if (j1 == 0) {
		if (j2 == S.items[i].size())	lenISecRemoved = S.routeLen[i] - dTime[S.items[i][j2 - 1]][0] - info.innerI - info.dwellISection;
		else							lenISecRemoved = S.routeLen[i] - dTime[S.items[i][j2 - 1]][S.items[i][j2]] - info.innerI - info.dwellISection;
	}
	else if (j2 == S.items[i].size())	lenISecRemoved = S.routeLen[i] - dTime[S.items[i][j1 - 1]][S.items[i][j1]] - dTime[S.items[i][j2 - 1]][0] + dTime[S.items[i][j1 - 1]][0] - info.innerI - info.dwellISection;
	else								lenISecRemoved = S.routeLen[i] - dTime[S.items[i][j1 - 1]][S.items[i][j1]] - dTime[S.items[i][j2 - 1]][S.items[i][j2]] + dTime[S.items[i][j1 - 1]][S.items[i][j2]] - info.innerI - info.dwellISection;
	//Now calculate result of inserting the x section at position j1 in route i.
	if (j1 == 0)
		if (j2 == S.items[i].size())	newLeni = lenISecRemoved + dTime[S.items[x][y2 - 1]][0] + info.innerX + info.dwellXSection;
		else							newLeni = lenISecRemoved + dTime[S.items[x][y2 - 1]][S.items[i][j2]] + info.innerX + info.dwellXSection;
	else if (j2 == S.items[i].size())	newLeni = lenISecRemoved - dTime[S.items[i][j1 - 1]][0] + dTime[S.items[i][j1 - 1]][S.items[x][y1]] + dTime[S.items[x][y2 - 1]][0] + info.innerX + info.dwellXSection;
	else								newLeni = lenISecRemoved - dTime[S.items[i][j1 - 1]][S.items[i][j2]] + dTime[S.items[i][j1 - 1]][S.items[x][y1]] + dTime[S.items[x][y2 - 1]][S.items[i][j2]] + info.innerX + info.dwellXSection;
	//and the same flipped
	if (j1 == 0)
		if (j2 == S.items[i].size())	newLeniF = lenISecRemoved + dTime[S.items[x][y1]][0] + info.innerXF + info.dwellXSection;
		else							newLeniF = lenISecRemoved + dTime[S.items[x][y1]][S.items[i][j2]] + info.innerXF + info.dwellXSection;
	else if (j2 == S.items[i].size())	newLeniF = lenISecRemoved - dTime[S.items[i][j1 - 1]][0] + dTime[S.items[i][j1 - 1]][S.items[x][y2 - 1]] + dTime[S.items[x][y1]][0] + info.innerXF + info.dwellXSection;
	else								newLeniF = lenISecRemoved - dTime[S.items[i][j1 - 1]][S.items[i][j2]] + dTime[S.items[i][j1 - 1]][S.items[x][y2 - 1]] + dTime[S.items[x][y1]][S.items[i][j2]] + info.innerXF + info.dwellXSection;
	if (newLeni <= newLeniF) info.flippedX = false;
	else info.flippedX = true;
	newLeni = minVal(newLeni, newLeniF);
	//Now calculate result of inserting the i section at position y1 in route x.
	if (y1 == 0)
		if (y2 == S.items[x].size())	newLenx = lenXSecRemoved + dTime[S.items[i][j2 - 1]][0] + info.innerI + info.dwellISection;
		else							newLenx = lenXSecRemoved + dTime[S.items[i][j2 - 1]][S.items[x][y2]] + info.innerI + info.dwellISection;
	else if (y2 == S.items[x].size())	newLenx = lenXSecRemoved - dTime[S.items[x][y1 - 1]][0] + dTime[S.items[x][y1 - 1]][S.items[i][j1]] + dTime[S.items[i][j2 - 1]][0] + info.innerI + info.dwellISection;
	else								newLenx = lenXSecRemoved - dTime[S.items[x][y1 - 1]][S.items[x][y2]] + dTime[S.items[x][y1 - 1]][S.items[i][j1]] + dTime[S.items[i][j2 - 1]][S.items[x][y2]] + info.innerI + info.dwellISection;
	//and the same flipped
	if (y1 == 0)
		if (y2 == S.items[x].size())	newLenxF = lenXSecRemoved + dTime[S.items[i][j1]][0] + info.innerIF + info.dwellISection;
		else							newLenxF = lenXSecRemoved + dTime[S.items[i][j1]][S.items[x][y2]] + info.innerIF + info.dwellISection;
	else if (y2 == S.items[x].size())	newLenxF = lenXSecRemoved - dTime[S.items[x][y1 - 1]][0] + dTime[S.items[x][y1 - 1]][S.items[i][j2 - 1]] + dTime[S.items[i][j1]][0] + info.innerIF + info.dwellISection;
	else								newLenxF = lenXSecRemoved - dTime[S.items[x][y1 - 1]][S.items[x][y2]] + dTime[S.items[x][y1 - 1]][S.items[i][j2 - 1]] + dTime[S.items[i][j1]][S.items[x][y2]] + info.innerIF + info.dwellISection;
	if (newLenx <= newLenxF) info.flippedI = false;
	else info.flippedI = true;
	newLenx = minVal(newLenx, newLenxF);
	
	if (S.commonStop[x][i]) {
		//If we are here we need to cope with any duplicates and re-evaluate the routes with their removal
		int y, j, pos;
		double internalX = 0.0, internalI = 0.0, dwellXSaving = 0.0, dwellISaving = 0.0;
		bool dupInXSec = false, dupInISec = false;
		tempVec1.clear(); tempVec2.clear();
		if (info.flippedX) { twoOpt(S.items[x], y1, y2 - 1); twoOpt(S.W[x], y1, y2 - 1); }
		if (info.flippedI) { twoOpt(S.items[i], j1, j2 - 1); twoOpt(S.W[i], j1, j2 - 1); }
		//Build up the actual x section that will be inserted into route i
		for (y = y1; y < y2; y++) {
			pos = checkForPresence(S, S.items[x][y], i, j1, j2);
			if (pos == -1) {
				//The item in route x at position y is not duplicated in route i, so it isn't removed
				tempVec1.push_back(S.items[x][y]);
				if (tempVec1.size() > 1) internalX += dTime[tempVec1[tempVec1.size() - 2]][tempVec1.back()];
			}
			else {
				//The item in route x at position y is duplicated in route i, so we don't copy it and we remove the associted stoppIng time
				dwellXSaving += calcDwellTime(0);
				dupInXSec = true;
			}
		}
		//And build up the actual i section that will be inserted into route x
		for (j = j1; j < j2; j++) {
			pos = checkForPresence(S, S.items[i][j], x, y1, y2);
			if (pos == -1) {
				//The item in route i at position j is not duplicated in route x, so it isn't removed
				tempVec2.push_back(S.items[i][j]);
				if (tempVec2.size() > 1) internalI += dTime[tempVec2[tempVec2.size() - 2]][tempVec2.back()];
			}
			else {
				//The item in route x at position y is duplicated in route i, so we don't copy it and we remove the associted stoppIng time
				dwellISaving += calcDwellTime(0);
				dupInISec = true;
			}
		}
		if (dupInXSec || dupInISec) {
			calcRealInterCost(newLenx, newLeni, S, x, y1, y2, i, j1, j2, tempVec1, tempVec2, internalX, internalI, info, lenXSecRemoved, lenISecRemoved);
			newLenx = newLenx - dwellISaving;
			newLeni = newLeni - dwellXSaving;
		}
		if (info.flippedX) { twoOpt(S.items[x], y1, y2 - 1); twoOpt(S.W[x], y1, y2 - 1); }
		if (info.flippedI) { twoOpt(S.items[i], j1, j2 - 1); twoOpt(S.W[i], j1, j2 - 1); }
	}
	newCost = S.cost - calcRCost(S.routeLen[x]) - calcRCost(S.routeLen[i]) + calcRCost(newLenx) + calcRCost(newLeni);
}

void evaluateVertexCopy(double &newCost, SOL &S, int i, int j, int x) {
	//Evaluate effect of copying v = S[i][j] into route x and then transferring some passengers to it
	//Need to assume that the num of people boarding at S[i][j] is >= 2 and that spare capacity in route x is >= 1
	if (S.W[i][j] < 2 || maxBusCapacity - S.passInRoute[x] < 1) {
		cout << "Error. Conditions not met for evaluateVertexCopy fn\n"; exit(1);
	}
	int v = S.items[i][j], pos = S.posInRoute[v][x], bestInsertPos = -1, spareCapX = maxBusCapacity - S.passInRoute[x], toTransfer;
	double minCost, inserCost;

	if (pos == -1 && S.items[x].size() > 0) {
		//Stop v is not in nonempty route x, so we look for the best point to insert it (before stop "bestInsertPos")
		minCost = dTime[v][S.items[x][0]];
		bestInsertPos = 0;
		for (int u = 1; u < S.items[x].size(); u++) {
			inserCost = dTime[S.items[x][u - 1]][v] + dTime[v][S.items[x][u]] - dTime[S.items[x][u - 1]][S.items[x][u]];
			if (inserCost < minCost) {
				minCost = inserCost;
				bestInsertPos = u;
			}
		}
		inserCost = dTime[S.items[x].back()][v] + dTime[v][0] - dTime[S.items[x].back()][0];
		if (inserCost < minCost) {
			minCost = inserCost;
			bestInsertPos = S.items[x].size();
		}
	}
	//Calculate how many passengers we'll transfer from v in route i, to v in route j
	if (S.routeLen[i] < maxJourneyTime) toTransfer = 1;
	else toTransfer = minVal(S.W[i][j] - 1, spareCapX);
	//Now calculate cost of the resultant solution if we were to do this move.
	double newLeni, newLenx;
	if (pos != -1) {
		//We're just transferring passengers between existing multistops
		newLeni = S.routeLen[i] - (dwellPerPassenger * toTransfer);
		newLenx = S.routeLen[x] + (dwellPerPassenger * toTransfer);
	}
	else if (S.items[x].empty()) {
		//We're inserting a copy of v into an empty route
		newLeni = S.routeLen[i] - (dwellPerPassenger * toTransfer);
		newLenx = S.routeLen[x] + dTime[v][0] + calcDwellTime(toTransfer);
	}
	else {
		//We're making a copy of v and putting it into a nonempty route
		newLeni = S.routeLen[i] - (dwellPerPassenger * toTransfer);
		if (bestInsertPos == 0)
			newLenx = S.routeLen[x] + dTime[v][S.items[x][0]] + calcDwellTime(toTransfer);
		else if (bestInsertPos == S.items[x].size())
			newLenx = S.routeLen[x] + dTime[S.items[x].back()][v] + dTime[v][0] + calcDwellTime(toTransfer) - dTime[S.items[x].back()][0];
		else
			newLenx = S.routeLen[x] + dTime[S.items[x][bestInsertPos - 1]][v] + dTime[v][S.items[x][bestInsertPos]] + calcDwellTime(toTransfer) - dTime[S.items[x][bestInsertPos - 1]][S.items[x][bestInsertPos]];
	}
	newCost = S.cost - calcRCost(S.routeLen[i]) - calcRCost(S.routeLen[x]) + calcRCost(newLeni) + calcRCost(newLenx);
}

bool localSearch(SOL &S, double &feasRatio, int &numMoves)
{
	int x, y1, y2, i, j1, j2, z, chosenMove;
	int besti, bestj1, bestj2, bestx, besty1, besty2, bestz, numBest = 0;
	double newCost = 0, bestCost = 0;
	long evalCnt = 0, evalFeasCnt = 0;
	bool checkedEmpty, bestflippedx, bestflippedi;
	EVALINFO info;
	feasRatio = 0.0;
	numMoves = 0;
	vector<vector<int> > tempW;
	vector<double> tempRouteLen;
	vector<int> tempPassInRoute;

	while (true) {
		chosenMove = 0;
		bestCost = DBL_MAX;
		numBest = 0;
		for (x = 0; x < S.items.size(); x++) {
			/*Inter-route operators. Check cost of swapping sections (S[x][y1]...S[x][y2-1]) and sections (S[i][j1]...S[i][j2-1]).
			where x != i. The latter section can be empty (j1==j2) in which case we insert (S[x][y1]...S[x][y2-1]) before the
			position S[i][j1]. Route i can also be empty. If there is more than one empty route i, only one of these is evaluated*/
			for (y1 = 0; y1 < S.items[x].size(); y1++) {
				info.innerX = 0.0;
				info.innerXF = 0.0;
				info.dwellXSection = 0.0;
				info.passXSection = 0;
				for (y2 = y1 + 1; y2 <= S.items[x].size(); y2++) {
					//Keep track of the total costs of the inernal edges in this section of route x (fwd and bkwds)
					if (y2 > y1 + 1) {
						info.innerX += dTime[S.items[x][y2 - 2]][S.items[x][y2 - 1]];
						info.innerXF += dTime[S.items[x][y2 - 1]][S.items[x][y2 - 2]];
					}
					//Also keep track of the total dwell times in this section of route x
					info.dwellXSection += calcDwellTime(S.W[x][y2 - 1]);
					info.passXSection += S.W[x][y2 - 1];
					checkedEmpty = false;
					for (i = 0; i < S.items.size(); i++) {
						if (x != i) {
							if (S.items[i].empty() && checkedEmpty == false) {
								//The neighbourhood operator involves one non-empty routes (x) and one empty route (i)
								evaluateInterEmpty(newCost, S, i, x, y1, y2, info);
								if (newCost <= bestCost) {
									if (newCost < bestCost) numBest = 0;
									if (rand() % (numBest + 1) == 0) {
										//Save the move with a certain probability
										chosenMove = 1;
										bestCost = newCost; besti = i; bestx = x; besty1 = y1; besty2 = y2; bestflippedx = info.flippedX;
									}
									numBest++;
								}
								checkedEmpty = true;
							}
							else {
								//The neighbourhood operator involves two non-empty routes, so loop through each section in route i
								for (j1 = 0; j1 < S.items[i].size(); j1++) {
									info.innerI = 0.0;
									info.innerIF = 0.0;
									info.dwellISection = 0.0;
									info.passISection = 0;
									for (j2 = j1; j2 <= S.items[i].size(); j2++) {
										//Keep track of the total costs of the inernal edges of this section of route i (fwd and bkwds)
										if (j2 > j1 + 1) {
											info.innerI += dTime[S.items[i][j2 - 2]][S.items[i][j2 - 1]];
											info.innerIF += dTime[S.items[i][j2 - 1]][S.items[i][j2 - 2]];
										}
										if (j2 > j1) {
											info.dwellISection += calcDwellTime(S.W[i][j2 - 1]);
											info.passISection += S.W[i][j2 - 1];
										}
										if (S.passInRoute[i] - info.passISection + info.passXSection <= maxBusCapacity && S.passInRoute[x] - info.passXSection + info.passISection <= maxBusCapacity) {
											//The proposed move will retain the validity of the route capactities,
											if (j1 == j2) {
												//Inserting a section from route x into route i
												evaluateInsert(newCost, S, i, j1, x, y1, y2, info);
												if (newCost <= bestCost) {
													if (newCost < bestCost) numBest = 0;
													if (rand() % (numBest + 1) == 0) {
														//Save the move with a certain probability
														chosenMove = 2;
														bestCost = newCost; besti = i; bestj1 = j1; bestj2 = j2; bestx = x; besty1 = y1; besty2 = y2; bestflippedx = info.flippedX; bestflippedi = info.flippedI;
													}
													numBest++;
												}
											}
											else {
												//Swapping a section from route x and a section of route i
												evaluateInter(newCost, S, i, j1, j2, x, y1, y2, info);
												if (newCost <= bestCost) {
													if (newCost < bestCost) numBest = 0;
													if (rand() % (numBest + 1) == 0) {
														//Save the move with a certain probability
														chosenMove = 3;
														bestCost = newCost; besti = i; bestj1 = j1; bestj2 = j2; bestx = x; besty1 = y1; besty2 = y2; bestflippedx = info.flippedX; bestflippedi = info.flippedI;
													}
													numBest++;
												}
											}
											evalFeasCnt++;
										}
										evalCnt++;
									}
								}
							}
						}
					}
				}
			}
			/*Inter-route operator that seeks to increase the number of multi-stops by copying stop v = S[i][j1] into route x at
			the best position, where x != i. Route x may already contain v, or may also be empty, but v should have at least 2 boarding
			passengers, and route x should have some spare capacity*/
			for (i = 0; i < S.items.size(); i++) {
				for (j1 = 0; j1 < S.items[i].size(); j1++) {
					if (i != x) {
						if (S.W[i][j1] > 1 && maxBusCapacity - S.passInRoute[x] >= 1) {
							evaluateVertexCopy(newCost, S, i, j1, x);
							if (newCost <= bestCost) {
								if (newCost < bestCost) numBest = 0;
								if (rand() % (numBest + 1) == 0) {
									//Save the move with a certain probability
									chosenMove = 7;
									bestCost = newCost; besti = i; bestj1 = j1; bestx = x;
								}
								numBest++;
							}
						}
					}
				}
			}
			//Now look at the inter-route operators
			if (!S.items[x].empty()) {
				for (y1 = 0; y1 < S.items[x].size(); y1++) {
					info.innerX = 0.0;
					info.innerXF = 0.0;
					for (y2 = y1; y2 < S.items[x].size(); y2++) {
						if (y1 < y2) {
							//Keep track of the total cost of the inernal edges of the section we are considering
							info.innerX += dTime[S.items[x][y2 - 1]][S.items[x][y2]];
							info.innerXF += dTime[S.items[x][y2]][S.items[x][y2 - 1]];
							//Now check the cost of swap
							evaluateSwapTwoOpt(newCost, S, x, y1, y2, 4, info);
							if (newCost <= bestCost) {
								if (newCost < bestCost) numBest = 0;
								if (rand() % (numBest + 1) == 0) {
									//Save the move with a certain probability
									chosenMove = 4;
									bestCost = newCost; bestx = x; besty1 = y1; besty2 = y2;
								}
								numBest++;
							}
							//And the cost of an inversion 
							evaluateSwapTwoOpt(newCost, S, x, y1, y2, 5, info);
							if (newCost <= bestCost) {
								if (newCost < bestCost) numBest = 0;
								if (rand() % (numBest + 1) == 0) {
									//Save the move with a certain probability
									chosenMove = 5;
									bestCost = newCost; bestx = x; besty1 = y1; besty2 = y2;
								}
								numBest++;
							}
						}
						//Now check cost of inserting section (y1,...y2) before point z
						for (z = 0; z <= S.items[x].size(); z++) {
							if (z == y1)
								z = y2 + 1;
							else {
								evaluateOrOpt(newCost, S, x, y1, y2, z, info);
								if (newCost <= bestCost) {
									if (newCost < bestCost) numBest = 0;
									if (rand() % (numBest + 1) == 0) {
										//Save the move with a certain probability
										chosenMove = 6;
										bestCost = newCost; bestx = x; besty1 = y1; besty2 = y2; bestz = z; bestflippedx = info.flippedX;
									}
									numBest++;
								}
							}
						}
					}
				}
			}
		}
		//All neighbourhoods evaluated. We now do the neighbourhood move. First, if no improvement has been found, end. 
		if (chosenMove == 0 || bestCost >= S.cost) {
			break;
		}
		//Otherwise, do the chosen move and repeat.
		if (chosenMove == 1)		doMove1(S, bestx, besty1, besty2, besti, bestCost, bestflippedx);
		else if (chosenMove == 2)	doMove2(S, bestx, besty1, besty2, besti, bestj1, bestCost, bestflippedx);
		else if (chosenMove == 3)	doMove3(S, bestx, besty1, besty2, besti, bestj1, bestj2, bestCost, bestflippedx, bestflippedi);
		else if (chosenMove == 4)	doMove4(S, bestx, besty1, besty2, bestCost);
		else if (chosenMove == 5) 	doMove5(S, bestx, besty1, besty2, bestCost);
		else if (chosenMove == 6)	doMove6(S, bestx, besty1, besty2, bestz, bestCost, bestflippedx);
		else if (chosenMove == 7)	doMove7(S, bestx, besti, bestj1, bestCost);
		numMoves++;
	}

	//We have finished the optimisation procedure
	if (evalCnt > 0) feasRatio = evalFeasCnt / double(evalCnt);
	else feasRatio = 0.0;
	
	if (S.numFeasibleRoutes >= S.items.size()) return true;
	else return false;
}

