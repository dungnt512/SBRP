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
extern string BingKey;
extern string distUnits;
extern double dwellPerPassenger;
extern double dwellPerStop;
extern double excessWeight;
extern double maxJourneyTime;
extern bool useMinCoverings;

//Some temporary vectors used in the covering functions
vector<int> Y, tempVec, perm;
vector<vector<int> > X;

int chooseRandomSet(vector<vector<int> > &X) {
	//Chooses any set with an element to cover, breaking ties randomly
	int i, pos = -1, numChoices = 0;
	for (i = 1; i < X.size(); i++) {
		if (X[i].size() > 0) {
			if (rand() % (numChoices + 1) == 0) {
				pos = i;
			}
			numChoices++;
		}
	}
	if (pos == -1) { cout << "Error in chooseRandomSet function. Ending...\n"; exit(1); }
	else return pos;
}

int chooseBiggestSet(vector<vector<int> > &X) {
	//Chooses the biggest set (stop with the most unvisited addresses), breaking ties randomly
	int i, max = 0, maxPos = -1, numChoices = 0;
	for (i = 1; i < X.size(); i++) {
		if (X[i].size() >= max) {
			if (X[i].size() > max) numChoices = 0;
			if (rand() % (numChoices + 1) == 0) {
				max = X[i].size();
				maxPos = i;
			}
			numChoices++;
		}
	}
	if (maxPos == -1) { cout << "Error in chooseBiggestSet function. Ending...\n"; exit(1); }
	else return maxPos;
}

void getClosestStops(vector<bool> &stopUsed) {
	//Creates a covering by simply taking the closest stop to each address
	int i;
	for (i = 0; i < addresses.size(); i++) {
		stopUsed[addrAdjList[i][0]] = true;
	}
}

void makeCoveringMinimal(vector<bool> &stopUsed) {
	//This checks the covering is minimal. If it is not, bus stops are removed in a random order to make it minimal. 
	int i, j, r, stop;
	tempVec.clear();
	Y.clear();
	Y.resize(addresses.size(), 0);
	//Make a list of all stops being used
	for (i = 1; i < stopUsed.size(); i++) {
		if (stopUsed[i]) tempVec.push_back(i);
	}
	//First, for each address count the number of adjacent stops in the solution and put this info in Y
	for (i = 0; i < tempVec.size(); i++) {
		stop = tempVec[i];
		for (j = 0; j < stopAdjList[stop].size(); j++) Y[stopAdjList[stop][j]] += 1;
	}
	//Now, for each used stop in a random order, see if removing this stop causes an address to have no suitable stop; in which 
	//case the stop is definately needed. If it is not needed, then it is removed
	randPermute(tempVec);
	for (i = 0; i < tempVec.size(); i++) {
		stop = tempVec[i];
		//Check if removing "stop" would give us an incomplete covering
		for (j = 0; j < stopAdjList[stop].size(); j++) {
			if (Y[stopAdjList[stop][j]] == 1) break;
		}
		if (j >= stopAdjList[stop].size()) {
			//If we are here then "stop" can be removed from the solution 
			for (r = 0; r < stopAdjList[stop].size(); r++) Y[stopAdjList[stop][r]]--;
			stopUsed[stop] = false;
		}
	}
}

void generateNewCovering(vector<bool> &stopUsed, int forbidden, int heuristic) {
	//Executes a greedy set covering algorithm to determine a subset of bus stops to use.
	//Uses the global variable "makeCoveringMinimal" to determine whether the set of bus stops should correspond to a minimal covering (invoking a procedure at the end)
	//"Forbidden" defines a single stop that should not be included in this covering (if it is -1 then all are allowed)
	//Heuristic: 1: choose set with most uncovered elements at each iteration
	//           2: choose any set with an uncovered element at each iteration
	int i, j, x, cnt = 0;
	X.clear();
	X.resize(stops.size());
	tempVec.clear();

	//Set up a multiset of sets X, each set is the adjacencies of a stop (in lexicographic order)	
	for (i = 1; i < stops.size(); i++) {
		for (j = 0; j < addresses.size(); j++) {
			if (addrStopAdj[j][i]) X[i].push_back(j);
		}
	}

	//This is optional and stops a particular stop from being selected as part of the covering
	if (forbidden >= 0 && forbidden < X.size()) {
		X[forbidden].clear();
	}

	//Now, if the stopUsed contains any true values, add these to the covering (they are already conributing to the covering)
	for(x = 1; x < stops.size(); x++) {
		if (stopUsed[x] == true) {
			cnt += X[x].size();
			//Remove the items in X[x] from all other sets X[i]
			for (i = 1; i < X.size(); i++) {
				if (i != x) {
					//Remove elements of X[x] from X[i] (i.e. Calculate (X[i] setminus X[x]) and put the results back into X[i])
					tempVec.clear();
					set_difference(X[i].begin(), X[i].end(), X[x].begin(), X[x].end(), back_inserter(tempVec));
					X[i].swap(tempVec);
				}
			}
			X[x].clear();
		}
	}

	if (cnt < addresses.size()) {
		//If we are here, the covering is incomplete, so we repeatedly select sets (bus stops) until we have a complete covering
		while (true) {
			if (heuristic == 1) {
				x = chooseBiggestSet(X);
			}
			else {
				x = chooseRandomSet(X);
			}
			cnt += X[x].size();
			stopUsed[x] = true;
			//Remove the items in X[x] from all other sets X[i]
			for (i = 1; i < X.size(); i++) {
				if (i != x) {
					tempVec.clear();
					set_difference(X[i].begin(), X[i].end(), X[x].begin(), X[x].end(), back_inserter(tempVec));
					X[i].swap(tempVec);
				}
			}
			X[x].clear();
			if (cnt >= addresses.size()) break;
		}
	}

	//If necesarry, now ensure that this covering is minimal
	if (useMinCoverings) {
		makeCoveringMinimal(stopUsed);
	}
}

int makeNewCovering(SOL &S) {
	//Delete some (non-required) stops and then repair via the set covering method
	int i;
	
	//Create a random permutation of the used stops that are not "required"
	tempVec.clear();
	for (i = 1; i < S.stopUsed.size(); i++) {
		if (S.stopUsed[i] == true && !stops[i].required) tempVec.push_back(i);
	}
	randPermute(tempVec);

	//If all stops are required, then end (we can't change the solution)
	if (tempVec.empty()) {
		return 0;
	}

	//This is the probability of removing a non-essential stop
	double p = 3 / double(tempVec.size());

	//Select the stops to delete. One stop is removed automatically, the rest according to binomial probabilities
	perm.clear();
	perm.push_back(tempVec[0]);
	for (i = 1; i < tempVec.size(); i++) {
		if (rand() / double(RAND_MAX) <= p) perm.push_back(tempVec[i]);
	}

	//Now delete these stops from the stopUsed array and find a new minimal covering. We forbid the first stop in
	//tempVec2 from being reselected to ensure the new set cover is different
	for (i = 0; i < perm.size(); i++) S.stopUsed[perm[i]] = false;
	generateNewCovering(S.stopUsed, perm[0], 2);

	//And finally rebuild the solution according to the new (minimal set of bus stops)
	rebuildSolution(S);
	return perm.size();
}