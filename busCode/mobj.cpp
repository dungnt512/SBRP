#include "mobj.h"

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
extern bool useMinCoverings;
extern double discreteLevel;
extern int verbosity;
extern vector<int> stopsToPack;
extern vector<int> weightOfStopsToPack;
vector<int> tVec, tVec2;

bool compareSlns(const SOL &lhs, const SOL &rhs) {
	return lhs.costWalk < rhs.costWalk;
}

void printDetails(list<SOL> &A, list<bool> &L, int its) {
	list<bool>::iterator it;
	list<SOL>::iterator AIt;
	int numVisited = 0, numFeas = 0, numInFront = A.size();
	AIt = A.begin();
	for (it = L.begin(); it != L.end(); ++it) {
		if (*it == true)
			numVisited++;
		if ((*AIt).numFeasibleRoutes == (*AIt).items.size()) {
			//Solution is feasible
			numFeas++;
		}
		++AIt;
	}
	cout << its << ") Archive size |A| =  " << numInFront << ", num visited solutions = " << numVisited << ". " << numFeas << " of these are feasible" << endl;
}

bool chooseUnvisitedSolution(list<SOL> &A, list<bool> &visited, SOL &S) {
	//Randomly chooses a member of the archive that has not yet been visited
	list<SOL>::iterator AIt, AChosen;
	list<bool>::iterator vIt, vChosen;
	vIt = visited.begin();
	AIt = A.begin();
	int numChoices = 0;
	while (vIt != visited.end()) {
		if (*vIt == false) {
			if (rand() % (numChoices + 1) == 0) {
				vChosen = vIt;
				AChosen = AIt;
			}
			numChoices++;
		}
		++AIt;
		++vIt;
	}
	if (numChoices == 0) {
		//No solutions are unvisited 
		return false;
	}
	else {
		//Return a random unvisited solution 
		*vChosen = true;
		S = *AChosen;
		return true;
	}
}
	
void updateA(list<SOL> &A, list<bool> &visited, SOL &S) {
	//Procedure that updates a mutually non-dominating archive A with a solution S
	list<SOL>::iterator AIt;
	list<bool>::iterator vIt;
	double k = double(S.items.size());
	double s = double(totalPassengers);
	double base = discreteLevel;
	AIt = A.begin();
	vIt = visited.begin();
	while (AIt != A.end()) {
		if (
			(
				S.cost / k <= (*AIt).cost / k
				&& 
				S.costWalk / s < (*AIt).costWalk / s
			)
			||
			(
				S.cost / k < (*AIt).cost / k
				&&
				S.costWalk / s <= (*AIt).costWalk / s
			)
			) {
			//S strictly dominates the solution (*AIt) in the archive, so delete (*AIt)
			AIt = A.erase(AIt);
			vIt = visited.erase(vIt);
		}
		else if (
			roundDown(S.cost / k, base) >= roundDown((*AIt).cost / k, base)
			&&
			roundDown(S.costWalk / s, base) >= roundDown((*AIt).costWalk / s, base)
			) {
			//S is dominated by a solution in A, so we will not add it, and can end now
			return;
		}
		else {
			++AIt;
			++vIt;
		}
	}
	//If we are here, then we must add S to A because it is mutually non dominating and sufficiently 
	//different with all slns remaining in A
	A.push_back(S);
	visited.push_back(false);
}

void calcSavingWhenAddingAStop(SOL &S, int v, double &saving, bool &addingStop) {
	//Calculate the savings in walking distance (if any) when adding bus stop v to solution S
	saving = 0;
	int j, addr;
	for (j = 0; j < stopAdjList[v].size(); j++) {
		addr = stopAdjList[v][j];
		if (wTime[addr][v] < wTime[addr][S.assignedTo[addr]]) {
			//Addr is closer to v than its current stop, so a saving can be made for all passengers at this address
			saving += (wTime[addr][S.assignedTo[addr]] - wTime[addr][v]) * addresses[addr].numPass;
		}
	}
	if (saving > 0) addingStop = true;
	else			addingStop = false; //Adding the stop makes no difference to peoples' walks
}

void addStop(int v, SOL &S, double saving) {
	int i, j, u, x, r, c, addr;
	//We are going to add a new stop v. First, we need to remove relevant passengers from their current stops and assign them to v
	for (j = 0; j < stopAdjList[v].size(); j++) {
		//Look at each address "addr" adjacent to v and consider the stop u it is currently assigned to
		addr = stopAdjList[v][j];
		u = S.assignedTo[addr];
		if (wTime[addr][v] < wTime[addr][u]) {
			//Addr is closer to v than u so a saving can be made. We do this by removing the x passengers of "addr" from occurences of u in S.W
			x = addresses[addr].numPass;
			S.numBoarding[v] += x;
			S.numBoarding[u] -= x;
			S.assignedTo[addr] = v;
			for (i = 0; i < S.routeOfStop[u].size(); i++) {
				r = S.routeOfStop[u][i];
				c = S.posInRoute[u][r];
				if (S.W[r][c] >= x) {
					S.W[r][c] -= x;
					break;
				}
				else {
					x -= S.W[r][c];
					S.W[r][c] = 0;
				}
			}
		}
	}
	//Now delete bus stops from S.items and S.W for which S.W[i][j] = 0 (if indeed there are any)
	//and recalculate the number of passengers in each route.
	for (i = 0; i < S.W.size(); i++) {
		S.passInRoute[i] = 0;
		j = 0;
		while (j < S.W[i].size()) {
			if (S.W[i][j] == 0) {
				S.W[i].erase(S.W[i].begin() + j);
				S.items[i].erase(S.items[i].begin() + j);
			}
			else {
				S.passInRoute[i] += S.W[i][j];
				j++;
			}
		}
	}
	//Now use BPP style procedure to pack stop v into the solution.
	binPacker(S.items, S.W, S.passInRoute, v, S.numBoarding[v]);
	//Having added v to the solution we now recalculate the auxiliary structures
	repopulateAuxiliaries(S);
	//and, update the costs
	S.cost = calcSolCostFromScratch(S);
	S.costWalk -= saving;
}

void calcSavingWhenRemovingAStop(SOL &S, int v, double &saving, bool doRepair, bool &deletingStop) {
	//Calculate the "savings" (which could be negative) of removing the non-compulsory stop v
	int i, j, addr, u, x, y;
	saving = 0;
	if (doRepair) {
		tVec.clear(); //Keeps a record of additional stops that are added (if any)
		tVec2.clear(); //This keeps a record of how the assignedTo array changes when we repair
	}
	for (i = 0; i < stopAdjList[v].size(); i++) {
		addr = stopAdjList[v][i];
		if (S.assignedTo[addr] == v) {
			//Addr is currently assigned to v, so we need to find another stop u for it (the closest used stop)
			for (j = 0; j < addrAdjList[addr].size(); j++) {
				u = addrAdjList[addr][j];
				if (S.stopUsed[u] && u != v) break; //Addr can be assigned to a stop u != v that is currently being used 
			}
			if (j >= addrAdjList[addr].size()) {
				//An additional stop is required for addr. Either find one, or end
				if (!doRepair) {
					deletingStop = false;
					return;
				}
				//No used stop is suitable for addr, so we assign addr to the closest unused stop u != v instead
				u = addrAdjList[addr][0];
				if (u == v) u = addrAdjList[addr][1];
				tVec.push_back(u);
				S.stopUsed[u] = true;
				tVec2.push_back(addr);
				tVec2.push_back(v);
				S.assignedTo[addr] = u;
				saving += wTime[addr][v] * addresses[addr].numPass;
				saving -= wTime[addr][u] * addresses[addr].numPass;
				//We also need to check if the addition of u affects the walking distances from any other adjacent addresses
				for (j = 0; j < stopAdjList[u].size(); j++) {
					//Check if address x, which is currently assigned to stop y, is closer to stop u
					x = stopAdjList[u][j];
					y = S.assignedTo[x];
					if (x != addr && y != v && wTime[x][u] < wTime[x][y]) {
						tVec2.push_back(x);
						tVec2.push_back(y);
						S.assignedTo[x] = u;
						saving += wTime[x][y] * addresses[x].numPass;
						saving -= wTime[x][u] * addresses[x].numPass;
					}	
				}
			}
			else {
				//Addr has been reassigned to the existing stop u, so the change in walking distance is easy
				if (doRepair) {
					tVec2.push_back(addr);
					tVec2.push_back(v);
					S.assignedTo[addr] = u;
				}
				saving += wTime[addr][v] * addresses[addr].numPass;
				saving -= wTime[addr][u] * addresses[addr].numPass;
			}		
		}
	}
	//If doing repair we now need to to reset the S.stopUsed and S.assignedTo arrays
	if (doRepair) {
		for (i = 0; i < tVec.size(); i++) {
			S.stopUsed[tVec[i]] = false;
		}
		i = tVec2.size() - 1;
		while (i > 0) {
			S.assignedTo[tVec2[i - 1]] = tVec2[i];
			i -= 2;
		}
	}
	deletingStop = true;
}

void removeStop(int v, SOL &S, double saving, bool doRepair) {
	//Remove the non-compulsory used stop v and reassign affected passengers to other stops.
	int i, j, r, c, addr, u, k = S.items.size(), min, x, y;
	//First remove passengers from stop v in the solution
	S.stopUsed[v] = false;
	S.numBoarding[v] = 0;
	for (i = 0; i < S.routeOfStop[v].size(); i++) {
		r = S.routeOfStop[v][i];
		c = S.posInRoute[v][r];
		S.passInRoute[r] -= S.W[r][c];
		S.W[r][c] = 0;
	}
	//Now assign passengers who were boarding v to other stops suitable stops u (this may involve adding new stops).
	for (i = 0; i < stopAdjList[v].size(); i++) {
		addr = stopAdjList[v][i];
		if (S.assignedTo[addr] == v) {
			for (j = 0; j < addrAdjList[addr].size(); j++) {
				u = addrAdjList[addr][j];
				if (S.stopUsed[u] && u != v) break;	//Addr can be assigned to a stop u != v that is currently being used 
			}
			if (j >= addrAdjList[addr].size()) {
				//No used stop is suitable for addr, so we assign addr to the closest unused stop u != v instead
				if (!doRepair) { cout << "Should not be here\n"; exit(1); }
				u = addrAdjList[addr][0];
				if (u == v) u = addrAdjList[addr][1];
				S.stopUsed[u] = true;
				S.assignedTo[addr] = u;
				S.numBoarding[u] = addresses[addr].numPass;
				//We now add stop u to the end of the emtiest route r (we do this now rather than let the BPP heuristic
				//sort it out later, because we might also want to use u again in a bit)
				min = S.passInRoute[0];
				r = 0;
				for (j = 1; j < k; j++) {
					if (S.passInRoute[j] < min) {
						min = S.passInRoute[j];
						r = j;
					}
				}				
				S.items[r].push_back(u);
				S.W[r].push_back(addresses[addr].numPass);
				S.passInRoute[r] += addresses[addr].numPass;
				S.routeOfStop[u].push_back(r);
				S.posInRoute[u][r] = S.items[r].size() - 1;
				//We now need to check if the addition of u affects the walking distances from any other adjacent addresses
				for (j = 0; j < stopAdjList[u].size(); j++) {
					//Check if address x, which is currently assigned to stop y, is actually closer to stop u
					x = stopAdjList[u][j];
					y = S.assignedTo[x];
					if (x != addr && y != v && wTime[x][u] < wTime[x][y]) {
						//Add passengers of address x to stop u
						S.assignedTo[x] = u;
						S.numBoarding[u] += addresses[x].numPass;
						S.W[r].back() += addresses[x].numPass;
						S.passInRoute[r] += addresses[x].numPass;
						//And remove them from stop y
						S.numBoarding[y] -= addresses[x].numPass;
						eliminateFromW(S, y, addresses[x].numPass);
					}
				}
			}
			else {
				//Addr has been reassigned to the existing stop u, so add it to the emptiest route containing u
				r = S.routeOfStop[u][0];
				min = S.passInRoute[r];
				for (j = 1; j < S.routeOfStop[u].size(); j++) {
					if (S.passInRoute[S.routeOfStop[u][j]] < min) {
						min = S.passInRoute[S.routeOfStop[u][j]];
						r = S.routeOfStop[u][j];
					}
				}
				c = S.posInRoute[u][r];
				S.W[r][c] += addresses[addr].numPass;
				S.passInRoute[r] += addresses[addr].numPass;
				S.assignedTo[addr] = u;
				S.numBoarding[u] += addresses[addr].numPass;
			}
		}
	}
	//Now delete all instances of where S.W = 0. This may mean that some stops that were previously being used may now not be.
	//tVec keeps track of how many instatnces of each stop are deleted. If all of them are, the stop is no longer used
	tVec.clear();
	tVec.resize(stops.size(), 0); 
	for (i = 0; i < S.W.size(); i++) {
		j = 0;
		while (j < S.W[i].size()) {
			if (S.W[i][j] == 0) {
				u = S.items[i][j];
				tVec[u]++;
				if (tVec[u] == S.routeOfStop[u].size()) {
					S.stopUsed[u] = false; //We are no longer using stop u in the solution
				}
				S.W[i].erase(S.W[i].begin() + j);
				S.items[i].erase(S.items[i].begin() + j);
			}
			else {
				j++;
			}
		}
	}
	//At this point, some routes may be overfull, so delete randomly chosen stops from each one
	stopsToPack.clear();
	weightOfStopsToPack.clear();
	for (i = 0; i < k; i++) {
		while (S.passInRoute[i] > maxBusCapacity) {
			j = rand() % (S.items[i].size());
			stopsToPack.push_back(S.items[i][j]);
			weightOfStopsToPack.push_back(S.W[i][j]);
			S.passInRoute[i] -= S.W[i][j];
			S.items[i].erase(S.items[i].begin() + j);
			S.W[i].erase(S.W[i].begin() + j);
		}
	}
	//Use BPP style procedure to pack all the children on to k buses.
	binPacker(S.items, S.W, S.passInRoute, stopsToPack, weightOfStopsToPack);

	//Finally, we need to repopulate the residual structures. 
	repopulateAuxiliaries(S);

	//...and calculate the costs
	S.cost = calcSolCostFromScratch(S);
	S.costWalk = S.costWalk - saving;
}

void doMultiObjOptimisation(list <SOL> &A) {
	
	//This takes an archive of solution(s) and runs the mobj process
	list<bool> visited;
	list<bool>::iterator vIt;
	list<SOL>::iterator AIt;
	int v, its = 1, numMoves, k = A.front().items.size();
	double saving, feasRatio;
	bool addingStop, deletingStop;
	SOL S, SPrime;
	
	//Mark the initial solution in the archive as unvisited
	visited.push_back(false);

	//Also add the solution where all students are given the shortest possible walk
	makeInitSol(S, k, 3);
	localSearch(S, feasRatio, numMoves);
	updateA(A, visited, S);
		
	if (verbosity >= 1) 
		cout << "\n\nNow using multiobjective techiniques to produce a range of solutions that use " << k << " buses.\n\n";
	
	while (true) {

		//Select a non-visited member S of the archive. If all are visited, end the algorithm.
		if (verbosity >= 1) {
			printDetails(A, visited, its);
		}
		if (chooseUnvisitedSolution(A, visited, S) == false) {
			break;
		}
					
		//If we are here, S is now a solution we will be visiting from (and has therefore been marked as visited)
		for (v = 1; v < stops.size(); v++) {
			if (!S.stopUsed[v]) {
				//Explore consequences of adding the currently unusued stop v
				calcSavingWhenAddingAStop(S, v, saving, addingStop);
				if (addingStop) {
					SPrime = S;
					addStop(v, SPrime, saving);
					localSearch(SPrime, feasRatio, numMoves);
					updateA(A, visited, SPrime);
				}
			}
			else {
				if (!stops[v].required) {
					//Explore consequences of removing stop the currently used, non-compulsory stop v
					calcSavingWhenRemovingAStop(S, v, saving, true, deletingStop);
					if (deletingStop) {
						SPrime = S;
						removeStop(v, SPrime, saving, true);
						localSearch(SPrime, feasRatio, numMoves);
						updateA(A, visited, SPrime);
					}
				}
			}
		}
		its++;
	}
	//Now sort the front and end
	A.sort(compareSlns);
}

