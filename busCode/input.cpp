#include "input.h"

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

//Functions for sorting an array of stop indexes according to their distance from addr (or stop)
int partitionStopsByDist(vector<int> &A, int left, int right, int who, int addr) {
	for (int i = left; i<right; ++i) {
		if (wTime[addr][A[i]] <= wTime[addr][who]) {
			swap(A[i], A[left]);
			left++;
		}
	}
	return left - 1;
}

void qSortStopsByDist(vector<int> &A, int left, int right, int addr) {
	if (left >= right) return;
	int middle = left + (right - left) / 2;
	swap(A[middle], A[left]);
	int midpoint = partitionStopsByDist(A, left + 1, right, A[left], addr);
	swap(A[left], A[midpoint]);
	qSortStopsByDist(A, left, midpoint, addr);
	qSortStopsByDist(A, midpoint + 1, right, addr);
}

int partitionAddressesByDist(vector<int> &A, int left, int right, int who, int stop) {
	for (int i = left; i<right; ++i) {
		if (wTime[A[i]][stop] <= wTime[who][stop]) {
			swap(A[i], A[left]);
			left++;
		}
	}
	return left - 1;
}

void qSortAddressesByDist(vector<int> &A, int left, int right, int stop) {
	if (left >= right) return;
	int middle = left + (right - left) / 2;
	swap(A[middle], A[left]);
	int midpoint = partitionAddressesByDist(A, left + 1, right, A[left], stop);
	swap(A[left], A[midpoint]);
	qSortAddressesByDist(A, left, midpoint, stop);
	qSortAddressesByDist(A, midpoint + 1, right, stop);
}

void trim(string &str) {
	//Trims any whitespce and commas from the left and right of the string.
	if (str.length() == 0) return;
	int l = 0, r = str.length() - 1;
	while (str[l] == ' ' || str[l] == ',' || str[l] == '\t') {
		l++;
	}
	while (str[r] == ' ' || str[r] == ',' || str[r] == '\t') {
		r--;
	}
	str = str.substr(l, r - l + 1);
}

void readInput(string &infile) {
	//Reads in the input file
	int i, j, x, y, numStops, numAddresses, numWalks;
	double d, t;
	string temp;

	totalPassengers = 0;

	ifstream inStream;
	inStream.open(infile);
	if (inStream.fail()) { cout << "ERROR OPENING INPUT FILE"; exit(1); }

	//First read the top line of the .bus file
	getline(inStream, temp, ',');
	numStops = stoi(temp);
	getline(inStream, temp, ',');
	numAddresses = stoi(temp);
	getline(inStream, temp, ',');
	numWalks = stoi(temp);
	getline(inStream, temp, ',');
	if (temp == "K") distUnits = "kms";
	else distUnits = "miles";
	getline(inStream, temp, ',');
	minEligibilityDist = stof(temp);
	getline(inStream, temp, ',');
	maxWalkDist = stof(temp);
	getline(inStream, temp, ',');
	trim(temp);
	//Read the rest of the line (doesn't do anything)
	getline(inStream, temp);

	cout << "Processing " << infile << " " << temp << "\n";

	//Now resize the arrays
	stops.resize(numStops);
	addresses.resize(numAddresses);
	dTime.resize(numStops, vector<double>(numStops));
	dDist.resize(numStops, vector<double>(numStops));
	wTime.resize(numAddresses, vector<double>(numStops, DBL_MAX));
	wDist.resize(numAddresses, vector<double>(numStops, DBL_MAX));
	
	//Now read information about the stops
	for (i = 0; i < numStops; i++) {
		getline(inStream, temp, ',');
		getline(inStream, temp, ',');
		stops[i].y = stod(temp);
		getline(inStream, temp, ',');
		stops[i].x = stod(temp);
		getline(inStream, temp);
		trim(temp);
		stops[i].label = temp;
		stops[i].required = false;
	}

	//And similalrly for the addresses
	for (i = 0; i < numAddresses; i++) {
		getline(inStream, temp, ',');
		getline(inStream, temp, ','); 
		addresses[i].y = stod(temp);
		getline(inStream, temp, ',');
		addresses[i].x = stod(temp);
		getline(inStream, temp, ',');
		addresses[i].numPass = stoi(temp);
		totalPassengers += addresses[i].numPass;
		getline(inStream, temp);
		trim(temp);
		addresses[i].label = temp;
	}

	//Now read the distances between all stop pairs
	for (i = 0; i < numStops; i++) {
		for (j = 0; j < numStops; j++) {
			getline(inStream, temp, ',');
			getline(inStream, temp, ',');
			getline(inStream, temp, ',');
			getline(inStream, temp, ',');
			dDist[i][j] = stod(temp);
			getline(inStream, temp);
			dTime[i][j] = stod(temp);
		}
	}

	//Read information about a walk time distance pair
	for (i = 0; i < numWalks; i++) {
		getline(inStream, temp, ',');
		getline(inStream, temp, ',');
		x = stoi(temp);
		getline(inStream, temp, ',');
		y = stoi(temp);
		getline(inStream, temp, ',');
		d = stod(temp);
		getline(inStream, temp);
		t = stod(temp);
		wTime[x][y] = t;
		wDist[x][y] = d;
	}
	inStream.close();

	//We have now read in all the input. 
	//Now create an Adj list and matrix specifying the stops and addresses that are adjacent (within maximum walking distance)
	addrStopAdj.resize(addresses.size(), vector<bool>(stops.size(), false));
	addrAdjList.resize(addresses.size(), vector<int>());
	for (i = 0; i < addresses.size(); i++) {
		for (j = 1; j < stops.size(); j++) {
			if (wDist[i][j] <= maxWalkDist) {
				addrStopAdj[i][j] = true;
				addrAdjList[i].push_back(j);
			}
		}
		if (addrAdjList[i].size() == 0) {
			cout << "Error. Address " << i << "(" << addresses[i].label << ") has no bus stop within " << maxWalkDist << " " << distUnits << ". Invalid input file\n";
			exit(1);
		}
		if (addrAdjList[i].size() == 1) {
			//Address adjacent to just one adjacent bus stop. So this stop is required in a solution
			stops[addrAdjList[i][0]].required = true;
		}
	}
	//Adj list specifying the addresses adjacent to each stop
	stopAdjList.resize(stops.size(), vector<int>());
	for (i = 1; i < stops.size(); i++) {
		for (j = 0; j < addresses.size(); j++) {
			if (wDist[j][i] <= maxWalkDist) stopAdjList[i].push_back(j);
		}
		if (stopAdjList[i].size() == 0) {
			cout << "Error. Stop " << i << "(" << stops[i].label << ") is isolated (more than " << maxWalkDist << " " << distUnits <<" from any address). Invalid input file.\n";
			//If the following exit statement is removed, the program will work just fine. It is there to let me know if the problem instance has not been generated correctly
			exit(1);
		}
	}
	//In each adjacencey list we now sort the items so that, on each row i, the stops (addresses) are in ascending order of distance
	for (i = 0; i < addresses.size(); i++) {
		qSortStopsByDist(addrAdjList[i], 0, addrAdjList[i].size(), i);
	}
	for (i = 1; i < stops.size(); i++) {
		qSortAddressesByDist(stopAdjList[i], 0, stopAdjList[i].size(), i);
	}
}

