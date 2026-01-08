#include "bpp.h"

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

void swapVals(int &x, int &y) {
	int z = x; x = y; y = z;
}

void quickRemove(int pos, vector<int> &itemWeight, vector<int> &itemLabel) {
	swapVals(itemWeight[pos], itemWeight.back());
	swapVals(itemLabel[pos], itemLabel.back());
	itemWeight.pop_back();
	itemLabel.pop_back();
}

int getLargestItem(vector<int> &itemWeight) {
	//Returns the position of the largest integer in a vector of integers
	int i, max = itemWeight[0], maxPos = 0;
	for (i = 1; i < itemWeight.size(); i++) {
		if (itemWeight[i] > max) {
			max = itemWeight[i];
			maxPos = i;
		}
	}
	return maxPos;
}

int posOfItemInBin(int item, vector<int> &items) {
	//Tells us the position of "item" in an array. Returns -1 if it isn't present
	int i;
	for (i = 0; i < items.size(); i++) {
		if (items[i] == item) return i;
	}
	return -1;
}

int chooseBinWithEnoughCapacity(vector<int> &binWeight, vector<vector<int> > &items, int v, int weightv, int &posOfV) {
	//Find the most suitable bin for item v. Do this by returning the first bin that has with adequate capacity 
	//and that already contains v. If such a bin does not exist, return the first bin with adequate capacity that 
	//does not contain v. Return -1 if neither exists.
	int i, k = binWeight.size(), binNoMultiStop = -1, pos;
	for (i = 0; i < k; i++) {
		if (binWeight[i] + weightv <= maxBusCapacity) {
			pos = posOfItemInBin(v, items[i]);
			if (pos != -1) {
				posOfV = pos;
				return i;
			}
			else if (binNoMultiStop == -1) {
				binNoMultiStop = i;
			}
		}
	}
	posOfV = -1;
	return binNoMultiStop;
}

int chooseEmptiestBin(vector<int> &binWeight, vector<vector<int> > &items, int v, int weightv, int &posOfV) {
	//This is used when no bin has adequate capacity. We therefore choose the emptiest bin that already contains v.
	//If none exists, just choose the emptiest bin. Assumes all bin weights are <= maxBusCapacity
	int i, k = binWeight.size(), minMulti = maxBusCapacity, minNoMulti = maxBusCapacity, minMultiPos = -1, minNoMultiPos = -1, pos;
	for (i = 0; i < k; i++) {
		pos = posOfItemInBin(v, items[i]);
		if (pos != -1) {
			if (binWeight[i] < minMulti) {
				minMulti = binWeight[i];
				posOfV = pos;
				minMultiPos = i;
			}
		}
		else {
			if (binWeight[i] < minNoMulti) {
				minNoMulti = binWeight[i];
				minNoMultiPos = i;
			}
		}
	}
	if (minMultiPos != -1) {
		return minMultiPos;
	}
	else {
		posOfV = -1;
		return minNoMultiPos;
	}
}

void binPacker(vector<vector<int> > &items, vector<vector<int> > &W, vector<int> &binSize, vector<int> &itemsToAdd, vector<int> &itemsToAddWeight) {
	//Generates an assignment of stops to buses / routes using BPP heuristics
	int pos, bin, spare, j;
	//Call the FFD-style BPP algorithm
	while (!itemsToAdd.empty()) {
		//Identify the largest item and search for a suitable bin
		pos = getLargestItem(itemsToAddWeight);
		bin = chooseBinWithEnoughCapacity(binSize, items, itemsToAdd[pos], itemsToAddWeight[pos], j);
		if (bin != -1) {
			//Bin with adequate capacity found. Assign item to bin and remove it from the itemWeight and itemLabel list.
			//If the item is already in the bin (j != -1), merge them, else just add it to the end
			if (j == -1) {
				items[bin].push_back(itemsToAdd[pos]);
				W[bin].push_back(itemsToAddWeight[pos]);
			}
			else {
				W[bin][j] += itemsToAddWeight[pos];
			}
			binSize[bin] += itemsToAddWeight[pos];
			quickRemove(pos, itemsToAddWeight, itemsToAdd);
		}
		else {
			//No single bin can accommodate the item, so use the bin with the most spare capacity for some of it
			//Again, if the item is already in the bin, merge them, else just add it to the end
			bin = chooseEmptiestBin(binSize, items, itemsToAdd[pos], itemsToAddWeight[pos], j);
			spare = maxBusCapacity - binSize[bin];
			if (j == -1) {
				items[bin].push_back(itemsToAdd[pos]);
				W[bin].push_back(spare);
			}
			else {
				W[bin][j] += spare;
			}
			binSize[bin] += spare;
			itemsToAddWeight[pos] -= spare;
		}
	}
}

void binPacker(vector<vector<int> > &items, vector<vector<int> > &W, vector<int> &binSize, int itemToPack, int itemToPackSize) {
	//Overloaded version of the above that packs just one item (bus stop)
	int bin, spare, j;
	while (true) {
		//Find a suitable bin
		bin = chooseBinWithEnoughCapacity(binSize, items, itemToPack, itemToPackSize, j);
		if (bin != -1) {
			//Assign item to bin i. (If the item is already in bin i, merge them)
			if (j == -1) {
				items[bin].push_back(itemToPack);
				W[bin].push_back(itemToPackSize);
			}
			else {
				W[bin][j] += itemToPackSize;
			}
			binSize[bin] += itemToPackSize;
			return;
		}
		else {
			//No single bin can accommodate the item, so use the bin with the most spare capacity for some of it
			bin = chooseEmptiestBin(binSize, items, itemToPack, itemToPackSize, j);
			spare = maxBusCapacity - binSize[bin];
			j = posOfItemInBin(itemToPack, items[bin]);
			if (j == -1) {
				items[bin].push_back(itemToPack);
				W[bin].push_back(spare);
			}
			else {
				W[bin][j] += spare;
			}
			binSize[bin] += spare;
			itemToPackSize -= spare;
		}
	}
}