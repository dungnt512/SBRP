#ifndef BPP_H
#define BPP_H

#include "main.h"

void binPacker(vector<vector<int> > &items, vector<vector<int> > &W, vector<int> &binSize, vector<int> &itemsToAdd, vector<int> &itemsToAddWeight);
void binPacker(vector<vector<int> > &items, vector<vector<int> > &W, vector<int> &binSize, int itemToPack, int itemToPackSize);

#endif //BPP
