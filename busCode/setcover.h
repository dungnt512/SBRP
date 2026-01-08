#ifndef SETCOVER_H
#define SETCOVER_H

#include "main.h"

void getClosestStops(vector<bool> &stopUsed);
void generateNewCovering(vector<bool> &stopUsed, int forbidden, int heuristic);
int makeNewCovering(SOL &S);

#endif //SETCOVER

