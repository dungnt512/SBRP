#ifndef INITSOL_H
#define INITSOL_H

#include "main.h"

void eliminateFromW(SOL &S, int v, int x);
void repopulateAuxiliaries(SOL &S);
void makeInitSol(SOL &S, int k, int heurisic);
void rebuildSolution(SOL &S);

#endif //INITSOL