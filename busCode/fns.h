#ifndef FNS_H
#define FNS_H

#include "main.h"

void randPermute(vector<int> &A);
double sumDouble(vector<double> &X);
bool approxEqual(double x, double y);
double calcDwellTime(int numPass);
double calcRCost(double l);
double calcRouteLenFromScratch(SOL &S, int route);
double calcSolCostFromScratch(SOL &S);
double calcWalkCostFromScratch(SOL &S);
int calcWSum(SOL &S, int v);
double roundUp(double x, double base);
double roundDown(double x, double base);
bool existsCommonStop(SOL &S, int r1, int r2);
bool containsOutlierStop(vector<int> &R);
void prettyPrintSol(SOL &S);
void checkSolutionValidity(SOL &S, bool shouldBeMinimal);
void calcMetrics(double &stopsPerAddr, double &addrPerStop);
int calcSingletonStops(SOL &S);
void getOutliers();

#endif //FNS