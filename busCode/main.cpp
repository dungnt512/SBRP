#include "main.h"

//Global variables
vector<STOP> stops;
vector<ADDR> addresses;
vector<bool> isOutlier;
vector<vector<double> > dDist;
vector<vector<double> > dTime;
vector<vector<double> > wDist;
vector<vector<double> > wTime;
vector<vector<int> > stopAdjList; //Gives a list of addresses adjacent to each stop
vector<vector<int> > addrAdjList; //Gives a list of stops adjacent to each address
vector<vector<bool> > addrStopAdj;
int totalPassengers, maxBusCapacity, kInit, timePerK;
string distUnits;
double maxWalkDist;
double minEligibilityDist;
double dwellPerPassenger;
double dwellPerStop;
double excessWeight;
double maxJourneyTime;
double discreteLevel;
int verbosity;
bool useMinCoverings;

void printSln(SOL &S) {
	//Writes details of a particular solution to the screen
	int i, j, k = S.items.size();
	bool containsOutlier = false;
	cout << "****************SOLUTION******************************\n";
	cout << "Number of buses = " << S.items.size() <<"\n"
		<< "Average walk time per person = " << (S.costWalk / double(totalPassengers)) / 60.0 << " mins.\n"		
		<< "Average route length = " << (sumDouble(S.routeLen) / double(k)) / 60.0 << " mins.\n"
		<< "The bus stops visited in each route, in order, are as follows:\n";
	for (i = 0; i < k; i++) {
		cout << "Route" << setw(3) << i << " (" << setw(3) << int(ceil(S.routeLen[i] / 60.0)) << " mins) = ( ";
		for (j = 0; j < S.items[i].size(); j++) {
			if (isOutlier[S.items[i][j]]) {
				cout << S.items[i][j] << "* ";
				containsOutlier = true;
			}
			else cout << S.items[i][j] << " ";
		}
		cout << ")\n";
	}
	if (containsOutlier) cout << "Outlier bus stops (compulsory bus stops more than m_t mins from the school) are marked with an *s.\n";
	cout << "The address -> bus stop assignments, in order, are as follows:\n{ ";
	for (i = 0; i < addresses.size(); i++) {
		cout << "(" << i << "," << S.assignedTo[i] << ") ";
	}
	cout << "}\n";
	cout << "******************************************************\n\n";
}

SOL ILS(int k, bool &foundFeas) {
	//The ILS algorithm for producing a solution using k buses.
	bool feasible = false;
	foundFeas = false;
	int i = 1;
	SOL S, bestS;
	double feasRatio;
	clock_t endTime;
	int numMoves, stopsDeleted, maxIts;
	//Decide if we're running the procedure to a time limit or iteration limit
	if (timePerK >= 0) {
		endTime = clock() + timePerK * CLOCKS_PER_SEC;
		maxIts = 0;
	}
	else {
		endTime = 0;
		maxIts = timePerK * -1;
	}
	//Produce an inital solution and move to a minimum
	makeInitSol(S, k, 1);
	feasible = localSearch(S, feasRatio, numMoves);
	if (feasible && !foundFeas) {
		//Feasibility has been found for the first time,
		foundFeas = true;
	}
	bestS = S;
	if (verbosity >= 2) {
		cout << "\n  k      it        Cost  #Feas #Empty       #Stops    StopsDel  MovesToMin    BestCost\n";
		cout << "-------------------------------------------------------------------------------------------------\n";
		cout << setw(3) << k << setw(8) << i << setw(12) << S.cost << setw(7) << S.numFeasibleRoutes << setw(7) << S.numEmptyRoutes << setw(10) << S.solSize << "/" << S.numUsedStops << setw(12) << "-" << setw(12) << numMoves << setw(12) << bestS.cost << "\n";
	}
	while(clock() < endTime || i <= maxIts) {
		//Peturb the solution and move to the minimum
		stopsDeleted = makeNewCovering(S);
		feasible = localSearch(S, feasRatio, numMoves);
		i++;
		if (feasible && !foundFeas) {
			//Feasibility has been found for the first time, so record the solution
			foundFeas = true;
			bestS = S;
		}
		else if (feasible && S.cost < bestS.cost) {
			//A new feasible solution has been found with an even better cost, so record it 
			bestS = S;
		}
		else if (!feasible && !foundFeas && S.cost < bestS.cost) {
			//Feasibility has not yet been found, but we have found a better infeasible solution so record it
			bestS = S;
		}
		//Note, we do not accept a new infeasible solution that has a better cost than a previously oberved feasible solution
		if (verbosity >= 2) {
			cout << setw(3) << k << setw(8) << i << setw(12) << S.cost << setw(7) << S.numFeasibleRoutes << setw(7) << S.numEmptyRoutes << setw(10) << S.solSize << "/" << S.numUsedStops << setw(12) << stopsDeleted << setw(12) << numMoves << setw(12) << bestS.cost << "\n";
		}
	}
	return bestS;
}


//Info output if different no parameters used
void usage() {
	cout << "School Bus Optimiser\n";
	cout << "--------------------\n";
	cout << "USAGE:\n"
		<< "---- Compulsory -------------------------------------------\n"
		<< "-i  <inFileName>         (must be a .bus file in the correct format. Do not include extension)\n"
		<< "---- Optional ---------------------------------------------\n"
		<< "-m  <double>             (Maximum bus journey time in minutes. Default = 45.0)\n"
		<< "-c                       (Maximum bus capacity. Default = 70)\n"
		<< "-t  <int>                (CPU time limit per-k in seconds. Default = 10. If negative value used, defines number of calls to LS (iterations) per k instead.)\n"
		<< "-D  <double>             (Discrete level. Minimum number of secs between each solution in the Pareto front. Larger values make runs faster but less accurate. Default = 10.0)\n"
		<< "-M                       (If present, bus stop subsets in Stage-1 must be minimal set coverings; else not.)\n"
		<< "-S                       (If present, only Stage 1 of the algorithm is run.)\n"
		<< "------------\n"
		<< "-d  <double> <double>    (Dwell time coefficients, seconds-per-passenger and seconds-per-stop resp. Defaults = 5.0 and 15.0)\n"
		<< "-r  <int>                (Random seed. Default = 1)\n"
		<< "-k  <int>                (Number of buses k to start at. Default is the lower bound (numStudents divided by busCapacity, rounded up to nearest integer)\n"
		<< "-v                       (Verbosity. Repeat for more output to the screen)\n"
		<< "------------------------------------------------------------\n";
	exit(0);
}

int main(int argc, char *argv[]){

	if(argc <=1){
		usage();
		exit(1);
	}

	//Determine run variables and set default values
	int i, totalTime, k = -1, seed = 1;
	string infile;
	bool foundFeas;
	verbosity = 0;
	dwellPerPassenger = 5.0;
	discreteLevel = 10.0;
	timePerK = 10;
	dwellPerStop = 15.0;
	maxBusCapacity = 70;
	useMinCoverings = false;
	bool stageOneOnly = false;
	double maxJourneyTimeMins = 45.0;
	list<SOL> A;
		
	try {
		//Read in all command line parameters. If there's an error, end immediately.
		for (i = 1; i < argc; i++) {
			if (strcmp("-r", argv[i]) == 0) {
				seed = atoi(argv[++i]);
			}
			else if (strcmp("-t", argv[i]) == 0) {
				timePerK = atoi(argv[++i]);
			}
			else if (strcmp("-d", argv[i]) == 0) {
				dwellPerPassenger = atof(argv[++i]);
				dwellPerStop = atof(argv[++i]);
			}
			else if (strcmp("-k", argv[i]) == 0) {
				k = atoi(argv[++i]);
			}
			else if (strcmp("-m", argv[i]) == 0) {
				maxJourneyTimeMins = atof(argv[++i]);
			}
			else if (strcmp("-S", argv[i]) == 0) {
				stageOneOnly = true;
			}
			else if (strcmp("-v", argv[i]) == 0) {
				verbosity++;
			}
			else if (strcmp("-c", argv[i]) == 0) {
				maxBusCapacity = atoi(argv[++i]);
			}
			else if (strcmp("-D", argv[i]) == 0) {
				discreteLevel = atof(argv[++i]);
			}
			else if (strcmp("-M", argv[i]) == 0) {
				useMinCoverings = true;
			}
			else if (strcmp("-i", argv[i]) == 0) {
				//read in the problem file (in the .bus format) and construct the relevant arrays.
				infile = argv[++i];
				string infileWithExtension = infile + ".bus";
				readInput(infileWithExtension);
			}
			else {
				cout << "Invalid input statement. ("<< argv[i] <<"). Please try again.\n";
				usage();
				exit(1);
			}
		}
	}
	catch (...) {
		cout << "Invalid input statement. Please try again.\n";
		usage();
		exit(1);
	}

	//Convert max journey time to seconds to be consistent with input files
	maxJourneyTime = maxJourneyTimeMins * 60.0;

	//By default the excess weight is set to the max journey time (in seconds)
	excessWeight = maxJourneyTime;

	//Set seed and start the clock	
	srand(seed);
	time_t startTime, endTime;
	startTime = clock();

	//Determine any bus stops that are outliers (i.e. far from the school) by populating the isOutlier vector
	getOutliers();

	//Determine initial number of buses kInit. This is either the LB or specified by the user 
	kInit = int(ceil(totalPassengers / double(maxBusCapacity)));
	if (k < kInit) k = kInit;

	//Algorithm Stage 1: Find a feasible solution --------------------------------------------------------
	foundFeas = false;
	SOL S;
	while(k <= addresses.size()) {
		if (verbosity >= 1) cout << "\nUsing ILS to find a feasible solution using " << k << " buses:" << endl;
		S = ILS(k, foundFeas);
		if (foundFeas) break;
		else  k++;
	}
	
	//Record how long it took to find a feasible solution and output some info
	endTime = clock();
	int midTime = (int)(((endTime - startTime) / double(CLOCKS_PER_SEC)) * 1000);
	if (verbosity >= 1) {
		cout << "\nILS method completed in " << midTime << " ms\n";
		checkSolutionValidity(S, true);
		if (verbosity >= 2) {
			cout << "\nHere is the best solution found by ILS:\n\n";
			printSln(S);
		}
	}

	if (stageOneOnly == false) {
		//We are now running Stage 2 (the multi-objective part) too. First we Add this single feasible solution to the archive A
		A.push_back(S);
		//Now do the multiobjective optimisation
		doMultiObjOptimisation(A);
		//Stop the clock
		endTime = clock();
		totalTime = (int)(((endTime - startTime) / double(CLOCKS_PER_SEC)) * 1000);
		if (verbosity >= 1) {
			cout << "\nRun completed in " << totalTime << " ms" << endl;
		}
	}
	cout << "Run details have been appended to log-results.txt" << endl;

	//Finally, output some information on the run to a log file
	ofstream resultsLog("log-results.txt", ios::app);
	double stopsPerAddr, addrPerStop;
	int numSingletonStops;
	calcMetrics(stopsPerAddr, addrPerStop);
	numSingletonStops = calcSingletonStops(S);
	
	//Information on the problem instance
	resultsLog << infile << "\t"
		<< stops.size() << "\t"
		<< addresses.size() << "\t"
		<< totalPassengers << "\t"
		<< distUnits << "\t"
		<< maxWalkDist << "\t"
		<< minEligibilityDist << "\t"
		<< stopsPerAddr << "\t"
		<< addrPerStop << "\t"
		<< dwellPerPassenger << "\t"
		<< dwellPerStop << "\t"
		<< maxBusCapacity << "\t";

	//Information on the run options used
	resultsLog << maxJourneyTimeMins << "\t"
		<< seed << "\t"
		<< kInit << "\t"
		<< timePerK << "\t"
		<< discreteLevel << "\t";
	if (useMinCoverings)resultsLog << "minCoveringsOnly\t";
	else				resultsLog << "AllCoverings\t";

	//Information on Stage 1's solution
	resultsLog << k << "\t"
		<< S.cost << "\t"
		<< S.numUsedStops << "\t"
		<< S.numUsedStops - numSingletonStops << "\t"
		<< S.solSize << "\t"
		<< S.numRoutesWithOutliers << "\t";
		if (foundFeas) resultsLog << "foundFeas\t";
		else resultsLog << "NoFeasFound\t";

	if (stageOneOnly == false) {
		//A is a sorted archive set containing all solutions found in the multiobjective optimisation process
		//First copy only the feasible solutions into a new front APrime
		list<SOL> APrime;
		list<SOL>::iterator AIt;
		for (AIt = A.begin(); AIt != A.end(); ++AIt) {
			if ((*AIt).numFeasibleRoutes == k) APrime.push_back(*AIt);
		}
		//Output some deatils to the log file
		resultsLog << A.size() << "\t"
			<< APrime.size() << "\t"
			<< totalTime << "\t";
		//Also add the costs of all solutions in APrime (the feasible solutions) to the logArchive
		cout << "Costs of solutions in the final archive set have been appended to log-archive.txt" << endl;
		ofstream archiveLog("log-archive.txt", ios::app);
		for (AIt = APrime.begin(); AIt != APrime.end(); ++AIt) {
			archiveLog << (*AIt).costWalk / double(totalPassengers) / 60.0 << "\t";
		}
		archiveLog << "\n";
		for (AIt = APrime.begin(); AIt != APrime.end(); ++AIt) {
			archiveLog << (*AIt).cost / double(k) / 60.0 << "\t";
		}
		archiveLog << "\n";
		if(verbosity >= 2) {
			cout << "\n\nHere are the " << APrime.size() << " feasible solutions in the final archive set: \n\n";
			for (AIt = APrime.begin(); AIt != APrime.end(); ++AIt) {
				printSln(*AIt);
			}
		}
	}
	resultsLog << "\n";
	resultsLog.close();
	
	//End of entire algorithm
	return 0;
}
