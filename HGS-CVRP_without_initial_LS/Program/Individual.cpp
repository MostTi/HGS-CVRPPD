#include "Individual.h" 

/*	Jonas Schrader
	- Representing the solutions through the search
	- Complete solutions including trip delimiters with their associated giant tours
*/

void Individual::evaluateCompleteCost()
{
	myCostSol = CostSol();
	orderIsFeasible = true;													// initial set to true	NEW
	for (int r = 0; r < params->nbVehicles; r++)
	{

		if (!chromR[r].empty())
		{
			std::vector<int> visitFlagVector(params->nbClients, 0);			// vector marking each visited stop with 1 (Index is the same as the customer index) NEW

			double distance = params->timeCost[0][chromR[r][0]];			// distance from depot to the first node
			
			visitFlagVector[chromR[r][0]] =  1;								// set visit-flag of the first client
			
			// check whether the pickup location has been serviced until this client
			// if not the solution is automatically not feasible
			if (params->cli[chromR[r][0]].pickupNum != -1) {
				if (visitFlagVector[params->cli[chromR[r][0]].pickupNum] != 1) {
					orderIsFeasible = false;
				}
			}

			double load = params->cli[chromR[r][0]].demand;					// demand of first client
			double service = params->cli[chromR[r][0]].serviceDuration;		// service duration of first client
			predecessors[chromR[r][0]] = 0;									// set predecessor of the first node to the depot

			for (int i = 1; i < (int)chromR[r].size(); i++)					// iterate through all clients
			{
				visitFlagVector[chromR[r][i]] = 1;							// NEW

				if (params->cli[chromR[r][i]].pickupNum != -1) {
					if (visitFlagVector[params->cli[chromR[r][i]].pickupNum] != 1) {
						orderIsFeasible = false;
					}
				}

				distance += params->timeCost[chromR[r][i-1]][chromR[r][i]];	// add distance from (i-1)th. client to the (i)th. client
				load += params->cli[chromR[r][i]].demand;					// add load ..
				service += params->cli[chromR[r][i]].serviceDuration;		// add service duration
				predecessors[chromR[r][i]] = chromR[r][i-1];				// set predecessor of (i)th. client
				successors[chromR[r][i-1]] = chromR[r][i];					// set successor of (i-1)th. client
			}

			successors[chromR[r][chromR[r].size()-1]] = 0;					// set successor of last client to the depot
			distance += params->timeCost[chromR[r][chromR[r].size()-1]][0];	// add distance from last client to the depot
			
			// OVERALL SOLUTION SPECIFICS
			myCostSol.distance += distance;									// add overall distance to whole solution distance
			myCostSol.nbRoutes++;											// increase number of routes for the whole solution

			// EXCESSES (CAPACITY, DURATION)
			if (load > params->vehicleCapacity) myCostSol.capacityExcess += load - params->vehicleCapacity;		// add capacity excess if existing
			if (distance + service > params->durationLimit) myCostSol.durationExcess += distance + service - params->durationLimit;		// add duration excess if existing
		
			// for (int i = 0; i < (int)chromR[r].size(); i++) {
			// 	std::cout << chromR[r][i];

			// 	if (params->cli[chromR[r][i]].pickupNum != -1) {
			// 		std::cout << "(" << params->cli[chromR[r][i]].pickupNum << ")";
			// 	}

			// 	std::cout << "->";
			// }

			// for (int i : visitFlagVector) {
			// 	std::cout << i << ' ';
			// }

			// std::cout << ' ' << orderIsFeasible << std::endl;
		}
	}

	// PENALIZED COST = (solution distance) + capPenality * capacityExcess + durPenality * durationExcess
	myCostSol.penalizedCost = myCostSol.distance + myCostSol.capacityExcess*params->penaltyCapacity + myCostSol.durationExcess*params->penaltyDuration;
	
	// FEASIBILITY CHECK
	isFeasible = (myCostSol.capacityExcess < MY_EPSILON && myCostSol.durationExcess < MY_EPSILON && orderIsFeasible);		// NEW
}

void Individual::removeProximity(Individual * indiv)
{
	auto it = indivsPerProximity.begin();
	while (it->second != indiv) ++it;
	indivsPerProximity.erase(it);
}

double Individual::brokenPairsDistance(Individual * indiv2)
{
	int differences = 0;
	for (int j = 1; j <= params->nbClients; j++)
	{
		if (successors[j] != indiv2->successors[j] && successors[j] != indiv2->predecessors[j]) differences++;
		if (predecessors[j] == 0 && indiv2->predecessors[j] != 0 && indiv2->successors[j] != 0) differences++;
	}
	return (double)differences/(double)params->nbClients;
}

double Individual::averageBrokenPairsDistanceClosest(int nbClosest) 
{
	double result = 0 ;
	int maxSize = std::min<int>(nbClosest, indivsPerProximity.size());
	auto it = indivsPerProximity.begin();
	for (int i=0 ; i < maxSize; i++)
	{
		result += it->first ;
		++it ;
	}
	return result/(double)maxSize ;
}

void Individual::exportCVRPLibFormat(std::string fileName)
{
	std::cout << "----- WRITING SOLUTION WITH VALUE " << myCostSol.penalizedCost << " IN : " << fileName << std::endl;
	std::ofstream myfile(fileName);
	if (myfile.is_open())
	{
		for (int k = 0; k < params->nbVehicles; k++)
		{
			if (!chromR[k].empty())
			{
				myfile << "Route #" << k+1 << ":"; // Route IDs start at 1 in the file format
				for (int i : chromR[k]) myfile << " " << i;
				myfile << std::endl;
			}
		}
		myfile << "Cost " << myCostSol.penalizedCost << std::endl;
		myfile << "Time " << (double)clock()/(double)CLOCKS_PER_SEC << std::endl;
	}
	else std::cout << "----- IMPOSSIBLE TO OPEN: " << fileName << std::endl;
}

bool Individual::readCVRPLibFormat(std::string fileName, std::vector<std::vector<int>> & readSolution, double & readCost)
{
	readSolution.clear();
	std::ifstream inputFile(fileName);
	if (inputFile.is_open())
	{
		std::string inputString;
		inputFile >> inputString;
		// Loops as long as the first line keyword is "Route"
		for (int r = 0; inputString == "Route" ; r++) 
		{
			readSolution.push_back(std::vector<int>());
			inputFile >> inputString;
			getline(inputFile, inputString);
			std::stringstream ss(inputString);
			int inputCustomer;
			while (ss >> inputCustomer) // Loops as long as there is an integer to read
				readSolution[r].push_back(inputCustomer);
			inputFile >> inputString;
		}
		if (inputString == "Cost")
		{
			inputFile >> readCost;
			return true;
		}
		else std::cout << "----- UNEXPECTED WORD IN SOLUTION FORMAT: " << inputString << std::endl;
	}
	else std::cout << "----- IMPOSSIBLE TO OPEN: " << fileName << std::endl;
	return false;
}

Individual::Individual(Params * params) : params(params)
{
	successors = std::vector <int>(params->nbClients + 1);
	predecessors = std::vector <int>(params->nbClients + 1);
	chromR = std::vector < std::vector <int> >(params->nbVehicles);
	chromT = std::vector <int>(params->nbClients);
	for (int i = 0; i < params->nbClients; i++) chromT[i] = i + 1;
	std::random_shuffle(chromT.begin(), chromT.end());
}

Individual::Individual()
{
	myCostSol.penalizedCost = 1.e30;
}