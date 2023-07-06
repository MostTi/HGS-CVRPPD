#include "Split.h" 

void Split::generalSplit(Individual * indiv, int nbMaxVehicles)
{
	// load of the giant tour for feasibility check
	int chromTLoad = 0;

	// reinitialize the predecessor matrix
	pred = std::vector<std::vector<int>>(params->nbVehicles + 1, std::vector <int>(params->nbClients + 1,0));

	// Do not apply Split with fewer vehicles than the trivial (LP) bin packing bound
	maxVehicles = std::max<int>(nbMaxVehicles, std::ceil(params->totalDemand/params->vehicleCapacity));

	maxVehicles = nbMaxVehicles;
	// Initialization of the data structures for the linear split algorithms
	// Direct application of the code located at https://github.com/vidalt/Split-Library


	for (int i = 1; i <= params->nbClients; i++)
	{
		cliSplit[i].okForSplit = false;		// can be done prettier
		cliSplit[i].demand = params->cli[indiv->chromT[i - 1]].demand;
		cliSplit[i].serviceTime = params->cli[indiv->chromT[i - 1]].serviceDuration;
		cliSplit[i].d0_x = params->timeCost[0][indiv->chromT[i - 1]];
		cliSplit[i].dx_0 = params->timeCost[indiv->chromT[i - 1]][0];
		if (i < params->nbClients) cliSplit[i].dnext = params->timeCost[indiv->chromT[i - 1]][indiv->chromT[i]];
		else cliSplit[i].dnext = -1.e30;
		sumLoad[i] = sumLoad[i - 1] + cliSplit[i].demand;
		sumService[i] = sumService[i - 1] + cliSplit[i].serviceTime;
		sumDistance[i] = sumDistance[i - 1] + cliSplit[i - 1].dnext;
		
		chromTLoad += cliSplit[i].demand;
		if (chromTLoad == 0) {
			cliSplit[i].okForSplit = true;
		}
	}

	// We first try the simple split, and then the Split with limited fleet if this is not successful
	if (splitSimple(indiv) == 0)
		splitLF(indiv);

	// Build up the rest of the Individual structure
	indiv->evaluateCompleteCost();
}

int Split::splitSimple(Individual * indiv) {

	// Reinitialize the potential structures
	potential[0][0] = 0;

	for (int i = 1; i <= params->nbClients; i++)
		potential[0][i] = 1.e30;

	// Simple Split using Bellman's algorithm in compliance with the pickup and delivery order 
	for (int i = 0; i < params->nbClients; i++) {
		
		double load = 0.;
		double distance = 0.;
		double serviceDuration = 0.;

		for (int j = i + 1; j <= params->nbClients && load <= 1.5 * params->vehicleCapacity ; j++) {

			load += cliSplit[j].demand;
			serviceDuration += cliSplit[j].serviceTime;
				
			if (j == i + 1) distance += cliSplit[j].d0_x;
			else distance += cliSplit[j - 1].dnext;
				
			double cost = distance + cliSplit[j].dx_0
				+ params->penaltyCapacity * std::max<double>(load - params->vehicleCapacity, 0.)
				+ params->penaltyDuration * std::max<double>(distance + cliSplit[j].dx_0 + serviceDuration - params->durationLimit, 0.);
				
			// additionally check if load is 0
			// if load is 0 the current subtour IS feasible because our tours can't violate the order of the pickup and delivery stops
			// and that's why the order is okay if the load is 0
			if ((potential[0][i] + cost < potential[0][j])) {

				potential[0][j] = potential[0][i] + cost;	// has to be improved in order to allow further better potentials

				if (load == 0 && cliSplit[i].okForSplit) {
					pred[0][j] = i;
				}
			}
		}
	}
	
	if (potential[0][params->nbClients] > 1.e29)
		exit(24);		//throw std::string("ERROR : no Split solution has been propagated until the last node");

	// Filling the chromR structure
	for (int k = params->nbVehicles - 1; k >= maxVehicles; k--)
		indiv->chromR[k].clear();

	int end = params->nbClients;

	for (int k = maxVehicles - 1; k >= 0; k--) {

		indiv->chromR[k].clear();
		int begin = pred[0][end];
		
		for (int ii = begin; ii < end; ii++)
			indiv->chromR[k].push_back(indiv->chromT[ii]);

		end = begin;
	}

	// Return OK in case the Split algorithm reached the beginning of the routes
	return (end == 0);
}

// Split for problems with limited fleet
int Split::splitLF(Individual * indiv) {

	// Initialize the potential structures
	potential[0][0] = 0;
	
	for (int k = 0; k <= maxVehicles; k++)
		for (int i = 1; i <= params->nbClients; i++)
			potential[k][i] = 1.e30;

	// Simple Split using Bellman's algorithm in compliance with the pickup and delivery order
	for (int k = 0; k < maxVehicles; k++) {

		for (int i = k; i < params->nbClients && potential[k][i] < 1.e29 ; i++) {

			double load = 0.;
			double serviceDuration = 0.;
			double distance = 0.;
				
			for (int j = i + 1; j <= params->nbClients && load <= 1.5 * params->vehicleCapacity ; j++) {

				load += cliSplit[j].demand;
				serviceDuration += cliSplit[j].serviceTime;
					
				if (j == i + 1) distance += cliSplit[j].d0_x;
				else distance += cliSplit[j - 1].dnext;
					
				double cost = distance + cliSplit[j].dx_0
							+ params->penaltyCapacity * std::max<double>(load - params->vehicleCapacity, 0.)
							+ params->penaltyDuration * std::max<double>(distance + cliSplit[j].dx_0 + serviceDuration - params->durationLimit, 0.);
					
				if (potential[k][i] + cost < potential[k + 1][j]) {
						
					potential[k + 1][j] = potential[k][i] + cost;

					if (load == 0 && cliSplit[i].okForSplit) {
						pred[k + 1][j] = i;
					}
				}
			}
		}
	}
	
	if (potential[maxVehicles][params->nbClients] > 1.e29) {
		std::cout << "ERROR split was propagated to the last node" << std::endl;
		exit(24);
	}

	// It could be cheaper to use a smaller number of vehicles
	double minCost = potential[maxVehicles][params->nbClients];
	int nbRoutes = maxVehicles;
	
	for (int k = 1; k < maxVehicles; k++)
		if (potential[k][params->nbClients] < minCost)
			{minCost = potential[k][params->nbClients]; nbRoutes = k;}

	// Filling the chromR structure
	for (int k = params->nbVehicles-1; k >= nbRoutes ; k--)
		indiv->chromR[k].clear();

	int end = params->nbClients;
	
	for (int k = nbRoutes - 1; k >= 0; k--) {

		indiv->chromR[k].clear();
		int begin = pred[k+1][end];
		
		for (int ii = begin; ii < end; ii++)
			indiv->chromR[k].push_back(indiv->chromT[ii]);
		
		end = begin;
	}

	// Return OK in case the Split algorithm reached the beginning of the routes
	return (end == 0);
}

Split::Split(Params * params): params(params)
{
	// Structures of the linear Split
	cliSplit = std::vector <ClientSplit>(params->nbClients + 1);
	sumDistance = std::vector <double>(params->nbClients + 1,0.);
	sumLoad = std::vector <double>(params->nbClients + 1,0.);
	sumService = std::vector <double>(params->nbClients + 1, 0.);
	potential = std::vector < std::vector <double> >(params->nbVehicles + 1, std::vector <double>(params->nbClients + 1,1.e30));
	pred = std::vector < std::vector <int> >(params->nbVehicles + 1, std::vector <int>(params->nbClients + 1,0));
}
