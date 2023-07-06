#include "Genetic.h"

/*	Jonas Schrader
	- main structure of the genetic algorithm and the crossover operator
*/

void Genetic::run(int maxIterNonProd, int timeLimit)
{	
	int nbIterNonProd = 1;
	for (int nbIter = 0 ; nbIterNonProd <= maxIterNonProd && clock()/CLOCKS_PER_SEC < timeLimit ; nbIter++)
	{	
		/* SELECTION AND CROSSOVER */
		crossoverOnePoint(offspring, population->getBinaryTournament(),population->getBinaryTournament());

		/* LOCAL SEARCH */
		localSearch->run(offspring, params->penaltyCapacity, params->penaltyDuration);
		// std::cout << "Offspring feasibility after local search: " << offspring->isFeasible << std::endl;
		bool isNewBest = population->addIndividual(offspring,true);
		if (!offspring->isFeasible && std::rand()%2 == 0) // Repair half of the solutions in case of infeasibility
		{
			localSearch->run(offspring, params->penaltyCapacity*10., params->penaltyDuration*10.);
			if (offspring->isFeasible) isNewBest = (population->addIndividual(offspring,false) || isNewBest);
		}

		/* TRACKING THE NUMBER OF ITERATIONS SINCE LAST SOLUTION IMPROVEMENT */
		if (isNewBest) nbIterNonProd = 1;
		else nbIterNonProd ++ ;

		/* DIVERSIFICATION, PENALTY MANAGEMENT AND TRACES */
		if (nbIter % 100 == 0) population->managePenalties() ;
		if (nbIter % 500 == 0) { 
			population->printState(nbIter, nbIterNonProd);
			
			Individual *best = population->getBestFound();

			if (best != NULL) {
				best->exportJsonFormat(params->pathToSolution, nbIter);
			}
		}

		/* FOR TESTS INVOLVING SUCCESSIVE RUNS UNTIL A TIME LIMIT: WE RESET THE ALGORITHM/POPULATION EACH TIME maxIterNonProd IS ATTAINED*/
		if (timeLimit != INT_MAX && nbIterNonProd == maxIterNonProd)
		{
			population->restart();
			nbIterNonProd = 1;
		}
	}
}

void Genetic::printTours(Individual *indiv) {

	for (uint j = 0; j < indiv->chromR.size(); j++) {
		
		std::vector<int> currTour = indiv->chromR[j];

		for (uint i = 0; i < currTour.size(); i++) {
			std::cout << currTour[i] << " ";
		}

		std::cout << "| ";
	}

	std::cout << std::endl;
}

// Implementing parent selection / one point crossover from (https://arxiv.org/pdf/1809.10584.pdf)
void Genetic::crossoverOnePoint(Individual * result, const Individual * parent1, const Individual * parent2)
{
	// Frequency table to track the customers which have been already inserted
	std::vector <bool> freqClient = std::vector <bool> (params->nbClients + 1, false);	// vector of size nbClienst+1 and all positions set to false
	std::vector <bool> missingDropOffLocations = std::vector <bool> (params->nbClients + 1, false);	// vector highlighting missing delivery locations after first parent traversal NEW

	// Picking end of the crossover zone
	int start = 0;									// one point crossover (Velasco et al. 2009 A Memetic Algorithm for a Pick-Up and Delivery Problem by Helicopter)	NEW
	int end = std::rand() % params->nbClients;
	while (end == start) end = std::rand() % params->nbClients;

	// Copy in place the elements from start to end (possibly "wrapping around" the end of the array)
	int j = start;
	while (j != (end + 1) % params->nbClients)
	{
		int currentClientIdx = parent1->chromT[j];

		result->chromT[j] = currentClientIdx;
		
		freqClient[currentClientIdx] = true;

		if (params->cli[currentClientIdx].getsPickedUp) {
			int dropOffLocIdx = params->cli[currentClientIdx].deliveryNum;

			missingDropOffLocations[dropOffLocIdx] = true;		// marking each needed drop off location
		}
		else {		// current client is a drop off location (params->cli[currentClientIdx].getsDelivered is TRUE)
			missingDropOffLocations[currentClientIdx] = false;
		}

		j++;
	}

	// Fill the remaining elements in the order given by the second parent
	// FIRST run (Add missing drop off locations)
	for (int i = 0; i < params->nbClients; i++)
	{
		int currentClientIdx = parent2->chromT[i];
		
		if (missingDropOffLocations[currentClientIdx] == true) {
			result->chromT[j] = currentClientIdx;
			
			missingDropOffLocations[currentClientIdx] = false;
			freqClient[currentClientIdx] = true;

			j++;
		}
	}

	// SECOND run (Add missing nodes)
	for (int i = 0; i < params->nbClients; i++)
	{
		int currentClientIdx = parent2->chromT[i];

		if (freqClient[currentClientIdx] == false)
		{
			result->chromT[j] = currentClientIdx;
			j++;
		}
	}

	// Completing the individual with the Split algorithm
	split->generalSplit(result, params->nbVehicles);		// NEW dunno y not //parent1->myCostSol.nbRoutes);
}

Genetic::Genetic(Params * params, Split * split, Population * population, LocalSearch * localSearch) : params(params), split(split), population(population), localSearch(localSearch)
{
	offspring = new Individual(params);
}

Genetic::~Genetic(void)
{ 
	delete offspring ;
}
