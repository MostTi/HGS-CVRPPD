#include "LocalSearch.h"

/*	Jonas Schrader
	- provides all functions needed for the local search (including SWAP* exploration procedure)
*/

// each individual has to be feasible in respect to the pickup and delivery precedence!! otherwise the moves are incorrect
void LocalSearch::run(Individual * indiv, double penaltyCapacityLS, double penaltyDurationLS)
{
	this->penaltyCapacityLS = penaltyCapacityLS;
	this->penaltyDurationLS = penaltyDurationLS;
	loadIndividual(indiv);

	// Shuffling the order of the nodes explored by the LS to allow for more diversity in the search
	std::random_shuffle(orderNodes.begin(), orderNodes.end());
	std::random_shuffle(orderRoutes.begin(), orderRoutes.end());

	for (int i = 1; i <= params->nbClients; i++)
		if (std::rand() % params->nbGranular == 0)  // Designed to use O(nbGranular x n) time overall to avoid possible bottlenecks
			std::random_shuffle(params->correlatedVertices[i].begin(), params->correlatedVertices[i].end());

	searchCompleted = false;
	for (loopID = 0; !searchCompleted && loopID < 10; loopID++)		// added max loop Id (has to be adapted in regards to testing the original HGS-CVRP but for now ok) TODO (not higher than 15)
	{
		searchCompleted = true;

		/* CLASSICAL ROUTE IMPROVEMENT (RI) MOVES SUBJECT TO A PROXIMITY RESTRICTION */
		for (int posU = 0; posU < params->nbClients; posU++)			// iterate over all nodes u randomly
		{
			nodeU = &clients[orderNodes[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;
			for (int posV = 0; posV < (int)params->correlatedVertices[nodeU->cour].size(); posV++)		// iterate over all its neighbors v (closest hn nodes; h is granularity factor)
			{
				nodeV = &clients[params->correlatedVertices[nodeU->cour][posV]];
				if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) // only evaluate moves involving routes that have been modified since last move evaluations for nodeU
				{
					// Randomizing the order of the neighborhoods within this loop does not matter much as we are already randomizing the order of the node pairs (and it's not very common to find improving moves of different types for the same node pair)
					setLocalVariablesRouteU();
					setLocalVariablesRouteV();

					if (routeU == routeV && move1()) continue; 									// RELOCATE	remove u from routeU and place it directly after v in routeV
					if (routeU == routeV && move2()) continue;  								// RELOCATE	remove u and u->next from routeU and place them directly after v in routeV
					if (routeU == routeV && move3()) continue;  								// RELOCATE	remove u and u->next from routeU and place them in reverse directly after v in routeV
					if (routeU == routeV && nodeUIndex <= nodeVIndex && move4()) continue;  	// SWAP	swap u and v from routeU and routeV
					if (routeU == routeV && move5()) continue;  								// SWAP	swap u and u->next with v from routeU and routeV
					if (routeU == routeV && nodeUIndex <= nodeVIndex && move6()) continue;  	// SWAP	swap u and u->next with v and v->next from routeU and routeV
					if (swapPair()) continue;													// TODO
					if (relocatePair()) continue;
					if (routeU == routeV && opt2k()) continue;
					// if (routeU == routeV && move7()) continue; // 2-OPT		// TODO
					// if (routeU != routeV && move8()) continue; // 2-OPT*		// TODO
					// if (routeU != routeV && move9()) continue; // 2-OPT*		// TODO

					// Trying moves that insert nodeU directly after the depot
					if (nodeV->prev->isDepot)
					{
						nodeV = nodeV->prev;		// nodeV is now the depot
						setLocalVariablesRouteV();
						if (routeU == routeV && move1()) continue; // RELOCATE
						if (routeU == routeV && move2()) continue; // RELOCATE
						if (routeU == routeV && move3()) continue; // RELOCATE
						if (relocatePair()) continue;		// TODO
						// if (routeU != routeV && move8()) continue; // 2-OPT*		TODO
						// if (routeU != routeV && move9()) continue; // 2-OPT*		TODO
					}
				}
			}

			/* MOVES INVOLVING AN EMPTY ROUTE -- NOT TESTED IN THE FIRST LOOP TO AVOID INCREASING TOO MUCH THE FLEET SIZE */
			if (loopID > 0 && !emptyRoutes.empty())
			{
				nodeV = routes[*emptyRoutes.begin()].depot;
				setLocalVariablesRouteU();
				setLocalVariablesRouteV();
				if (routeU == routeV && move1()) continue; // RELOCATE
				if (routeU == routeV && move2()) continue; // RELOCATE
				if (routeU == routeV && move3()) continue; // RELOCATE
				if (relocatePair()) continue;		// TODO
				// if (move9()) continue; // 2-OPT*		// TODO
			}
		}

		/* (SWAP*) MOVES LIMITED TO ROUTE PAIRS WHOSE CIRCLE SECTORS OVERLAP */
		// for (int rU = 0; rU < params->nbVehicles; rU++)
		// {
		// 	routeU = &routes[orderRoutes[rU]];
		// 	int lastTestSWAPStarRouteU = routeU->whenLastTestedSWAPStar;
		// 	routeU->whenLastTestedSWAPStar = nbMoves;
		// 	for (int rV = 0; rV < params->nbVehicles; rV++)
		// 	{
		// 		routeV = &routes[orderRoutes[rV]];
		// 		if (routeU->nbCustomers > 0 && routeV->nbCustomers > 0 && routeU->cour < routeV->cour && (loopID == 0 || std::max<int>(routeU->whenLastModified, routeV->whenLastModified) > lastTestSWAPStarRouteU))
		// 			if (CircleSector::overlap(routeU->sector, routeV->sector))
		// 				swapStar();
		// 	}
		// }
	}

	// Register the solution produced by the LS in the individual
	exportIndividual(indiv);
}

// set properties for node u and route u
void LocalSearch::setLocalVariablesRouteU()
{
	routeU = nodeU->route;									// set routeU
	nodeX = nodeU->next;									// set next node to u (u->next)
	nodeXNextIndex = nodeX->next->cour;						// set next index to next node of u (u->next->next.idx)
	nodeUIndex = nodeU->cour;								// set index of u (u->idx)
	nodeUPrevIndex = nodeU->prev->cour;						// set index of prev node of u (u->prev.idx)
	nodeXIndex = nodeX->cour;								// set index of next node of u (u->next.idx)
	isBoardingPoint = params->cli[nodeUIndex].getsPickedUp;	// set value determining the meaning of this stop NEW
	loadU    = params->cli[nodeUIndex].demand;				// set demand of u (u.demand)
	serviceU = params->cli[nodeUIndex].serviceDuration;		// set service duration of u (u.serviceDuration)
	loadX	 = params->cli[nodeXIndex].demand;				// set demand of next node of u (u->next.demand)
	serviceX = params->cli[nodeXIndex].serviceDuration;		// set service duration of next node of u (u->next.serviceDuration)

	// new
	Node * currentNode = nodeU;

	// TODO could be done faster (in initialization of the nodes; could be avoided at all -> no additional complexity!)
	if (isBoardingPoint) {
		while (currentNode->cour != params->cli[nodeUIndex].deliveryNum) {		// searching the drop off point of the client
			currentNode = currentNode->next;
		}
	}
	else {
		while (currentNode->cour != params->cli[nodeUIndex].pickupNum) {		// searching the client to be picked up
			currentNode = currentNode->prev;
		}
	}

	partnerNodeU = currentNode;
	partnerNodeUIndex = partnerNodeU->cour;
	loadPartnerNodeU = params->cli[partnerNodeUIndex].demand;
	servicePartnerNodeU = params->cli[partnerNodeUIndex].serviceDuration;
}

// set properties for node v and route v
void LocalSearch::setLocalVariablesRouteV()
{
	routeV = nodeV->route;
	nodeY = nodeV->next;
	nodeYNextIndex = nodeY->next->cour;
	nodeVIndex = nodeV->cour;
	nodeVPrevIndex = nodeV->prev->cour;
	nodeYIndex = nodeY->cour;
	isBoardingPoint = params->cli[nodeVIndex].getsPickedUp;	// set value determining the meaning of this stop NEW
	loadV    = params->cli[nodeVIndex].demand;
	serviceV = params->cli[nodeVIndex].serviceDuration;
	loadY	 = params->cli[nodeYIndex].demand;
	serviceY = params->cli[nodeYIndex].serviceDuration;


	// new
	if (nodeV->isDepot) return;		// there cannot be a patner node of the depot

	Node * currentNode = nodeV;

	if (isBoardingPoint) {
		while (currentNode->cour != params->cli[nodeVIndex].deliveryNum) {		// searching the drop off point of the client
			currentNode = currentNode->next;
		}
	}
	else {
		while (currentNode->cour != params->cli[nodeVIndex].pickupNum) {		// searching the client to be picked up
			currentNode = currentNode->prev;
		}
	}

	partnerNodeV = currentNode;
	partnerNodeVIndex = partnerNodeV->cour;
	loadPartnerNodeV = params->cli[partnerNodeVIndex].demand;
	servicePartnerNodeV = params->cli[partnerNodeVIndex].serviceDuration;
}

// relocate move ~ remove u from route u and place it after v in route v
bool LocalSearch::move1()
{
	// NEW
	if (nodeU->isBoardingNode && nodeU->dropOffNode->position <= nodeV->position) return false;		// delivery location of u is before or equal to v
	if (nodeU->isDropOffNode && nodeU->boardingNode->position > nodeV->position) return false;				// pickup location of u is after v

	// dist(u->prev; u->next) - dist(u->prev; u) - dist(u; u->next)
	double costSuppU = params->timeCost[nodeUPrevIndex][nodeXIndex] - params->timeCost[nodeUPrevIndex][nodeUIndex] - params->timeCost[nodeUIndex][nodeXIndex];
	// dist(v; u) + dist(u; v->next) - dist(v; v->next)
	double costSuppV = params->timeCost[nodeVIndex][nodeUIndex] + params->timeCost[nodeUIndex][nodeYIndex] - params->timeCost[nodeVIndex][nodeYIndex];

	if (routeU != routeV)
	{
		costSuppU += penaltyExcessDuration(routeU->duration + costSuppU - serviceU)
			+ penaltyExcessLoad(routeU->load - loadU)
			- routeU->penalty;

		costSuppV += penaltyExcessDuration(routeV->duration + costSuppV + serviceU)
			+ penaltyExcessLoad(routeV->load + loadU)
			- routeV->penalty;
	}

	if (costSuppU + costSuppV > -MY_EPSILON) return false;
	if (nodeUIndex == nodeYIndex) return false;

	insertNode(nodeU, nodeV);
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (routeU != routeV) updateRouteData(routeV);
	return true;
}

// 1 -> 2 -> v -> 3 -> 4			 // v -> p -> d  // v -> p -> 3 -> d // v -> p -> 3 -> 4 -> d
// 5 -> p -> 6 -> d -> 7
// relocate move ~ move u and partnerNode (order depending on pickup and delivery precedence) u from routeU after v in routeV (at most three positions after v)
bool LocalSearch::relocatePair() {
	Node *boardingNode, *dropOffNode, *bestInsertPosition;
	int boardingNodeIndex, boardingNodePrevIndex, boardingNodeNextIndex, dropOffNodeIndex, dropOffNodePrevIndex, dropOffNodeNextIndex;
	double tempCost1, tempCost2, tempCost3;

	if (nodeU->isBoardingNode) {
		boardingNode = nodeU;
		dropOffNode = partnerNodeU;
	}
	else {
		boardingNode = partnerNodeU;
		dropOffNode = nodeU;
	}

	
	boardingNodeIndex = boardingNode->cour;
	boardingNodePrevIndex = boardingNode->prev->cour;
	boardingNodeNextIndex = boardingNode->next->cour;

	dropOffNodeIndex = dropOffNode->cour;
	dropOffNodePrevIndex = dropOffNode->prev->cour;
	dropOffNodeNextIndex = dropOffNode->next->cour;

	double costSuppU, costSuppV;

	// cost calculation (https://arxiv.org/pdf/2107.05189.pdf)
	if (boardingNodeNextIndex == dropOffNodeIndex) {		// drop off node is directly after boarding node
		costSuppU = params->timeCost[boardingNodePrevIndex][dropOffNodeNextIndex] 
					- params->timeCost[boardingNodePrevIndex][boardingNodeIndex] 
					- params->timeCost[boardingNodeIndex][boardingNodeNextIndex]
					- params->timeCost[dropOffNodeIndex][dropOffNodeNextIndex];
	}
	else {
		costSuppU = params->timeCost[boardingNodePrevIndex][boardingNodeNextIndex]
					+ params->timeCost[dropOffNodePrevIndex][dropOffNodeNextIndex]
					- params->timeCost[boardingNodePrevIndex][boardingNodeIndex]
					- params->timeCost[boardingNodeIndex][boardingNodeNextIndex]
					- params->timeCost[dropOffNodePrevIndex][dropOffNodeIndex]
					- params->timeCost[dropOffNodeIndex][dropOffNodeNextIndex];
	}

	tempCost1 = params->timeCost[nodeVIndex][boardingNodeIndex]				// v -> bn -> don -> y
				+ params->timeCost[boardingNodeIndex][dropOffNodeIndex]
				+ params->timeCost[dropOffNodeIndex][nodeYIndex] 
				- params->timeCost[nodeVIndex][nodeYIndex];

	costSuppV = tempCost1;
	bestInsertPosition = boardingNode;			// node after which the drop off node should be inserted

	if (!nodeY->isDepot) {			
		tempCost2 = params->timeCost[nodeVIndex][boardingNodeIndex]				// v -> bn -> y -> don -> ynext
					+ params->timeCost[boardingNodeIndex][nodeYIndex]
					+ params->timeCost[nodeYIndex][dropOffNodeIndex]
					+ params->timeCost[dropOffNodeIndex][nodeYNextIndex]
					- params->timeCost[nodeVIndex][nodeYIndex]
					- params->timeCost[nodeYIndex][nodeYNextIndex];

		if (tempCost2 < costSuppV) {
			costSuppV = tempCost2;
			bestInsertPosition = nodeY;
		}
	}
	else {
		tempCost2 = __DBL_MAX__;
	}

	if (!nodeY->isDepot && !nodeY->next->isDepot) {
		tempCost3 = params->timeCost[nodeVIndex][boardingNodeIndex]				// v -> bn -> y -> ynext -> don -> ynextnext
					+ params->timeCost[boardingNodeIndex][nodeYIndex]
					+ params->timeCost[nodeYNextIndex][dropOffNodeIndex]
					+ params->timeCost[dropOffNodeIndex][nodeY->next->next->cour]
					- params->timeCost[nodeVIndex][nodeYIndex]
					- params->timeCost[nodeYNextIndex][nodeY->next->next->cour];

		if (tempCost3 < costSuppV) {
			costSuppV = tempCost3;
			bestInsertPosition = nodeY->next;
		}
	}
	else {
		tempCost3 = __DBL_MAX__;
	}

	
	if (routeU != routeV) {
		costSuppU += penaltyExcessLoad(routeU->load - loadU - loadPartnerNodeU) - routeU->penalty;
		costSuppV += penaltyExcessLoad(routeV->load + loadU + loadPartnerNodeU) - routeV->penalty;
	}

	if (costSuppU + costSuppV > -MY_EPSILON) return false;
	if (nodeVIndex == dropOffNodeIndex
		|| nodeVIndex == boardingNodeIndex
		|| nodeYIndex == dropOffNodeIndex
		|| nodeYIndex == boardingNodeIndex
		|| (!nodeY->isDepot && (nodeYNextIndex == dropOffNodeIndex || nodeYIndex == boardingNodeIndex))
		|| (!nodeY->isDepot && !nodeY->next->isDepot && (nodeY->next->next->cour == dropOffNodeIndex || nodeY->next->next->cour == boardingNodeIndex))) return false;

	insertNode(boardingNode, nodeV);
	insertNode(dropOffNode, bestInsertPosition);
	nbMoves++;
	searchCompleted = false;
	updateRouteData(routeU);
	if (routeU != routeV) updateRouteData(routeV);
	
	return true;
}

// relocate move ~ remove u and u->next from route u and place them after v in route v
bool LocalSearch::move2()
{ 
	// NEW
	if (nodeU->isBoardingNode 
		&& nodeU->dropOffNode->position != nodeX->position 
		&& nodeU->dropOffNode->position <= nodeV->position
		) return false;				// delivery location of u is before or equal to v
	
	if (nodeU->isDropOffNode 
		&& (nodeU->boardingNode->position > nodeV->position || nodeU->boardingNode->position == nodeX->position)
		) return false;				// pickup location of u is after v or u->next is the pickup location of u

	if (nodeX->isBoardingNode 
		&& nodeX->dropOffNode->position <= nodeV->position
		) return false;				// delivery location of u->next is before or equal to V

	if (nodeX->isDropOffNode 
		&& nodeX->boardingNode->position > nodeV->position
		) return false;				// pickup location of u->next is after v

	if (nodeX->isBoardingNode
		&& nodeX->dropOffNode->position == nodeU->position
		) return false;				// u is the delivery location of u->next

	// dist(u->prev; u->next->next) - dist(u->prev; u) - dist(u->next; u->next->next)
	double costSuppU = params->timeCost[nodeUPrevIndex][nodeXNextIndex] - params->timeCost[nodeUPrevIndex][nodeUIndex] - params->timeCost[nodeXIndex][nodeXNextIndex];

	// dist(v; u) + dist(u->next; v->next) - dist(v; v->next)
	double costSuppV = params->timeCost[nodeVIndex][nodeUIndex] + params->timeCost[nodeXIndex][nodeYIndex] - params->timeCost[nodeVIndex][nodeYIndex];

	if (routeU != routeV)
	{
		costSuppU += penaltyExcessDuration(routeU->duration + costSuppU - params->timeCost[nodeUIndex][nodeXIndex] - serviceU - serviceX)
			+ penaltyExcessLoad(routeU->load - loadU - loadX)
			- routeU->penalty;

		costSuppV += penaltyExcessDuration(routeV->duration + costSuppV + params->timeCost[nodeUIndex][nodeXIndex] + serviceU + serviceX)
			+ penaltyExcessLoad(routeV->load + loadU + loadX)
			- routeV->penalty;
	}

	if (costSuppU + costSuppV > -MY_EPSILON) return false;
	if (nodeU == nodeY || nodeV == nodeX || nodeX->isDepot) return false;

	insertNode(nodeU, nodeV);
	insertNode(nodeX, nodeU);
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (routeU != routeV) updateRouteData(routeV);
	return true;
}

// relocate move ~ remove u and u->next from route u and place them in reverse order after v in route v  v->x->u->y
bool LocalSearch::move3()
{
	if (nodeU->isBoardingNode 
		&& nodeU->dropOffNode->position <= nodeV->position
		) return false;				// delivery location of u is before or equal to v

	if (nodeU->isDropOffNode 
		&& nodeU->boardingNode->position != nodeX->position 
		&& nodeU->boardingNode->position > nodeV->position
		) return false;				// pickup location of u is after v and not u->next

	if (nodeX->isBoardingNode
		&& nodeX->dropOffNode->position != nodeU->position
		&& nodeX->dropOffNode->position <= nodeV->position
		) return false;				// delivery location of u->next is before or equal to v and not u

	if (nodeX->isDropOffNode
		&& (nodeX->boardingNode->position == nodeU->position || nodeX->boardingNode->position > nodeV->position)
		) return false;				// pickup location of u->next is either u or after v

	// dist(u->prev; u->next->next) - dist(u->prev; u) - dist(u; u->next) - dist(u->next; u->next->next)
	double costSuppU = params->timeCost[nodeUPrevIndex][nodeXNextIndex] - params->timeCost[nodeUPrevIndex][nodeUIndex] - params->timeCost[nodeUIndex][nodeXIndex] - params->timeCost[nodeXIndex][nodeXNextIndex];
	// dist(v; u->next) + dist(u->next; u) + dist(u; v->next) - dist(v; v->next)
	double costSuppV = params->timeCost[nodeVIndex][nodeXIndex] + params->timeCost[nodeXIndex][nodeUIndex] + params->timeCost[nodeUIndex][nodeYIndex] - params->timeCost[nodeVIndex][nodeYIndex];

	if (routeU != routeV)
	{
		costSuppU += penaltyExcessDuration(routeU->duration + costSuppU - serviceU - serviceX)
			+ penaltyExcessLoad(routeU->load - loadU - loadX)
			- routeU->penalty;

		costSuppV += penaltyExcessDuration(routeV->duration + costSuppV + serviceU + serviceX)
			+ penaltyExcessLoad(routeV->load + loadU + loadX)
			- routeV->penalty;
	}

	if (costSuppU + costSuppV > -MY_EPSILON) return false;
	if (nodeU == nodeY || nodeX == nodeV || nodeX->isDepot) return false;

	insertNode(nodeX, nodeV);
	insertNode(nodeU, nodeX);
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (routeU != routeV) updateRouteData(routeV);
	return true;
}

// swap move ~ swap u and v in route u and route v
bool LocalSearch::move4()
{
	if (nodeU->isBoardingNode 
		&& nodeU->dropOffNode->position <= nodeV->position
		) return false;				// delivery location of u is before or equal to v

	if (nodeU->isDropOffNode 
		&& nodeU->boardingNode->position >= nodeV->position
		) return false;				// pickup location of u is after or equal to v

	if (nodeV->isBoardingNode 
		&& nodeV->dropOffNode->position <= nodeU->position
		) return false;				// delivery location of v is before or equal to u

	if (nodeV->isDropOffNode 
		&& nodeV->boardingNode->position >= nodeU->position
		) return false;				// pickup location of v is after u

	// dist(u->prev; v) + dist(v; u->next) - dist(u->prev; u) - dist(u; u->next)
	double costSuppU = params->timeCost[nodeUPrevIndex][nodeVIndex] + params->timeCost[nodeVIndex][nodeXIndex] - params->timeCost[nodeUPrevIndex][nodeUIndex] - params->timeCost[nodeUIndex][nodeXIndex];
	// dist(v->prev; u) + dist(u; v->next) - dist(v->prev; v) - dist(v; v->next)
	double costSuppV = params->timeCost[nodeVPrevIndex][nodeUIndex] + params->timeCost[nodeUIndex][nodeYIndex] - params->timeCost[nodeVPrevIndex][nodeVIndex] - params->timeCost[nodeVIndex][nodeYIndex];

	if (routeU != routeV)
	{
		costSuppU += penaltyExcessDuration(routeU->duration + costSuppU + serviceV - serviceU)
			+ penaltyExcessLoad(routeU->load + loadV - loadU)
			- routeU->penalty;

		costSuppV += penaltyExcessDuration(routeV->duration + costSuppV - serviceV + serviceU)
			+ penaltyExcessLoad(routeV->load + loadU - loadV)
			- routeV->penalty;
	}

	if (costSuppU + costSuppV > -MY_EPSILON) return false;
	if (nodeUIndex == nodeVPrevIndex || nodeUIndex == nodeYIndex) return false;

	swapNode(nodeU, nodeV);
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (routeU != routeV) updateRouteData(routeV);
	return true;
}

// swap move ~ remove u and u->next from route u and swap it with v from route v
bool LocalSearch::move5()
{
	if (nodeU->isBoardingNode 
		&& nodeU->dropOffNode->position <= nodeV->position 
		&& nodeU->dropOffNode->position != nodeX->position
		) return false;				// delivery location of u is before or equal to v and not u->next

	if (nodeU->isDropOffNode 
		&& (nodeU->boardingNode->position >= nodeV->position || nodeU->boardingNode->position == nodeX->position)
		) return false;				// pickup location of u is after or equal to v or it is u->next

	if (nodeV->isBoardingNode 
		&& nodeV->dropOffNode->position <= nodeU->position
		) return false;				// delivery location of v is before u

	if (nodeV->isDropOffNode 
		&& nodeV->boardingNode->position >= nodeU->position
		) return false;				// pickup location of v is after or equal to u

	if (nodeX->isBoardingNode
		&& nodeX->dropOffNode->position <= nodeV->position
		) return false;				// delivery location of u->next is before or equal to v

	if (nodeX->isDropOffNode
		&& nodeX->boardingNode->position >= nodeV->position
		) return false;				// pickup location of u->next is after or equal to v

	// dist(u->prev; v) + dist(v; u->next->next) - dist(u->prev; u) - dist(u->next; u->next->next)
	double costSuppU = params->timeCost[nodeUPrevIndex][nodeVIndex] + params->timeCost[nodeVIndex][nodeXNextIndex] - params->timeCost[nodeUPrevIndex][nodeUIndex] - params->timeCost[nodeXIndex][nodeXNextIndex];
	// dist(v->prev; u) + dist(u->next; v->next) - dist(v->prev; v) - dist(v; v->next)
	double costSuppV = params->timeCost[nodeVPrevIndex][nodeUIndex] + params->timeCost[nodeXIndex][nodeYIndex] - params->timeCost[nodeVPrevIndex][nodeVIndex] - params->timeCost[nodeVIndex][nodeYIndex];

	if (routeU != routeV)
	{
		costSuppU += penaltyExcessDuration(routeU->duration + costSuppU - params->timeCost[nodeUIndex][nodeXIndex] + serviceV - serviceU - serviceX)
			+ penaltyExcessLoad(routeU->load + loadV - loadU - loadX)
			- routeU->penalty;

		costSuppV += penaltyExcessDuration(routeV->duration + costSuppV + params->timeCost[nodeUIndex][nodeXIndex] - serviceV + serviceU + serviceX)
			+ penaltyExcessLoad(routeV->load + loadU + loadX - loadV)
			- routeV->penalty;
	}

	if (costSuppU + costSuppV > -MY_EPSILON) return false;
	if (nodeU == nodeV->prev || nodeX == nodeV->prev || nodeU == nodeY || nodeX->isDepot) return false;

	swapNode(nodeU, nodeV);
	insertNode(nodeX, nodeU);
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (routeU != routeV) updateRouteData(routeV);
	return true;
}


/*
Es gibt 4 Moeglichkeiten:

Seien uB, uD, vB und vD wie folgt:
	uB := firstBoardingNode
	uD := firstDropOffNode
	vB := secondBoardingNode
	vD := secondDropOffNode

1. uB directly after uD: 1 -> 2 -> uB -> uD -> 3 -> 4		und vB directly after vD: 1 -> 2 -> vB -> vD -> 3 -> 4
2. uB not directly after uD: 1 -> uB -> 2 -> uD -> 3 -> 4	und vB not directly after vD: 1 -> vB -> 2 -> vD -> 3 -> 4
3. uB not directly after uD: 1 -> uB -> 2 -> uD -> 3 -> 4	und vB directly after vD: 1 -> 2 -> vB -> vD -> 3 -> 4
4. uB directly after uD: 1 -> 2 -> uB -> uD -> 3 -> 4		und vB not directly after vD: 1 -> vB -> 2 -> vD -> 3 -> 4

*/

bool LocalSearch::swapPair() {
	Node *firstBoardingNode, *firstDropOffNode, *secondBoardingNode, *secondDropOffNode;
	int fbnIndex, fbnPrevIndex, fbnNextIndex, fdonIndex, fdonPrevIndex, fdonNextIndex, sbnIndex, sbnPrevIndex, sbnNextIndex, sdonIndex, sdonPrevIndex, sdonNextIndex;

	if (nodeU->isBoardingNode) {
		firstBoardingNode = nodeU;
		firstDropOffNode = partnerNodeU;
	}
	else {
		firstBoardingNode = partnerNodeU;
		firstDropOffNode = nodeU;
	}

	if (nodeV->isBoardingNode) {
		secondBoardingNode = nodeV;
		secondDropOffNode = partnerNodeV;
	}
	else {
		secondBoardingNode = partnerNodeV;
		secondDropOffNode = nodeV;
	}

	
	fbnIndex = firstBoardingNode->cour;
	fbnPrevIndex = firstBoardingNode->prev->cour;
	fbnNextIndex = firstBoardingNode->next->cour;

	fdonIndex = firstDropOffNode->cour;
	fdonPrevIndex = firstDropOffNode->prev->cour;
	fdonNextIndex = firstDropOffNode->next->cour;

	sbnIndex = secondBoardingNode->cour;
	sbnPrevIndex = secondBoardingNode->prev->cour;
	sbnNextIndex = secondBoardingNode->next->cour;

	sdonIndex = secondDropOffNode->cour;
	sdonPrevIndex = secondDropOffNode->prev->cour;
	sdonNextIndex = secondDropOffNode->next->cour;

	double costSuppU, costSuppV;

	if (fbnNextIndex != fdonIndex) {					// drop off point is not directly after the boarding point
		costSuppU = params->timeCost[fbnPrevIndex][sbnIndex]
					+ params->timeCost[sbnIndex][fbnNextIndex]
					+ params->timeCost[fdonPrevIndex][sdonIndex]
					+ params->timeCost[sdonIndex][fdonNextIndex]
					- params->timeCost[fbnPrevIndex][fbnIndex]
					- params->timeCost[fbnIndex][fbnNextIndex]
					- params->timeCost[fdonPrevIndex][fdonIndex]
					- params->timeCost[fdonIndex][fdonNextIndex];
	}
	else {																// drop off point is directly after the boarding point
		costSuppU = params->timeCost[fbnPrevIndex][sbnIndex]
					+ params->timeCost[sbnIndex][sdonIndex]
					+ params->timeCost[sdonIndex][fdonNextIndex]
					- params->timeCost[fbnPrevIndex][fbnIndex]
					- params->timeCost[fbnIndex][fbnNextIndex]
					- params->timeCost[fdonIndex][fdonNextIndex];
	}

	if (sbnNextIndex != sdonIndex) {
		costSuppV = params->timeCost[sbnPrevIndex][fbnIndex] 
					+ params->timeCost[fbnIndex][sbnNextIndex]
					+ params->timeCost[sdonPrevIndex][fdonIndex]
					+ params->timeCost[fdonIndex][sdonNextIndex]
					- params->timeCost[sbnPrevIndex][sbnIndex]
					- params->timeCost[sbnIndex][sbnNextIndex]
					- params->timeCost[sdonPrevIndex][sdonIndex]
					- params->timeCost[sdonIndex][sdonNextIndex];
	}
	else {
		costSuppV = params->timeCost[sbnPrevIndex][fbnIndex]
					+ params->timeCost[fbnIndex][fdonIndex]
					+ params->timeCost[fdonIndex][sdonNextIndex]
					- params->timeCost[sbnPrevIndex][sbnIndex]
					- params->timeCost[sbnIndex][sbnNextIndex]
					- params->timeCost[sdonIndex][sdonNextIndex];
	}

	if (routeU != routeV) {
		costSuppU += penaltyExcessLoad(routeU->load + loadV + loadPartnerNodeV - loadU - loadPartnerNodeU)
						- routeU->penalty;

		costSuppV += penaltyExcessLoad(routeV->load + loadU + loadPartnerNodeU - loadV - loadPartnerNodeV)
						- routeV->penalty;
	}

	if (costSuppU + costSuppV > -MY_EPSILON) return false;
	if (nodeU->isDepot || partnerNodeU->isDepot || nodeV->isDepot || partnerNodeV->isDepot || fbnNextIndex == sbnIndex || fbnNextIndex == sdonIndex
		|| fdonNextIndex == sbnIndex || fdonNextIndex == sdonIndex || sbnNextIndex == fbnIndex || sbnNextIndex == fdonIndex || sdonNextIndex == fbnIndex
		|| sdonNextIndex == fdonIndex) return false;

	swapNode(firstBoardingNode, secondBoardingNode);
	swapNode(firstDropOffNode, secondDropOffNode);
	nbMoves++;		// Incrementing move counter before updating route data
	searchCompleted = false;
	
	updateRouteData(routeU);

	if (routeU != routeV) updateRouteData(routeV);

	return true;
}

// swap move ~ swap u and u->next from route u with v and v->next from route v
bool LocalSearch::move6()
{
	if (nodeU->isBoardingNode 
		&& nodeU->dropOffNode->position <= nodeY->position 
		&& nodeU->dropOffNode->position != nodeX->position
		) return false;					// delivery location of u is before or equal to v->next and not nodeX

	if (nodeU->isDropOffNode 
		&& (nodeU->boardingNode->position >= nodeV->position || nodeU->boardingNode->position == nodeX->position)
		) return false;					// pickup location of u is after or equal to v or nodeX

	if (nodeX->isBoardingNode
		&& nodeX->dropOffNode->position <= nodeY->position
		) return false;					// delivery location of u->next is before or equal to v->next

	if (nodeX->isDropOffNode
		&& nodeX->boardingNode->position >= nodeV->position
		) return false;					// pickup location of u->next is after or equal to v

	if (nodeV->isBoardingNode 
		&& nodeV->dropOffNode->position <= nodeX->position 
		&& nodeV->dropOffNode->position != nodeY->position
		) return false;					// delivery location of v is before or equal to u->next and not nodeY

	if (nodeV->isDropOffNode 
		&& (nodeV->boardingNode->position >= nodeU->position || nodeV->boardingNode->position == nodeY->position)
		) return false;					// pickup location of v is after or equal to u or is nodeY

	if (nodeY->isBoardingNode
		&& nodeY->dropOffNode->position <= nodeX->position
		) return false;					// delivery location of v->next is before or equal to u->next

	if (nodeY->isDropOffNode
		&& nodeY->boardingNode->position >= nodeU->position
		) return false;					// pickup location of v->next is after or equal to u

	// dist(u->prev; v) + dist(v->next; u->next->next) - dist(u->prev; u) - dist(u->next; u->next->next)
	double costSuppU = params->timeCost[nodeUPrevIndex][nodeVIndex] + params->timeCost[nodeYIndex][nodeXNextIndex] - params->timeCost[nodeUPrevIndex][nodeUIndex] - params->timeCost[nodeXIndex][nodeXNextIndex];
	// dist(v->prev; u) + dist(u->next; v->next->next) - dist(v->prev; v) - dist(v->next; v->next->next)
	double costSuppV = params->timeCost[nodeVPrevIndex][nodeUIndex] + params->timeCost[nodeXIndex][nodeYNextIndex] - params->timeCost[nodeVPrevIndex][nodeVIndex] - params->timeCost[nodeYIndex][nodeYNextIndex];

	if (routeU != routeV)
	{
		costSuppU += penaltyExcessDuration(routeU->duration + costSuppU - params->timeCost[nodeUIndex][nodeXIndex] + params->timeCost[nodeVIndex][nodeYIndex] + serviceV + serviceY - serviceU - serviceX)
			+ penaltyExcessLoad(routeU->load + loadV + loadY - loadU - loadX)
			- routeU->penalty;

		costSuppV += penaltyExcessDuration(routeV->duration + costSuppV + params->timeCost[nodeUIndex][nodeXIndex] - params->timeCost[nodeVIndex][nodeYIndex] - serviceV - serviceY + serviceU + serviceX)
			+ penaltyExcessLoad(routeV->load + loadU + loadX - loadV - loadY)
			- routeV->penalty;
	}

	if (costSuppU + costSuppV > -MY_EPSILON) return false;
	if (nodeX->isDepot || nodeY->isDepot || nodeY == nodeU->prev || nodeU == nodeY || nodeX == nodeV || nodeV == nodeX->next) return false;

	swapNode(nodeU, nodeV);
	swapNode(nodeX, nodeY);
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (routeU != routeV) updateRouteData(routeV);
	return true;
}


bool LocalSearch::opt2k() {

	if (nodeU->position > nodeV->position) return false;		// if u is after v
	if (nodeU->next == nodeV) return false;

	// temp array for the nodes between u (exclusive) and v (inclusive)
	// example: 1 - 2 - u - 3 - 4 - 5 - 6 - v - 7		-> tempTourIndex.size() = 5
	int size = nodeV->position - nodeU->position;
	std::vector<int> tempTourIndex (size, -1);
	std::vector<int> originalIndex (size, -1);
	Node *begin, *end;
	double cost;

	begin = nodeX;
	end = nodeV;

	uint i = 0;
	int j = tempTourIndex.size() -1;

	while (begin->position < end->position) {

		// get the first node from the beginning that is ok for the move
		while (!isNodeOkFor2kOpt(begin, begin->position, end->position) && begin != end) {
			tempTourIndex[i] = begin->cour;
			originalIndex[i++] = begin->cour;
			begin = begin->next;
		}

		// get the first node from the end (in reverse order) that is ok for the move
		while (!isNodeOkFor2kOpt(end, begin->position, end->position) && begin != end) {
			tempTourIndex[j] = end->cour;
			originalIndex[j--] = end->cour;
			end = end->prev;
		}

		if (begin->position > end->position) break;
		if (begin->position == end->position) {
			tempTourIndex[i] = begin->cour;		// set the missing entry
			originalIndex[i++] = begin->cour;
			break;
		}

		// swap the begin and end node
		tempTourIndex[i] = end->cour;
		tempTourIndex[j] = begin->cour;
		originalIndex[i++] = begin->cour;
		originalIndex[j--] = end->cour;

		begin = begin->next;
		end = end->prev;

		if (begin->position == end->position) {
			tempTourIndex[i] = begin->cour;		// set the missing entry
			originalIndex[i++] = begin->cour;
			break;
		}
	}

	// calculate cost and deside if it is worth it or not
	i = 0;

	cost = params->timeCost[nodeUIndex][tempTourIndex[i]]
			- params->timeCost[nodeUIndex][nodeXIndex];

	while (i < tempTourIndex.size()-1) {
		cost += params->timeCost[tempTourIndex[i]][tempTourIndex[i+1]]
				- params->timeCost[originalIndex[i]][originalIndex[i+1]];

		i++;
	}	

	cost += params->timeCost[tempTourIndex[i]][nodeYIndex]
			- params->timeCost[originalIndex[i]][nodeYIndex];

	if (cost > -MY_EPSILON) return false;

	// do the actual relocating and swapping
	begin = nodeX;
	end = nodeV;

	while (begin->position < end->position) {

		while (!isNodeOkFor2kOpt(begin, begin->position, end->position)) {
			begin = begin->next;
		}

		while (!isNodeOkFor2kOpt(end, begin->position, end->position)) {
			end = end->prev;
		}

		if (begin->position >= end->position) break;

		begin = begin->next;
		end = end->prev;

		if (!(begin == end->next)) {
			swapNode(begin->prev, end->next);
		}
		else {
			insertNode(begin->prev, end->next);
		}
	}

	nbMoves++;
	searchCompleted = false;
	updateRouteData(routeU);

	return true;
}


/**
 * this method checks if a node can take part at a 2-opt move
 * 
 * node is ok <=>
 * 		<=> node is boarding node and the drop off node is behind the current end node
 * 		<=> node is drop off node and the boarding node is before the current begin node
 * 
 * node is not ok <=>
 * 		<=> node is boarding node and the drop off node is before or equal to the current end node
 * 		<=> node is drop off node and the boarding node is behind or equal to the current begin node
 */
bool LocalSearch::isNodeOkFor2kOpt(Node *node, int beginPos, int endPos) {

	if (node->isBoardingNode && node->dropOffNode->position > endPos) return true;
	if (node->isDropOffNode && node->boardingNode->position < beginPos) return true;

	return false;
}


// 2-OPT move (route u = route v) ~ 1->2->3->4->5->6->7->8->9 (u=3; v=7) => 1->2->3->7->6->5->4->8->9
bool LocalSearch::move7()
{
	if (nodeU->position > nodeV->position) return false;		// if u is after v

	if (nodeV->isBoardingNode && nodeV->dropOffNode->position <= nodeU->position) return false;
	if (nodeV->isDropOffNode && (nodeV->boardingNode->position > nodeV->position || (nodeV->boardingNode->position < nodeV->position && nodeV->boardingNode->position > nodeU->position))) return false;

	// u is before v
	// dist(u; v) + dist(u->next; v->next) - dist(u; u->next) - dist(v; v->next) + v.cumRevDist - u->next.cumRevDist
	double cost = params->timeCost[nodeUIndex][nodeVIndex] 
							+ params->timeCost[nodeXIndex][nodeYIndex] 
							- params->timeCost[nodeUIndex][nodeXIndex] 
							- params->timeCost[nodeVIndex][nodeYIndex] 
							+ nodeV->cumulatedReversalDistance 
							- nodeX->cumulatedReversalDistance;

	if (cost > -MY_EPSILON) return false;
	if (nodeU->next == nodeV) return false;

	Node * nodeNum = nodeX->next;
	nodeX->prev = nodeNum;
	nodeX->next = nodeY;

	while (nodeNum != nodeV)
	{
		Node * temp = nodeNum->next;
		nodeNum->next = nodeNum->prev;
		nodeNum->prev = temp;
		nodeNum = temp;
	}

	nodeV->next = nodeV->prev;
	nodeV->prev = nodeU;
	nodeU->next = nodeV;
	nodeY->prev = nodeX;

	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	return true;
}

// NEW
void LocalSearch::printRoute() {

	Node *depotU = routeU->depot;
	Node *depotV = routeV->depot;
	Node *current = depotU->next;

	std::cout << ">>>> NodeU = " << nodeU->cour <<  " ~ NodeV = " << nodeVIndex  << std::endl;
	std::cout << "Route U:" << std::endl;
	std::cout << depotU->cour << "->";

	while (!current->isDepot) {
		std::cout << current->cour << "->";
		current = current->next;
	}

	std::cout << current->cour << std::endl;

	std::cout << "Route V:" << std::endl;
	std::cout << depotV->cour << "->";
	current = depotV->next;

	while (!current->isDepot) {
		std::cout << current->cour << "->";
		current = current->next;
	}

	std::cout << current->cour << std::endl;

	std::cout << "=====================================================================" << std::endl;
}


// 2-OPT* move
bool LocalSearch::move8()
{

	std::cout << "2-OPT* Start" << std::endl;
	printRoute();
	
	double cost = params->timeCost[nodeUIndex][nodeVIndex] + params->timeCost[nodeXIndex][nodeYIndex] - params->timeCost[nodeUIndex][nodeXIndex] - params->timeCost[nodeVIndex][nodeYIndex]
		+ penaltyExcessDuration(nodeU->cumulatedTime + nodeV->cumulatedTime + nodeV->cumulatedReversalDistance + params->timeCost[nodeUIndex][nodeVIndex])
		+ penaltyExcessDuration(routeU->duration - nodeU->cumulatedTime - params->timeCost[nodeUIndex][nodeXIndex] + routeU->reversalDistance - nodeX->cumulatedReversalDistance + routeV->duration - nodeV->cumulatedTime - params->timeCost[nodeVIndex][nodeYIndex] + params->timeCost[nodeXIndex][nodeYIndex])
		+ penaltyExcessLoad(nodeU->cumulatedLoad + nodeV->cumulatedLoad)
		+ penaltyExcessLoad(routeU->load + routeV->load - nodeU->cumulatedLoad - nodeV->cumulatedLoad)
		- routeU->penalty - routeV->penalty
		+ nodeV->cumulatedReversalDistance + routeU->reversalDistance - nodeX->cumulatedReversalDistance;

	if (cost > -MY_EPSILON) return false;

	Node * depotU = routeU->depot;
	Node * depotV = routeV->depot;
	Node * depotUFin = routeU->depot->prev;
	Node * depotVFin = routeV->depot->prev;
	Node * depotVSuiv = depotV->next;

	Node * temp;
	Node * xx = nodeX;
	Node * vv = nodeV;

	while (!xx->isDepot)
	{
		temp = xx->next;
		xx->next = xx->prev;
		xx->prev = temp;
		xx->route = routeV;
		xx = temp;
	}

	while (!vv->isDepot)
	{
		temp = vv->prev;
		vv->prev = vv->next;
		vv->next = temp;
		vv->route = routeU;
		vv = temp;
	}

	nodeU->next = nodeV;
	nodeV->prev = nodeU;
	nodeX->next = nodeY;
	nodeY->prev = nodeX;

	if (nodeX->isDepot)
	{
		depotUFin->next = depotU;
		depotUFin->prev = depotVSuiv;
		depotUFin->prev->next = depotUFin;
		depotV->next = nodeY;
		nodeY->prev = depotV;
	}
	else if (nodeV->isDepot)
	{
		depotV->next = depotUFin->prev;
		depotV->next->prev = depotV;
		depotV->prev = depotVFin;
		depotUFin->prev = nodeU;
		nodeU->next = depotUFin;
	}
	else
	{
		depotV->next = depotUFin->prev;
		depotV->next->prev = depotV;
		depotUFin->prev = depotVSuiv;
		depotUFin->prev->next = depotUFin;
	}

	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	updateRouteData(routeV);
	std::cout << "2-OPT* End" << std::endl;
	printRoute();
	return true;
}

// 2-OPT* move
bool LocalSearch::move9()
{
	
	double cost = params->timeCost[nodeUIndex][nodeYIndex] + params->timeCost[nodeVIndex][nodeXIndex] - params->timeCost[nodeUIndex][nodeXIndex] - params->timeCost[nodeVIndex][nodeYIndex]
		+ penaltyExcessDuration(nodeU->cumulatedTime + routeV->duration - nodeV->cumulatedTime - params->timeCost[nodeVIndex][nodeYIndex] + params->timeCost[nodeUIndex][nodeYIndex])
		+ penaltyExcessDuration(routeU->duration - nodeU->cumulatedTime - params->timeCost[nodeUIndex][nodeXIndex] + nodeV->cumulatedTime + params->timeCost[nodeVIndex][nodeXIndex])
		+ penaltyExcessLoad(nodeU->cumulatedLoad + routeV->load - nodeV->cumulatedLoad)
		+ penaltyExcessLoad(nodeV->cumulatedLoad + routeU->load - nodeU->cumulatedLoad)
		- routeU->penalty - routeV->penalty;

	if (cost > -MY_EPSILON) return false;

	Node * depotU = routeU->depot;
	Node * depotV = routeV->depot;
	Node * depotUFin = depotU->prev;
	Node * depotVFin = depotV->prev;
	Node * depotUpred = depotUFin->prev;

	Node * count = nodeY;
	while (!count->isDepot)
	{
		count->route = routeU;
		count = count->next;
	}

	count = nodeX;
	while (!count->isDepot)
	{
		count->route = routeV;
		count = count->next;
	}

	nodeU->next = nodeY;
	nodeY->prev = nodeU;
	nodeV->next = nodeX;
	nodeX->prev = nodeV;

	if (nodeX->isDepot)
	{
		depotUFin->prev = depotVFin->prev;
		depotUFin->prev->next = depotUFin;
		nodeV->next = depotVFin;
		depotVFin->prev = nodeV;
	}
	else
	{
		depotUFin->prev = depotVFin->prev;
		depotUFin->prev->next = depotUFin;
		depotVFin->prev = depotUpred;
		depotVFin->prev->next = depotVFin;
	}

	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	updateRouteData(routeV);
	return true;
}

// swap* move
bool LocalSearch::swapStar()
{
	SwapStarElement myBestSwapStar;

	// Preprocessing insertion costs
	preprocessInsertions(routeU, routeV);
	preprocessInsertions(routeV, routeU);

	// Evaluating the moves
	for (nodeU = routeU->depot->next; !nodeU->isDepot; nodeU = nodeU->next)
	{
		for (nodeV = routeV->depot->next; !nodeV->isDepot; nodeV = nodeV->next)
		{
			double deltaPenRouteU = penaltyExcessLoad(routeU->load + params->cli[nodeV->cour].demand - params->cli[nodeU->cour].demand) - routeU->penalty;
			double deltaPenRouteV = penaltyExcessLoad(routeV->load + params->cli[nodeU->cour].demand - params->cli[nodeV->cour].demand) - routeV->penalty;

			// Quick filter: possibly early elimination of many SWAP* due to the capacity constraints/penalties and bounds on insertion costs
			if (deltaPenRouteU + nodeU->deltaRemoval + deltaPenRouteV + nodeV->deltaRemoval <= 0)
			{
				SwapStarElement mySwapStar;
				mySwapStar.U = nodeU;
				mySwapStar.V = nodeV;

				// Evaluate best reinsertion cost of U in the route of V where V has been removed
				double extraV = getCheapestInsertSimultRemoval(nodeU, nodeV, mySwapStar.bestPositionU);

				// Evaluate best reinsertion cost of V in the route of U where U has been removed
				double extraU = getCheapestInsertSimultRemoval(nodeV, nodeU, mySwapStar.bestPositionV);

				// Evaluating final cost
				mySwapStar.moveCost = deltaPenRouteU + nodeU->deltaRemoval + extraU + deltaPenRouteV + nodeV->deltaRemoval + extraV
					+ penaltyExcessDuration(routeU->duration + nodeU->deltaRemoval + extraU + params->cli[nodeV->cour].serviceDuration - params->cli[nodeU->cour].serviceDuration)
					+ penaltyExcessDuration(routeV->duration + nodeV->deltaRemoval + extraV - params->cli[nodeV->cour].serviceDuration + params->cli[nodeU->cour].serviceDuration);

				if (mySwapStar.moveCost < myBestSwapStar.moveCost)
					myBestSwapStar = mySwapStar;
			}
		}
	}

	// Including RELOCATE from nodeU towards routeV (costs nothing to include in the evaluation at this step since we already have the best insertion location)
	// Moreover, since the granularity criterion is different, this can lead to different improving moves
	for (nodeU = routeU->depot->next; !nodeU->isDepot; nodeU = nodeU->next)
	{
		SwapStarElement mySwapStar;
		mySwapStar.U = nodeU;
		mySwapStar.bestPositionU = bestInsertClient[routeV->cour][nodeU->cour].bestLocation[0];
		double deltaDistRouteU = params->timeCost[nodeU->prev->cour][nodeU->next->cour] - params->timeCost[nodeU->prev->cour][nodeU->cour] - params->timeCost[nodeU->cour][nodeU->next->cour];
		double deltaDistRouteV = bestInsertClient[routeV->cour][nodeU->cour].bestCost[0];
		mySwapStar.moveCost = deltaDistRouteU + deltaDistRouteV
			+ penaltyExcessLoad(routeU->load - params->cli[nodeU->cour].demand) - routeU->penalty
			+ penaltyExcessLoad(routeV->load + params->cli[nodeU->cour].demand) - routeV->penalty
			+ penaltyExcessDuration(routeU->duration + deltaDistRouteU - params->cli[nodeU->cour].serviceDuration)
			+ penaltyExcessDuration(routeV->duration + deltaDistRouteV + params->cli[nodeU->cour].serviceDuration);

		if (mySwapStar.moveCost < myBestSwapStar.moveCost)
			myBestSwapStar = mySwapStar;
	}

	// Including RELOCATE from nodeV towards routeU
	for (nodeV = routeV->depot->next; !nodeV->isDepot; nodeV = nodeV->next)
	{
		SwapStarElement mySwapStar;
		mySwapStar.V = nodeV;
		mySwapStar.bestPositionV = bestInsertClient[routeU->cour][nodeV->cour].bestLocation[0];
		double deltaDistRouteU = bestInsertClient[routeU->cour][nodeV->cour].bestCost[0];
		double deltaDistRouteV = params->timeCost[nodeV->prev->cour][nodeV->next->cour] - params->timeCost[nodeV->prev->cour][nodeV->cour] - params->timeCost[nodeV->cour][nodeV->next->cour];
		mySwapStar.moveCost = deltaDistRouteU + deltaDistRouteV
			+ penaltyExcessLoad(routeU->load + params->cli[nodeV->cour].demand) - routeU->penalty
			+ penaltyExcessLoad(routeV->load - params->cli[nodeV->cour].demand) - routeV->penalty
			+ penaltyExcessDuration(routeU->duration + deltaDistRouteU + params->cli[nodeV->cour].serviceDuration)
			+ penaltyExcessDuration(routeV->duration + deltaDistRouteV - params->cli[nodeV->cour].serviceDuration);

		if (mySwapStar.moveCost < myBestSwapStar.moveCost)
			myBestSwapStar = mySwapStar;
	}

	if (myBestSwapStar.moveCost > -MY_EPSILON) return false;

	// Applying the best move in case of improvement
	if (myBestSwapStar.bestPositionU != NULL) insertNode(myBestSwapStar.U, myBestSwapStar.bestPositionU);
	if (myBestSwapStar.bestPositionV != NULL) insertNode(myBestSwapStar.V, myBestSwapStar.bestPositionV);
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	updateRouteData(routeV);
	return true;
}

double LocalSearch::getCheapestInsertSimultRemoval(Node * U, Node * V, Node *& bestPosition)
{
	ThreeBestInsert * myBestInsert = &bestInsertClient[V->route->cour][U->cour];
	bool found = false;

	// Find best insertion in the route such that V is not next or pred (can only belong to the top three locations)
	bestPosition = myBestInsert->bestLocation[0];
	double bestCost = myBestInsert->bestCost[0];
	found = (bestPosition != V && bestPosition->next != V);
	if (!found && myBestInsert->bestLocation[1] != NULL)
	{
		bestPosition = myBestInsert->bestLocation[1];
		bestCost = myBestInsert->bestCost[1];
		found = (bestPosition != V && bestPosition->next != V);
		if (!found && myBestInsert->bestLocation[2] != NULL)
		{
			bestPosition = myBestInsert->bestLocation[2];
			bestCost = myBestInsert->bestCost[2];
			found = true;
		}
	}

	// Compute insertion in the place of V
	double deltaCost = params->timeCost[V->prev->cour][U->cour] + params->timeCost[U->cour][V->next->cour] - params->timeCost[V->prev->cour][V->next->cour];
	if (!found || deltaCost < bestCost)
	{
		bestPosition = V->prev;
		bestCost = deltaCost;
	}

	return bestCost;
}

void LocalSearch::preprocessInsertions(Route * R1, Route * R2)
{
	for (Node * U = R1->depot->next; !U->isDepot; U = U->next)
	{
		// Performs the preprocessing
		U->deltaRemoval = params->timeCost[U->prev->cour][U->next->cour] - params->timeCost[U->prev->cour][U->cour] - params->timeCost[U->cour][U->next->cour];
		if (R2->whenLastModified > bestInsertClient[R2->cour][U->cour].whenLastCalculated)
		{
			bestInsertClient[R2->cour][U->cour].reset();
			bestInsertClient[R2->cour][U->cour].whenLastCalculated = nbMoves;
			bestInsertClient[R2->cour][U->cour].bestCost[0] = params->timeCost[0][U->cour] + params->timeCost[U->cour][R2->depot->next->cour] - params->timeCost[0][R2->depot->next->cour];
			bestInsertClient[R2->cour][U->cour].bestLocation[0] = R2->depot;
			for (Node * V = R2->depot->next; !V->isDepot; V = V->next)
			{
				double deltaCost = params->timeCost[V->cour][U->cour] + params->timeCost[U->cour][V->next->cour] - params->timeCost[V->cour][V->next->cour];
				bestInsertClient[R2->cour][U->cour].compareAndAdd(deltaCost, V);
			}
		}
	}
}

void LocalSearch::insertNode(Node * U, Node * V)
{
	U->prev->next = U->next;
	U->next->prev = U->prev;
	V->next->prev = U;
	U->prev = V;
	U->next = V->next;
	V->next = U;
	U->route = V->route;
}

void LocalSearch::swapNode(Node * U, Node * V)
{
	Node * myVPred = V->prev;
	Node * myVSuiv = V->next;
	Node * myUPred = U->prev;
	Node * myUSuiv = U->next;
	Route * myRouteU = U->route;
	Route * myRouteV = V->route;

	myUPred->next = V;
	myUSuiv->prev = V;
	myVPred->next = U;
	myVSuiv->prev = U;

	U->prev = myVPred;
	U->next = myVSuiv;
	V->prev = myUPred;
	V->next = myUSuiv;

	U->route = myRouteV;
	V->route = myRouteU;
}

// just going through and assigning position, cumulated values, etc.
void LocalSearch::updateRouteData(Route * myRoute)
{
	int myplace = 0;
	double myload = 0.;
	double mytime = 0.;
	double myReversalDistance = 0.;
	double cumulatedX = 0.;
	double cumulatedY = 0.;

	Node * mynode = myRoute->depot;
	mynode->position = 0;
	mynode->cumulatedLoad = 0.;
	mynode->cumulatedTime = 0.;
	mynode->cumulatedReversalDistance = 0.;

	bool firstIt = true;
	
	while (!mynode->isDepot || firstIt)
	{
		mynode = mynode->next;
		myplace++;
		mynode->position = myplace;
		myload += params->cli[mynode->cour].demand;
		mytime += params->timeCost[mynode->prev->cour][mynode->cour] + params->cli[mynode->cour].serviceDuration;
		myReversalDistance += params->timeCost[mynode->cour][mynode->prev->cour] - params->timeCost[mynode->prev->cour][mynode->cour] ;
		mynode->cumulatedLoad = myload;
		mynode->cumulatedTime = mytime;
		mynode->cumulatedReversalDistance = myReversalDistance;
		
		if (!mynode->isDepot)
		{
			cumulatedX += params->cli[mynode->cour].coordX;
			cumulatedY += params->cli[mynode->cour].coordY;
			if (firstIt) myRoute->sector.initialize(params->cli[mynode->cour].polarAngle);
			else myRoute->sector.extend(params->cli[mynode->cour].polarAngle);
		}
		
		firstIt = false;
	}

	myRoute->duration = mytime;
	myRoute->load = myload;
	myRoute->penalty = penaltyExcessDuration(mytime) + penaltyExcessLoad(myload);
	myRoute->nbCustomers = myplace-1;
	myRoute->reversalDistance = myReversalDistance;
	// Remember "when" this route has been last modified (will be used to filter unnecessary move evaluations)
	myRoute->whenLastModified = nbMoves ;

	if (myRoute->nbCustomers == 0)
	{
		myRoute->polarAngleBarycenter = 1.e30;
		emptyRoutes.insert(myRoute->cour);
	}
	else
	{
		myRoute->polarAngleBarycenter = atan2(cumulatedY/(double)myRoute->nbCustomers - params->cli[0].coordY, cumulatedX/(double)myRoute->nbCustomers - params->cli[0].coordX);
		emptyRoutes.erase(myRoute->cour);
	}
}

// NEW
void LocalSearch::setDeliveryAndPickupNodes(Route * route) {

	std::vector<Node*> routeVector(params->nbClients + 1);

	Node *currentStop = route->depot->next;

	while (!currentStop->isDepot && currentStop != NULL) {
		routeVector[currentStop->cour] = currentStop;
		currentStop = currentStop->next;

	}

	currentStop = route->depot->next;

	while (!currentStop->isDepot) {
		if (currentStop->isBoardingNode) currentStop->dropOffNode = routeVector[currentStop->dropOffIdx];
		if (currentStop->isDropOffNode) currentStop->boardingNode = routeVector[currentStop->boardingIdx];

		currentStop = currentStop->next;
	}
}

void LocalSearch::loadIndividual(Individual * indiv)
{
	emptyRoutes.clear();
	nbMoves = 0; 

	for (int r = 0; r < params->nbVehicles; r++)
	{
		Node * myDepot = &depots[r];
		Node * myDepotFin = &depotsEnd[r];
		Route * myRoute = &routes[r];
		myDepot->prev = myDepotFin;
		myDepotFin->next = myDepot;

		if (!indiv->chromR[r].empty())
		{
			Node * myClient = &clients[indiv->chromR[r][0]];
			myClient->dropOffIdx = params->cli[indiv->chromR[r][0]].deliveryNum;				// NEW
			if (myClient->dropOffIdx != -1) myClient->isBoardingNode = true;					// NEW
			myClient->boardingIdx = params->cli[indiv->chromR[r][0]].pickupNum;					// NEW
			if (myClient->boardingIdx != -1) myClient->isDropOffNode = true;						// NEW
			myClient->route = myRoute;
			myClient->prev = myDepot;
			myDepot->next = myClient;

			for (int i = 1; i < (int)indiv->chromR[r].size(); i++)
			{
				Node * myClientPred = myClient;
				myClient = &clients[indiv->chromR[r][i]];
				myClient->dropOffIdx = params->cli[indiv->chromR[r][i]].deliveryNum;				// NEW
				if (myClient->dropOffIdx != -1) myClient->isBoardingNode = true;					// NEW
				myClient->boardingIdx = params->cli[indiv->chromR[r][i]].pickupNum;					// NEW
				if (myClient->boardingIdx != -1) myClient->isDropOffNode = true;						// NEW
				myClient->prev = myClientPred;
				myClientPred->next = myClient;
				myClient->route = myRoute;
			}
			myClient->next = myDepotFin;
			myDepotFin->prev = myClient;
		}
		else
		{
			myDepot->next = myDepotFin;
			myDepotFin->prev = myDepot;
		}

		updateRouteData(&routes[r]);
		setDeliveryAndPickupNodes(&routes[r]);

		routes[r].whenLastTestedSWAPStar = -1;

		for (int i = 1; i <= params->nbClients; i++) // Initializing memory structures
			bestInsertClient[r][i].whenLastCalculated = -1;
	}

	for (int i = 1; i <= params->nbClients; i++) // Initializing memory structures
		clients[i].whenLastTestedRI = -1;
}

void LocalSearch::exportIndividual(Individual * indiv)
{
	std::vector < std::pair <double, int> > routePolarAngles ;
	for (int r = 0; r < params->nbVehicles; r++)
		routePolarAngles.push_back(std::pair <double, int>(routes[r].polarAngleBarycenter, r));
	std::sort(routePolarAngles.begin(), routePolarAngles.end()); // empty routes have a polar angle of 1.e30, and therefore will always appear at the end

	int pos = 0;
	for (int r = 0; r < params->nbVehicles; r++)
	{
		indiv->chromR[r].clear();
		Node * node = depots[routePolarAngles[r].second].next;
		while (!node->isDepot)
		{
			indiv->chromT[pos] = node->cour;
			indiv->chromR[r].push_back(node->cour);
			node = node->next;
			pos++;
		}
	}

	indiv->evaluateCompleteCost();
}

LocalSearch::LocalSearch(Params * params) : params (params)
{
	clients = std::vector < Node >(params->nbClients + 1);
	routes = std::vector < Route >(params->nbVehicles);
	depots = std::vector < Node >(params->nbVehicles);
	depotsEnd = std::vector < Node >(params->nbVehicles);
	bestInsertClient = std::vector < std::vector <ThreeBestInsert> >(params->nbVehicles, std::vector <ThreeBestInsert>(params->nbClients + 1));

	for (int i = 0; i <= params->nbClients; i++) 
	{ 
		clients[i].cour = i; 
		clients[i].isDepot = false; 
	}
	for (int i = 0; i < params->nbVehicles; i++)
	{
		routes[i].cour = i;
		routes[i].depot = &depots[i];
		depots[i].cour = 0;
		depots[i].isDepot = true;
		depots[i].route = &routes[i];
		depotsEnd[i].cour = 0;
		depotsEnd[i].isDepot = true;
		depotsEnd[i].route = &routes[i];
	}
	for (int i = 1 ; i <= params->nbClients ; i++) orderNodes.push_back(i);
	for (int r = 0 ; r < params->nbVehicles ; r++) orderRoutes.push_back(r);
}

