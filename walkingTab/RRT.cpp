/*
 * Copyright (c) 2010, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** 
 * @file RRT.cpp
 * @author Tobias Kunz, Can Erdogan
 * @date Jan 31, 2013
 * @brief The generic RRT implementation. It can be inherited for modifications to collision
 * checking, sampling and etc.
 */

#include "RRT.h"
#include <robotics/World.h>
#include <robotics/Robot.h>
#include <kinematics/Dof.h>

using namespace std;
using namespace Eigen;
using namespace robotics;

namespace walking {

/* ********************************************************************************************* */
RRT::RRT(World* world,
		 int robot,
		 const std::vector<int> &dofs,
		 const VectorXd &root,
		 double stepSize) : world(world),
		 	 	 	 	 	robot(robot),
		 	 	 	 	 	dofs(dofs),
		 	 	 	 	 	ndim(dofs.size()),
		 	 	 	 	 	stepSize(stepSize),
		 	 	 	 	 	index(flann::KDTreeSingleIndexParams())
{

	// RRT-star
	ballRadiusLast = 1000;
	ballRadiusMax = 2;
	ballRadiusConstant = 25;
	goalReached = false;
	xGoal_ind = -1;

	version = 0;


	// Reset the random number generator and add the given start configuration to the flann structure
	srand(time(NULL));

	addNode(root, -1);

}

RRT::RRT(World* world,
		 int robot,
		 const std::vector<int> &dofs,
		 const VectorXd &root,
		 const VectorXd &goal,
		 double stepSize) : world(world),
						   	robot(robot),
						    dofs(dofs),
						    ndim(dofs.size()),
						    stepSize(stepSize),
						    index(flann::KDTreeSingleIndexParams())
{

	// RRT-star
	ballRadiusLast = 1000;
	ballRadiusMax = 2;
	ballRadiusConstant = 25;
	goalReached = false;

	xGoal = goal;
	xGoal_ind = -1;

	version = 0;

	// Reset the random number generator and add the given start configuration to the flann structure
	srand(time(NULL));

	int xRoot_ind = addNode_sharp(root, -1);

	gScore_[xRoot_ind] = 0;
	rhs_[xRoot_ind] = 0;
	hScore_[xRoot_ind] = computeHeuristics_sharp(root,goal);

	rootedFromInitialHeapNode_[xRoot_ind]->data = xRoot_ind;
	rootedFromInitialHeapNode_[xRoot_ind]->key = computeKey_sharp(xRoot_ind);

	rootedFromInitialHeapState_[xRoot_ind] = OUTSIDE_HEAP;

}

/* ********************************************************************************************* */
RRT::RRT(World* world,
		 int robot,
		 const std::vector<int> &dofs,
		 const std::vector<VectorXd> &roots,
		 double stepSize) : world(world),
		 	 	 	 	 	robot(robot),
		 	 	 	 	 	dofs(dofs),
		 	 	 	 	 	ndim(dofs.size()),
		 	 	 	 	 	stepSize(stepSize),
		 	 	 	 	 	index(flann::KDTreeSingleIndexParams())
{
	// RRT-star
	ballRadiusLast = 1000;
	ballRadiusMax = 2;
	ballRadiusConstant = 25;
	goalReached = false;
	xGoal_ind = -1;

	version = 0;

	// Reset the random number generator and add the given start configurations to the flann structure
	srand(time(NULL));
	for(int i = 0; i < roots.size(); i++)
	{
		addNode(roots[i], -1);
	}
}

/* ********************************************************************************************* */
RRT::RRT(World* world,
		 int robot,
		 const std::vector<int> &dofs,
		 const vector<VectorXd> &roots,
		 const VectorXd &goal,
		 double stepSize): world(world),
		 	 	 	 	   robot(robot),
		 	 	 	 	   dofs(dofs),
		 	 	 	 	   ndim(dofs.size()),
		 	 	 	 	   stepSize(stepSize),
		 	 	 	 	   index(flann::KDTreeSingleIndexParams())
{
	// RRT-star
	ballRadiusLast = 1000;
	ballRadiusMax = 2;
	ballRadiusConstant = 25;
	goalReached = false;

	xGoal = goal;
	xGoal_ind = -1;

	version = 0;

	int xRoot_ind;
	// Reset the random number generator and add the given start configurations to the flann structure
	srand(time(NULL));
	for(int i = 0; i < roots.size(); i++)
	{
		xRoot_ind = addNode_sharp(roots[i], -1);

		gScore_[xRoot_ind] = 0;
		rhs_[xRoot_ind] = 0;
		hScore_[xRoot_ind] = computeHeuristics_sharp(roots[i], goal);

		rootedFromInitialHeapNode_[xRoot_ind]->data = xRoot_ind;
		rootedFromInitialHeapNode_[xRoot_ind]->key = computeKey_sharp(xRoot_ind);

		rootedFromInitialHeapState_[xRoot_ind] = OUTSIDE_HEAP;
	}
}

/* ********************************************************************************************* */
bool RRT::connect() {
	VectorXd qtry = getRandomConfig();
	return connect(qtry);
}

/* ********************************************************************************************* */
bool
RRT::connect(const VectorXd &target)
{

	// Get the index of the nearest neighbor in the tree to the given target
	int NNidx = getNearestNeighbor(target);

	// Keep taking steps towards the target until a collision happens
	StepResult result = STEP_PROGRESS;
	while(result == STEP_PROGRESS)
	{
		result = tryStepFromNode(target, NNidx);
		NNidx = configVector.size() - 1;
	}

	return (result == STEP_REACHED);
}

/* ********************************************************************************************* */
RRT::StepResult
RRT::tryStep()
{
	VectorXd qtry = getRandomConfig();
	return tryStep(qtry);
}

/* ********************************************************************************************* */
RRT::StepResult
RRT::tryStep(const VectorXd &qtry)
{
	int NNidx = getNearestNeighbor(qtry);
	return tryStepFromNode(qtry, NNidx);
}

/* ********************************************************************************************* */
RRT::StepResult
RRT::tryStepFromNode(const VectorXd &qtry, int NNidx)
{

	// Get the configuration of the nearest neighbor and check if already reached
	const VectorXd& qnear = *(configVector[NNidx]);

	if((qtry - qnear).norm() < stepSize)
	{
		return STEP_REACHED;
	}

	// Create the new node: scale the direction vector to stepSize and add to qnear
	VectorXd qnew = qnear + stepSize * (qtry - qnear).normalized();

	// Check for collision, make changes to the qNew and create intermediate points if necessary
	// NOTE: This is largely implementation dependent and in default, no points are created.
	list<VectorXd> intermediatePoints;
	bool collisionClear = newConfig(intermediatePoints, qnew, qnear, qtry);

	if(!collisionClear)
		return STEP_COLLISION;

	// Add the intermediate nodes and the final new node to the tree
	list <VectorXd>::iterator it = intermediatePoints.begin();
	for(; it != intermediatePoints.end(); it++) 
		NNidx = addNode(*it, NNidx);

	addNode(qnew, NNidx);

	return STEP_PROGRESS;
}

bool
RRT::extendTowardsState_star(int xFrom_ind,
							 const VectorXd &xTowards,
							 VectorXd &xExtended,
							 double &edgeCost)
{
	int numSteps = 10;

	//std::cout << xFrom_ind << std::endl;
	// xNearest_ind
	// Get the configuration of the qFrom and check if already reached
	const VectorXd& xFrom = *(configVector[xFrom_ind]);

	if((xTowards - xFrom).norm() < stepSize)
		xExtended = xTowards;
	else
		xExtended = xFrom + stepSize * (xTowards - xFrom).normalized();

	if (!hasCollisions_star(xFrom, xExtended, numSteps))
	{
		edgeCost = (xExtended - xFrom).norm();
		return true;
	}
	else
		return false;

}

// this function creates a new node in the tree
bool
RRT::extendTowardsState_star(int xFrom_ind,
							 const Eigen::VectorXd &xTowards,
							 int &xExtended_ind,
							 double &edgeCost)
{
	Eigen::VectorXd xExtended;

	if (!extendTowardsState_star(xFrom_ind, xTowards, xExtended, edgeCost))
		return false;

	xExtended_ind = addNode(xExtended, xFrom_ind, edgeCost);
	return true;
}

void
RRT::updateBallRadius_star()
{
	int n = this->configVector.size();

	ballRadiusLast = ballRadiusConstant*(std::pow(log(1+(double)(n))/((double)(n)), 1.0/ndim));

	if (ballRadiusLast > ballRadiusMax)
		ballRadiusLast = ballRadiusMax;
}

bool
RRT::hasCollisions_star(const VectorXd qInitial,
					    const VectorXd qFinal,
					    int numSteps)
{
	VectorXd disc = (qFinal-qInitial)/((double)numSteps);

	VectorXd qCurrent = qInitial;

	for (int i = 0; i <= numSteps; ++i)
	{
		world->getRobot(robot)->setConfig(dofs, qCurrent);

		if (world->checkCollision())
			return true;
		qCurrent += disc;
	}

	// no collision detected
	return false;
}

bool
RRT::findMinStateInSet_star(const Eigen::VectorXd &xTowards,
						    const std::vector<int> &xList_ind,
						    int &xMin_ind,
						    std::vector<int> &reachableXList_ind,
						    std::vector<double> &edgeCosts
						    )
{
	int numNeighbors, xFrom_ind ;

	Eigen::VectorXd xExtended,
					xDiff;

	double epsilon = 0.001,
		   minCostFromRootXTowards = std::numeric_limits<double>::infinity(),
		   costToComeXTowards,
		   costToComeXFrom,
		   edgeCost;

	bool found = false;

	reachableXList_ind.clear();
	edgeCosts.clear();

	numNeighbors = xList_ind.size();

	for (int i = 0; i < numNeighbors; ++i)
	{
		xFrom_ind = xList_ind[i];
		costToComeXFrom = computeCostFromRoot_star(xFrom_ind);

		if (!extendTowardsState_star(xFrom_ind, xTowards, xExtended, edgeCost))
			continue;

		if ((xTowards-xExtended).norm() < epsilon) // i.e., xTowards can be reached from xFrom fully
		{
			found = true;

			reachableXList_ind.push_back(xFrom_ind);
			edgeCosts.push_back(edgeCost);

			costToComeXTowards = costToComeXFrom + edgeCost;

			if (costToComeXTowards < minCostFromRootXTowards)
			{
				minCostFromRootXTowards = costToComeXTowards ;
				xMin_ind = xFrom_ind;

				//parentVector[xTowards_ind] = xFrom_ind;
				//costFromParent[xTowards_ind] = edgeCost;
			}

		}
	}

	return found;

}

bool
RRT::rewireTree_star(int xFrom_ind,
					 std::vector<int> &reachableXList_ind,
					 std::vector<double> &edgeCosts)
{

	Eigen::VectorXd xExtended,
					xTowards;

	double epsilon = 0.001,
		   newCostFromRootXTowards,
		   costToComeXTowards,
		   costToComeXFrom;

	int xTowards_ind,
		numNeighbors;

	costToComeXFrom = computeCostFromRoot_star(xFrom_ind);
	//std::cout << "g[xFrom_ind] : " << costToComeXFrom << std::endl;;
	numNeighbors = reachableXList_ind.size();

	for (int i = 0; i < numNeighbors; ++i)
	{
		xTowards_ind = reachableXList_ind[i];

		if (xTowards_ind == xFrom_ind || parentVector[xTowards_ind] == -1)
			continue;

		//xTowards = *(configVector[xTowards_ind]);

		//if (!extendTowardsState_star(xFrom_ind, xTowards, xExtended, edgeCost));
		//	continue;

		//if ((xTowards-xExtended).norm() < epsilon) // i.e., xTowards can be reached from xFrom fully
		//{
		costToComeXTowards = costToComeXFrom + edgeCosts[i];

		//std::cout << "new : " << costToComeXTowards
		//		  << " vs old : " << computeCostFromRoot_star(xTowards_ind) << std::endl;

		if (costToComeXTowards < computeCostFromRoot_star(xTowards_ind))
		{
			parentVector[xTowards_ind] = xFrom_ind;
			costFromParent[xTowards_ind] = edgeCosts[i];

			//std::cout << "rewired" << std::endl;
		}

		//}
	}
}

int
RRT::replan_sharp(int max_iter)
{
    HeapNode<int,CostKey> *xMin_hn,
    					  *xU_hn,
    					  *xS_hn;

    int xU_ind,
    	xS_ind;

    int numNeighbors,
    	nVertexExpanded;

    Edge *edge;

    double newRhs;

    CostKey xMin_key,
    		xGoal_key;

    if (!rootedFromInitialHeap_.isEmpty())
    {
        xMin_hn = rootedFromInitialHeap_.findMin();
        xMin_key = xMin_hn->key;
    }
    else
    {
        xMin_key.key1 = std::numeric_limits<double>::infinity();
        xMin_key.key2 = std::numeric_limits<double>::infinity();
    }

    xGoal_key = getGoalKey_sharp();

    int numUpdates = 0;
    int iter = 0;

    while((iter < max_iter) && (xMin_key < xGoal_key))
    {
        xU_hn = rootedFromInitialHeap_.deleteMin();       // This function corresponds to U.Pop()

        xU_ind = xU_hn->data;

        rootedFromInitialHeapState_[xU_ind] = OUTSIDE_HEAP;

        gScore_[xU_ind] = rhs_[xU_ind];

        numNeighbors = (outgoingEdges_[xU_ind])->size();

        std::list<Edge*>::iterator itEdge;

        for (itEdge = outgoingEdges_[xU_ind]->begin(); itEdge != outgoingEdges_[xU_ind]->end(); ++itEdge)
        {
            edge = *itEdge;
            xS_ind = edge->head;

            //if (xS_ind->state_.x_[0] == 8 && s->state_.x_[1] == 8 )
            //    std::cout << "goal is reached\n";

            newRhs = gScore_[xU_ind] + edge->cost;

            if (newRhs < rhs_[xS_ind])
            {
                if (parentVector[xS_ind] != -1) // remove xS_ind from children list of its parent
                {
                	int xS_parent_ind = parentVector[xS_ind];
                	children_[xS_parent_ind]->remove(xS_ind);
                }

                parentVector[xS_ind] = xU_ind;
                children_[xU_ind]->push_back(xS_ind);

                rhs_[xS_ind] = newRhs;

                updateHeapState_sharp(xS_ind);

                ++numUpdates;
            }
        }

        if (!rootedFromInitialHeap_.isEmpty())
        {
            xMin_hn = rootedFromInitialHeap_.findMin();
            xMin_key = xMin_hn->key;
        }
        else
        {
            xMin_key.key1 = std::numeric_limits<double>::infinity();
            xMin_key.key2 = std::numeric_limits<double>::infinity();
        }

        xGoal_key = getGoalKey_sharp();

    	++iter;
    }

    return numUpdates;
}

void
RRT::updateHeapState_sharp(int xU_ind)
{
	if ( gScore_[xU_ind] != rhs_[xU_ind] ) // the node is inconsistent, so it will be inserted into the heap
	{
		// or heap->isContain(xU_ind) check this later!!!
		if ( rootedFromInitialHeapState_[xU_ind] == INSIDE_HEAP ) // the heap_node is already in the heap
		{
			// first remove the heap_node from heap
			if ( rootedFromInitialHeapNode_[xU_ind] == rootedFromInitialHeap_.findMin() )
				rootedFromInitialHeap_.deleteMin();

			rootedFromInitialHeapNode_[xU_ind]->remove();
		}
		else
		{
			rootedFromInitialHeapState_[xU_ind] = INSIDE_HEAP;
		}
		// insert the heap_node with new key
		rootedFromInitialHeapNode_[xU_ind]->key = computeKey_sharp(xU_ind);
		rootedFromInitialHeap_.insertVertex(rootedFromInitialHeapNode_[xU_ind]);
	}
	else if ( gScore_[xU_ind] == rhs_[xU_ind] && rootedFromInitialHeapState_[xU_ind] == INSIDE_HEAP)
	{
		// the new update in rhs & g_score, makes the node consistent,
		// so it must be removed from the heap
		if ( rootedFromInitialHeapNode_[xU_ind] == rootedFromInitialHeap_.findMin() )
			rootedFromInitialHeap_.deleteMin();

		rootedFromInitialHeapNode_[xU_ind]->remove();

		rootedFromInitialHeapState_[xU_ind] = OUTSIDE_HEAP;
	}

}

/* ********************************************************************************************* */
bool
RRT::newConfig(list<VectorXd> &intermediatePoints,
			   VectorXd &qnew,
			   const VectorXd &qnear,
			   const VectorXd &qtarget)
{
	return !checkCollisions(qnew);
}

/* ********************************************************************************************* */
int
RRT::addNode(const VectorXd &qnew,
				  int parentId,
				  double edgeCost)
{
	
	// Update the graph vector
	VectorXd* temp = new VectorXd(qnew);
	configVector.push_back(temp);
	parentVector.push_back(parentId);
	costFromParent.push_back(edgeCost);

	// Update the underlying flann structure (the kdtree)
	unsigned int id = configVector.size() - 1;

	if(id == 0) 
		index.buildIndex(flann::Matrix<double>((double*)temp->data(), 1, temp->size()));
	else
		index.addPoints(flann::Matrix<double>((double*)temp->data(), 1, temp->size()));
	
	activeNode = id;

	return id;
}

int
RRT::addNode_sharp(const VectorXd &xNew,
				   int xNew_parent_ind)
{

	// Update the graph vector
	VectorXd* temp = new VectorXd(xNew);


	configVector.push_back(temp);

	parentVector.push_back(xNew_parent_ind);
	children_.push_back( new std::list<int> ());

	incomingEdges_.push_back(new std::list<Edge*>);
	outgoingEdges_.push_back(new std::list<Edge*>);


	gScore_.push_back(0);
	rhs_.push_back(0);
	hScore_.push_back(0);

	rootedFromInitialHeapNode_.push_back( new HeapNode<int,CostKey> ());
	rootedFromInitialHeapState_.push_back(OUTSIDE_HEAP);


	// Update the underlying flann structure (the kdtree)
	unsigned int id = configVector.size() - 1;

	if(id == 0)
		index.buildIndex(flann::Matrix<double>((double*)temp->data(), 1, temp->size()));
	else
		index.addPoints(flann::Matrix<double>((double*)temp->data(), 1, temp->size()));

	activeNode = id;
	return id;
}

/* ********************************************************************************************* */
inline int
RRT::getNearestNeighbor(const VectorXd &qsamp)
{
	int nearest;
	double distance;
	const flann::Matrix<double> queryMatrix((double*)qsamp.data(), 1, qsamp.size());
	flann::Matrix<int> nearestMatrix(&nearest, 1, 1);
	flann::Matrix<double> distanceMatrix(flann::Matrix<double>(&distance, 1, 1));
	index.knnSearch(queryMatrix, nearestMatrix, distanceMatrix, 1, 
		flann::SearchParams(flann::FLANN_CHECKS_UNLIMITED));
	activeNode = nearest;
	return nearest;
}

inline int
RRT::findNearestNeighbor(const Eigen::VectorXd &q)
{
	int qNearest_idx;
	double distance;

	const flann::Matrix<double> queryMatrix((double*)q.data(), 1, q.size());
	flann::Matrix<int> nearestMatrix(&qNearest_idx, 1, 1);
	flann::Matrix<double> distanceMatrix(flann::Matrix<double>(&distance, 1, 1));

	index.knnSearch(queryMatrix,
					nearestMatrix,
					distanceMatrix,
					1,
					flann::SearchParams(flann::FLANN_CHECKS_UNLIMITED));

	activeNode = qNearest_idx;

	return qNearest_idx;
}

/* ********************************************************************************************* */
inline bool
RRT::getNearNeighbors_star(const VectorXd &q,
						   double ballRadius,
						   std::vector<int> &nearNeighbors)
{
	double distance;
	int numNeighbors;

	//std::cout << "q: [ " << q.transpose() << " ]" << std::endl;
	const flann::Matrix<double> queryMatrix((double*)q.data(), 1, q.size());
	//flann::Matrix<int> nearMatrix;		// these are for pre-allocated case
	//flann::Matrix<double> distanceMatrix;
	std::vector< std::vector<int> > nearMatrix;
	std::vector< std::vector<double> > distanceMatrix;

	nearNeighbors.clear();

	//std::cout << "pre: " << nearNeighbors.size() << std::endl;

	numNeighbors = index.radiusSearch(
					   queryMatrix,
					   nearMatrix,
					   distanceMatrix,
					   ballRadius,
					   flann::SearchParams(flann::FLANN_CHECKS_UNLIMITED, 0.0, false)
					   );

	//std::cout << "num_neighbors : " << numNeighbors
	//		  << " near size : [" << nearMatrix[0].size()<< " ]" << std::endl;

	if (numNeighbors <= 0)
		return false;

	for (int i = 0; i < numNeighbors; ++i)
	{	//std::cout << nearMatrix[0][i] << std::endl;
		nearNeighbors.push_back(nearMatrix[0][i]);
	}

	return true;
}

double
RRT::computeBallRadiusConstant_star()
{
	double sigma = 0.0;

	if (ndim % 2 == 0)
		sigma = std::pow(M_PI, ndim/2.0)*factorial(ndim/2);
	else
		sigma = std::pow(M_PI, floor(ndim/2.0))
				*std::pow(2, ceil(ndim/2.0))
				*factorial(ndim,2.0);

	return sigma;

}

double
RRT::computeCostFromRoot_star(int x_ind)
{
	int xCurrent_ind;
	double distance = 0.0;

	xCurrent_ind = x_ind;

	while(xCurrent_ind  != -1)
	{
		distance = distance + costFromParent[xCurrent_ind];
		xCurrent_ind = parentVector[xCurrent_ind];
	}

	return distance;
}

bool
RRT::extend_sharp(const Eigen::VectorXd &xTarget)
{
	int xNearest_ind,
		xFrom_ind,
		numNeighbors,
		numSteps = 10,
		numReachableNeighbors;

	bool includeDecision;

	Eigen::VectorXd xFrom;

	double temp_rhs,
		   edgeCost;

	std::vector<int> nearNeighbors,
					 reachableNeighbors;

	CostKey xGoal_key;

	Edge *edge;

	// begin: for the new node
	int xNew_ind = -1;
	Eigen::VectorXd xNew;

	int xNew_parent_ind = -1;

	std::list<int> * xNew_children;

	double xNew_gScore = std::numeric_limits<double>::infinity();
	double xNew_rhs = std::numeric_limits<double>::infinity();
	double xNew_hScore = std::numeric_limits<double>::infinity();

	HeapState xNew_hs = OUTSIDE_HEAP;
    HeapNode<int,CostKey> *xNew_hn;


	// end: for the new node


	// A.2. Find the nearest node
	xNearest_ind = getNearestNeighbor(xTarget);

	// A.3. Extend the node towards the sampled state -- just compute the extended state

	if (!extendTowardsState_sharp(xNearest_ind, xTarget, xNew, stepSize, numSteps))
		return false;

	xNew_hScore = computeHeuristics_sharp(xNew, xGoal);

    // A.4. Compute the set of all close nodes around extended_state
	if (!getNearNeighbors_star(xNew, ballRadiusLast, nearNeighbors))
		nearNeighbors.push_back(xNearest_ind);

	// A.5. Pick the node to be extended


	numNeighbors = nearNeighbors.size();

	for (int i = 0; i < numNeighbors; ++i)
	{
		xFrom_ind = nearNeighbors[i];
		xFrom = *configVector[xFrom_ind];

		if (!hasCollisions_star(xFrom, xNew, numSteps))
		{
			reachableNeighbors.push_back(xFrom_ind);

			edgeCost = computeCost_sharp(xFrom, xNew);
			temp_rhs = gScore_[xFrom_ind] + edgeCost;

			if (temp_rhs < xNew_rhs)
			{
				xNew_rhs = temp_rhs;
				xNew_parent_ind = xFrom_ind;
			}
		}
	}

	xGoal_key = getGoalKey_sharp();

    if (version == 0)
    {
    	includeDecision = true;

    	if (xNew_parent_ind == -1)	// newNode->rhs_ is already infinity
    	{
    		xNew_parent_ind = xNearest_ind;
    		xNew_rhs = std::numeric_limits<double>::infinity();
    	}
    }
    else if (version == 1)
    {
    	includeDecision = (xNew_parent_ind != -1);
    }
  	else if (version == 2)
    {
    	CostKey xNew_parent_key = computeKey_sharp(xNew_parent_ind);

    	includeDecision = (xNew_parent_ind != -1) && (xNew_parent_key < xGoal_key);
    }
  	else if (version == 3)
    {
    	CostKey xNew_key;

    	xNew_key.key2 = std::min(xNew_gScore, xNew_rhs);
    	xNew_key.key1 =  xNew_key.key2 + xNew_hScore;

  		includeDecision = (xNew_parent_ind !=-1) && (xNew_key < xGoal_key);
    }

    if (includeDecision)
    {
    	// create a new node on the tree and insert into the kd-tree

    	xNew_ind = addNode_sharp(xNew, xNew_parent_ind);

    	children_[xNew_parent_ind]->push_back(xNew_ind);

    	gScore_[xNew_ind] = xNew_gScore;
    	rhs_[xNew_ind] = xNew_rhs;
    	hScore_[xNew_ind] = xNew_hScore;

    	//xNew_incomingEdges = new std::list<Edge*> ();
    	//xNew_outgoingEdges = new std::list<Edge*> ();

    	numReachableNeighbors = reachableNeighbors.size();

        for (int i = 0; i < numReachableNeighbors; ++i)
        {
            xFrom_ind = reachableNeighbors[i];
            xFrom = *configVector[xFrom_ind];

            edgeCost = computeCost_sharp(xFrom, xNew);

            edge = new Edge(xFrom_ind, xNew_ind, edgeCost);

            outgoingEdges_[xFrom_ind]->push_back(edge);
            incomingEdges_[xNew_ind]->push_back(edge);

            //this is for unidirected graphs
            edge = new Edge(xNew_ind, xFrom_ind, edgeCost);
            outgoingEdges_[xNew_ind]->push_back(edge);
            incomingEdges_[xFrom_ind]->push_back(edge);
        }

        // inserting into the heap
        rootedFromInitialHeapNode_[xNew_ind]->data = xNew_ind;
        rootedFromInitialHeapNode_[xNew_ind]->key = computeKey_sharp(xNew_ind);

        rootedFromInitialHeapState_[xNew_ind] = OUTSIDE_HEAP;

        updateHeapState_sharp(xNew_ind);



        // Check if the goal is reached and create the path, if so

		if ( !goalReached && (xGoal - xNew).norm() < 0.001)
		{
			goalReached = true;
			xGoal_ind = xNew_ind;
		}

		return true;

    }
    else
    {
    	// no need to delete any node.

    	return false;
    }
}

bool
RRT::extendTowardsState_sharp(int xFrom_ind,
							 const VectorXd& xTowards,
							 VectorXd &xExtended,
							 double steerLength,
							 int numSteps)
{

	//std::cout << xFrom_ind << std::endl;
	// xNearest_ind
	// Get the configuration of the qFrom and check if already reached
	double distance;
	VectorXd xFrom = *(configVector[xFrom_ind]);

	distance = (xTowards - xFrom).norm();

	if( distance < steerLength)
		xExtended = xTowards;
	else
		xExtended = xFrom + steerLength * (xTowards - xFrom).normalized();

	return !hasCollisions_star(xFrom, xExtended, numSteps);

}

CostKey
RRT::getGoalKey_sharp()
{
	CostKey xGoal_key;

	if (goalReached)
		xGoal_key = computeKey_sharp(xGoal_ind);
	else
	{
		xGoal_key.key1 = std::numeric_limits<double>::infinity();
		xGoal_key.key2 = std::numeric_limits<double>::infinity();
	}

	return xGoal_key;
}

double
RRT::computeCost_sharp(const Eigen::VectorXd &xFrom,
						const Eigen::VectorXd &xTo)
{
	return (xTo - xFrom).norm();
}

double
RRT::computeHeuristics_sharp(const Eigen::VectorXd &xFrom,
		  	  	  	    const Eigen::VectorXd &xTo)
{
	return (xTo - xFrom).norm();
}

CostKey
RRT::computeKey_sharp(int x_ind)
{
    CostKey k;

    k.key2 = std::min(gScore_[x_ind], rhs_[x_ind]);
    k.key1 = k.key2 + hScore_[x_ind];

    return k;
}

/* ********************************************************************************************* */
// random # between min & max
inline double RRT::randomInRange(double min, double max) {
	if(min == max) return min;
	return min + ((max-min) * ((double)rand() / ((double)RAND_MAX + 1)));
}

/* ********************************************************************************************* */
VectorXd
RRT::getRandomConfig()
{
	// Samples a random point for qtmp in the configuration space, bounded by the provided 
	// configuration vectors (and returns ref to it)

	VectorXd config(ndim);
	for (int i = 0; i < ndim; ++i)
	{
		config[i] = randomInRange(world->getRobot(robot)->getDof(dofs[i])->getMin(), 
			world->getRobot(robot)->getDof(dofs[i])->getMax());
	}
	return config;
}

VectorXd
RRT::getRandomPromisingConfig()
{

	std::vector<int> nearNeighbors;

	VectorXd xNew(ndim), xRandom(ndim), xMiddle(ndim);

	xRandom = VectorXd::Zero(ndim);

	//std::cout << "xRandom : "
	//		  << xRandom << std::endl;

	int xNew_ind,
		xNeighbor_ind,
		xParent_ind,
		xChild_ind,
		numChildren,
		numNeighbors,
		numPromisingNeighbors;

	CostKey xNew_key,
			xGoal_key,
			xNeighbor_key;

	double len_min, len_max, r, r_min, r_max, d;


	xGoal_key = getGoalKey_sharp();

	xNew_key.key1 = std::numeric_limits<double>::infinity();
	xNew_key.key2 = std::numeric_limits<double>::infinity();

	numChildren = 0;

	while (!(xNew_key < xGoal_key && numChildren != 0))
	{
		xNew_ind = (int) randomInRange(1, configVector.size()-0.01);  // zero is the xRoot_ind
		xNew_key = computeKey_sharp(xNew_ind);
		numChildren = children_[xNew_ind]->size();
	}

	/*
	xParent_ind = parentVector[xNew_ind];
	xChild_ind = (int) randomInRange(0, numChildren-0.01);

	xMiddle = (*configVector[xParent_ind]) + (*configVector[xChild_ind])/2.0;


	d = randomInRange(0.05, 0.2);

	xRandom = xNew + d*(xMiddle - xNew).normalized();

	for (int i = 0; i < ndim; ++i)
	{
		len_min = world->getRobot(robot)->getDof(dofs[i])->getMin();
		len_max = world->getRobot(robot)->getDof(dofs[i])->getMax();

		if (xRandom[i] < len_min)
			xRandom[i] = len_min;

		if (xRandom[i] > len_max)
			xRandom[i] = len_max;

	}

	return xRandom;
*/

	/*
	getNearNeighbors_star(xNew, ballRadiusLast, nearNeighbors);

	numNeighbors = nearNeighbors.size();

	numPromisingNeighbors = 0;

	for (int i = 0; i < numNeighbors; ++i)
	{
		xNeighbor_ind = nearNeighbors[i];
		xNeighbor_key = computeKey_sharp(xNeighbor_ind);

		if (xNeighbor_ind != xNew_ind && (xNeighbor_key < xGoal_key))
		{
			xRandom = xRandom + (*configVector[xNeighbor_ind]);
			++numPromisingNeighbors;
		}
	}

	*/


	Edge *edge;
	std::list<Edge*>::reverse_iterator rit;

	numPromisingNeighbors = 0;
	xRandom = xNew;

	//&& numPromisingNeighbors < 10

	for (rit = outgoingEdges_[xNew_ind]->rbegin(); rit != outgoingEdges_[xNew_ind]->rend() && numPromisingNeighbors < 10; ++rit)
	{
		xNeighbor_ind = (*rit)->head;
		xNeighbor_key = computeKey_sharp(xNeighbor_ind);

		if (xNeighbor_key < xGoal_key)
		{
			xRandom = xRandom + (*configVector[xNeighbor_ind]);
			++numPromisingNeighbors;
		}
	}

	if (numPromisingNeighbors > 0)
	{
		xRandom = xRandom/numPromisingNeighbors;

		//std::cout << "sample: x_center " << xNew_ind
		//		  << " num_neigh " << numPromisingNeighbors << std::endl;

		return xRandom;
	}




	// put a hyper cube centered on Xnew

	for (int i = 0; i < ndim; ++i)
	{
		len_min = world->getRobot(robot)->getDof(dofs[i])->getMin();
		len_max = world->getRobot(robot)->getDof(dofs[i])->getMax();
		r = (len_max - len_min)/20;

		r_min = xNew[i]-r/2;
		r_max = xNew[i]+r/2;

		if (r_min < len_min)
			r_min = len_min;

		if (r_max > len_max)
			r_max = len_max;

		xRandom[i] = randomInRange(r_min, r_max);
	}

	return xRandom;
}


/* ********************************************************************************************* */
VectorXd
RRT::getRandomConfig(const Vector3d &current_com)
{
	// Samples a random point for qtmp in the configuration space, bounded by the provided
	// configuration vectors (and returns ref to it)
	VectorXd config(ndim);

	for (int i = 0; i < 3; ++i)
	{
		config[i] = randomInRange(current_com[i]-0.25, current_com[i]+0.25);
	}


	for (int i = 0; i < ndim-3; ++i)
	{
		std::cout << "dofs : " << dofs[i+3]
		          << " min : " << world->getRobot(robot)->getDof(dofs[i+3])->getMin()
		          << " max : " << world->getRobot(robot)->getDof(dofs[i+3])->getMax() << "\n";

		config[i+3] = randomInRange(world->getRobot(robot)->getDof(dofs[i+3])->getMin(),
			world->getRobot(robot)->getDof(dofs[i+3])->getMax());

		std::cout << " config " << config.transpose() << "\n";
	}
	return config;
}

/* ********************************************************************************************* */
double RRT::getGap(const VectorXd &target) {
	return (target - *(configVector[activeNode])).norm();
}

/* ********************************************************************************************* */
void RRT::tracePath(int node, std::list<VectorXd> &path, bool reverse) {

	// Keep following the "linked list" in the given direction
	int x = node;
	while(x != -1) {
		if(!reverse) path.push_front(*(configVector[x]));	
		else path.push_back(*(configVector[x]));
		x = parentVector[x];
	}
}

/* ********************************************************************************************* */
bool
RRT::checkCollisions(const VectorXd &c)
{
	world->getRobot(robot)->setConfig(dofs, c);
	return world->checkCollision();
}

/* ********************************************************************************************* */
size_t RRT::getSize() {
	return configVector.size();
}

/* ********************************************************************************************* */
inline void RRT::saveLine (char* l1, char* l2, size_t off, const VectorXd& n1, const VectorXd& n2) {
	if(n1.size() == 2) {
		sprintf(l1 + off, "%+02.3lf %+02.3lf ", n1(0), n1(1));
		sprintf(l2 + off, "%+02.3lf %+02.3lf ", n2(0), n2(1));
	}
	else if(n1.size() == 3) {
		sprintf(l1 + off, "%+02.3lf %+02.3lf %+02.3lf ", n1(0), n1(1), n1(2));
		sprintf(l2 + off, "%+02.3lf %+02.3lf %+02.3lf ", n2(0), n2(1), n2(2));
	}
}

/* ********************************************************************************************* */
inline void RRT::drawLine (FILE* f, size_t numDofs, const char* color, size_t i, bool last) {
	if(numDofs == 2) {
		fprintf(f, "\".data\" u %lu:%lu with linespoints notitle ps 1 pt 6 lc rgb '#%s'%s ", 
			2*i+1, 2*i+2, color, (last ? "" : ","));
	}
	else if (numDofs == 3) {
		fprintf(f, "\".data\" u %lu:%lu:%lu with linespoints notitle ps 1 pt 6 lc rgb '#%s'%s ", 
			3*i+1, 3*i+2, 3*i+3, color, (last ? "" : ","));
	}
}

/* ********************************************************************************************* */
void RRT::draw (const RRT* t1, const RRT* t2) {

	// Check the size of a data point - we can only visualize 2 or 3
	size_t numDofs = t1->dofs.size();
	if((numDofs != 2) && (numDofs != 3)) return;

	// ====================================================================
	// Write the data to a file

	// File contains two lines, one for each endpoint of an edge
	FILE* data = fopen(".data", "w");
	size_t step = numDofs * 7;											// 7 characters per double
	size_t numEdges1 = (t1->configVector.size() - 1);
	size_t numEdges2 = ((t2 == NULL) ? 0 : t2->configVector.size() - 1);
	char* line1 = new char[(numEdges1 + numEdges2 + 5) * step];
	char* line2 = new char[(numEdges1 + numEdges2 + 5) * step];

	// Write each node and its parent in the respective lines
	size_t lineIndex = 0;
	const RRT* trees [2] = {t1, t2};
	for(size_t t = 0; t < 2; t++) {

		// Skip the tree if not there
		if(trees[t] == NULL) continue;

		// Draw the edges
		size_t numEdges = trees[t]->configVector.size() - 1;
		for(size_t i = 0; i < numEdges; i++, lineIndex += step) {
			const VectorXd& node = *trees[t]->configVector[i + 1];
			const VectorXd& parent = *trees[t]->configVector[trees[t]->parentVector[i + 1]];
			saveLine(line1, line2, lineIndex, node, parent);
		}
	}

	// Print the start to draw with a special color (we draw a 0 length edge)
	const VectorXd& goal = *t1->configVector[0];
	saveLine(line1, line2, lineIndex, goal, goal);
	lineIndex += step;

	// Write the lines to the file
	fprintf(data, "%s\n", line1);
	fprintf(data, "%s\n", line2);
	fclose(data);

	delete[] line1;
	delete[] line2;

	// ====================================================================
	// Run gnuplot with the pipe call

	// Open gnuplot in a shell terminal
	FILE* gnuplot;
	gnuplot = fopen(".gnuplot", "w");

	// Set options
	fprintf(gnuplot, "");
	fprintf(gnuplot, "set terminal wxt size 600,600;\n");
	fprintf(gnuplot, "set xrange [-3.14:3.14];\n");
	fprintf(gnuplot, "set yrange [-3.14:3.14];\n");
	fprintf(gnuplot, "set size ratio -1;\n");
	if(numDofs == 3) {
		fprintf(gnuplot, "set zrange [-3.14:3.14];\n");
		fprintf(gnuplot, "set xyplane at -3.14;\n");
	}

	// Draw the edges in the file but leave the last edge to draw a special color for the goal
	fprintf(gnuplot, "%s ", ((numDofs == 2) ? "plot" : "splot"));
	for(size_t i = 0; i < numEdges1; i++) 
		drawLine(gnuplot, numDofs, "0000ff", i);
	for(size_t i = 0; i < numEdges2; i++) 
		drawLine(gnuplot, numDofs, "00ffff", i + numEdges1);
	
	// Draw the goal point (fake edge)
	drawLine(gnuplot, numDofs, "00ff00", numEdges1 + numEdges2, true); 

	// Close the pipe
	fprintf(gnuplot, "\n");
	fprintf(gnuplot, "\n");
	fclose(gnuplot);

	// Make the system call
	int status = system("gnuplot -persist .gnuplot");
	assert((status != -1) && "Error in system call in RRT::draw()");
}

}	//< End of namespace
