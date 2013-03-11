/**
 * @file PathPlanner.h
 * @author Tobias Kunz (?), Can Erdogan
 * @date Jan 30, 2013
 * @brief Contains the path planner class definition which is templated on a RRT implementation and
 * creates an interface to generate trajectories with different RRT algorithms such as goal-biased,
 * bidirectional, connect and etc.
 */

#include <Eigen/Core>
#include <iostream>
#include <limits>
#include <list>
#include <vector>
#include <robotics/Robot.h>
#include <robotics/World.h>
#include "RRT.h"

namespace walking {

/* ********************************************************************************************* */
/// The path planner class - a common interface to motion planners
template <class R = RRT>
class PathPlanner
{
public:

	bool connect;            ///< Whether we take a small step or shoot for the target node
	bool bidirectional;      ///< Whether two trees try to meet each other or one goes for the goal
	double stepSize;        ///< Step size from a node in the tree to the random/goal node
	double goalBias;        ///< Choose btw goal and random value (for goal-biased search)
	size_t maxNodes;        ///< Maximum number of iterations the sampling would continue
	robotics::World* world;  ///< The world that the robot is in (for obstacles and etc.)

	// NOTE: It is useful to keep the rrts around after planning for reuse, analysis, and etc.
	R* start_rrt;            ///< The rrt for unidirectional search
	R* goal_rrt;              ///< The second rrt if bidirectional search is executed

public:

	/// The default constructor
	PathPlanner() : world(NULL) {}

	/// The desired constructor - you should use this one.
	PathPlanner(robotics::World& world,
				bool bidirectional_ = true,
				bool connect_ = true,
				double stepSize_ = 0.1,
				size_t maxNodes_ = 1e6,
				double goalBias_ = 0.3): world(&world),
										 bidirectional(bidirectional_),
										 connect(connect_),
										 stepSize(stepSize_),
										 maxNodes(maxNodes_),
										 goalBias(goalBias_)
	{}

	/// The destructor
	~PathPlanner() {}

	/// Plan a path from a single start configuration to a single goal
	bool
  	planPath(int robotId,
  			 const std::vector<int> &dofs,
		     const Eigen::VectorXd &start,
		     const Eigen::VectorXd &goal,
		     std::list<Eigen::VectorXd> &path)
	{
		std::vector<Eigen::VectorXd> startVector, goalVector;
		startVector.push_back(start);
		goalVector.push_back(goal);
		return planPath(robotId, dofs, startVector, goalVector, path);
	}

	/// Plan a path from a single start configuration to a single goal
	bool
	planPath_star(int robotId,
			 	  const std::vector<int> &dofs,
			 	  const Eigen::VectorXd &start,
			 	  const Eigen::VectorXd &goal,
			 	  std::list<Eigen::VectorXd> &path)
	{
		std::vector<Eigen::VectorXd> startVector, goalVector;
		startVector.push_back(start);
		goalVector.push_back(goal);
		return planPath_star(robotId, dofs, startVector, goalVector, path);
	}

	/// Plan a path from a single start configuration to a single goal
	bool
	planPath_sharp(int robotId,
				  const std::vector<int> &dofs,
				  const Eigen::VectorXd &start,
				  const Eigen::VectorXd &goal,
				  std::list<Eigen::VectorXd> &path)
	{
		std::vector<Eigen::VectorXd> startVector, goalVector;
		startVector.push_back(start);
		goalVector.push_back(goal);
		return planPath_sharp(robotId, dofs, startVector, goalVector, path);
	}

	/// Plan a path from a single start configuration to a goal_com (center of mass)
	bool planPath(int robotId,
				 const std::vector<int> &dofs,
				 const Eigen::VectorXd &start,
				 const Eigen::Vector3d &goal_com,
				 std::list<Eigen::VectorXd> &path)
	{
		std::vector<Eigen::VectorXd> startVector;
		startVector.push_back(start);

		return planPath(robotId, dofs, startVector, goal_com, path);
	}



	/// Plan a path from a _set_ of start configurations to a _set_ of goals
	bool planPath(int robotId,
				  const std::vector<int> &dofs,
				  const std::vector<Eigen::VectorXd> &start,
				  const std::vector<Eigen::VectorXd> &goal,
				  std::list<Eigen::VectorXd> &path);

	/// Plan a path from a _set_ of start configurations to a _set_ of goals
	bool
	planPath_star(int robotId,
				  const std::vector<int> &dofs,
				  const std::vector<Eigen::VectorXd> &start,
				  const std::vector<Eigen::VectorXd> &goal,
				  std::list<Eigen::VectorXd> &path);

	bool
	planPath_sharp(int robotId,
				  const std::vector<int> &dofs,
				  const std::vector<Eigen::VectorXd> &start,
				  const std::vector<Eigen::VectorXd> &goal,
				  std::list<Eigen::VectorXd> &path);


	/// Plan a path from a single start configuration to a goal_com (center of mass)
	bool
	planPath(int robotId,
				 const std::vector<int> &dofs,
				 const std::vector<Eigen::VectorXd> &start,
				 const Eigen::Vector3d &goal_com,
				 std::list<Eigen::VectorXd> &path);

private:

  /// Performs a unidirectional RRT with the given options.
	bool planSingleTreeRrt(int robot,
						   const std::vector<int> &dofs,
						   const std::vector<Eigen::VectorXd> &start,
						   const Eigen::VectorXd &goal,
						   std::list<Eigen::VectorXd> &path);

	bool
	planSingleTreeRrt_star(int robot,
  				   	  	  const std::vector<int> &dofs,
  						  const std::vector<Eigen::VectorXd> &start,
  						  const Eigen::VectorXd &goal,
  						  std::list<Eigen::VectorXd> &path);

	bool
	planSingleTreeRrt_sharp(int robot,
  				   	  	  const std::vector<int> &dofs,
  						  const std::vector<Eigen::VectorXd> &start,
  						  const Eigen::VectorXd &goal,
  						  std::list<Eigen::VectorXd> &path);


	bool
	planSingleTreeRrt(int robot,
					  const std::vector<int> &dofs,
					  const std::vector<Eigen::VectorXd> &start,
					  const Eigen::Vector3d &goal_com,
					  std::list<Eigen::VectorXd> &path);

  /// Performs bidirectional RRT with the given options.
  /// NOTE This algorithm has several different popular implementations. The implementation in the
  /// kinodynamic paper (1999) by LaValle and Kuffner extend the two RRTs towards a common random
  /// configurations whereas here, first, start rrt extends towards a random node and creates
  /// some node N. Afterwards, the second rrt extends towards _the node N_ and they continue
  /// swapping roles.
  bool planBidirectionalRrt(int robot, const std::vector<int> &dofs,
    const std::vector<Eigen::VectorXd> &start, const std::vector<Eigen::VectorXd> &goal,
    std::list<Eigen::VectorXd> &path);
};

/* ********************************************************************************************* */
template <class R>
bool
PathPlanner<R>::planPath(int robotId,
						 const std::vector<int> &dofs,
						 const std::vector<Eigen::VectorXd> &start,
						 const std::vector<Eigen::VectorXd> &goal,
						 std::list<Eigen::VectorXd> &path)
{

	Eigen::VectorXd savedConfiguration = world->getRobot(robotId)->getConfig(dofs);

	// ====================================================================
	// Check for collisions in the start and goal configurations

	// Sift through the possible start configurations and eliminate those that are in collision
	std::vector<Eigen::VectorXd> feasibleStart;

	for(unsigned int i = 0; i < start.size(); i++)
	{
		world->getRobot(robotId)->setConfig(dofs, start[i]);

		if(!world->checkCollision())
			feasibleStart.push_back(start[i]);
	}

	// Return false if there are no feasible start configurations
	if(feasibleStart.empty())
	{
		printf("WARNING: PathPlanner: Feasible start points are empty!\n");
		return false;
	}

	// Sift through the possible goal configurations and eliminate those that are in collision
	std::vector<Eigen::VectorXd> feasibleGoal;

	for(unsigned int i = 0; i < goal.size(); i++)
	{
		world->getRobot(robotId)->setConfig(dofs, goal[i]);

		if(!world->checkCollision())
			feasibleGoal.push_back(goal[i]);
	}

	// Return false if there are no feasible goal configurations
	if(feasibleGoal.empty())
	{
		printf("WARNING: PathPlanner: Feasible goal points are empty!\n");
		return false;
	}

	// ====================================================================
	// Make the correct RRT algorithm for the given method

	// Direct the search towards single or bidirectional
	bool result = false;

	if(bidirectional)
		result = planBidirectionalRrt(robotId, dofs, feasibleStart, feasibleGoal, path);
	else {
		if(feasibleGoal.size() > 1)
			fprintf(stderr, "WARNING: planPath is using ONLY the first goal!\n");

		result = planSingleTreeRrt(robotId, dofs, feasibleStart, feasibleGoal.front(), path);
	}

	// Restore previous robot configuration
	world->getRobot(robotId)->setConfig(dofs, savedConfiguration);

	return result;
}


/// Plan a path from a _set_ of start configurations to a _set_ of goals
template <class R>
bool
PathPlanner<R>::planPath_star(int robotId,
						 const std::vector<int> &dofs,
						 const std::vector<Eigen::VectorXd> &start,
						 const std::vector<Eigen::VectorXd> &goal,
						 std::list<Eigen::VectorXd> &path)

{

	Eigen::VectorXd savedConfiguration = world->getRobot(robotId)->getConfig(dofs);

	// ====================================================================
	// Check for collisions in the start and goal configurations

	// Sift through the possible start configurations and eliminate those that are in collision
	std::vector<Eigen::VectorXd> feasibleStart;

	for(unsigned int i = 0; i < start.size(); i++)
	{
		world->getRobot(robotId)->setConfig(dofs, start[i]);

		if(!world->checkCollision())
			feasibleStart.push_back(start[i]);
	}

	// Return false if there are no feasible start configurations
	if(feasibleStart.empty())
	{
		printf("WARNING: PathPlanner: Feasible start points are empty!\n");
		return false;
	}

	// Sift through the possible goal configurations and eliminate those that are in collision
	std::vector<Eigen::VectorXd> feasibleGoal;

	for(unsigned int i = 0; i < goal.size(); i++)
	{
		world->getRobot(robotId)->setConfig(dofs, goal[i]);

		if(!world->checkCollision())
			feasibleGoal.push_back(goal[i]);
	}

	// Return false if there are no feasible goal configurations
	if(feasibleGoal.empty())
	{
		printf("WARNING: PathPlanner: Feasible goal points are empty!\n");
		return false;
	}

	// ====================================================================
	// Make the correct RRT algorithm for the given method

	// Direct the search towards single or bidirectional
	bool result = false;


	if(feasibleGoal.size() > 1)
		fprintf(stderr, "WARNING: planPath is using ONLY the first goal!\n");

	result = planSingleTreeRrt_star(robotId, dofs, feasibleStart, feasibleGoal.front(), path);


	// Restore previous robot configuration
	world->getRobot(robotId)->setConfig(dofs, savedConfiguration);

	return result;
}

template <class R>
bool
PathPlanner<R>::planPath_sharp(int robotId,
						 	   const std::vector<int> &dofs,
						 	   const std::vector<Eigen::VectorXd> &start,
						 	   const std::vector<Eigen::VectorXd> &goal,
						 	   std::list<Eigen::VectorXd> &path)

{

	Eigen::VectorXd savedConfiguration = world->getRobot(robotId)->getConfig(dofs);

	// ====================================================================
	// Check for collisions in the start and goal configurations

	// Sift through the possible start configurations and eliminate those that are in collision
	std::vector<Eigen::VectorXd> feasibleStart;

	for(unsigned int i = 0; i < start.size(); i++)
	{
		world->getRobot(robotId)->setConfig(dofs, start[i]);

		if(!world->checkCollision())
			feasibleStart.push_back(start[i]);
	}

	// Return false if there are no feasible start configurations
	if(feasibleStart.empty())
	{
		printf("WARNING: PathPlanner: Feasible start points are empty!\n");
		return false;
	}

	// Sift through the possible goal configurations and eliminate those that are in collision
	std::vector<Eigen::VectorXd> feasibleGoal;

	for(unsigned int i = 0; i < goal.size(); i++)
	{
		world->getRobot(robotId)->setConfig(dofs, goal[i]);

		if(!world->checkCollision())
			feasibleGoal.push_back(goal[i]);
	}

	// Return false if there are no feasible goal configurations
	if(feasibleGoal.empty())
	{
		printf("WARNING: PathPlanner: Feasible goal points are empty!\n");
		return false;
	}

	// ====================================================================
	// Make the correct RRT algorithm for the given method

	// Direct the search towards single or bidirectional
	bool result = false;


	if(feasibleGoal.size() > 1)
		fprintf(stderr, "WARNING: planPath is using ONLY the first goal!\n");

	result = planSingleTreeRrt_sharp(robotId, dofs, feasibleStart, feasibleGoal.front(), path);


	// Restore previous robot configuration
	world->getRobot(robotId)->setConfig(dofs, savedConfiguration);

	return result;
}

/// Plan a path from a single start configuration to a goal_com (center of mass)
template <class R>
bool
PathPlanner<R>::planPath(int robotId,
			 	 	 	 const std::vector<int> &dofs,
			 	 	 	 const std::vector<Eigen::VectorXd> &start,
			 	 	 	 const Eigen::Vector3d &goal_com,
			 	 	 	 std::list<Eigen::VectorXd> &path)
{

	Eigen::VectorXd savedConfiguration = world->getRobot(robotId)->getConfig(dofs);

	// ====================================================================
	// Check for collisions in the start and goal configurations

	// Sift through the possible start configurations and eliminate those that are in collision
	std::vector<Eigen::VectorXd> feasibleStart;

	for(unsigned int i = 0; i < start.size(); i++)
	{
		world->getRobot(robotId)->setConfig(dofs, start[i]);

		if(!world->checkCollision())
			feasibleStart.push_back(start[i]);
	}

	// Return false if there are no feasible start configurations
	if(feasibleStart.empty())
	{
		printf("WARNING: PathPlanner: Feasible start points are empty!\n");
		return false;
	}

	// ====================================================================
	// Make the correct RRT algorithm for the given method

	// Direct the search towards single or bidirectional
	bool result = false;

	result = planSingleTreeRrt(robotId, dofs, feasibleStart, goal_com, path);

	// Restore previous robot configuration
	world->getRobot(robotId)->setConfig(dofs, savedConfiguration);

	return result;
}



/* ********************************************************************************************* */
template <class R>
bool
PathPlanner<R>::planSingleTreeRrt(int robot,
								  const std::vector<int> &dofs,
								  const std::vector<Eigen::VectorXd> &start,
								  const Eigen::VectorXd &goal,
								  std::list<Eigen::VectorXd> &path)
{

	const bool debug = false;

	// Initialize the RRT
	start_rrt = new R (world, robot, dofs, start, stepSize);

	// Expand the tree until the goal is reached or the max # nodes is passed
	typename R::StepResult result = R::STEP_PROGRESS;
	double smallestGap = std::numeric_limits<double>::infinity();
	size_t numNodes = start_rrt->getSize();

	while(numNodes <= maxNodes)
	{

		// Get the target node based on the bias
		Eigen::VectorXd target;
		double randomValue = ((double) rand()) / RAND_MAX;

		if(randomValue < goalBias)
			target = goal;
		else
			target = start_rrt->getRandomConfig();

		// Based on the method, either attempt to connect to the target directly or take a small step
		if(connect)
			start_rrt->connect(target);
		else
			start_rrt->tryStep(target);

		// Check if the goal is reached and create the path, if so
		double gap = start_rrt->getGap(goal);
		if(gap < stepSize)
		{
			if(debug)
				std::cout << "Returning true, reached the goal" << std::endl;

			start_rrt->tracePath(start_rrt->activeNode, path);
			printf("numNodes: %lu\n", numNodes);

			return true;
		}

		// Update the number of nodes
		numNodes = start_rrt->getSize();
	}

	if(debug)
		printf("numNodes: %lu\n", numNodes);

	// Maximum # of iterations are reached and path is not found - failed.
	return false;
}

/* ********************************************************************************************* */
template <class R>
bool
PathPlanner<R>::planSingleTreeRrt_star(int robot,
   								   	  const std::vector<int> &dofs,
									  const std::vector<Eigen::VectorXd> &start,
									  const Eigen::VectorXd &xGoal,
									  std::list<Eigen::VectorXd> &path)
{


	const bool debug = false;

	// Initialize the RRT
	start_rrt = new R (world, robot, dofs, start, stepSize);

	// Expand the tree until the goal is reached or the max # nodes is passed
	typename R::StepResult result = R::STEP_PROGRESS;
	double smallestGap = std::numeric_limits<double>::infinity();
	size_t numNodes = start_rrt->getSize();

	double randomValue;

	////
	double edgeCost;

	VectorXd xTarget,
			 xNew;

	int xNearest_ind,
		xFrom_ind,
		xNew_ind;



	bool hasNeighbors,
		 hasClosestNeighbor;


	maxNodes = 200;
	start_rrt->ballRadiusMax = 0.20;
	start_rrt->ballRadiusConstant = start_rrt->computeBallRadiusConstant_star();

	int maxIter = 5000;
	int numStates = dofs.size();

	for (int iter = 0; iter <= maxIter; ++iter)
	{
		if ((iter %100) == 0)
		{
			double optcost = 0.0;

			if (start_rrt->goalReached)
			{
				optcost = start_rrt->computeCostFromRoot_star(start_rrt->xGoal_ind);
			}


			std::cout << iter << " : "
					  << " num_nodes: " << numNodes
					  << " r: " << start_rrt->ballRadiusLast
					  << " optcost: " << optcost << std::endl;
		}

		// A.0. Sample a configuration
	    // Get the target node based on the bias

	    randomValue = ((double) rand()) / RAND_MAX;

	    if(!start_rrt->goalReached && randomValue < goalBias)
	    //if(randomValue < goalBias )
	    	xTarget = xGoal;
	    else{
	    	/*
			if (numNodes == maxNodes) // try to connect the goal state at the last iteration
			{
				xTarget = xGoal;
				maxNodes = maxNodes - 1; // this will make sure to exit if goal is not included
			}
			else*/
				xTarget = start_rrt->getRandomConfig();
	    }


		// A.1. Calculate the ball radius constant
		//TODO:compute this for n-dim space
		start_rrt->ballRadiusLast = start_rrt->ballRadiusConstant
            * (pow(log(1+(double)(numNodes))/((double)(numNodes)), 1.0/numStates));

		if (start_rrt->ballRadiusLast >= start_rrt->ballRadiusMax)
			start_rrt->ballRadiusLast = start_rrt->ballRadiusMax;

		// A.2. Find the nearest node
		xNearest_ind = start_rrt->getNearestNeighbor(xTarget);


		// A.3. Extend the node towards the sampled state -- just compute the extended state

		if (!start_rrt->extendTowardsState_star(xNearest_ind, xTarget, xNew, edgeCost))
			continue;

        // A.4. Compute the set of all close nodes around extended_state
		std::vector<int> nearNeighbors;
		hasNeighbors = start_rrt->getNearNeighbors_star(xNew, start_rrt->ballRadiusLast, nearNeighbors);

		//if (nearNeighbors.size() > 20)
		//	std::cout << "neigh size: " << nearNeighbors.size() << " - "
		//				<< "[" << xNew.transpose() << "]" << std::endl;

		// A.5. Pick the node to be extended
		if (hasNeighbors)
		{
			int xMin_ind;

			std::vector<int> reachableNeighbors;

			std::vector<double> edgeCosts;

			//hasClosestNeighbor = start_rrt->findMinStateInSet_star(xNew, nearNeighbors, xMin_ind);
			start_rrt->findMinStateInSet_star(xNew,
											  nearNeighbors,
											  xMin_ind,
											  reachableNeighbors,
											  edgeCosts);

			// A.6. Extend the appropriate node
			if(!start_rrt->extendTowardsState_star(xMin_ind, xNew, xNew_ind, edgeCost))
				continue;

			// A.7. Rewire the tree if possible
			start_rrt->rewireTree_star(xNew_ind, reachableNeighbors, edgeCosts);

			//std::cout << "has neighbors\n";
		}
		else
		{
			// A.6. Extend the appropriate node

			if(!start_rrt->extendTowardsState_star(xNearest_ind, xNew, xNew_ind, edgeCost))
				continue;
		}


		// Check if the goal is reached and create the path, if so

		if ( !start_rrt->goalReached && (xGoal - *(start_rrt->configVector[xNew_ind])).norm() < 0.01)
		{
			std::cout << "Returning true, reached the goal" << std::endl;
			start_rrt->goalReached = true;
			start_rrt->xGoal_ind = xNew_ind;
		}

		numNodes = start_rrt->getSize();
	}

	/*
	Eigen::VectorXd xDiff = xGoal - xNew;
	double epsilon = 0.001;

	if (xDiff.norm() < epsilon) // goal state is reached
	{
		if(debug)
			std::cout << "Returning true, reached the goal" << std::endl;

		start_rrt->tracePath(xNew_ind, path);
		return true;
	}

	if(debug)
		printf("numNodes: %lu\n", numNodes);
	*/

  	  // Maximum # of iterations are reached and path is not found - failed.

	if (start_rrt->goalReached) // goal state is reached
	{
		start_rrt->tracePath(start_rrt->xGoal_ind, path);
		return true;
	}

	return false;
}


template <class R>
bool
PathPlanner<R>::planSingleTreeRrt_sharp(int robot,
   								   	  const std::vector<int> &dofs,
									  const std::vector<Eigen::VectorXd> &start,
									  const Eigen::VectorXd &xGoal,
									  std::list<Eigen::VectorXd> &path)
{
	const bool debug = false;

	// Initialize the RRT
	start_rrt = new R (world, robot, dofs, start, xGoal, stepSize);

	// Expand the tree until the goal is reached or the max # nodes is passed
	typename R::StepResult result = R::STEP_PROGRESS;

	double prob,
		   smallestGap = std::numeric_limits<double>::infinity();

	size_t numNodes = start_rrt->getSize();


	VectorXd xTarget,
			 xNew;

	int xNearest_ind,
		xFrom_ind,
		xNew_ind,
		max_iter,
		numUpdates;


	max_iter = 3000;
	start_rrt->ballRadiusMax = 0.45;
	start_rrt->ballRadiusConstant = start_rrt->computeBallRadiusConstant_star()/15;
	start_rrt->version = 3;

	for (int iter = 0; iter < max_iter; ++iter)
	{
		if ((iter %100) == 0)
		{
			double optcost = 0.0;

			if (start_rrt->goalReached)
			{
				optcost = start_rrt->rhs_[start_rrt->xGoal_ind];
			}
			else
			{
				int xNearest_ind = start_rrt->getNearestNeighbor(xGoal);

				optcost = -(xGoal - *(start_rrt->configVector[xNearest_ind])).norm();
			}

			std::cout << iter << " : "
					  << " num_nodes: " << numNodes
					  << " r: " << start_rrt->ballRadiusLast
					  << " optcost: " << optcost << std::endl;
		}

		// A.0. Sample a configuration
	    // Get the target node based on the bias

	    prob = ((double) rand()) / RAND_MAX;

	    if(!start_rrt->goalReached)
	    {
	     	prob = ((double) rand()) / RAND_MAX;

	    	if (prob < goalBias)
	    		xTarget = xGoal;
	    	else
	    		xTarget = start_rrt->getRandomConfig();
	    }
	    else
	    {
	    	prob = ((double) rand()) / RAND_MAX;

	    	if ( prob < 0.3)
	    		xTarget = start_rrt->getRandomPromisingConfig();
	    	else
	    		xTarget = start_rrt->getRandomConfig();
	    }

	    start_rrt->updateBallRadius_star();

		if (!start_rrt->extend_sharp(xTarget))
			continue;

		// A.7. Replan the tree if possible
		numUpdates = start_rrt->replan_sharp(max_iter);

		/*
		if (numUpdates > 0)
			std::cout << iter << " : "
				  << "num_updates : " << numUpdates << std::endl;
		*/

		numNodes = start_rrt->getSize();
	}

	if (start_rrt->goalReached) // goal state is reached
	{
		start_rrt->tracePath(start_rrt->xGoal_ind, path);
		return true;
	}

	return false;
}


template <class R>
bool
PathPlanner<R>::planSingleTreeRrt(int robot,
									   const std::vector<int> &dofs,
									   const std::vector<Eigen::VectorXd> &start,
									   const Eigen::Vector3d &goal_com,
									   std::list<Eigen::VectorXd> &path)
{

	const bool debug = false;


	// Initialize the RRT

	start_rrt = new R (world, robot, dofs, start, stepSize);

	// Expand the tree until the goal is reached or the max # nodes is passed
	typename R::StepResult result = R::STEP_PROGRESS;
	double smallestGap = std::numeric_limits<double>::infinity();
	size_t numNodes = start_rrt->getSize();

	while(numNodes <= maxNodes)
	{
		// Get the target node based on the bias
		Eigen::VectorXd target;
		double randomValue = ((double) rand()) / RAND_MAX;

		//if(randomValue < goalBias) target = goal;
		//else target = start_rrt->getRandomConfig();
		target = start_rrt->getRandomConfig(goal_com);

		// Based on the method, either attempt to connect to the target directly or take a small step
		connect = 0;
		if(connect)
			start_rrt->connect(target);
		else
			start_rrt->tryStep(target);

		// Check if the goal is reached and create the path, if so
		std::cout << "qnew: [ " << (start_rrt->configVector[start_rrt->activeNode])->transpose() << "]";

		world->getRobot(robot)->setConfig(dofs, *(start_rrt->configVector[start_rrt->activeNode]));

		viewer->DrawGLScene();

		std::cout << " -- com: [ " << world->getRobot(robot)->getWorldCOM().transpose() << " ]\n";
		char che;
		//std::cin >> che;

		Eigen::Vector3d delta_com;

		delta_com = world->getRobot(robot)->getWorldCOM() - goal_com;

		delta_com[2] = 0.0;

		double gap = delta_com.norm();
		double eps = 0.001;

		std::cout << " - gap: " << gap << "\n";

		if(gap < eps)
		{
			if(debug)
				std::cout << "Returning true, reached the goal " << start_rrt->getSize() << std::endl;

			start_rrt->tracePath(start_rrt->activeNode, path);

			std::cout << "Number of nodes: " << start_rrt->getSize() << std::endl;
			return true;
		}

		// Update the number of nodes
		numNodes = start_rrt->getSize();
	}



	if(debug)
		printf("numNodes: %lu\n", numNodes);
		// Maximum # of iterations are reached and path is not found - failed.
	return false;
}

/* ********************************************************************************************* */
template <class R>
bool PathPlanner<R>::planBidirectionalRrt(int robot, const std::vector<int> &dofs,
    const std::vector<Eigen::VectorXd> &start, const std::vector<Eigen::VectorXd> &goal,
    std::list<Eigen::VectorXd> &path) {

  const bool debug = false;

  // Initialize both the start and goal RRTs.
  // NOTE: We use the pointers for the RRTs to swap their roles in extending towards a target
  // (random or goal) node.
  start_rrt = new R(world, robot, dofs, start, stepSize);
  goal_rrt = new R(world, robot, dofs, goal, stepSize);
  R* rrt1 = start_rrt;
  R* rrt2 = goal_rrt;

  // Expand the tree until the trees meet or the max # nodes is passed
  double smallestGap = std::numeric_limits<double>::infinity();
  size_t numNodes = rrt1->getSize() + rrt2->getSize();
  while(numNodes < maxNodes) {

    // Swap the roles of the two RRTs. Remember, the first rrt reaches out to a target node and
    // creates a new node and the second rrt reaches to _the new node_.
    R* temp = rrt1;
    rrt1 = rrt2;
    rrt2 = temp;

     // Get the target node based on the bias
    Eigen::VectorXd target;
    double randomValue = ((double) rand()) / RAND_MAX;
    if(randomValue < goalBias) target = goal[0];
    else target = rrt1->getRandomConfig();

    // Based on the method, rrt1 either attempt to connect to the target directly or takes a step
    if(connect) rrt1->connect(target);
    else rrt1->tryStep(target);

    // rrt2 uses the last added node of rrt1 as a target and reaches out to it (connect or step)
    // NOTE: If a node was not added, the nearest neighbor to the random node in rrt1 is used.
    // NOTE: connect(x) and tryStep(x) functions return true if rrt2 can add the given node
    // in the tree. In this case, this would imply that the two trees meet.
    bool treesMet = false;
    const Eigen::VectorXd& rrt2target = *(rrt1->configVector[rrt1->activeNode]);
    if(connect) treesMet = rrt2->connect(rrt2target);
    else treesMet = (rrt2->tryStep(rrt2target) == R::STEP_REACHED);

    // Check if the trees have met and create the path, if so.
    if(treesMet) {
      start_rrt->tracePath(start_rrt->activeNode, path);
      goal_rrt->tracePath(goal_rrt->activeNode, path, true);

      std::cout << "number of nodes: " << start_rrt->getSize() << std::endl;
      return true;
    }

    // Update the number of nodes in the two trees
    numNodes = rrt1->getSize() + rrt2->getSize();

    // Print the gap between the trees in debug mode
    if(debug) {
      double gap = rrt2->getGap(*(rrt1->configVector[rrt1->activeNode]));
      if(gap < smallestGap) {
        smallestGap = gap;
        std::cout << "Gap: " << smallestGap << "  Sizes: " << start_rrt->configVector.size()
          << "/" << goal_rrt->configVector.size() << std::endl;
      }
    }
  }

  // Maximum # of iterations are reached and path is not found - failed.
  return false;
}

}  //< End of namespace
