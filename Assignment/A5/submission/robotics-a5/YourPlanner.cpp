#include "YourPlanner.h"
#include <rl/plan/SimpleModel.h>
#include <rl/plan/Sampler.h>
#include <iostream>
#include <rl/plan/Verifier.h>
#include <rl/plan/Viewer.h>
#include <boost/make_shared.hpp>


// constructor
YourPlanner::YourPlanner() :
	RrtConConBase(),
	// Added for addaptive RRT
	alpha(0.05f),
	lower(2.0f),
	radius(20.0f)
{
	// Variable for Find NN
	this->deltaDistance = 1.0;
	this->exhaustedThresh = 100;
	this->clusterDistanceThresh = 3.0f;
}


// destructor
YourPlanner::~YourPlanner() {}


// getter for the name of this planner
::std::string YourPlanner::getName() const { return "Your Planner"; }


// generate random a random configuration (sample)
void YourPlanner::choose(::rl::math::Vector& chosen)
{
	// assign a random sample to the chosen vector
	chosen = this->sampler->generate();
}


// try to connect the nearest neighbor with the sample considering collisions inbetween
RrtConConBase::Vertex YourPlanner::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
	// get distance from the nearest neighbor to the sample
	::rl::math::Real distance = nearest.second;

	// variable for the to be used step size
	::rl::math::Real step = distance;

	// default criteria for the extension loop
	bool reached = false;

	// check whether distance of the nearest neighbor to the sample is smaller or equal to the given configuration step size
	if (step <= this->delta)
	{
		// set reached true to break the loop as no further extension towards the sample is necessary - do one iteration to get to the position of the sample/chosen
		reached = true;
	}
	else
	{
		// use given configuration step size
		step = this->delta;
	}

	// vector pointer to store sample information for further extension steps
	::rl::plan::VectorPtr last = ::std::make_shared< ::rl::math::Vector >(this->model->getDof());

	// move "last" along the line q<->chosen by distance "step / distance"
	this->model->interpolate(*tree[nearest.first].q, chosen, step / distance, *last);
	this->model->setPosition(*last);
	this->model->updateFrames();

	// check whether extended step collides
	if (this->model->isColliding())
	{
		// increase extension fails
		tree[nearest.first].exhaustCount++;

		// stop the extension in case of collision
		return NULL;
	}

	// vector to store sample information for further extension steps
	::rl::math::Vector next(this->model->getDof());

	// compute further extension steps
	while (!reached)
	{
		// recompute step size and distance of the nearest neighbor and the sample to compute an extension step
		distance = this->model->distance(*last, chosen);
		step = distance;

		// check whether distance of the nearest neighbor to the sample is smaller or equal to the given configuration step size
		if (step <= this->delta)
		{
			// set reached true to break the loop as no further extension towards the sample is necessary - do one iteration to get to the position of the sample/chosen
			reached = true;
		}
		else
		{
			// use given configuration step size
			step = this->delta;
		}

		// interpolate a step along the line "last<->sample" by the distance "step / distance"
		this->model->interpolate(*last, chosen, step / distance, next);
		this->model->setPosition(next);
		this->model->updateFrames();

		// check whether extended step collides
		if (this->model->isColliding())
		{
			// increase extension fails
			tree[nearest.first].exhaustCount++;

			// stop the extension in case of collision
			break;
		}

		// set computed extension as new nearest neighbor for further steps in direction of the sample
		*last = next;
	}

	// "last" now points to the vertex where the connect step collided with the environment
	// add extended sample to the tree and return the sample
	Vertex connected = this->addVertex(tree, last);
	this->addEdge(nearest.first, connected, tree);
	return connected;
}


// Added for addaptive RRT
enum Solver { ADD_RRT_CON_CON, RRT_CON_CON_BASE };

bool
YourPlanner::solve()
{
	// kinodynamic planning
	// 1. generate random sample
	// 2. find closest node in selected tree (init/goal)
	// 3. expand selected tree towards the sample
	// 4. add node from expansion to the selected tree and try to connect both trees with each other

	Solver mode = ADD_RRT_CON_CON;

	switch (mode)
	{
	case ADD_RRT_CON_CON:
		return this->addaptiveSolve();

	case RRT_CON_CON_BASE:
		return RrtConConBase::solve();

	default:
		return 0;
	}

}

// Implementation of the Find NN Algorithm
RrtConConBase::Neighbor
YourPlanner::nearest(const Tree& tree, const ::rl::math::Vector& chosen)
{

	//create an empty pair <Vertex, distance> to return
	Neighbor p(Vertex(), (::std::numeric_limits<::rl::math::Real>::max)());

	//create an empty pair <Vertex, distance> as fallback, for the case that every vertex in that cluster is exhausted
	Neighbor p_fallback(Vertex(), (::std::numeric_limits<::rl::math::Real>::max)());


	//vector of vertices for storing nearest cluster
	std::vector<RrtConConBase::Vertex* >* nearestCluster;
	//variable to hold the current minimum distance value
	::rl::math::Real minCentroidDistance = ::std::numeric_limits<::rl::math::Real>::max();


	//check which tree we're working with
	if (&tree == &this->tree[0])
	{
		//iterate through clusters of tree A
		for (auto& cluster : this->vertexClusterA)
		{
			//calculate distance between chosen and cluster centroid
			::rl::math::Real d = this->model->transformedDistance(chosen, *tree[*cluster.first].q);

			//check distance and update nearest cluster, if needed
			if (d < minCentroidDistance)
			{
				minCentroidDistance = d;
				nearestCluster = cluster.second;
			}
		}
	}
	else
	{
		//same if we're looking at tree B
		for (auto& cluster : this->vertexClusterB)
		{
			::rl::math::Real d = this->model->transformedDistance(chosen, *tree[*cluster.first].q);

			if (d < minCentroidDistance)
			{
				minCentroidDistance = d;
				nearestCluster = cluster.second;
			}
		}
	}

	//iterate through all vertices of chosen nearest cluster
	for (auto& vertex : *nearestCluster)
	{
		//calculate distance between chosen and vertices of the nearest cluster
		::rl::math::Real d = this->model->transformedDistance(chosen, *tree[*vertex].q);

		//check if distance is closer as current best
		if (d < p.second)
		{
			//check if vertex within exhaustion threshold
			if (tree[*vertex].exhaustCount <= this->exhaustedThresh)
			{
				p.first = *vertex;
				p.second = d;
			}
			// else check if better than current fallback
			else if (d < p_fallback.second)
			{
				p_fallback.first = *vertex;
				p_fallback.second = d;
			}
		}
	}

	//check if vertex within in threshold was not found, than use fallback
	if (p.second == ::std::numeric_limits<::rl::math::Real>::max())
	{
		p.first = p_fallback.first;
		p.second = p_fallback.second;
	}

	// Compute the square root of distance
	p.second = this->model->inverseOfTransformedDistance(p.second);

	return p;
}

// Implementation of addaptive RRT
bool YourPlanner::addaptiveSolve()
{
	// start a timer to measure time needed for the road to be found
	this->time = ::std::chrono::steady_clock::now();

	// set the time limit for the planer to find a path to 120 seconds -
	// TurtorialPlanSystem.cpp (accidently) sets it to 1200
	this->duration = ::std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<float>(120.0));

	// define the roots of both trees
	this->begin[0] =
		this->addVertex(this->tree[0], ::std::make_shared<::rl::math::Vector>(
			*this->start)); // Changed ::boost to ::std
	this->begin[1] =
		this->addVertex(this->tree[1], ::std::make_shared<::rl::math::Vector>(
			*this->goal)); // Changed ::boost to ::std

	Tree* a = &this->tree[0];
	Tree* b = &this->tree[1];

	// define a vector for the random samples
	::rl::math::Vector chosen(this->model->getDof());

	// try to find a path within the time limit
	while ((::std::chrono::steady_clock::now() - this->time) < this->duration)
	{
		// first grow tree a and then try to connect to tree b, then swap roles and
		// grow tree b and try to connect to tree a - ensure if time is up, you end
		// up with selected tree being tree a
		for (::std::size_t j = 0; j < 2; ++j)
		{
			Neighbor aNearest;

			do {
				// sample a random configuration
				this->choose(chosen);

				// find the nearest neighbour in the tree
				aNearest = this->nearest(*a, chosen);
			} while (aNearest.second > (*a)[aNearest.first].radius);

			// try to connect the nearest neighbour to a configuration in direction of
			// the sample to tree a
			Vertex aConnected = this->connect(*a, aNearest, chosen);

			// if a new node was added to tree a
			if (NULL != aConnected)
			{
				// increase radius of the vertex
				if ((*a)[aNearest.first].radius <
					::std::numeric_limits<::rl::math::Real>::max())
				{
					(*a)[aNearest.first].radius *= (1.0f + this->alpha);
				}

				// try to connect the the newly added configuration to tree b
				Neighbor bNearest = this->nearest(*b, chosen);
				Vertex bConnected = this->connect(*b, bNearest, *(*a)[aConnected].q);

				// if a new node was added to tree b
				if (NULL != bConnected)
				{
					// check if both trees can be connected with each other
					if (this->areEqual(*(*a)[aConnected].q, *(*b)[bConnected].q))
					{
						this->end[0] = &this->tree[0] == a ? aConnected : bConnected;
						this->end[1] = &this->tree[1] == b ? bConnected : aConnected;

						// return as a path is found thereby
						return true;
					}
				}
			}
			else
			{
				// decrease radius of the vertex
				if ((*a)[aNearest.first].radius <
					::std::numeric_limits<::rl::math::Real>::max())
				{
					(*a)[aNearest.first].radius *= (1.0f - this->alpha);
					(*a)[aNearest.first].radius =
						::std::max(this->lower, (*a)[aNearest.first].radius);
				}
				else
				{
					(*a)[aNearest.first].radius = this->radius;
				}
			}

			// swap the roles of tree a and tree b
			::std::swap(a, b);
		}
	}

	// return false if time limit is exceeded
	return false;
}

// AddVertex function for find NN
RrtConConBase::Vertex
YourPlanner::addVertex(RrtConConBase::Tree& tree, const ::rl::plan::VectorPtr& q)
{
	//use base addVertex functionality
	RrtConConBase::Vertex addedVertex = RrtConConBase::addVertex(tree, q);

	// variable for vertex cluster
	auto* vertexClusterVector = new std::vector<RrtConConBase::Vertex* >();

	// copy newly added vertex to have memory control
	auto* clusterVertex = new RrtConConBase::Vertex(addedVertex);

	//check current tree
	if (&tree == &this->tree[0])
	{
		//check if there arent clusters yet - create new cluster then
		if (this->vertexClusterA.empty())
		{
			vertexClusterVector->push_back(clusterVertex);
			this->vertexClusterA[clusterVertex] = vertexClusterVector;
		}
		else
		{
			// remember if, vertex was added to a cluster
			bool inserted = false;
			// iterate through all clusters
			for (auto& cluster : this->vertexClusterA)
			{
				// calculate distance to cluster centroid
				::rl::math::Real distance = this->model->transformedDistance(*tree[*cluster.first].q, *q);
				//check if distance within threshold
				if (distance <= this->clusterDistanceThresh)
				{
					//add vertex to the cluster and stop
					vertexClusterVector = cluster.second;
					vertexClusterVector->push_back(clusterVertex);
					this->vertexClusterA[cluster.first] = vertexClusterVector;
					inserted = true;
					break;
				}
			}

			// if not matching cluster was found, create new cluster
			if (!inserted)
			{
				vertexClusterVector->push_back(clusterVertex);
				this->vertexClusterA[clusterVertex] = vertexClusterVector;
			}
		}
	}
	else
	{
		//same if tree B
		if (this->vertexClusterB.empty())
		{
			vertexClusterVector->push_back(clusterVertex);
			this->vertexClusterB[clusterVertex] = vertexClusterVector;
		}
		else
		{
			bool inserted = false;
			for (auto& cluster : this->vertexClusterB) {
				::rl::math::Real distance = this->model->transformedDistance(*tree[*cluster.first].q, *q);
				if (distance <= this->clusterDistanceThresh) {
					vertexClusterVector = cluster.second;
					vertexClusterVector->push_back(clusterVertex);
					this->vertexClusterB[cluster.first] = vertexClusterVector;
					inserted = true;
					break;
				}
			}

			if (!inserted)
			{
				vertexClusterVector->push_back(clusterVertex);
				this->vertexClusterB[clusterVertex] = vertexClusterVector;
			}
		}
	}

	return addedVertex;
}
