/*
 * @Author: Jingsheng Lyu
 * @Date: 2021-02-28 09:51:22
 * @LastEditors: Jingsheng Lyu
 * @LastEditTime: 2021-02-28 10:04:57
 * @FilePath: /robotics-a5-rlysander/YourPlanner.h
 * @Github: https://github.com/jingshenglyu
 * @Web: https://jingshenglyu.github.io/
 * @E-Mail: jingshenglyu@gmail.com
 */
#ifndef _YOUR_PLANNER_H_
#define _YOUR_PLANNER_H_

#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

#include "RrtConConBase.h"

using namespace ::rl::plan;

/**
*	The implementation of your planner.
*	modify any of the existing methods to improve planning performance.
*/
class YourPlanner : public RrtConConBase
{
public:
  YourPlanner();

  virtual ~YourPlanner();

  virtual ::std::string getName() const;

  bool solve();

  // Added for addaptive RRT
  bool addaptiveSolve();

  // Added for addaptive RRT
  /** Radius expansion factor. */
  ::rl::math::Real alpha;
  
  /** Lower bound for radius. */
  ::rl::math::Real lower;
  
  /** Initial vertex radius. */
  ::rl::math::Real radius;


protected:
  void choose(::rl::math::Vector& chosen);
  RrtConConBase::Vertex connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);
  RrtConConBase::Neighbor nearest(const Tree& tree, const ::rl::math::Vector& chosen);
  RrtConConBase::Vertex addVertex(Tree& tree, const ::rl::plan::VectorPtr& q);


private:

  // For find NN
  double deltaDistance;
  int exhaustedThresh;
	
	// The variable used to assign a vertex to the threshold distance of a cluster.
  ::rl::math::Real clusterDistanceThresh;

  // Represents maps for tree A and B , which hold <centroid, vector of vertices>
  std::map<RrtConConBase::Vertex*, std::vector<RrtConConBase::Vertex* >* > vertexClusterA;
  std::map<RrtConConBase::Vertex*, std::vector<RrtConConBase::Vertex* >* > vertexClusterB;

};

#endif // _YOUR_PLANNER_H_
