

/*
 * @Author       : Jingsheng Lyu
 * @Date         : 2020-11-05 20:36:14
 * @LastEditors  : Jingsheng Lyu
 * @LastEditTime : 2020-11-05 22:20:52
 * @FilePath     : /JingshengLyu_Assignment0/code_current/SpringMass.cpp
 * @Github       : https://github.com/jingshenglyu
 * @Web          : https://jingshenglyu.github.io/
 * @E-Mail       : jingshenglyu@gmail.com
 */
#include "SpringMass.h"

#include <ostream>
#include <fstream>

using namespace std;

// define gravity constant
const double SpringMass::GRAVITY = 10;
const double SpringMass::SPRING_CONST = 7;
const double SpringMass::MASS = 30;

// TODO SpringMass constructor
  
SpringMass::SpringMass(double pos_init, double vel_init, double pos_eqm, double vel_eqm)
{ 
    //init 
    position = pos_init;
    velocity = vel_init;

    position_eqm = pos_eqm;
    velocity_eqm = vel_eqm;

    time = 0;

    //record state into <vector>traj
    Vec2d state;
    state.x = position;
    state.y = velocity;
    traj.push_back(state);

}

SpringMass::~SpringMass() {
  // TODO clean up stuff from HEAP  
}

// TODO SpringMass simulation step
int SpringMass::step() {

    //Step by step simulation
    velocity = velocity - (SPRING_CONST/MASS) * (position - position_eqm);
    position = position + velocity;
    time = time + 1;

    //record state into <vector>traj
    Vec2d state;
    state.x = position;
    state.y = velocity;
    traj.push_back(state);
    return time;
}

// TODO SpringMass configuration getter
bool SpringMass::getConfiguration(int t, Vec2d& state) const {
  if (t <= time)
  {
    //take the state from <vector>traj
    Vec2d state_t = traj[t];
    state.x = state_t.x;
    state.y = state_t.y;
    return true;
  }

  else
  {
    return false;
  }

}

// TODO SpringMass current simulation time getter
int SpringMass::getCurrentSimulationTime() const {

    return time;  //return the current time
}



