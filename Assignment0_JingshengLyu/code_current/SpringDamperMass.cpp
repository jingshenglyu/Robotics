

/*
 * @Author       : Jingsheng Lyu
 * @Date         : 2020-11-05 20:36:14
 * @LastEditors  : Jingsheng Lyu
 * @LastEditTime : 2020-11-05 22:30:35
 * @FilePath     : /JingshengLyu_Assignment0/code_current/SpringDamperMass.cpp
 * @Github       : https://github.com/jingshenglyu
 * @Web          : https://jingshenglyu.github.io/
 * @E-Mail       : jingshenglyu@gmail.com
 */
#include "SpringDamperMass.h"

// TODO
// Define your methods here

int SpringDamperMass::step()
{
    //Step by step on equation
    velocity = velocity - (damping_coeff/MASS) * velocity - (SPRING_CONST/MASS) * (position - position_eqm);
    position = position + velocity;
    time = time + 1;

    //record state into traj
    Vec2d state;
    state.x = position;
    state.y = velocity;
    traj.push_back(state);

    return time;
}
