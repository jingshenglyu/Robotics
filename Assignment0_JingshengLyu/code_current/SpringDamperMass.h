

/*
 * @Author       : Jingsheng Lyu
 * @Date         : 2020-11-05 20:36:14
 * @LastEditors  : Jingsheng Lyu
 * @LastEditTime : 2020-11-05 22:24:28
 * @FilePath     : /JingshengLyu_Assignment0/code_current/SpringDamperMass.h
 * @Github       : https://github.com/jingshenglyu
 * @Web          : https://jingshenglyu.github.io/
 * @E-Mail       : jingshenglyu@gmail.com
 */
#ifndef SPRING_DAMPER_MASS__H__
#define SPRING_DAMPER_MASS__H__

#include "SpringMass.h"

class SpringDamperMass : public SpringMass {
public: 

  /**
   * @brief Initialize object initial position and velocity,  
   * object position and velocity when the spring is unstretched (equilibrium state)
   * and the damping coefficient
   */
  SpringDamperMass(double pos_init, double vel_init, double pos_eqm, double vel_eqm,
           double _damping_coeff) : SpringMass(pos_init, vel_init, pos_eqm, vel_eqm)
  {
    //TODO
    damping_coeff = _damping_coeff;
  }

  // TODO define your methods here
  int step();

private:
  /**
   * Damping coefficient for damper
   */ 
  double damping_coeff;
  
  // TODO define additional members (if necessary) here

};


#endif 
