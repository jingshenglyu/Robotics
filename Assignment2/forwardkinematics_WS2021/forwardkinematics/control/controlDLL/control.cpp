// controlDLL.cpp : Defines the entry point for the DLL application.
//
#include "servo.h"
#include "param.h"
#include "control.h"
//#include "UiAgent.h"
#include "PrVector.h"
#include "PrMatrix.h"
#include "Utils.h" // misc. utility functions, such as toRad, toDeg, etc.
#include <math.h>
#include <algorithm>
using std::min;
using std::max;


struct CubicSpline
{
	double t0, tf;
	PrVector a0, a1, a2, a3; 
};
CubicSpline spline;

// Compute total trajectory length
void computeTf (GlobalVariables& gv) 
{
   // gv.tau = -gv.G - gv.kp*(gv.q - gv.qd) - gv.kv * gv.dq;
   float res[gv.dof];
   for (int i=0;i<gv.dof;i++){
      res[i]=gv.dqmax[i]/gv.ddqmax[i];
   }
   float *tf_relative = std::min_element(res, res + gv.dof);
   
   spline.t0=gv.curTime;
   spline.tf=spline.t0+*tf_relative;

   spline.a0=gv.q;
   // spline.a1=0; //initialised to 0?
   spline.a2=3*(gv.qd-gv.q)/(pow(*tf_relative,2));
   spline.a3=-2*(gv.qd-gv.q)/(pow(*tf_relative,3));
}

// double computeTf(GlobalVariables& gv)
// {
// 	// B2
//    double tf=0;
// 	for(int i=0;i<3;i++)
// 	{
// 		tf =max(tf, max(sqrt(6*abs(gv.q[i]-gv.qd[i])/gv.ddqmax[i]),3*abs(gv.q[i]-gv.qd[i])/(2*gv.dqmax[i])));
// 	}
	
// 	// return the tf and curent time
// 	return double(tf+gv.curTime);
// }

void PrintDebug(GlobalVariables& gv);

// *******************************************************************
// Initialization functions
// *******************************************************************

void InitControl(GlobalVariables& gv) 
{
   // This code runs before the first servo loop
}
 
void PreprocessControl(GlobalVariables& gv)  
{
   // This code runs on every servo loop, just before the control law
   
   
  // 
    if ((gv.dof == 3) || (gv.dof == 6)) {

        //get the correct joint angles depending on the current mode:
        double q1,q2,q3;
        if (gv.dof == 3) {
            q1 = gv.q[0];
            q2 = gv.q[1];
            q3 = gv.q[2];
        } else if (gv.dof == 6) {
            q1 = gv.q[1];
            q2 = gv.q[2];
            q3 = gv.q[4];
        }

        PrVector3 g123 = PrVector3(0,0,0); //Variable that holds the torque exerted by gravity for each joint

        //Compute g123 here!        
         double r1, r2, r3, l1, l2, l3, m1, m2, m3, g;
         r1 = R2; 
         r2 = 0.189738;
         r3 = R6;
         l1 = L2;
         l2 = L3;
         l3 = L6;
         m1 = M2;
         m2 = M3+M4+M5;
         m3 = M6;
         g = -9.81;
         double c1 = cos(q1);
         double c12 = -cos(q1 + q2 + 3.14/2);
         double c123 = -cos( q1 + q2 + 3.14/2+ q3);

         g123[0] = -(m1*(r1*c1)+m2*(l1*c1+r2*c12)+m3*(l1*c1+l2*c12+r3*c123))*g;
         g123[1] = -(m2*(r2*c12)+m3*(l2*c12+r3*c123))*g;
         g123[2] = -(m3*r3*c123)*g;

        //maps the torques to the right joint indices depending on the current mode:
        if (gv.dof == 3) {
            gv.G[0] = g123[0];
            gv.G[1] = g123[1];
            gv.G[2] = g123[2];
        } else if (gv.dof == 6) {
            gv.G[1] = g123[0];
            gv.G[2] = g123[1];
            gv.G[4] = g123[2];
        }
        // printing example, do not leave print inthe handed in solution 
//        printVariable(g123, "g123");
    } else {
        gv.G = PrVector(gv.G.size());
    }   
}

void PostprocessControl(GlobalVariables& gv) 
{
   // This code runs on every servo loop, just after the control law
}

void initFloatControl(GlobalVariables& gv) 
{
    // Control Initialization Code Here
}

void initOpenControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNjholdControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initJholdControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNjmoveControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initJmoveControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNjgotoControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initJgotoControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNjtrackControl(GlobalVariables& gv) 
{
   computeTf(gv);
}
// {
// 	// Control Initialization Code Here

//    // computation of time
//    spline.t0 = gv.curTime;
//    spline.tf = computeTf(gv);

//    //compute the spline parameters, process of the computation is on the doc.
//    spline.a0 = gv.q;
//    spline.a1 = gv.dq;
//    spline.a2 = 3*(gv.qd-gv.q)/pow(spline.tf-spline.t0,2) - 2*gv.dq/(spline.tf-spline.t0)- gv.dqd/(spline.tf-spline.t0);
//    spline.a3 = -2*(gv.qd-gv.q)/pow(spline.tf-spline.t0,3) + (gv.dqd + gv.dq)/pow(spline.tf-spline.t0,2);

// }

void initJtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNxtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initXtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initNholdControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initHoldControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNgotoControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initGotoControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initNtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initTrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initPfmoveControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initLineControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initProj1Control(GlobalVariables& gv) 
{
	// Control Initialization Code Here

   // desired joint configuration in [rad]
	gv.qd[0] = 0.096;
	gv.qd[1] = 0.097;
	gv.qd[2] = -1.061;

   initNjtrackControl(gv);
   
}

void initProj2Control(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initProj3Control(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}


// *******************************************************************
// Control laws
// *******************************************************************

void noControl(GlobalVariables& gv)
{
}

void floatControl(GlobalVariables& gv)
{   
	gv.tau = -gv.G;
   // this only works on the real robot unless the function is changed to use cout
	// the handed in solution must not contain any printouts
//	PrintDebug(gv);
}

void openControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void njholdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}  

void jholdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void njmoveControl(GlobalVariables& gv)
{
   gv.tau=-gv.kp*(gv.q-gv.qd);

   //floatControl(gv);  // Remove this line when you implement this controller
}

void jmoveControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void njgotoControl(GlobalVariables& gv) 
{	
   gv.tau = - gv.G - gv.kp * (gv.q - gv.qd);
   //floatControl(gv);  // Remove this line when you implement this controller
}

void jgotoControl(GlobalVariables& gv) 
{
   gv.tau = -gv.G - gv.kp*(gv.q - gv.qd) - gv.kv * gv.dq;
   //floatControl(gv);  // Remove this line when you implement this controller
}

void njtrackControl(GlobalVariables& gv) 
{
   double t=gv.curTime-spline.t0;
	if(gv.curTime < spline.tf)
	{
		// updating the desired position and velocity
		gv.qd = spline.a0 + t*spline.a1 + pow(t,2)*spline.a2 + pow(t,3)*spline.a3;
		gv.dqd = 2*t*spline.a2 + 3*pow(t,2)*spline.a3;  
      gv.tau = -gv.G - gv.kp*(gv.q - gv.qd) - gv.kv*(gv.dq-gv.dqd);
	}
	else
	{
		floatControl(gv);  // Remove this line when you implement this controller
   }
}

void jtrackControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void nxtrackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void xtrackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void nholdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void holdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void ngotoControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void gotoControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void ntrackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void trackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void pfmoveControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void lineControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void proj1Control(GlobalVariables& gv) 
{
   njtrackControl(gv);  // Remove this line when you implement proj1Control
}

void proj2Control(GlobalVariables& gv) 
{
   PrVector3 force = PrVector3(0,0,0);
   double t_init;
   double ang_vel=2.0*3.14/5.0;
   double t=gv.curTime-t_init;
   double angle=ang_vel*t;
    
    
   gv.xd[0]=0.45+0.20*sin(angle);
   gv.xd[1]=0.60+0.20*cos(angle);
   gv.xd[2]=0.0;
    
   gv.dxd[0]=0.20*cos(angle)*ang_vel;
   gv.dxd[1]= -0.20*sin(angle)*ang_vel;
   gv.dxd[2]=0.0;

   force=gv.kp*(gv.xd-gv.x)+gv.kv*(gv.dxd-gv.dx);    
   gv.tau=gv.Jtranspose*force-gv.G;
   
   
   // Remove this line when you implement proj2Control
}

void proj3Control(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement proj3Control
}

// *******************************************************************
// Debug function
// *******************************************************************

void PrintDebug(GlobalVariables& gv)
{
   // Replace this code with any debug information you'd like to get
   // when you type "pdebug" at the prompt.
   printf( "This sample code prints the torque and mass\n" );
   gv.tau.display( "tau" );
   gv.A.display( "A" );
}

#ifdef WIN32
// *******************************************************************
// XPrintf(): Replacement for printf() which calls ui->VDisplay()
// whenever the ui object is available.  See utility/XPrintf.h.
// *******************************************************************

int XPrintf( const char* fmt, ... )
{
  int returnValue;
  va_list argptr;
  va_start( argptr, fmt );

  returnValue = vprintf( fmt, argptr );

  va_end( argptr );
  return returnValue;
}
#endif //#ifdef WIN32

/********************************************************

END OF DEFAULT STUDENT FILE 

ADD HERE ALL STUDENT DEFINED AND AUX FUNCTIONS 

*******************************************************/
