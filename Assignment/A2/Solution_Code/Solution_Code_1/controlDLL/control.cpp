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
#include <cmath>

using std::min;
using std::max;
using std::sin;
using std::cos;


void PrintDebug(GlobalVariables& gv);

// *******************************************************************
// Initialization functions
// *******************************************************************

void printV(const char* label, double val)
{
   fprintf(stdout,"%s %f\n", label, val);
}

void printV(const char* label, PrVector& val)
{ 
   fprintf(stdout,"\n%s\t:\n",label);
   for (int i = 0; i < val.size(); i++)
   { 
      fprintf(stdout,"[%d]:\t",i);    
      printV("",val[i]);
   }

}
void printV(const char* label, PrMatrix& val)
{ 
   fprintf(stdout,"\n%s\t:\n",label);
   for (int i = 0; i < val.column(); i++)
   { 
      for(int j = 0; j < val.row(); j++){
         fprintf(stdout,"%f\t",val[i][j]); 
      }       
      fprintf(stdout,"\n");  
   }

}

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
         double r1, r2, r3, l1, l2, l3, 
                  m1, m2, m3, g, c1, c12, c123;

         r1 = R2;    r2 = 0.189738;    r3 = R6;
         l1 = L2;    l2 = L3;          l3 = L6;
         m1 = M2;    m2 = M3 + M4 + M5;m3 = M6;
         g = -9.81;
         c1 = cos(q1);   c12 = -cos(q1 + q2 + 3.14/2);     c123 = -cos(q1 + q2 + 3.14/2 + q3);    
         // printV("gv.q",gv.q);
         // printV("c1",c1);

         g123[0] = -(r1*c1*m1 + m2*(l1*c1 + r2*c12) + m3*(l1*c1 + l2*c12 + r3*c123))*g;
         g123[1] = -(r2*c12*m2 + m3*(l2*c12 + r3*c123))*g;
         g123[2] = -r3*c123*m3*g;

        if (gv.dof == 3) {
            gv.G[0] = g123[0];
            gv.G[1] = g123[1];
            gv.G[2] = g123[2];
        } else if (gv.dof == 6) {
            gv.G[1] = g123[0];
            gv.G[2] = g123[1];
            gv.G[4] = g123[2];
        }

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
    printV("gv.J",gv.J);
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

struct CubicSpline {
   double t0 , tf ;
   PrVector a0 , a1 , a2 , a3 ;
};

CubicSpline spline;
// Compute total trajectory time interval
void computeTf (GlobalVariables& gv) 
{
   // gv.tau = -gv.G - gv.kp*(gv.q - gv.qd) - gv.kv * gv.dq;
   float res[gv.dof];
   for (int i=0;i<gv.dof;i++){
      res[i]=4*gv.dqmax[i]/gv.ddqmax[i];  //which time will satisfy all 3 joints constraints
   }
   float *tf_relative = std::max_element(res, res + gv.dof);
   
   spline.t0=gv.curTime;
   spline.tf=spline.t0+*tf_relative;

   spline.a0=gv.q;
   // spline.a1=0; //initialised to 0?
   spline.a2=3*(gv.qd-gv.q)/(pow(*tf_relative,2));
   spline.a3=-2*(gv.qd-gv.q)/(pow(*tf_relative,3));
}

void initNjtrackControl(GlobalVariables& gv) 
{
   computeTf(gv);
}

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
   if (gv.dof == 3) 
   {
      gv.qd[0]= 0.096;
      gv.qd[1]= 0.967;
      gv.qd[2]= -1.061;
   } else if (gv.dof == 6) {
      gv.qd[1]= 0.096;
      gv.qd[2]= 0.967;
      gv.qd[4]= -1.061;
   }
   initNjtrackControl(gv);

	// Control Initialization Code Here
}

struct Circle {
   void step(double t){
      
      x[0]= 0.6 + 0.2*cos(dBetta*(t-t0));
      x[1]= 0.35 + 0.2*sin(dBetta*(t-t0));
      x[2]= 0; //Specifying the orientation of end frame
      dx[0]= -dBetta*0.2*sin(dBetta*(t-t0));
      dx[1]= dBetta*0.2*cos(dBetta*(t-t0));
      dx[2]= 0; //Specifying the orientation of end frame 
   }
   
   double t0;
   PrVector3 x,dx;
   private:
   static constexpr double dBetta=2*3.14/5;
   
};

struct Circle_acc {
   void step(double t){
      
      if ((t-t0)<=td){              // 0 < t <= td
         Pfi = ddBetta*pow((t-t0),2)/2;
         dPfi= ddBetta*(t-t0);
      } else if ((t-t0)<=3*td){     // td < t <= 3*td
         Pfi = 3.14 + dBetta*(t-t0);
         dPfi= dBetta;
      } else if ((t-t0)<=4*td){     // 3*td < t <= 4*td
         Pfi = 3.14 + (3.14 - ddBetta*pow((4*td-(t-t0)),2)/2);
         dPfi= -ddBetta*(4*td-(t-t0));
      }
      
      x[0]= 0.6 + 0.2*cos(Pfi);
      x[1]= 0.35 + 0.2*sin(Pfi);
      x[2]= 0;
      dx[0]= -dPfi*0.2*sin(Pfi);
      dx[1]= dPfi*0.2*cos(Pfi);
      dx[2]= 0;  
   }
   void calc_tf(){
      tf=t0+4*td;
   }

   double t0,tf;
   PrVector3 x,dx;
   private:
   static constexpr double ddBetta=2*3.14/25;
   static constexpr double dBetta=2*3.14/5;
   static constexpr double td = 5;
   double Pfi, dPfi;

   
};


Circle circle;
Circle_acc circle_acc;

void initProj2Control(GlobalVariables& gv) 
{
   circle.t0=gv.curTime;

	// Control Initialization Code Here
}

void initProj3Control(GlobalVariables& gv) 
{
   circle_acc.t0=gv.curTime;
   circle_acc.calc_tf();
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
   gv.tau=-gv.G;
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
}

void jmoveControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void njgotoControl(GlobalVariables& gv) 
{	
   gv.tau = -gv.G - gv.kp*(gv.q-gv.qd);
}

void jgotoControl(GlobalVariables& gv) 
{
   
   gv.tau = -gv.G - gv.kp*(gv.q-gv.qd) - gv.kv*gv.dq;
}

void njtrackControl(GlobalVariables& gv) 
{
   if (gv.curTime<=spline.tf){
      PrVector qd,dqd;
      qd=spline.a0+spline.a2*pow(gv.curTime-spline.t0,2)+spline.a3*pow(gv.curTime-spline.t0,3);
      dqd=2*spline.a2*(gv.curTime-spline.t0)+3*spline.a3*pow(gv.curTime-spline.t0,2);
      gv.tau = -gv.G - gv.kp*(gv.q - qd) - gv.kv * (gv.dq-dqd);
   } else {
      floatControl(gv); 
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
   PrVector F;
   circle.step(gv.curTime);
   F=-gv.kp*(gv.x-circle.x)-gv.kv*(gv.dx-circle.dx);
   gv.Jtranspose.multiply(F,gv.tau);

}

void proj3Control(GlobalVariables& gv) 
{
   PrVector F;
   if (gv.curTime<=circle_acc.tf){
      circle_acc.step(gv.curTime);
      F=-gv.kp*(gv.x-circle_acc.x)-gv.kv*(gv.dx-circle_acc.dx);
      gv.Jtranspose.multiply(F,gv.tau); 
   } else {  
      floatControl(gv);  
   }
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