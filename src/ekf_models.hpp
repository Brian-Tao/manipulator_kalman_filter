
#ifndef __EKF_MODELS_HPP__
#define __EKF_MODELS_HPP__

#include <iostream>
#include <iomanip>


struct jnt_array{ 
   //enum {JNT_POS, JNT_VEL, JNT_ACC};
   double q[7];
};


/**
   State of the robot
*/
struct State{

  double q[7];
  double qd[7];
  double qdd[7];

/*
  friend std::ostream& operator << ( std::ostream& os, const State& s){

    os << std::setw(15) << "POS_X: " << s.x[POS_X]
       << std::setw(15) << "POS_Y: " << s.x[POS_Y]
       << std::setw(15) << "POS_Z: " << s.x[POS_Z] << std::endl
       << std::setw(15) << "ROT_R: " << s.x[ROT_R] 
       << std::setw(15) << "ROT_P: " << s.x[ROT_P]
       << std::setw(15) << "ROT_Y: " << s.x[ROT_Y] << std::endl;

    return os;

  }
*/

};

/**
   Fill in the value of the process covariance matrix. The 
   performance of your EKF will vary based on the values you provide here.
   \param[out] Q Covariance matrix of the system.
   \param state_in    The current state estimate
   \param dt          Delta time
*/
void sys_evaluate_Q( double Q[21][21], const State& state, double dt );

/**
   Fill in the value of the measurement covariance matrix. The 
   performance of your EKF will vary based on the values you provide here.
   \param[out] R Covariance matrix of the sensors.
   \param state_in    The current state estimate
*/
void meas_evaluate_R( double R[14][14], const State& state );

/**
   @brief
   Evaluate the system function.
   Compute the process model.
   This function returns the prediction of the next state based on the
   current state estimate and the commmand input (linear/angular velocities).
   \param state_in    The current state estimate
   \param dt          Delta time
*/
State sys_evaluate_g(const State& state_in, double dt);

void sys_evaluate_G( double G[6][6], const State& state, double v, double w, double dt );

/**
   @brief calculate jnt pos measurement
   \param state The state estimate
   \return      Joint position
*/
jnt_array meas_evaluate_jnt_pos(const State& state);

/**
   @brief calculate jnt vel measurement.
   \param state_in The current state estimate
   \return         Joint velocity.
*/
jnt_array meas_evaluate_jnt_vel(const State& state);

void meas_evaluate_Hgps( double Hgps[3][3], const State& state );

void meas_evaluate_Himu( double Himu[3][3], const State& state );

#endif
