
#include "ekf_models.hpp"

#include <tf/tf.h>

/**
   TODO
   Fill in the value of the process covariance matrix. The rows/columns of VMVt are
   in the following order [POS_X POS_Y POS_Z ROT_R ROT_P ROT_Y ].
   \param[out]  Covariance matrix of the system.
   \param state_in    The current state estimate
   \param dt          Delta time
*/
void sys_evaluate_Q(double Q[21][21], const State& state, double dt) {
    // TODO fill in the matrix Q
    for (int i = 0; i < 21; ++i) {
        for (int j = 0; j < 21; ++j) {
            if (i == j) {
                Q[i][j] = 1.0;
            } else {
                Q[i][j] = 0.0;
            }
        }
    }

    // joint position variance
    Q[0][0] = 1e-5;
    Q[1][1] = 1e-5;
    Q[2][2] = 1e-5;
    Q[3][3] = 1e-5;
    Q[4][4] = 1e-5;
    Q[5][5] = 1e-5;
    Q[6][6] = 1e-5;

    // joint velocity variance
    Q[7][7] = 1e-5;
    Q[8][8] = 1e-5;
    Q[9][9] = 1e-5;
    Q[10][10] = 1e-5;
    Q[11][11] = 1e-5;
    Q[12][12] = 1e-5;
    Q[13][13] = 1e-5;

    // joint acceleration variance
    Q[14][14] = 1e-5;
    Q[15][15] = 1e-5;
    Q[16][16] = 1e-5;
    Q[17][17] = 1e-5;
    Q[18][18] = 1e-5;
    Q[19][19] = 1e-5;
    Q[20][20] = 1e-5;
}

/**
   TODO
   Fill in the value of the measurement covariance matrix. The rows/columns of C
   are in the following order [POS_X POS_Y POS_Z ROT_R ROT_P ROT_Y ]
   \param[out] R Covariance matrix of the sensors.
   \param state_in    The current state estimate
*/
void meas_evaluate_R(double R[14][14], const State& state) {
    // TODO fill in the matrix R
    for (int i = 0; i < 14; ++i) {
        for (int j = 0; j < 14; ++j) {
            if (i == j) {
                R[i][j] = 1.0;
            } else {
                R[i][j] = 0.0;
            }
        }
    }

    // joint position measurement variance
    R[0][0] = 1e-5;
    R[1][1] = 1e-5;
    R[2][2] = 1e-5;
    R[3][3] = 1e-5;
    R[4][4] = 1e-5;
    R[5][5] = 1e-5;
    R[6][6] = 1e-5;

    // joint velocity measurement variance
    R[7][7] = 1e-3;
    R[8][8] = 1e-3;
    R[9][9] = 1e-3;
    R[10][10] = 1e-3;
    R[11][11] = 1e-3;
    R[12][12] = 1e-3;
    R[13][13] = 1e-3;
}

/**
   @brief
   Evaluate the system function.
   Compute the process model.
   This function returns the prediction of the next state based on the
   current state estimate and the commmand input (linear/angular velocities).
   \param state_in    The current state estimate
   \param dt          Delta time
*/
State sys_evaluate_g(const State& state_in, double dt) {
    State state_out;

    for (int i = 0; i < 7; ++i) {
        state_out.q[i] = state_in.q[i] + dt * state_in.qd[i] + 0.5 * dt * dt * state_in.qdd[i];
        state_out.qd[i] = state_in.qd[i] + dt * state_in.qdd[i];
        state_out.qdd[i] = state_in.qdd[i];
    }

    return state_out;
}

void sys_evaluate_G(double G[6][6], const State& state, double v, double w, double dt) {
    // not needed for now
}

/**
   @brief calculate jnt pos measurement
   \param state The state estimate
   \return      Joint position
*/
jnt_array meas_evaluate_jnt_pos(const State& state) {
    jnt_array jnt_pos;

    for (int i = 0; i < 7; ++i) {
        jnt_pos.q[i] = state.q[i];
    };

    return jnt_pos;
}

/**
   @brief calculate jnt vel measurement.
   \param state_in The current state estimate
   \return         Joint velocity.
*/
jnt_array meas_evaluate_jnt_vel(const State& state) {
    // TODO
    // Given the prior estimate state, determine the expected RPY measurement rpy
    jnt_array jnt_vel;

    for (int i = 0; i < 7; ++i) {
        jnt_vel.q[i] = state.qd[i];
    };

    return jnt_vel;
}

void meas_evaluate_Hgps(double Hgps[3][3], const State& state) {
    // not needed for now
}

void meas_evaluate_Himu(double Himu[3][3], const State& state) {
    // not needed for now
}
