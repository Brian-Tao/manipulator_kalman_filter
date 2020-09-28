
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
void sys_evaluate_Q(double Q[14][14], const State& state, double dt) {
    // TODO fill in the matrix Q
    for (int i = 0; i < 14; ++i) {
        for (int j = 0; j < 14; ++j) {
            if (i == j) {
                Q[i][j] = 1.0;
            } else {
                Q[i][j] = 0.0;
            }
        }
    }

    // joint position variance
    Q[0][0] = 0.1;
    Q[1][1] = 0.1;
    Q[2][2] = 0.1;
    Q[3][3] = 0.1;
    Q[4][4] = 0.1;
    Q[5][5] = 0.1;
    Q[6][6] = 0.1;

    // joint velocity variance
    Q[7][7] = 0.1;
    Q[8][8] = 0.1;
    Q[9][9] = 0.1;
    Q[10][10] = 0.1;
    Q[11][11] = 0.1;
    Q[12][12] = 0.1;
    Q[13][13] = 0.1;

    // joint acceleration variance
    // Q[14][14] = 10.0;
    // Q[15][15] = 10.0;
    // Q[16][16] = 10.0;
    // Q[17][17] = 10.0;
    // Q[18][18] = 10.0;
    // Q[19][19] = 10.0;
    // Q[20][20] = 10.0;
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
    R[0][0] = 1e-2;
    R[1][1] = 1e-2;
    R[2][2] = 1e-2;
    R[3][3] = 1e-2;
    R[4][4] = 1e-2;
    R[5][5] = 1e-2;
    R[6][6] = 1e-2;

    // joint velocity measurement variance
    R[7][7] = 10;
    R[8][8] = 10;
    R[9][9] = 10;
    R[10][10] = 10;
    R[11][11] = 10;
    R[12][12] = 10;
    R[13][13] = 10;

    // joint acceleration measurement variance
    // R[14][14] = 10.0;
    // R[15][15] = 10.0;
    // R[16][16] = 10.0;
    // R[17][17] = 10.0;
    // R[18][18] = 10.0;
    // R[19][19] = 10.0;
    // R[20][20] = 10.0;
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
        state_out.q[i] = state_in.q[i] + dt * state_in.qd[i];
        state_out.qd[i] = state_in.qd[i];
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
