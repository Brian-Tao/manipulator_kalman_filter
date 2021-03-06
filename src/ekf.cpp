/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Wim Meeussen */

#include "ekf.hpp"

using namespace MatrixWrapper;
using namespace BFL;
using namespace tf;
using namespace std;

// constructor
EKF::EKF() : prior_(NULL), filter_(NULL), initialized(false) {
    // create system model
    ColumnVector sysNoise_Mu(14);
    sysNoise_Mu = 0;
    SymmetricMatrix sysNoise_Cov(14);
    sysNoise_Cov = 0;

    // process_covariance( C );
    for (int r = 1; r <= 14; r++) {
        sysNoise_Cov(r, r) = 1e-1;
    }

    Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);
    sys_pdf_ = new SystemPDF(system_Uncertainty);
    sys_model_ = new AnalyticSystemModelGaussianUncertainty(sys_pdf_);

    // create measurement model
    ColumnVector measNoise_Mu(14);
    measNoise_Mu = 0;
    SymmetricMatrix measNoise_Cov(14);
    measNoise_Cov = 0;

    // measurement_covariance( C );
    for (int r = 1; r <= 14; r++) {
        measNoise_Cov(r, r) = 1.0;
    }

    Gaussian measurement_Uncertainty(measNoise_Mu, measNoise_Cov);
    meas_pdf_ = new MeasurementPDF(measurement_Uncertainty);
    meas_model_ = new AnalyticMeasurementModelGaussianUncertainty(meas_pdf_);
}

// destructor
EKF::~EKF() {
    if (filter_)
        delete filter_;
    if (prior_)
        delete prior_;
    if (sys_pdf_)
        delete sys_pdf_;
    if (sys_model_)
        delete sys_model_;
    if (meas_pdf_)
        delete meas_pdf_;
    if (meas_model_)
        delete meas_model_;
}

// initialize prior density of filter
void EKF::initialize() {
    ColumnVector prior_Mu(14);
    prior_Mu = 0.0;
    // prior_Mu(3) = 0.23;

    // set a good covariance
    SymmetricMatrix prior_Cov(14);
    prior_Cov = 0.0;
    for (unsigned int i = 1; i <= 14; i++) {
        for (unsigned int j = 1; j <= 14; j++) {
            if (i == j) {
                prior_Cov(i, j) = 1.0;
            }
        }
    }

    prior_ = new Gaussian(prior_Mu, prior_Cov);
    filter_ = new ExtendedKalmanFilter(prior_);

    filter_time_old = ros::Time::now();
    initialized = true;
}

// return the posterior estimate
State EKF::get_posterior() const {
    ColumnVector mu = filter_->PostGet()->ExpectedValueGet();

    State res;

    for (int i = 1; i <= 7; ++i) {
        res.q[i - 1] = mu(i);
        res.qd[i - 1] = mu(i + 7);
    }

    return res;
}

// update filter
bool EKF::update(const ros::Time& filter_time, const ColumnVector& z, const ColumnVector& u) {
    // only update filter for time later than current filter time
    /*
    double dt = (filter_time - filter_time_old).toSec();
    if (dt == 0) return false;
    if (dt <  0){
      ROS_INFO("Will not update robot pose with time %f sec in the past.", dt);
      return false;
    }
    ROS_DEBUG("Update filter at time %f with dt %f", filter_time.toSec(), dt);
    */
    // std::cout << u(1) << " " << u(8) << std::endl;
    // std::cout << z(1) << " " << z(8) << std::endl;
    filter_->Update(sys_model_, u, meas_model_, z);
    filter_time_old = filter_time;

    return true;
}
