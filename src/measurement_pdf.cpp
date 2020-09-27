// Copyright (C) 2008 Wim Meeussen <meeussen at willowgarage com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation; either version 2.1 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
//

#include "measurement_pdf.hpp"

#include <bfl/wrappers/rng/rng.h>  // Wrapper around several rng libraries
#include <tf/tf.h>

#include "ekf_models.hpp"

namespace BFL {

using namespace MatrixWrapper;

MeasurementPDF::MeasurementPDF(const Gaussian& additiveNoise)
    : AnalyticConditionalGaussianAdditiveNoise(additiveNoise) {}

MeasurementPDF::~MeasurementPDF() {}

ColumnVector MeasurementPDF::ExpectedValueGet() const {
    ColumnVector state = ConditionalArgumentGet(0);
    ColumnVector z(14);
    z = 0.0;

    for (int i = 1; i <= 14; ++i) {
        z(i) = state(i);
    }

    return z;
}

Matrix MeasurementPDF::dfGet(unsigned int i) const {
    Matrix df(14, 21);
    df = 0;

    for (int i = 1; i <= 14; ++i) {
        for (int j = 1; j <= 21; ++j) {
            if (i == j) {
                df(i, j) = 1.0;
            }else {
                df(i, j) = 0.0; 
            }
        }
    }

    return df;
}

MatrixWrapper::SymmetricMatrix MeasurementPDF::CovarianceGet() const {
    ColumnVector state = ConditionalArgumentGet(0);

    // copy to the state
    State state_in;
    for (size_t i = 1; i <= 7; i++) {
        state_in.q[i - 1] = state(i);
        state_in.qd[i - 1] = state(i + 7);
        state_in.qdd[i - 1] = state(i + 14);
    }

    double R[14][14];
    meas_evaluate_R(R, state_in);

    SymmetricMatrix measR(14, 14);
    for (int r = 1; r <= 14; r++) {
        for (int c = 1; c <= 14; c++) {
            measR(r, c) = R[r - 1][c - 1];
        }
    }

    return measR;
}

}  // namespace BFL
