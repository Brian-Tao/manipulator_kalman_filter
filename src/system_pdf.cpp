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

#include "system_pdf.hpp"
#include "ekf_models.hpp"
#include <bfl/wrappers/rng/rng.h> // Wrapper around several rng libraries

namespace BFL
{

  using namespace MatrixWrapper;
  
  SystemPDF::SystemPDF( const Gaussian& additiveNoise )
    : AnalyticConditionalGaussianAdditiveNoise( additiveNoise, 2 ){}
  
  SystemPDF::~SystemPDF(){}


  // This method gets called from AnalyticSystemModelGaussianUncertainty::PredictionGet
  // Argument 0 is set to x (posterior expected value)
  // Argument 1 is set to u
  ColumnVector SystemPDF::ExpectedValueGet() const{
    
    ColumnVector state = ConditionalArgumentGet(0);
    ColumnVector torque = ConditionalArgumentGet(1); // not used yet

    // copy to the state (1 indexed to 0 indexed)
    State state_in;
    for (size_t i = 1; i <= 7; i++) {
        state_in.q[i - 1] = state(i);
        state_in.qd[i + 6] = state(i + 7);
        state_in.qdd[i + 13] = state(i + 14);
    }

    // Call the state prediction
    State state_out = sys_evaluate_g( state_in, torque(8) );

    // copy back to the state
    for (size_t i = 1; i <= 7; i++) {
        state(i) = state_out.q[i - 1];
        state(i + 7) = state_out.qd[i - 1];
        state(i + 14) = state_out.qdd[i - 1];
    }

    return state;

  }


  // This method gets called from AnalyticSystemModelGaussianUncertainty::df_dxGet
  // Argument 0 is set to x (posterior expected value)
  // Argument 1 is set to u
  Matrix SystemPDF::dfGet(unsigned int i) const{

    Matrix df( 21, 21 );
    df = 0;

    return df;

  }

  MatrixWrapper::SymmetricMatrix SystemPDF::CovarianceGet() const{

    ColumnVector state = ConditionalArgumentGet(0);
    ColumnVector torque = ConditionalArgumentGet(1); // not used for now

    // copy to the state
    State state_in;
    for (size_t i = 1; i <= 7; i++) {
        state_in.q[i - 1] = state(i);
        state_in.qd[i + 6] = state(i + 7);
        state_in.qdd[i + 13] = state(i + 14);
    }

    double Q[21][21];
    sys_evaluate_Q( Q, state_in, torque(8) );
    
    SymmetricMatrix sysQ( 21, 21 );
    for( int r=1; r<=21; r++){
      for( int c=1; c<=21; c++){
        sysQ(r,c) = Q[r-1][c-1];
      }
    }
    
    return sysQ;
  }
  
}//namespace BFL

