/**
 * @file utilities.cpp
 * @author Wojciech Babicki (wojciech.babicki@op.pl)
 * @brief 
 * @version 0.1
 * @date 2024-07-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "utilities.hpp"
#include "globals.hpp"

void calculate_difference_of_structs(const State_variables_struct* state1, const State_variables_struct* state2, float result[6])
{
    result[0] = state1->x - state2->x;
    result[1] = state1->x_dot - state2->x_dot;
    result[2] = state1->theta_p - state2->theta_p;
    result[3] = state1->theta_p_dot - state2->theta_p_dot;
    result[4] = state1->psi - state2->psi;
    result[5] = state1->psi_dot - state2->psi_dot;
}

void multiplyVectorByMatrix(const float stateVector[6], const float matrix[2][6], float controlVector[2])
{
  // Initialize control vector array
  controlVector[0] = 0.0;
  controlVector[1] = 0.0;

  // multiply state vector with LQR matrix
  for (int i = 0; i < 2; ++i)
  {
    for (int j = 0; j < 6; ++j)
    {
      controlVector[i] += stateVector[j] * matrix[i][j];
    }
  }
}

int signum(float value)
{
  if (value > 0.0)
  {
    return 1;
  }
  else if (value < 0.0)
  {
    return -1;
  }
  else
  {
    return 0;
  }
}

float saturation(float value, float limit)
{
  if (value > limit)
  {
    return limit;
  }
  else if (value < -limit)
  {
    return -limit;
  }
  else
  {
    return value;
  }
}
