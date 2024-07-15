/**
 * @file utilities.hpp
 * @author Wojciech Babicki (wojciech.babicki@op.pl)
 * @brief 
 * @version 0.1
 * @date 2024-07-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef UTILITIES_HPP
#define UTILITIES_HPP

#include "globals.hpp"
/**
 * @brief function implements error calculation as input for LQR regulator
 * 
 * @param state1 
 * @param state2 
 * @param result 
 */
void calculate_difference_of_structs(const State_variables_struct* state1, const State_variables_struct* state2, float result[6]);
/**
 * @brief function implements multiplying matrixes: 2x6 * 6x1 (for LQR regulator)
 * 
 * @param stateVector 
 * @param matrix 
 * @param controlVector 
 */
void multiplyVectorByMatrix(const float stateVector[6], const float matrix[2][6], float controlVector[2]);
/**
 * @brief function implements signum function (for static friction)
 * 
 * @param value 
 * @return int 
 */
int signum(float value);
/**
 * @brief function implements saturation (for controller voltage)
 * 
 * @param value 
 * @param limit 
 * @return float 
 */
float saturation(float value, float limit);


#endif // UTILITIES_HPP
