#include <math.h>
#include <iostream>
#include <iomanip>      // std::setw
#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  p_error = 0;
  i_error = 0;
  d_error = 0;
  prev_d_error = 0;

  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  
  twiddle = false;

  //Ki = 0;
  //Kd = 0;

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  p_error = cte;
  d_error = cte - prev_d_error;
  prev_d_error = cte;
  i_error += cte;

  // A self contained twiddle algorithm
  if (twiddle) {
    static double counter = 0;
    counter++;
    static double cost = 0;
    cost += abs(cte);
    // 1000 iterations seem sufficient for one round
    // Use the average error for the data
    if (counter > 1000) {
      SM(cost/counter);
      counter = 0;
      cost = 0;
    }
  }

}

double PID::TotalError() {
  // return steering angle using P, D & I parameters
  return -(p_error * Kp + i_error * Ki + d_error * Kd);
}

void PID::SM(double total_error) {

  // A simple state machine with 'if' conditions
  static int state = 0;
  // Control parameter under test
  static int p = 0;
  static double best_error = total_error;
  static double overall_best_error = total_error;
  // Parameter increments
  static double dp[] = { 0.05, 0.5, 1.0 };

  double K[] = { Kp, Kd, Ki};
  
  #define K_LEN 2

  if (dp[p] < 0.02)
    return;
  if (best_error < 0.05)
    return;
  
if (total_error < overall_best_error) {
  overall_best_error = total_error;
 
  std::cout << "SM: Best Error " << std::setw(7) << std::left << overall_best_error << " Kp " << std::setw(7) << std::left << Kp <<
       " Kd " << std::setw(7) << std::left << Kd << " Ki " << std::setw(7) << std::left << Ki << std::endl;

 }  
  std::cout << "SM: state " << state << " p " << p << " terror " << std::setw(7) << total_error 
  << " berr " << std::setw(7) << best_error;
  std::cout << " Kp " << std::left << std::setw(7) << Kp << " Kd " << std::left << std::setw(7) << Kd << " Ki " << std::left << std::setw(7) << Ki << 
            " p " << std::left << std::setw(7) << dp[0] << " i " << std::left << std::setw(7) << dp[1] << " d " << std::setw(7) << dp[2] << std::endl;


  if (state == 0) {
      // State 0 and State 1 are same now - it was not in the initial version
      //best_error = total_error;
      //state = 1;
      // First increment
      K[p] += dp[p];
      state = 2;      
  }
  else if (state == 1) {  
    // State 0 and State 1 are same now - it was not in the initial version
    // First increment
    K[p] += dp[p];
    state = 2;
  }
  else if (state == 2) {
    if (total_error < best_error) {
      best_error = total_error;
      // Increasing parameter seem to work, increase the increments
      dp[p] *= 1.1;
      if (p == K_LEN-1) 
        state = 0;
      else 
        state = 1;
      // Switch to next parameter
      p = (p + 1) % K_LEN;
    } 
    else {
      // Increasing doesn't work, reduce the increment to original value
      K[p] -= 2.0 * dp[p];
      state = 3;
    }
  }
  else {
    if (total_error < best_error) {
      best_error = total_error;
      // Error has reduced, increase the increment
      dp[p] *= 1.1;
    }
    else {
      // Increasing doesn't work, reduce the increment 
      K[p] += dp[p];
      dp[p] *= 0.9;
    }
    if (p == K_LEN-1) 
      state = 0;
    else 
      state = 1;
    // Switch to next parameter
    p = (p + 1) % K_LEN;
  }
  Kp = K[0];
  Ki = K[2];
  Kd = K[1];
}