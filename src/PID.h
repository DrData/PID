#ifndef PID_H
#define PID_H

#include "Twiddle.h"

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  std::vector<double> vErrors;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  std::vector<double> vParams;
  std::vector<double> vdeltaP;

  /*
  * Twiddle Paramters
  */
  long long nSteps_;
  long long nMaxSteps_;
  long burn_in_;

  bool optimize_;
  bool isEstimatingErrorDone(int nsteps);
  double best_err_;
  double err_;
  double last_err_;

  bool reset_;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, std::vector<double> vdp, bool optimize);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte, double velocity, bool &reset);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  double GetLastError();

  bool IsOptimizing() const
  {
	  return tw_.IsOptimizating();
  }

  /*
  *
  */
  Twiddle tw_;
};

#endif /* PID_H */
