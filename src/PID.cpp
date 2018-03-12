#include "PID.h"
#include <stdio.h>
#include <iostream>
using namespace std;

PID::PID() : p_error(0.0), i_error(0.0),
d_error(0), Kp(1.0), Ki(0.001), Kd(.5)
{}

PID::~PID() {}

void PID::Init(double _Kp, double _Ki, double _Kd, std::vector<double> vdp, bool optimize)
{
	this->Kp = _Kp;
	this->Ki = _Ki;
	this->Kd = _Kd;
	vParams.push_back(_Kp);
	vParams.push_back(_Ki);
	vParams.push_back(_Kd);
	vdeltaP.push_back(vdp[0]);
	vdeltaP.push_back(vdp[1]);
	vdeltaP.push_back(vdp[2]);
	tw_.Initialize(0.2, vdeltaP, vParams);
	tw_.SetIsOptimizing(optimize);
}

void PID::UpdateError(double cte, double v, bool &reset)
{
	// Do Twiddle if optimizing
	if( tw_.IsOptimizating() ) tw_.Update(cte, v, reset);

	if (reset)
	{
		// Process the cte
		p_error = 0;
		d_error = cte - p_error;
		p_error = cte;
		i_error = cte;

		reset_ = true;

		// Update parameters from the current optimization step
		Kp = vParams[0];
		Ki = vParams[1];
		Kd = vParams[2];
	}
	else
	{
		// Process the cte
		reset_ = false;
		d_error = cte - p_error;
		p_error = cte;
		i_error += cte;
	}
}

double PID::TotalError()
{
	if (tw_.IsOptimizating()  && reset_)
	{
		double terr = -Kp * p_error - Kd * d_error - Ki * i_error;
		
		Kp = vParams[0];
		Ki = vParams[1];
		Kd = vParams[2];

		return terr;
	}
	return -Kp * p_error - Kd * d_error - Ki * i_error;
}


double PID::GetLastError()
{
	return last_err_;
}

