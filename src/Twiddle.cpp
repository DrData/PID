#include "Twiddle.h"
#include <vector>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <fstream>

using namespace std;

Twiddle::Twiddle() : nitr(0), debug_(false), is_optimizing_(false)
, avg_err_(0.0), nSteps_(650), nAvg_(0), nBurnIn_(0),
is_initialized_(false), current_state_(eOpt_INIT), pvParams_(NULL)
, vdp_(std::vector<double>())
{
}

Twiddle::~Twiddle()
{

}

void Twiddle::Initialize(double tol, std::vector<double> & deltaParam, vector<double> &Params)
{
	// Parameters/Hyperparameters
	dtol_ = tol;
	pvParams_ = &Params;
	vdp_ = deltaParam;

	// Track Twiddle State machine
	current_state_ = eOpt_INIT;

	// Intialize error accumulation
	InitErrorEstimation();
	n_cur = 0;
	is_initialized_ = false;
	best_err_ = 999999.0;
	_BURNIN_ = fmin(nSteps_ / 2, 100);
	bestParams_ = Params;
}

void Twiddle::InitErrorEstimation()
{
	this->nAvg_ = 0;
	avg_err_ = 0.0;
	nBurnIn_ = 0;
	qVelo.empty();
}


bool Twiddle::EstimatingError(double err, double velocity, bool &reset)
{

	if (++nBurnIn_ < _BURNIN_ ) return true;

	// Check if the car is moving, if not abandon the trial
	if (qVelo.size() == 10) qVelo.pop_back();
	qVelo.push_front(velocity);

	if (qVelo.size() > 5)
	{
		double sv = 0.0;
		for (int i =0; i < qVelo.size(); i++) sv += qVelo[i];
		sv /= qVelo.size();
		if (sv < 0.02)
		{
			avg_err_ = 99999.9;
			reset = true;
			return false;
		}
	}

	// Check if the car is off the road
	// CTE is greater than 6.5 can't be on the road
	// so abandon trial
	if (fabs(err) > 6.5)
	{
		avg_err_ = 99999.9;
		reset = true;
		return false;
	}

	// Accumulate squared error
	avg_err_ += err*err;
	nAvg_++;

	if (nAvg_ == nSteps_)
	{
		// mean summed squared error
		avg_err_ /= (double) nAvg_;
		reset = true;
		return false;
	}
	reset = false;
	return true;
}

bool Twiddle::Update(double err, double velocity, bool &reset)
{
	// Computing the error, if keep going, then returen
	// if done proceed to state evaluation below
	if (EstimatingError(err, fabs(velocity), reset)) return true;

	// Get current parameter values
	std::vector<double> &vParams_ = *pvParams_;

	if (current_state_ == eOpt_INIT)
	{
		best_err_ = this->GetLatestEstimate(); // set this for output
	}

	cout << endl << "Interation: " << nitr++ << endl;
	cout << "vp=(" << vParams_[0] << ", " << vParams_[1] << ", " << vParams_[2] << ") " << endl;
	cout << "dp=(" << vdp_[0] << ", " << vdp_[1] << ", " << vdp_[2] << ") " << endl;
	cout << "best_err = " << best_err_ << endl;
	cout << "best vp=(" << bestParams_[0] << ", " << bestParams_[1] << ", " << bestParams_[2] << ") " << endl;

	switch (current_state_)
	{
	case eOpt_INIT:
		best_err_ = this->GetLatestEstimate();
		n_cur = 0;
		vParams_[n_cur] += vdp_[n_cur];
		InitErrorEstimation();
		reset = true;
		current_state_ = eOpt_STEP_1;
		break;

	case eOpt_STEP_1:
		this->err_ = this->GetLatestEstimate();
		if (err_ < best_err_)
		{
			best_err_ = err_;
			bestParams_ = vParams_;
			vdp_[n_cur] *= 1.1;

			// Advance to next parameter
			// Iterating over the number of parameters
			n_cur++; // n_curr is the current parameter 
			if (n_cur == vParams_.size())
			{
				// Looped over all parameters
				// so check if we are done
				double sum = 0;
				for (double _dp : vdp_) sum += _dp;
				if (sum < dtol_)
				{
					// Done so stop
					is_optimizing_ = false;
					SaveToFile();
					return false;
				}
				else
				{
					n_cur = 0;
				}
			}

			// Since this was an improvement
			// try bigger change in same parameter
			vParams_[n_cur] += vdp_[n_cur];
			current_state_ = eOpt_STEP_1;
		}
		else
		{
			// Not an improvement
			// Reset and try the other direction
			vParams_[n_cur] -= 2.0 * vdp_[n_cur];
			current_state_ = eOpt_STEP_2;
		}
		// Compute the next error given delta to parameter
		this->InitErrorEstimation();
		reset = true;
		break;

	case eOpt_STEP_2:
		this->err_ = this->GetLatestEstimate();
		if (err_ < best_err_)
		{
			// Other direction is improvement so keep
			// and try bigger step
			best_err_ = err_;
			bestParams_ = vParams_;

			vdp_[n_cur] *= 1.1;
		}
		else
		{
			// Other direction didn't work,
			// try smaller step
			vParams_[n_cur] += vdp_[n_cur];
			vdp_[n_cur] *= 0.9;
		}

		// Advance to next parameter
		// Iterating over the number of parameters
		n_cur++; // n_curr is the current parameter 
		if (n_cur == vParams_.size())
		{
			// Looped over all parameters
			// so check if we are done
			double sum = 0;
			for (double _dp : vdp_) sum += _dp;
			if (sum < dtol_)
			{
				// Done so stop
				is_optimizing_ = false;
				SaveToFile();
				return false;
			}
			else
			{
				n_cur = 0;
			}
		}


		// Change paramter
		vParams_[n_cur] += vdp_[n_cur];

		// Get estimate
		InitErrorEstimation();
		reset = true;
		current_state_ = eOpt_STEP_1;
		break;

	default:
		break;
	}

	return true;

}

void Twiddle::SaveToFile()
{

	string fname = "./optimzer_results.txt";
	fstream outf(fname.c_str());
	for (int i = 0; i < this->bestParams_.size(); i++)
	{
		outf << bestParams_[i] << ", " << endl;
	}
	outf << endl;
	outf.close();
	return;
}

