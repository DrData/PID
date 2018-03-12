#pragma once

#include <vector>
#include <deque>

using namespace std;

class Twiddle
{
public:
	Twiddle();
	~Twiddle();

	enum eOptState
	{
		eOpt_INIT,
		eOpt_STEP_1,
		eOpt_STEP_2,
		eOpt_STEP_3,
		eOpt_STEP_4,
		eOpt_STEP_5,
	};

	void Initialize(double tol, std::vector<double> & deltaParam, vector<double> &Params);

	bool Update(double err, double velocity, bool &reset);
	bool EstimatingError(double err, double velocity, bool &reset);
	void InitErrorEstimation();
	double GetLatestEstimate() const { return this->avg_err_; }
	void GetCurrentParams(std::vector<double> &vparams);
	bool IsOptimizating()  const { return is_optimizing_;}
	void SetIsOptimizing(bool bflag = true) { is_optimizing_ = bflag; }
	void SaveToFile();

private:
	long long nitr;
	bool debug_;
	bool is_optimizing_;
	double avg_err_;
	int nSteps_;
	int nAvg_;
	int nBurnIn_;
	bool is_initialized_;
	int _BURNIN_;
	
	deque<double> qVelo;

	eOptState current_state_;
	int cur_param_index_;
	bool finished_;

	double dtol_;
	std::vector<double> *pvParams_;
	std::vector<double> vdp_;

	std::vector<double> bestParams_;


	int nDim;
	double best_err_;
	double err_;

	int n_cur;
};
