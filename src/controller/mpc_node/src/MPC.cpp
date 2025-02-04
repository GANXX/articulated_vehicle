/*
# Copyright 2018 HyphaROS Workshop.
# Developer: HaoChih, LIN (hypha.ros@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
*/

#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <Eigen/Core>

// The program use fragments of code from
// https://github.com/udacity/CarND-MPC-Quizzes

using CppAD::AD;
using namespace std;

// =========================================
// FG_eval class definition implementation.
// =========================================
class FG_eval 
{
    public:
        // Fitted polynomial coefficients
        Eigen::VectorXd coeffs;

        double _Lf, _Lr, _dt, _ref_cte, _ref_epsi, _ref_vel; 
        double  _w_cte, _w_epsi, _w_vel, _w_delta, _w_accel, _w_delta_d, _w_accel_d;
        int _mpc_steps, _x_start, _y_start, _psi_start, _v_start, _gama_start, _cte_start, _epsi_start, _delta_start, _a_start;

        // Constructor
        FG_eval(Eigen::VectorXd coeffs) 
        { 
            this->coeffs = coeffs; 

            // Set default value    
            _Lf = 1.5; 
            _Lr = 1.49; 
            _dt = 0.1;  // in sec
            _ref_cte   = 0;
            _ref_epsi  = 0;
            _ref_vel   = 1.0; // m/s
            _w_cte     = 100;
            _w_epsi    = 100;
            _w_vel     = 100;
            _w_delta   = 100;
            _w_accel   = 50;
            _w_delta_d = 0;
            _w_accel_d = 0;

            _mpc_steps   = 20;
            _x_start     = 0;
            _y_start     = _x_start + _mpc_steps;
            _psi_start   = _y_start + _mpc_steps;
            _v_start     = _psi_start + _mpc_steps;
			_gama_start  = _v_start + _mpc_steps;
            _cte_start   = _gama_start + _mpc_steps;
            _epsi_start  = _cte_start + _mpc_steps;
            _delta_start = _epsi_start + _mpc_steps;
            _a_start     = _delta_start + _mpc_steps - 1;
        }

        // Load parameters for constraints
        void LoadParams(const std::map<string, double> &params)
        {
            _dt = params.find("DT") != params.end() ? params.at("DT") : _dt;
            _Lf = params.find("LF") != params.end() ? params.at("LF") : _Lf;
            _Lr = params.find("LR") != params.end() ? params.at("LR") : _Lr;
            _mpc_steps = params.find("STEPS") != params.end()    ? params.at("STEPS") : _mpc_steps;
            _ref_cte   = params.find("REF_CTE") != params.end()  ? params.at("REF_CTE") : _ref_cte;
            _ref_epsi  = params.find("REF_EPSI") != params.end() ? params.at("REF_EPSI") : _ref_epsi;
            _ref_vel   = params.find("REF_V") != params.end()    ? params.at("REF_V") : _ref_vel;
            
            _w_cte   = params.find("W_CTE") != params.end()   ? params.at("W_CTE") : _w_cte;
            _w_epsi  = params.find("W_EPSI") != params.end()  ? params.at("W_EPSI") : _w_epsi;
            _w_vel   = params.find("W_V") != params.end()     ? params.at("W_V") : _w_vel;
            _w_delta = params.find("W_DELTA") != params.end() ? params.at("W_DELTA") : _w_delta;
            _w_accel = params.find("W_A") != params.end()     ? params.at("W_A") : _w_accel;
            _w_delta_d = params.find("W_DDELTA") != params.end() ? params.at("W_DDELTA") : _w_delta_d;
            _w_accel_d = params.find("W_DA") != params.end()     ? params.at("W_DA") : _w_accel_d;

            _x_start     = 0;
            _y_start     = _x_start + _mpc_steps;
            _psi_start   = _y_start + _mpc_steps;
            _v_start     = _psi_start + _mpc_steps;
			_gama_start  = _v_start + _mpc_steps;
            _cte_start   = _gama_start + _mpc_steps;
            _epsi_start  = _cte_start + _mpc_steps;
            _delta_start = _epsi_start + _mpc_steps;
            _a_start     = _delta_start + _mpc_steps - 1;
            
            //cout << "\n!! FG_eval Obj parameters updated !! " << _mpc_steps << endl; 
        }

        // MPC implementation (cost func & constraints)
        typedef CPPAD_TESTVECTOR(AD<double>) ADvector; 
        // fg: function that evaluates the objective and constraints using the syntax       
		//在构造fg[x]
        void operator()(ADvector& fg, const ADvector& vars) 
        {
            
            // fg[0] for cost function
			// fg[0]就是优化函数 fg[1...2]是约束条件
			// var[]是自变量
		   /*******构造目标函数***********/
            fg[0] = 0;
            for (int i = 0; i < _mpc_steps; i++) {
              fg[0] += _w_cte * CppAD::pow(vars[_cte_start + i] - _ref_cte, 2); // cross deviation error 偏差值的加权量
              fg[0] += _w_epsi * CppAD::pow(vars[_epsi_start + i] - _ref_epsi, 2); // heading error 航向误差的加权量
              fg[0] += _w_vel * CppAD::pow(vars[_v_start + i] - _ref_vel, 2); // speed error  与预期速度的加权量
			  if(i<_mpc_steps-2){
				 fg[0] += _w_delta_d * CppAD::pow(vars[_delta_start + i + 1] - vars[_delta_start + i], 2);
				 fg[0] += _w_accel_d * CppAD::pow(vars[_a_start + i + 1] - vars[_a_start + i], 2);
			  }

            }

            // Minimize the use of actuators.
			// 执行机构也做了相应的加权量
            //for (int i = 0; i < _mpc_steps - 1; i++) {
              //fg[0] += _w_delta * CppAD::pow(vars[_delta_start + i], 2); //对转角的加权量
              //fg[0] += _w_accel * CppAD::pow(vars[_a_start + i], 2);     //对加速度的加权量
            //}

            // Minimize the value gap between sequential actuations.最大限度地减少顺序启动之间的价值差距。
            //for (int i = 0; i < _mpc_steps - 2; i++) {
              //fg[0] += _w_delta_d * CppAD::pow(vars[_delta_start + i + 1] - vars[_delta_start + i], 2);
              //fg[0] += _w_accel_d * CppAD::pow(vars[_a_start + i + 1] - vars[_a_start + i], 2);
            //}
            /*************************/

            // fg[x] for constraints
            // Initial constraints
            fg[1 + _x_start] = vars[_x_start];
            fg[1 + _y_start] = vars[_y_start];
            fg[1 + _psi_start] = vars[_psi_start];
            fg[1 + _gama_start] = vars[_gama_start];
            fg[1 + _v_start] = vars[_v_start];
            fg[1 + _cte_start] = vars[_cte_start];
            fg[1 + _epsi_start] = vars[_epsi_start];

            // Add system dynamic model constraint
            for (int i = 0; i < _mpc_steps - 1; i++)
            {
                // The state at time t+1 .
                AD<double> x1    = vars[_x_start + i + 1];
                AD<double> y1    = vars[_y_start + i + 1];
                AD<double> psi1  = vars[_psi_start + i + 1];
                AD<double> gama1 = vars[_gama_start + i + 1];
                AD<double> v1    = vars[_v_start + i + 1];
                AD<double> cte1  = vars[_cte_start + i + 1];
                AD<double> epsi1 = vars[_epsi_start + i + 1];

                // The state at time t.
                AD<double> x0    = vars[_x_start + i];
                AD<double> y0    = vars[_y_start + i];
                AD<double> psi0  = vars[_psi_start + i];
                AD<double> gama0 = vars[_gama_start + i];
                AD<double> v0    = vars[_v_start + i];
                AD<double> cte0  = vars[_cte_start + i];
                AD<double> epsi0 = vars[_epsi_start + i];

                // Only consider the actuation at time t.
				// 这两个就是最后要求的自变量
                AD<double> delta0 = vars[_delta_start + i];//转向角速度
                AD<double> a0     = vars[_a_start + i];//加速度

                AD<double> f0 = 0.0;
				//coeffs[]应该是参考路径的样条曲线的拟合方程的前项系数，ｆ(x0)=y0是车辆在x坐标下的y的参考坐标
				//所以f0-y0就是车辆的位置偏差值
				//同理psides0是f0的导数,也就是车辆的航向角的tan值
				//coeffs产生的样条曲线是以baselink为坐标系的,所以x一直是车辆的行驶方向
				AD<double> yumiao_dis = x0;
				if(i ==0) 
                for (int i = 0; i < coeffs.size(); i++) 
                {
                    f0 += coeffs[i] * CppAD::pow(yumiao_dis, i);
					//cout<<"f0\t"<<f0<<"yumiao_dis\t"<<yumiao_dis<<endl;
                }
                AD<double> psides0 = 0.0;
                for (int i = 1; i < coeffs.size(); i++) 
                {
                    psides0 += i*coeffs[i] * CppAD::pow(yumiao_dis, i-1); // f'(yumiao_dis)
                }
                psides0 = CppAD::atan(psides0);

                fg[2 + _x_start + i]    = x1 - (x0 + v0 * CppAD::cos(psi0) * _dt); //车辆x方向的约束
                fg[2 + _y_start + i]    = y1 - (y0 + v0 * CppAD::sin(psi0) * _dt); //车辆y方向的约束
                fg[2 + _gama_start + i]  = gama1 - (gama0 + delta0 * _dt);			   //车辆铰接角度的约束
				AD<double> L = (_Lf*CppAD::cos(gama0)+_Lr);
                fg[2 + _psi_start + i]  = psi1 - (psi0 + v0*CppAD::sin(gama0)*_dt/L+delta0*_dt*_Lr/L);  //车辆角度的约束
				fg[2 + _v_start + i]    = v1 - (v0 + a0 * _dt);					   //车辆速度的约束
				//计算横向误差和航向角误差的
				//f0是i时刻车辆参考点的y坐标(相对于车身坐标系),y0是i时刻车辆自身的y坐标(i=0时候y0=0)
				//cte1 是i+1时刻的横向误差值,所以cte1-cte0 = 0建立约束条件
                fg[2 + _cte_start + i]  = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * _dt)); //横向误差
                fg[2 + _epsi_start + i] = epsi1 - ((psi0 - psides0) + delta0 * _dt);//航向角误差
            }
        }
};

// ====================================
// MPC class definition implementation.
// ====================================
MPC::MPC() 
{
    // Set default value    
    _mpc_steps = 20;
    _max_steering = 0.139; // Maximal steering radian (~30 deg)
	_max_gama = 0.7;
    _max_throttle = 1.0; // Maximal throttle accel
    _bound_value  = 1.0e3; // Bound value for other variables

    _x_start     = 0;
    _y_start     = _x_start + _mpc_steps;
    _psi_start   = _y_start + _mpc_steps;
    _v_start     = _psi_start + _mpc_steps;
	_gama_start  = _v_start + _mpc_steps;
	_cte_start   = _gama_start + _mpc_steps;
    _epsi_start  = _cte_start + _mpc_steps;
    _delta_start = _epsi_start + _mpc_steps;
    _a_start     = _delta_start + _mpc_steps - 1;

}

void MPC::LoadParams(const std::map<string, double> &params)
{
    _params = params;
    //Init parameters for MPC object
    _mpc_steps = _params.find("STEPS") != _params.end() ? _params.at("STEPS") : _mpc_steps;
    _max_steering = _params.find("MAXSTR") != _params.end() ? _params.at("MAXSTR") : _max_steering;
    _max_throttle = _params.find("MAXTHR") != _params.end() ? _params.at("MAXTHR") : _max_throttle;
    _max_gama = _params.find("MAXGAMA") != _params.end() ? _params.at("MAXGAMA") : _max_gama;
    _bound_value  = _params.find("BOUND") != _params.end()  ? _params.at("BOUND") : _bound_value;
    
    _x_start     = 0;
    _y_start     = _x_start + _mpc_steps;
    _psi_start   = _y_start + _mpc_steps;
    _v_start     = _psi_start + _mpc_steps;
	_gama_start  = _v_start + _mpc_steps;
	_cte_start   = _gama_start + _mpc_steps;
    _epsi_start  = _cte_start + _mpc_steps;
    _delta_start = _epsi_start + _mpc_steps;
    _a_start     = _delta_start + _mpc_steps - 1;

    //cout << "\n!! MPC Obj parameters updated !! " << endl; 
}


vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) 
{
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;
    const double x      = state[0];
    const double y      = state[1];
    const double psi    = state[2];
    const double v      = state[3];
    const double gama   = state[4];
    const double cte    = state[5];
    const double epsi   = state[6];
    // Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
	// 需要优化的变量
    size_t n_vars = _mpc_steps * 7 + (_mpc_steps - 1) * 2;
    // Set the number of constraints
    size_t n_constraints = _mpc_steps * 7;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
	// 给待求解变量的初始值
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) 
    {
        vars[i] = 0;
    }

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    // Set lower and upper limits for variables.
    for (int i = 0; i < _gama_start; i++) 
    {
        vars_lowerbound[i] = -_bound_value;
        vars_upperbound[i] =  _bound_value;
    }
    for (int i = _cte_start; i < _delta_start; i++) 
    {
        vars_lowerbound[i] = -_bound_value;
        vars_upperbound[i] =  _bound_value;
    }
	// set loewr and upper limits for articulate angle
    for (int i = _gama_start; i < _cte_start; i++) 
    {
        vars_lowerbound[i] = -_max_gama;
        vars_upperbound[i] =  _max_gama;
    }
    // The upper and lower limits of delta are set to -25 and 25
    // degrees (values in radians).
    for (int i = _delta_start; i < _a_start; i++) 
    {
        vars_lowerbound[i] = -_max_steering;
        vars_upperbound[i] =  _max_steering;
    }
    // Acceleration/decceleration upper and lower limits
    for (int i = _a_start; i < n_vars; i++)  
    {
        vars_lowerbound[i] = -_max_throttle;
        vars_upperbound[i] =  _max_throttle;
    }


    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[_x_start] = x;
    constraints_lowerbound[_y_start] = y;
    constraints_lowerbound[_psi_start] = psi;
    constraints_lowerbound[_gama_start] = gama;
    constraints_lowerbound[_v_start] = v;
    constraints_lowerbound[_cte_start] = cte;
    constraints_lowerbound[_epsi_start] = epsi;

    constraints_upperbound[_x_start] = x;
    constraints_upperbound[_y_start] = y;
    constraints_upperbound[_psi_start] = psi;
    constraints_upperbound[_gama_start] = gama;
    constraints_upperbound[_v_start] = v;
    constraints_upperbound[_cte_start] = cte;
    constraints_upperbound[_epsi_start] = epsi;

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);
    fg_eval.LoadParams(_params);
    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.etc_data
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Cost
    auto cost = solution.obj_value;
    this->mpc_x = {};
    this->mpc_y = {};
    for (int i = 0; i < _mpc_steps; i++) 
    {
        this->mpc_x.push_back(solution.x[_x_start + i]);
        this->mpc_y.push_back(solution.x[_y_start + i]);
    }
    vector<double> result;
	//cout<<solution.x[60]<<"\t"<<solution.x[80]<<"\t"<<solution.x[100]<<"\t"<<solution.x[120]<<"\t"<<solution.x[139]<<"\t"<<endl;
    result.push_back(solution.x[_delta_start]);
    result.push_back(solution.x[_a_start]);
    return result;
}
