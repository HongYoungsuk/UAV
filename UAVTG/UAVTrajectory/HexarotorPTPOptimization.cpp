#include "HexarotorPTPOptimization.h"

using namespace std;
using namespace irLib::irMath;
using namespace irLib::irDyn;

namespace UAVTG
{
	namespace UAVTrajectory
	{
		/*!
			HexarotorPTPOptimization class
		*/
		void HexarotorPTPOptimization::makeObjectiveFunction()
		{
			_objectiveFunc = FunctionPtr(new HexarotorInputObjective(this));
		}

		void HexarotorPTPOptimization::makeIneqConstraintFunction()
		{
			_InputIneqFunc = FunctionPtr(new HexarotorInputConstraint(this));
			_IneqFunc = FunctionPtr(new AugmentedFunction());
			static_pointer_cast<AugmentedFunction>(_IneqFunc)->addFunction(_InputIneqFunc);
			for (unsigned int i = 0; i < _SphereObstacleConFunc.size(); i++)
				static_pointer_cast<AugmentedFunction>(_IneqFunc)->addFunction(_SphereObstacleConFunc[i]);
			//_IneqFunc = _SphereObstacleConFunc[0];
		}
		
		SharedResource * HexarotorPTPOptimization::CreateSharedResource()
		{
			_shared = new HexarotorSharedResource(this);
			return _shared;
		}

		/*!
			HexarotorSharedResource class
		*/
		void HexarotorSharedResource::update(const irLib::irMath::VectorX & params)
		{
			if (_params.size() == 0 || !_params.isApprox(params))
			{
				_params = params;

				// make b-spline using new parameter values
				makeBSpline(_params);

				for (unsigned int i = 0; i < _PTPOptimizer->_numOfSamples; i++)
				{
					_ParamState[i]->_q = _qSpline(_PTPOptimizer->_integrator->GetPoints()[i]);
					_ParamState[i]->_qdot = _qdotSpline(_PTPOptimizer->_integrator->GetPoints()[i]) / _PTPOptimizer->_tf;
					_ParamState[i]->_qddot = _qddotSpline(_PTPOptimizer->_integrator->GetPoints()[i]) / (_PTPOptimizer->_tf * _PTPOptimizer->_tf);

					if (!(_PTPOptimizer->_setTravelingTime)) // if final time(tf) is parameter
					{
						calculatedqdtf(_params(_params.size() - 1));

						_ParamState[i]->_dqdp = _dqdp[i];


						_ParamState[i]->_dqdotdp = _dqdotdp[i] / _PTPOptimizer->_tf;
						_ParamState[i]->_dqdotdp.col(_PTPOptimizer->_dimOfParams - 1) = _ParamState[i]->_dqdotdp.col(_PTPOptimizer->_dimOfParams - 1) -
							_qdotSpline(_PTPOptimizer->_integrator->GetPoints()[i]) / _PTPOptimizer->_tf / _PTPOptimizer->_tf;

						_ParamState[i]->_dqddotdp = _dqddotdp[i] / (_PTPOptimizer->_tf * _PTPOptimizer->_tf);
						_ParamState[i]->_dqddotdp.col(_PTPOptimizer->_dimOfParams - 1) = _ParamState[i]->_dqddotdp.col(_PTPOptimizer->_dimOfParams - 1) -
							2 * _qddotSpline(_PTPOptimizer->_integrator->GetPoints()[i]) / (_PTPOptimizer->_tf * _PTPOptimizer->_tf * _PTPOptimizer->_tf);
					}
					else // tf is variable
					{
						_ParamState[i]->_dqdp = _dqdp[i];
						_ParamState[i]->_dqdotdp = _dqdotdp[i] / _PTPOptimizer->_tf;
						_ParamState[i]->_dqddotdp = _dqddotdp[i] / (_PTPOptimizer->_tf * _PTPOptimizer->_tf);
					}

					_PTPOptimizer->_SE3Params->calculateVelocityValues(_ParamState[i], _UAVState[i]);
					
					_input[i] = _PTPOptimizer->_UAV->solveUAVInverseDynamics(_UAVState[i]->_T, _UAVState[i]->_V, _UAVState[i]->_Vdot);
					_dinputdp[i] = _PTPOptimizer->_UAV->solveUAVDiffInverseDynamics(_UAVState[i]->_T, _UAVState[i]->_V, _UAVState[i]->_Vdot,
						_UAVState[i]->_dVdp, _UAVState[i]->_dVdotdp, _UAVState[i]->_Vp);
				}
			}
		}

		/*!
			Torque objective function class
		*/
		irLib::irMath::VectorX HexarotorInputObjective::func(const irLib::irMath::VectorX & params) const
		{
			const std::vector<VectorX>& input = _optimizer->_shared->getinput(params);
			const VectorX& weight = _optimizer->_integrator->GetWeights();
			VectorX fval = VectorX::Zero(1);
			
			for (unsigned int i = 0; i < _optimizer->_numOfSamples; i++)
			{
				fval(0) += weight(i) * input[i].squaredNorm();
			}
			fval(0) *= _optimizer->_tf;
			//cout << "fval : " << fval(0) << endl;
			//fval(0) *= 0.5;
			
			//static int _i = 0;
			//cout << "i : " << _i++ << ", fval : " << fval(0) << endl;
			return fval;
		}

		irLib::irMath::MatrixX HexarotorInputObjective::Jacobian(const irLib::irMath::VectorX & params) const
		{
			const vector<VectorX>& input = _optimizer->_shared->getinput(params);
			const std::vector<MatrixX>& dinputdp = _optimizer->_shared->getdinputdp(params);
			const VectorX& weight = _optimizer->_integrator->GetWeights();
			MatrixX jacobian = MatrixX::Zero(1, params.size());

			for (unsigned int i = 0; i < _optimizer->_numOfSamples; i++)
			{
				jacobian += weight(i) * 2 * input[i].transpose()*dinputdp[i];
			}
			jacobian *= _optimizer->_tf;
			//jacobian *= 0.5;

			if (!(_optimizer->_setTravelingTime))
			{
				Real fval = func(params)(0);
				fval /= _optimizer->_tf;
				jacobian(0, params.size() - 1) += fval;
			}

			return jacobian;
		}

		/*!
			Input constraint function class
		*/
		irLib::irMath::VectorX HexarotorInputConstraint::func(const irLib::irMath::VectorX & params) const
		{
			const vector<VectorX>& input = _optimizer->_shared->getinput(params);
			VectorX fval(_optimizer->_UAV->_dof * 2);
			Real upper, lower;

			for (unsigned int i = 0; i < _optimizer->_UAV->_dof; i++)
			{
				// value initialization
				fval(i) = -1.0;
				fval(_optimizer->_UAV->_dof + i) = -1.0;

				upper = RealMin;
				lower = RealMax;
				for (unsigned int j = 0; j < _optimizer->_numOfSamples; j++)
				{
					if (upper < input[j](i))
					{
						upper = input[j](i);
					}
					if (lower > input[j](i))
					{
						lower = input[j](i);
					}
				}

				fval(i) = upper - _optimizer->_UAV->_umax[i];
				fval(_optimizer->_UAV->_dof + i) = lower * (-1) + _optimizer->_UAV->_umin[i];
			}
			return fval;
		}

		irLib::irMath::MatrixX HexarotorInputConstraint::Jacobian(const irLib::irMath::VectorX & params) const
		{
			const vector<VectorX>& input = _optimizer->_shared->getinput(params);
			const std::vector<MatrixX>& dinputdp = _optimizer->_shared->getdinputdp(params);
			MatrixX jacobian(_optimizer->_UAV->_dof * 2, params.size());
			Real upper, lower;
			unsigned int upperIdx, lowerIdx;

			for (unsigned int i = 0; i < _optimizer->_UAV->_dof; i++)
			{
				upper = RealMin;
				lower = RealMax;
				for (unsigned int j = 0; j < _optimizer->_numOfSamples; j++)
				{
					if (upper < input[j](i))
					{
						upper = input[j](i);
						upperIdx = j;
					}
					if (lower > input[j](i))
					{
						lower = input[j](i);
						lowerIdx = j;
					}
				}

				jacobian.row(i) = dinputdp[upperIdx].row(i);
				jacobian.row(_optimizer->_UAV->_dof + i) = -dinputdp[lowerIdx].row(i);
			}
			return jacobian;
		}

	}
}