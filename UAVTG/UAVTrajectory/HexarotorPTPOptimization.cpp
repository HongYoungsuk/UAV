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
			_objectiveFunc = FunctionPtr(new HexarotorTorqueObjective(this));
		}

		void HexarotorPTPOptimization::makeIneqConstraintFunction()
		{
			_IneqFunc = FunctionPtr(new HexarotorInputConstraint(this));
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
				makeBSpline(_params);

				for (unsigned int i = 0; i < _PTPOptimizer->_numOfSamples; i++)
				{
					_ParamState[i]->_q = _qSpline(_PTPOptimizer->_integrator->GetPoints()[i]);
					_ParamState[i]->_qdot = _qdotSpline(_PTPOptimizer->_integrator->GetPoints()[i]);
					_ParamState[i]->_qddot = _qddotSpline(_PTPOptimizer->_integrator->GetPoints()[i]);
					
					_ParamState[i]->_dqdp = _dqdp[i];
					_ParamState[i]->_dqdotdp = _dqdotdp[i];
					_ParamState[i]->_dqddotdp = _dqddotdp[i];

					_PTPOptimizer->_SE3Params->calculateVelocityValues(_ParamState[i], _UAVState[i]);
					
					_tau[i] = _PTPOptimizer->_UAV->solveUAVInverseDynamics(_UAVState[i]->_T, _UAVState[i]->_V, _UAVState[i]->_Vdot);
				}

			}
		}

		/*!
			Torque objective function class
		*/
		irLib::irMath::VectorX HexarotorTorqueObjective::func(const irLib::irMath::VectorX & params) const
		{
			return irLib::irMath::VectorX();
		}

		irLib::irMath::MatrixX HexarotorTorqueObjective::Jacobian(const irLib::irMath::VectorX & params) const
		{
			return irLib::irMath::MatrixX();
		}



		/*!
			Input constraint function class
		*/
		irLib::irMath::VectorX HexarotorInputConstraint::func(const irLib::irMath::VectorX & params) const
		{
			return irLib::irMath::VectorX();
		}

		irLib::irMath::MatrixX HexarotorInputConstraint::Jacobian(const irLib::irMath::VectorX & params) const
		{
			return irLib::irMath::MatrixX();
		}

	}
}