#include "PTPOptimization.h"

using namespace std;
using namespace irLib::irMath;
using namespace irLib::irDyn;
using namespace UAVTG::UAVDyn;
using namespace UAVTG::UAVTrajectory;

namespace UAVTG
{
	namespace UAVTrajectory
	{
		
		PTPOptimization::PTPOptimization(UAVTG::UAVDyn::UAVModel * UAV, const unsigned int orderOfBSpline, const unsigned int numOfOptCP, const unsigned int numOfSamples)
			: _UAV(UAV), _orderOfBSpline(orderOfBSpline), _numOfOptCP(numOfOptCP), _numOfSamples(numOfSamples)
		{
			_SE3Params = new SE3Parametrization();

			_numOfCP = _numOfOptCP + _constraintorder * 2;
			_numOfKnots = _numOfCP + _orderOfBSpline;
			_knots.resize(_numOfKnots);
			_si = 0;
			_sf = 1;
			LOGIF(_numOfCP > 2 * _orderOfBSpline, "EnergyOptimization::EnergyOptimization(). The number of control points is not enough.");

			_initialState.resize(_SE3Params->getParamDof(), 3);
			_finalState.resize(_SE3Params->getParamDof(), 3);
			_initialState.setZero();
			_finalState.setZero();

			_dimOfParams = _numOfOptCP * _SE3Params->getParamDof();

		}

		PTPOptimization::~PTPOptimization() 
		{
			delete _SE3Params;
		}

		void PTPOptimization::setTravelingTime(irLib::irMath::Real tf)
		{
			_tf = tf;
			_setTravelingTime = true;
			_dimOfParams += 1;
		}

		void PTPOptimization::setInitialState(const irLib::irMath::MatrixX & initialState)
		{
			LOGIF(initialState.cols() == 3 && initialState.rows() == 6, "setInintialState function error, dimension of input initialState is wrong.");
			_initialState = initialState;
		}

		void PTPOptimization::setFinalState(const irLib::irMath::MatrixX & finalState)
		{
			LOGIF(finalState.cols() == 3 && finalState.rows() == 6, "setFinalState function error, dimension of input finalState is wrong.");
			_finalState = finalState;
		}

		void PTPOptimization::makeBSplineKnots()
		{
			for (unsigned int i = 0; i < _orderOfBSpline; i++)
			{
				_knots(i) = _si;
				_knots(_numOfKnots - i - 1) = _sf;
			}

			Real delta = _sf / (_numOfKnots - 2 * _orderOfBSpline + 1);
			for (unsigned int i = 0; i < _numOfKnots - 2 * _orderOfBSpline; i++)
				_knots(_orderOfBSpline + i) = delta * (i + 1);
		}

		void PTPOptimization::calculateBoundaryCondition(irLib::irMath::Real tf)
		{
			Real delta = tf / (_numOfKnots - 2 * _orderOfBSpline);

			_initialCP.resize(3);
			_initialCP[0] = _initialState.col(0);
			_initialCP[1] = delta / (_orderOfBSpline - 1)*_initialState.col(1) + _initialCP[0];
			_initialCP[2] = 2 * delta*(delta / (_orderOfBSpline - 1) / (_orderOfBSpline - 2)*_initialState.col(2) + _initialCP[1] * (1 / (2 * delta) + 1 / delta) - _initialCP[0] / delta);

			_finalCP.resize(3);
			_finalCP[0] = _finalState.col(0);
			_finalCP[1] = -delta / (_orderOfBSpline - 1)*_finalState.col(1) + _finalCP[0];
			_finalCP[2] = 2 * delta*(delta / (_orderOfBSpline - 1) / (_orderOfBSpline - 2)*_finalState.col(2) + _finalCP[1] * (1 / (2 * delta) + 1 / delta) - _finalCP[0] / delta);
		}

		void PTPOptimization::makeInitialParams(irLib::irMath::VectorX & initParam)
		{


		}

		void PTPOptimization::makeObjectiveFunction()
		{
		}

		void PTPOptimization::makeIneqConstraintFunction()
		{
		}

		void PTPOptimization::generateTrajectory()
		{

			
		}

		SharedResource::SharedResource(PTPOptimization * PTPOptimzer)
		{


		}
	}
}