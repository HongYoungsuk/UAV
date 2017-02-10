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
			_constraintorder = 3;

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
			_setTravelingTime = false;
			_tf = 10;
		}

		PTPOptimization::~PTPOptimization()
		{
			delete _SE3Params;
			delete _shared;
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
			// HAVE TO DO
		}

		void PTPOptimization::generateTrajectory()
		{
			makeBSplineKnots();
			LOG("Complete making B-Spline knots.");

			calculateBoundaryCondition(_tf);
			LOG("Complete making boundary conditions.");

			_integrator = new GaussianQuadrature(_numOfSamples, _si, _sf);
			LOG("Integrator ready.");

			CreateSharedResource();
			LOG("Complete shared initialization.");

			makeObjectiveFunction();
			makeIneqConstraintFunction();
			LOG("Optimization ready.");

			VectorX initX;
			makeInitialParams(initX);
			LOG("Initial guess ready.");

			cout << "Initial objective function value : " << _objectiveFunc->func(initX) << endl << endl;

			_optimizer.setObjectiveFunction(_objectiveFunc);
			_optimizer.setInequalityConstraint(_IneqFunc);
			LOG("Start optimization.");
			_optimizer.solve(initX, VectorX(), VectorX());
			LOG("Finish optimization.");

			cout << "Final objective function value : " << _objectiveFunc->func(_optimizer.resultX) << endl << endl;
		}


		/*!
			SharedResource Class functions
		*/
		SharedResource::SharedResource(PTPOptimization * PTPOptimzer)
		{
			// variable initialization
			_PTPOptimizer = PTPOptimzer;
			for (unsigned int i = 0; i < _PTPOptimizer->_numOfSamples; i++)
			{
				UAVStatePtr uavState = UAVStatePtr(new UAVState(_PTPOptimizer->_numOfSamples));
				ParamStatePtr paramState = ParamStatePtr(new ParamState(_PTPOptimizer->_numOfSamples));
				_UAVState.push_back(uavState);
				_ParamState.push_back(paramState);
			}
			_cp.resize(_PTPOptimizer->_SE3Params->getParamDof(), _PTPOptimizer->_dimOfParams);
			_tau.resize(_PTPOptimizer->_numOfSamples, VectorX::Zero(_PTPOptimizer->_UAV->_dof));
			_dtaudp.resize(_PTPOptimizer->_numOfSamples, MatrixX::Zero(_PTPOptimizer->_UAV->_dof, _PTPOptimizer->_dimOfParams));
			_dqdp.resize(_PTPOptimizer->_numOfSamples, MatrixX::Zero(_PTPOptimizer->_SE3Params->getParamDof(), _PTPOptimizer->_dimOfParams));
			_dqdotdp.resize(_PTPOptimizer->_numOfSamples, MatrixX::Zero(_PTPOptimizer->_SE3Params->getParamDof(), _PTPOptimizer->_dimOfParams));
			_dqddotdp.resize(_PTPOptimizer->_numOfSamples, MatrixX::Zero(_PTPOptimizer->_SE3Params->getParamDof(), _PTPOptimizer->_dimOfParams));

			MatrixX cp(1, _PTPOptimizer->_numOfCP);
			//bool checkMatrixSize = false;

			// calculate _dqdp variables and dPdP
			for (unsigned int i = 0; i < _PTPOptimizer->_numOfOptCP; i++)
			{
				cp.setZero();
				cp(0, _PTPOptimizer->_constraintorder + i) = 1.0; // boundary control point 3개 이후에 1 값을 넣어준다
				_qSpline = BSpline<-1, -1, -1>(_PTPOptimizer->_knots, cp);
				_qdotSpline = _qSpline.derivative();
				_qddotSpline = _qdotSpline.derivative();

				//if (!checkMatrixSize)
				//{
				//	_dPdP.resize(_qSpline.getControlPoints().cols() - _PTPOptimizer->_constraintorder * 2, _PTPOptimizer->_numOfOptCP);
				//	_dQdP.resize(_qdotSpline.getControlPoints().cols() - _PTPOptimizer->_constraintorder * 2 + 2, _PTPOptimizer->_numOfOptCP);
				//	_dRdP.resize(_qddotSpline.getControlPoints().cols() - _PTPOptimizer->_constraintorder * 2 + 4, _PTPOptimizer->_numOfOptCP);
				//	checkMatrixSize = true;
				//}
				//_dPdP.col(i) = _qSpline.getControlPoints().block(0, 3, 1, _dPdP.rows()).transpose();
				//_dQdP.col(i) = _qdotSpline.getControlPoints().block(0, 2, 1, _dQdP.rows()).transpose();
				//_dRdP.col(i) = _qddotSpline.getControlPoints().block(0, 1, 1, _dRdP.rows()).transpose();

				for (unsigned int j = 0; j < _PTPOptimizer->_numOfSamples; j++)
				{
					VectorX dqdp = _qSpline(_PTPOptimizer->_integrator->GetPoints()[j]);
					VectorX dqdotdp = _qdotSpline(_PTPOptimizer->_integrator->GetPoints()[j]);
					VectorX dqddotdp = _qddotSpline(_PTPOptimizer->_integrator->GetPoints()[j]);

					for (unsigned int k = 0; k < _PTPOptimizer->_SE3Params->getParamDof(); k++)
					{
						_dqdp[j](k, _PTPOptimizer->_numOfOptCP*k + i) = dqdp[0];
						_dqdotdp[j](k, _PTPOptimizer->_numOfOptCP*k + i) = dqdotdp[0];
						_dqddotdp[j](k, _PTPOptimizer->_numOfOptCP*k + i) = dqddotdp[0];
					}
				}
			}

			// calculate _P, _Q and _R
			// 계산할 필요 없을듯..???
		}

		void SharedResource::makeBSpline(const irLib::irMath::VectorX & params)
		{
			if (!(_PTPOptimizer->_setTravelingTime))
				_PTPOptimizer->calculateBoundaryCondition(params(_PTPOptimizer->_dimOfParams - 1));

			_cp.col(0) = _PTPOptimizer->_initialCP[0];
			_cp.col(1) = _PTPOptimizer->_initialCP[1];
			_cp.col(2) = _PTPOptimizer->_initialCP[2];
			_cp.col(_cp.cols() - 1) = _PTPOptimizer->_finalCP[0];
			_cp.col(_cp.cols() - 2) = _PTPOptimizer->_finalCP[1];
			_cp.col(_cp.cols() - 3) = _PTPOptimizer->_finalCP[2];

			for (unsigned int i = 0; i < _PTPOptimizer->_SE3Params->getParamDof(); i++)
			{
				for (unsigned int j = 0; j < _PTPOptimizer->_numOfOptCP; j++)
				{
					_cp(i, _PTPOptimizer->_constraintorder + j) = params(_PTPOptimizer->_numOfOptCP*i + j);
				}
			}

			_qSpline = BSpline<-1, -1, -1>(_PTPOptimizer->_knots, _cp);
			_qdotSpline = _qSpline.derivative();
			_qddotSpline = _qdotSpline.derivative();
		}

		const std::vector<irLib::irMath::VectorX>& SharedResource::gettau(const irLib::irMath::VectorX & params)
		{
			update(params);
			return _tau;
		}

		const std::vector<irLib::irMath::MatrixX>& SharedResource::getdtaudp(const irLib::irMath::VectorX & params)
		{
			update(params);
			return _dtaudp;
		}
	}
}