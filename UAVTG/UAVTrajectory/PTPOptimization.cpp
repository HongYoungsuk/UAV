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

			_dimOfParams = _numOfOptCP * _SE3Params->getParamDof() + 1; // +1 means final time
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
			_dimOfParams -= 1;
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

		void PTPOptimization::setSphereObstacleInequalityFun(const irLib::irMath::Vector3 & center, const irLib::irMath::Real radius)
		{
			FunctionPtr sphereObstacleIneqCon = FunctionPtr(new SphereObstacleConstraint(this, center, radius));
			_SphereObstacleConFunc.push_back(sphereObstacleIneqCon);
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
			Real delta = tf / (_numOfKnots - 2 * _orderOfBSpline + 1);

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
			initParam.resize(_dimOfParams);
			initParam.setZero();
			
			srand(time(NULL));
			for (unsigned int i = 0; i < _SE3Params->getParamDof(); i++)
			{
				for (unsigned int j = 0; j < _numOfOptCP; j++)
				{
#ifdef USE_LIN_INIT
					initParam(_numOfOptCP * i + j) = (_finalCP[2](i) - _initialCP[2](i)) / (_numOfOptCP + 1) * (j + 1) + _initialCP[2](i);
#else
					if (j == 0)
						initParam(_numOfOptCP * i + j) = makeRandLU(_initialCP[2](i), (_finalCP[2](i) - _initialCP[2](i)) / (_numOfOptCP + 1) * (j + 1) + _initialCP[2](i));
					else
						initParam(_numOfOptCP * i + j) = makeRandLU(initParam(_numOfOptCP * i + j - 1), (_finalCP[2](i) - _initialCP[2](i)) / (_numOfOptCP + 1) * (j + 1) + _initialCP[2](i));
					//if (j == 0)
					//	initParam(_numOfOptCP * i + j) = makeRandLU(_initialCP[2](i), _finalCP[2](i));
					//else
					//	initParam(_numOfOptCP * i + j) = makeRandLU(initParam(_numOfOptCP * i + j - 1), _finalCP[2](i));
#endif
				}
			}

			//cout << "initParam size : " << initParam.size() << endl;
			//cout << endl; cout << initParam << endl << endl;

			//initParam << 0.428571, 0.857143, 1.28571, 1.71429, 2.14286, 2.57143, 
			//	0.428571, 0.857143, 1.28571, 1.71429, 2.14286, 2.57143,
			//	//1.214286, 1.42857, 2.14286, 2.85714, 3.57143, 4.28571,
			//	0.714286, 1.42857, 2.14286, 2.85714, 3.57143, 4.28571,
			//	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

			//cout << endl; cout << initParam << endl << endl;


			if(!_setTravelingTime)
				initParam(initParam.size() - 1) = _tf;
		}

		void PTPOptimization::generateTrajectory()
		{
			makeBSplineKnots();
			LOG("Complete making B-Spline knots.");

			calculateBoundaryCondition(_tf);
			LOG("Complete making boundary conditions.");

			_integrator = new GaussianQuadrature(_numOfSamples, _si, _sf);
			//_integrator = new EulerIntegrator(_numOfSamples, _si, _sf);
			LOG("Integrator ready.");

			CreateSharedResource();
			LOG("Complete shared initialization.");

			makeObjectiveFunction();
			makeIneqConstraintFunction();
			LOG("Optimization ready.");

			VectorX initX;
			makeInitialParams(initX);
			LOG("Initial guess ready.");

			//////////////////////////////////////////////////////////
			// calculate initial trajectory
			_shared->makeBSpline(initX);
			for (unsigned int i = 0; i < _numOfSamples; i++)
			{
				_initialTrajectory.push_back(_shared->_qSpline(_integrator->GetPoints()[i]));
			}
			//////////////////////////////////////////////////////////


			cout << "Initial objective function value : " << _objectiveFunc->func(initX) << endl << endl;

			_optimizer.setObjectiveFunction(_objectiveFunc);
			_optimizer.setInequalityConstraint(_IneqFunc);
			//_optimizer.setInequalityConstraint(_SphereObstacleConFunc[0]);
			LOG("================== Start optimization. ======================");
			_optimizer.solve(initX, VectorX(), VectorX());
			LOG("================== Finish optimization. =====================");

			//cout << "result params" << endl << _optimizer.resultX << endl << endl;

			//////////////////////////////////////////////////////////
			// Inequality constraint test
			unsigned int cnt = 0;
			VectorX inequalFval = _IneqFunc->func(_optimizer.resultX);
			//cout << "inequality function values" << endl << inequalFval << endl << endl;
			for (unsigned int i = 0; i < inequalFval.size(); i++)
			{
				if (inequalFval(i) > 0.0)
					cnt++;
			}
			//cout << "cnt : " << cnt << endl << endl;
			if (cnt == 0)
				cout << "cnt : " << cnt << ", All inequality constraints are satisfied." << endl;
			else
				cout << "cnt : " << cnt << ", All inequality constraints are not satisfied." << endl;
			//////////////////////////////////////////////////////////

			//////////////////////////////////////////////////////////
			// calculate final trajectory
			_shared->makeBSpline(_optimizer.resultX);
			for (unsigned int i = 0; i < _numOfSamples; i++)
			{
				_finalTrajectory.push_back(_shared->_qSpline(_integrator->GetPoints()[i]));
			}
			//////////////////////////////////////////////////////////


			//cout << "optimization parameters" << endl << _optimizer.resultX << endl << endl;
			cout << "Traveling time : " << _tf << endl;
			cout << "Final objective function value : " << _objectiveFunc->func(_optimizer.resultX) << endl;
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
				UAVStatePtr uavState = UAVStatePtr(new UAVState(_PTPOptimizer->_dimOfParams));
				ParamStatePtr paramState = ParamStatePtr(new ParamState(_PTPOptimizer->_dimOfParams));
				_UAVState.push_back(uavState);
				_ParamState.push_back(paramState);
			}
			_cp.resize(_PTPOptimizer->_SE3Params->getParamDof(), _PTPOptimizer->_numOfCP); _cp.setZero();
			_input.resize(_PTPOptimizer->_numOfSamples, VectorX::Zero(_PTPOptimizer->_UAV->_dof));
			_dinputdp.resize(_PTPOptimizer->_numOfSamples, MatrixX::Zero(_PTPOptimizer->_UAV->_dof, _PTPOptimizer->_dimOfParams));
			_dqdp.resize(_PTPOptimizer->_numOfSamples, MatrixX::Zero(_PTPOptimizer->_SE3Params->getParamDof(), _PTPOptimizer->_dimOfParams));
			_dqdotdp.resize(_PTPOptimizer->_numOfSamples, MatrixX::Zero(_PTPOptimizer->_SE3Params->getParamDof(), _PTPOptimizer->_dimOfParams));
			_dqddotdp.resize(_PTPOptimizer->_numOfSamples, MatrixX::Zero(_PTPOptimizer->_SE3Params->getParamDof(), _PTPOptimizer->_dimOfParams));

			// calculate _dqdp variables
			MatrixX cp(1, _PTPOptimizer->_numOfCP);			
			for (unsigned int i = 0; i < _PTPOptimizer->_numOfOptCP; i++)
			{
				cp.setZero();
				cp(0, _PTPOptimizer->_constraintorder + i) = 1.0; // boundary control point 3개 이후에 1 값을 넣어준다
				_qSpline = BSpline<-1, -1, -1>(_PTPOptimizer->_knots, cp);
				_qdotSpline = _qSpline.derivative();
				_qddotSpline = _qdotSpline.derivative();

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

			// calculate dPdP
			// calculate _P, _Q and _R
			// 계산할 필요 없을듯..???
		}

		void SharedResource::calculatedqdtf(const irLib::irMath::Real tf)
		{
			MatrixX cp(_PTPOptimizer->_SE3Params->getParamDof(), _PTPOptimizer->_numOfCP);
			Real delta = tf / (_PTPOptimizer->_numOfKnots - 2 * _PTPOptimizer->_orderOfBSpline + 1);
			
			cp.setZero();
			cp.col(1) = delta / (_PTPOptimizer->_orderOfBSpline - 1) * _PTPOptimizer->_initialState.col(1);
			cp.col(2) = 3 * cp.col(1) + 4 * tf * delta * delta / ((_PTPOptimizer->_orderOfBSpline - 1) * (_PTPOptimizer->_orderOfBSpline - 2)) * _PTPOptimizer->_initialState.col(2);
			cp.col(_PTPOptimizer->_numOfCP - 2) = -delta / (_PTPOptimizer->_orderOfBSpline - 1) * _PTPOptimizer->_finalState.col(1);
			cp.col(_PTPOptimizer->_numOfCP - 3) = 3 * cp.col(_PTPOptimizer->_numOfCP - 2) + 4 * tf * delta * delta / ((_PTPOptimizer->_orderOfBSpline - 1) * (_PTPOptimizer->_orderOfBSpline - 2)) * _PTPOptimizer->_finalState.col(2);

			BSpline<-1, -1, -1> qSpline(_PTPOptimizer->_knots, cp);
			BSpline<-1, -1, -1> qdotSpline = qSpline.derivative();
			BSpline<-1, -1, -1> qddotSpline = qdotSpline.derivative();

			for (unsigned int i = 0; i < _PTPOptimizer->_numOfSamples; i++)
			{
				_dqdp[i].col(_dqdp[i].cols() - 1) = qSpline(_PTPOptimizer->_integrator->GetPoints()[i]) - 
					_qdotSpline(_PTPOptimizer->_integrator->GetPoints()[i])*(_PTPOptimizer->_integrator->GetPoints()[i])/tf;
				_dqdotdp[i].col(_dqdotdp[i].cols() - 1) = qdotSpline(_PTPOptimizer->_integrator->GetPoints()[i]) - 
					_qddotSpline(_PTPOptimizer->_integrator->GetPoints()[i])*(_PTPOptimizer->_integrator->GetPoints()[i]) / tf;
				_dqddotdp[i].col(_dqddotdp[i].cols() - 1) = qddotSpline(_PTPOptimizer->_integrator->GetPoints()[i]) - 
					_qdddotSpline(_PTPOptimizer->_integrator->GetPoints()[i])*(_PTPOptimizer->_integrator->GetPoints()[i]) / tf;
			}
		}

		void SharedResource::makeBSpline(const irLib::irMath::VectorX & params)
		{
			if (!(_PTPOptimizer->_setTravelingTime))
			{
				_PTPOptimizer->_tf = params(_PTPOptimizer->_dimOfParams - 1);
				_PTPOptimizer->calculateBoundaryCondition(_PTPOptimizer->_tf);
			}

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
			_qdddotSpline = _qddotSpline.derivative();
		}

		const std::vector<irLib::irMath::VectorX>& SharedResource::getinput(const irLib::irMath::VectorX & params)
		{
			update(params);
			return _input;
		}

		const std::vector<irLib::irMath::MatrixX>& SharedResource::getdinputdp(const irLib::irMath::VectorX & params)
		{
			update(params);
			return _dinputdp;
		}

		/*
			Sphere obstacle constraint functions
		*/
		irLib::irMath::VectorX SphereObstacleConstraint::func(const irLib::irMath::VectorX & params) const
		{
			_optimizer->_shared->update(params);
			VectorX fval(_optimizer->_numOfSamples); fval.setZero();
			VectorX q;
			Real x, y, z;
			for (unsigned int i = 0; i < _optimizer->_numOfSamples; i++)
			{
				//q = _optimizer->_shared->_qSpline(_optimizer->_integrator->GetPoints()[i]);
				//x = q(0); y = q(1); z = q(2);
				//cout << "x : " << x << ", y : " << y << ", z : " << z << endl;
				x = _optimizer->_shared->_ParamState[i]->_q(0);
				y = _optimizer->_shared->_ParamState[i]->_q(1);
				z = _optimizer->_shared->_ParamState[i]->_q(2);
				fval(i) = -pow((x - _center(0)), 2) - pow((y - _center(1)), 2) - pow((z - _center(2)), 2) + pow(_radius, 2);
			}
			//cout << "fval" << endl << fval << endl << endl;
			return fval;
		}

		irLib::irMath::MatrixX SphereObstacleConstraint::Jacobian(const irLib::irMath::VectorX & params) const
		{
			_optimizer->_shared->update(params);
			MatrixX jacobian(_optimizer->_numOfSamples, params.size());
			jacobian.setZero();

			VectorX q;
			Real x, y, z;

			for (unsigned int i = 0; i < _optimizer->_numOfSamples; i++)
			{
				//q = _optimizer->_shared->_qSpline(_optimizer->_integrator->GetPoints()[i]);
				//x = q(0); y = q(1); z = q(2);
				x = _optimizer->_shared->_ParamState[i]->_q(0);
				y = _optimizer->_shared->_ParamState[i]->_q(1);
				z = _optimizer->_shared->_ParamState[i]->_q(2);
				jacobian.row(i) = -2 * (x - _center(0)) * _optimizer->_shared->_ParamState[i]->_dqdp.row(0)
					- 2 * (y - _center(1)) * _optimizer->_shared->_ParamState[i]->_dqdp.row(1)
					- 2 * (z - _center(2)) * _optimizer->_shared->_ParamState[i]->_dqdp.row(2);
			}

			return jacobian;
		}
	}
}