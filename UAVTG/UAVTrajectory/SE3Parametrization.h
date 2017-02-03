#pragma once

#include <irDyn\SerialOpenChain.h>

namespace UAVTG
{
	namespace UAVTrajectory
	{
		class UAVState;
		class ParamState;
		class SE3Parametrization;
		
		typedef std::shared_ptr<UAVState> UAVStatePtr;
		typedef std::shared_ptr<ParamState> ParamStatePtr;

		class UAVState
		{
		public:
			UAVState(unsigned int numOfParams)
			{
				_numOfParams = numOfParams;
				_Vp.resize(6, _numOfParams);
				_dVdp.resize(6, _numOfParams);
				_dVdotdp.resize(6, _numOfParams);
				_dVddotdp.resize(6, _numOfParams);
			}
			~UAVState(){}

			irLib::irMath::SE3 _T;
			irLib::irMath::se3 _V;
			irLib::irMath::se3 _Vdot;
			irLib::irMath::se3 _Vddot;

			irLib::irMath::MatrixX _Vp;
			irLib::irMath::MatrixX _dVdp;
			irLib::irMath::MatrixX _dVdotdp;
			irLib::irMath::MatrixX _dVddotdp;

			unsigned int _numOfParams;

		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};

		class ParamState
		{
		public:
			ParamState(unsigned int numOfParams)
			{
				_numOfParams = numOfParams;
				_q.resize(6);
				_qdot.resize(6);
				_qddot.resize(6);
				_qdddot.resize(6);

				_dqdp.resize(6, _numOfParams);
				_dqdotdp.resize(6, _numOfParams);
				_dqddotdp.resize(6, _numOfParams);
				_dqdddotdp.resize(6, _numOfParams);
			}
			~ParamState(){}

			irLib::irMath::VectorX _q;
			irLib::irMath::VectorX _qdot;
			irLib::irMath::VectorX _qddot;
			irLib::irMath::VectorX _qdddot;

			irLib::irMath::MatrixX _dqdp;
			irLib::irMath::MatrixX _dqdotdp;
			irLib::irMath::MatrixX _dqddotdp;
			irLib::irMath::MatrixX _dqdddotdp;

			unsigned int _numOfParams;
		};


		class SE3Parametrization : public irLib::irDyn::SerialOpenChain
		{
		public:
			SE3Parametrization();
			~SE3Parametrization() {}
			
			/*
				Given joint value q(member variable of ParamStatePtr), calculate SE3 T
			*/
			void calculateT(ParamStatePtr&, UAVStatePtr&);
			
			/*
				Given q, qdot, ... , dqdddotdp(member variables of ParamStatePtr), calculate V, Vdot, ..., dVddotdp(member variables of UAVStatePtr)
			*/
			void calculateVelocityValues(ParamStatePtr&, UAVStatePtr&);

			/*!
				Get functions
			*/
			unsigned int getParamDof() const { return _ParamDof; }

		private:
			unsigned int _ParamDof;
			irLib::irDyn::StatePtr _state;
		};






	}

}