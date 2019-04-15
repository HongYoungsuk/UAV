#pragma once

#include <time.h>
#include <irMath\Function.h>
#include <irMath\Constant.h>
#include <irMath\GaussianQuadrature.h>
#include <irMath\EulerIntegrator.h>
#include <irMath\Interpolation.h>
#include <irMath\NonlinearOptimization.h>
#include <irUtils\Diagnostic.h>
#include <UAVDyn\UAV.h>
#include "SE3Parametrization.h"

#define USE_LIN_INIT

namespace UAVTG
{
	namespace UAVTrajectory
	{
		class PTPOptimization;
		class SharedResource;

		class SphereObstacleConstraint;

		class PTPOptimization
		{
			friend class SharedResource;
			friend class SphereObstacleConstraint;
		public:
			PTPOptimization(UAVTG::UAVDyn::UAVModel* UAV, const unsigned int orderOfBSpline = 5, const unsigned int numOfOptCP = 5, const unsigned int numOfSamples = 30);
			~PTPOptimization();

			/*!
				Set traveling time function
				if traveling time is not set, final time is considered as optimal variable
			*/
			void setTravelingTime(irLib::irMath::Real tf);
			
			/*!
				Set functions
			*/
			void setInitialState(const irLib::irMath::MatrixX& initialState);
			void setFinalState(const irLib::irMath::MatrixX& finalState);
			void setSphereObstacleInequalityFun(const irLib::irMath::Vector3& center, const irLib::irMath::Real radius);

			/*!
				Make B-spline knots
			*/
			void makeBSplineKnots();

			/*!
				Calculate the boundary condition of B-spline controlpoints using q, qdot, qddot
			*/
			void calculateBoundaryCondition(irLib::irMath::Real tf);

			/*!
				Make initial spline
			*/
			void makeInitialParams(irLib::irMath::VectorX& initParam);

			/*!
				Make objective function
			*/
			virtual void makeObjectiveFunction() = 0;

			/*!
				Make Inequality constraint functions
			*/
			virtual void makeIneqConstraintFunction() = 0;

			/*!
				Creat shared resource
			*/
			virtual SharedResource* CreateSharedResource() = 0;

			/*!
				Generate joint trajectory through optimization process
			*/
			virtual void generateTrajectory();

			/*
				
			*/
			irLib::irMath::Real makeRandLU(irLib::irMath::Real lower, irLib::irMath::Real upper)
			{
				return lower + (upper - lower)*((double)rand() / 32767.0);
			}
		public:
			/*
				Nonlinear optimizer
			*/
			irLib::irMath::NonlinearOptimization _optimizer;
			
			/*
				Target UAV(system) for optimization
			*/
			UAVTG::UAVDyn::UAVModel* _UAV;

			/*!
				SE3 parametrization ( six dof serial open chain )
			*/
			UAVTG::UAVTrajectory::SE3Parametrization* _SE3Params;

			/*
				variable for supervising the resources
			*/
			SharedResource* _shared;			

			/*!
				Objective and contraint functions
			*/
			irLib::irMath::FunctionPtr _objectiveFunc;
			std::vector<irLib::irMath::FunctionPtr> _SphereObstacleConFunc;
			irLib::irMath::FunctionPtr _InputIneqFunc;
			irLib::irMath::FunctionPtr _IneqFunc;
			
			/*!
				traveling time
			*/
			irLib::irMath::Real _tf;
			bool _setTravelingTime;

			/*!
				Initial and final state variables
				Serial open chain joint position, velocity and acceleration
			*/
			irLib::irMath::MatrixX _initialState;
			irLib::irMath::MatrixX _finalState;

			/*!
				Bspline variables
			*/
			unsigned int _constraintorder;
			unsigned int _orderOfBSpline;
			unsigned int _numOfOptCP;
			unsigned int _numOfCP;
			unsigned int _numOfKnots;
			irLib::irMath::VectorX _knots;
			irLib::irMath::Real _si;
			irLib::irMath::Real _sf;
			std::vector<irLib::irMath::VectorX> _initialCP;
			std::vector<irLib::irMath::VectorX> _finalCP;

			/*!
				integrator variables
			*/
			irLib::irMath::LinearIntegrator* _integrator;
			unsigned int _numOfSamples;

			/*!
				variables related to parameters
			*/
			irLib::irMath::VectorX _initialParams;
			irLib::irMath::VectorX _finalParams;
			unsigned int _dimOfParams;

			/*!
				Trajectories
			*/
			std::vector<irLib::irMath::VectorX> _initialTrajectory;
			std::vector<irLib::irMath::VectorX> _finalTrajectory;
		};

		class SharedResource
		{
		public:
			SharedResource(PTPOptimization* PTPOptimzer);
			~SharedResource(){}
			
			void calculatedqdtf(const irLib::irMath::Real tf);
			virtual void makeBSpline(const irLib::irMath::VectorX& params);
			virtual void update(const irLib::irMath::VectorX& params) = 0;
			const std::vector<irLib::irMath::VectorX>& getinput(const irLib::irMath::VectorX& params);
			const std::vector<irLib::irMath::MatrixX>& getdinputdp(const irLib::irMath::VectorX& params);
		public:
			PTPOptimization* _PTPOptimizer;

			/*!
				Optimized parameters (vectorialized b-spline control points)
			*/
			irLib::irMath::VectorX _params;

			/*!
				B-spline control points
			*/
			irLib::irMath::MatrixX _cp;

			/*!
				States(SE3, generalized velocity ...) of UAV
			*/
			std::vector<UAVTG::UAVTrajectory::UAVStatePtr> _UAVState;

			/*!
				Serial open chain state(joint pos, vel, acc ..)
			*/
			std::vector<UAVTG::UAVTrajectory::ParamStatePtr> _ParamState;

			/*!
				UAV motor torque
			*/
			std::vector<irLib::irMath::VectorX> _input;
			
			/*!
				Bspline
			*/
			irLib::irMath::BSpline<-1, -1, -1> _qSpline;
			irLib::irMath::BSpline<-1, -1, -1> _qdotSpline;
			irLib::irMath::BSpline<-1, -1, -1> _qddotSpline;
			irLib::irMath::BSpline<-1, -1, -1> _qdddotSpline;

			/*!
				Other variables
			*/
			std::vector<irLib::irMath::MatrixX> _dinputdp;
			std::vector<irLib::irMath::MatrixX> _dqdp;				///< this _dqdp means derivative of q_tilda r.w.t parameters
			std::vector<irLib::irMath::MatrixX> _dqdotdp;
			std::vector<irLib::irMath::MatrixX> _dqddotdp;

			// 필요 없을 거 같은데..
			//irLib::irMath::MatrixX _dPdP;
			//irLib::irMath::MatrixX _dQdP;
			//irLib::irMath::MatrixX _dRdP;
			//std::vector<irLib::irMath::VectorX> _P;
			//std::vector<irLib::irMath::VectorX> _Q;
			//std::vector<irLib::irMath::VectorX> _R;

		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};

		class SphereObstacleConstraint : public irLib::irMath::Function
		{
		public:
			SphereObstacleConstraint(PTPOptimization* optimizer, const irLib::irMath::Vector3& center, const irLib::irMath::Real radius)
				: _optimizer(optimizer), _center(center), _radius(radius) {}
			const irLib::irMath::Vector3& getCenter() const { return _center; }
			const irLib::irMath::Real getRadius() const { return _radius; }

			irLib::irMath::VectorX func(const irLib::irMath::VectorX& params) const;
			irLib::irMath::MatrixX Jacobian(const irLib::irMath::VectorX& params) const;
		public:
			irLib::irMath::Vector3 _center;
			irLib::irMath::Real _radius;
			PTPOptimization* _optimizer;

		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};

	}

	

}
