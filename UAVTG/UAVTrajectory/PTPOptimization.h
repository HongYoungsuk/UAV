#pragma once

#include <irMath\Function.h>
#include <irMath\Constant.h>
#include <irMath\GaussianQuadrature.h>
#include <irMath\Interpolation.h>
#include <irMath\NonlinearOptimization.h>
#include <irUtils\Diagnostic.h>
#include <UAVDyn\UAV.h>
#include "SE3Parametrization.h"

namespace UAVTG
{
	namespace UAVTrajectory
	{
		class PTPOptimization;
		class SharedResource;
		
		class TorqueObjective;
		class InputConstraint;

		class PTPOptimization
		{
			friend class SharedResource;
			friend class TorqueObjective;
			friend class InputConstraint;
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
				void makeObjectiveFunction();

			/*!
				Make Inequality constraint functions
			*/
			void makeIneqConstraintFunction();

			/*!
				Generate joint trajectory through optimization process
			*/
			virtual void generateTrajectory();


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
			irLib::irMath::FunctionPtr _IneqFunc;
			irLib::irMath::FunctionPtr _linearIneqFunc;
			irLib::irMath::FunctionPtr _nonlinearIneqFunc;
			
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
		};

		class SharedResource
		{
		public:
			SharedResource(PTPOptimization* PTPOptimzer);
			~SharedResource(){}

		public:
			PTPOptimization* _PTPOptimizer;

			/*!
				States(SE3, generalized velocity ...) of UAV
			*/
			std::vector<UAVTG::UAVTrajectory::UAVStatePtr> _UAVState;

			/*!
				Serial open chain state(joint pos, vel, acc ..)
			*/
			std::vector<UAVTG::UAVTrajectory::ParamStatePtr> _ParamState;
			
			/*!
				Bspline
			*/
			irLib::irMath::BSpline<-1, -1, -1> _qSpline;
			irLib::irMath::BSpline<-1, -1, -1> _qdotSpline;
			irLib::irMath::BSpline<-1, -1, -1> _qddotSpline;

			/*!
				Other variables
			*/
			irLib::irMath::MatrixX _dPdP;
			irLib::irMath::MatrixX _dQdP;
			irLib::irMath::MatrixX _dRdP;
			std::vector<irLib::irMath::VectorX> _P;
			std::vector<irLib::irMath::VectorX> _Q;
			std::vector<irLib::irMath::VectorX> _R;
		};
	}

	class TorqueObjective : public irLib::irMath::Function
	{
	public:
		TorqueObjective(PTPOptimization* PTPOptimizer) : _PTPOptimizer(PTPOptimizer) {}
		irLib::irMath::VectorX func(const irLib::irMath::VectorX& params) const;
		irLib::irMath::MatrixX Jacobian(const irLib::irMath::VectorX& params) const;
		PTPOptimization * _PTPOptimizer;
	};

	class InputConstraint : public irLib::irMath::Function
	{
	public:
		InputConstraint(PTPOptimization* PTPOptimizer) : _PTPOptimizer(PTPOptimizer) {}
		irLib::irMath::VectorX func(const irLib::irMath::VectorX& params) const;
		irLib::irMath::MatrixX Jacobian(const irLib::irMath::VectorX& params) const;
		PTPOptimization * _PTPOptimizer;
	};

}
