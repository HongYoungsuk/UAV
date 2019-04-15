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

namespace UAVTG
{
	namespace UAVTrajectory
	{
		class WayPointOptimization;
		class WayPointNode;
		class WayPointSector;

		typedef std::shared_ptr<WayPointSector> WayPointSectorPtr;

		const enum WAYPOINTCONDITION
		{
			TIMEFREE_ORIENTFREE,
			TIMEFREE_ORIENTFIXED,
			TIMEFIXED_ORIENTFREE,
			TIMEFIXED_ORIENTFIXED
		};

		class WayPointOptimization
		{
			friend class WayPointNode;
		public:
			WayPointOptimization(UAVTG::UAVDyn::UAVModel* UAV, WAYPOINTCONDITION waypointCondition, const unsigned int orderOfBSpline = 5, const unsigned int numOfOptCP = 3, const unsigned int numOfSamples = 30);
			~WayPointOptimization();

			void setFinalTimes(const std::vector<irLib::irMath::Real>& tfvec) { _tfvec = tfvec; }
			void addFinalTimes(const irLib::irMath::Real tf) { _tfvec.push_back(tf); }
			void setNodes(const std::vector<WayPointNode>& nodes) 
			{ 
				_nodes = nodes;
				_numOfNodes = _nodes.size();
			}
			void addNodes(const WayPointNode& nodes) 
			{ 
				_nodes.push_back(nodes); 
				_numOfNodes += 1;
			}
			
			virtual void makeObjectiveFunction() = 0;
			virtual void makeIneqConstraintFunction() = 0;
			virtual void generateTrajectory();

			// trajectory generation 에서 time vector 랑 node 개수 비교해서 개수가 다르면 에러 띄우는 메세지 --> 예외처리
			// trajectory generation 에서 문제 잘 setting 되었는지 처음에 확인해주는 작업 꼭 넣어줄 것!
			// trajectory generation 에서 각 sector 에 tf 나누는 작업넣어주기

		private:
			void makeBSplineKnots();
			void makeSectors();


		private:
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

			/*!
				Objective and contraint functions
			*/
			irLib::irMath::FunctionPtr _objectiveFunc;
			irLib::irMath::FunctionPtr _InputIneqFunc;
			irLib::irMath::FunctionPtr _IneqFunc;

			/*!
				Node(initial point, waypoints, and final points)
				Sector(Object containing information about Bspline)
			*/
			std::vector<WayPointNode> _nodes;
			//std::vector<WayPointSector> _sectors;
			std::vector<WayPointSectorPtr> _sectors;
			unsigned int _numOfNodes;
			unsigned int _numOfSectors;

			/*!
				Traveling time
			*/
			std::vector<irLib::irMath::Real> _tfvec;
			irLib::irMath::Real _totaltime;

			/*!
				varialbe indicating whether final time of waypoints is fixed or not,
				and whether orientation of waypoints is fixed or not
			*/
			WAYPOINTCONDITION _waypointCondition;

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

			/*!
				the number of integration sample points
			*/
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

		/*!
			Class description : WayPointNode class means points through which UAV is passing, such as initial point, waypoints, and final point
			This class contains the information about points such as position, velocity and acceleration
		*/
		class WayPointNode
		{
		public:
			WayPointNode(const irLib::irMath::VectorX& q, const irLib::irMath::VectorX& qdot, const irLib::irMath::VectorX& qddot) : _q(q), _qdot(qdot), _qddot(qddot) {}

			void setq(const irLib::irMath::VectorX& q) { _q = q; }
			void setqdot(const irLib::irMath::VectorX& qdot) { _qdot = qdot; }
			void setqddot(const irLib::irMath::VectorX& qddot) { _qddot = qddot; }

			const irLib::irMath::VectorX& getq() const { return _q; }
			const irLib::irMath::VectorX& getqdot() const { return _qdot; }
			const irLib::irMath::VectorX& getqddot() const { return _qddot; }

		private:
			irLib::irMath::VectorX _q;
			irLib::irMath::VectorX _qdot;
			irLib::irMath::VectorX _qddot;
		};

		/*!
			Class description : WayPointSector class means Bspline which connects two nodes.
			This class contains the information about Bspline such as order and control points of Bspline.
			This class also contains the information about time-series such as dqdp, SE3, and se3 of UAV.
		*/
		class WayPointSector
		{
		public:
			WayPointSector(WayPointOptimization* WPOptimizer, const irLib::irMath::Real tf);
			~WayPointSector();


		public:
			WayPointOptimization* _WPOptimizer;

			/*!
				Traveling time of Bspline
			*/
			irLib::irMath::Real _tf;

			/*!
				Optimized parameters (vectorialized b-spline control points)
			*/
			irLib::irMath::VectorX _params;

			/*!
				B-spline control points
			*/
			irLib::irMath::MatrixX _cp;

			/*!
				Boundary control points
			*/
			std::vector<irLib::irMath::VectorX> _initialCP;
			std::vector<irLib::irMath::VectorX> _finalCP;

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
			irLib::irMath::BSpline<-1, -1, -1> _qtildaSpline;
			irLib::irMath::BSpline<-1, -1, -1> _qdottildaSpline;
			irLib::irMath::BSpline<-1, -1, -1> _qddottildaSpline;
			irLib::irMath::BSpline<-1, -1, -1> _qdddottildaSpline;

			/*!
				derivative variables
			*/
			std::vector<irLib::irMath::MatrixX> _dinputdp;
			std::vector<irLib::irMath::MatrixX> _dqtildcp;				///< this _dqtildcp means derivative of q_tilda r.w.t only control points
			std::vector<irLib::irMath::MatrixX> _dqdottildcp;
			std::vector<irLib::irMath::MatrixX> _dqddottildcp;
			std::vector<irLib::irMath::MatrixX> _dqtildp;				///< this _dqtildp means derivative of q_tilda r.w.t parameters
			std::vector<irLib::irMath::MatrixX> _dqdottildp;
			std::vector<irLib::irMath::MatrixX> _dqddottildp;

			/*!
				integrator variables
			*/
			std::shared_ptr<irLib::irMath::LinearIntegrator> _integrator;
		};
	}
}