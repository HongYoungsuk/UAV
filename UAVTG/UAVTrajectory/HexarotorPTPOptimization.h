#pragma once

#include "PTPOptimization.h"

#define USE_MAX_IN_INPUTCNT
#define USE_ANALTIC_JACOBIAN

static int cntForEvaluation = 0;
static std::vector<irLib::irMath::Real> gFvals = std::vector<irLib::irMath::Real>();
static std::vector<irLib::irMath::VectorX> gParams = std::vector<irLib::irMath::VectorX>();

namespace UAVTG
{
	namespace UAVTrajectory
	{
		class HexarotorPTPOptimization;
		class HexarotorSharedResource;

		class HexarotorInputObjective;
		class HexarotorInputConstraint;
			   		 
		class HexarotorPTPOptimization : public PTPOptimization
		{
			friend class HexarotorSharedResource;
			friend class HexarotorInputObjective;
			friend class HexarotorInputConstraint;
		public:
			HexarotorPTPOptimization(UAVTG::UAVDyn::Hexarotor* UAV, const unsigned int orderOfBSpline = 5, const unsigned int numOfOptCP = 5, const unsigned int numOfSamples = 30)
				: PTPOptimization(UAV, orderOfBSpline, numOfOptCP, numOfSamples) {}
			~HexarotorPTPOptimization() {}

			virtual void makeObjectiveFunction();
			virtual void makeIneqConstraintFunction();
			virtual SharedResource* CreateSharedResource();
			
			//////////////////////////////////////////////////
			//////////////////////////////////////////////////
			void checkAllInequalityConstraint();
			int _cntForEvaluation;
			std::vector<irLib::irMath::Real> _fvals;
			std::vector<irLib::irMath::VectorX> _params;
			//////////////////////////////////////////////////
			//////////////////////////////////////////////////
		};

		class HexarotorSharedResource : public SharedResource
		{
		public:
			HexarotorSharedResource(HexarotorPTPOptimization* PTPOptimizer) : SharedResource(PTPOptimizer) {}
			~HexarotorSharedResource() {}

			virtual void update(const irLib::irMath::VectorX& params);
		};

		class HexarotorInputObjective : public irLib::irMath::Function
		{
		public:
			HexarotorInputObjective(HexarotorPTPOptimization* optimizer) : _optimizer(optimizer) {}
			irLib::irMath::VectorX func(const irLib::irMath::VectorX& params) const;
#ifdef USE_ANALTIC_JACOBIAN
			irLib::irMath::MatrixX Jacobian(const irLib::irMath::VectorX& params) const;
#endif
			HexarotorPTPOptimization* _optimizer;
		};

		class HexarotorInputConstraint : public irLib::irMath::Function
		{
		public:
			HexarotorInputConstraint(HexarotorPTPOptimization* optimizer) : _optimizer(optimizer) {}
			irLib::irMath::VectorX func(const irLib::irMath::VectorX& params) const;
#ifdef USE_ANALTIC_JACOBIAN
			irLib::irMath::MatrixX Jacobian(const irLib::irMath::VectorX& params) const;
#endif
			HexarotorPTPOptimization* _optimizer;
		};


	}
}