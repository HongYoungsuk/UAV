#pragma once

#include "PTPOptimization.h"

namespace UAVTG
{
	namespace UAVTrajectory
	{
		class HexarotorPTPOptimization;
		class HexarotorSharedResource;

		class HexarotorTorqueObjective;
		class HexarotorInputConstraint;

		class HexarotorPTPOptimization : public PTPOptimization
		{
			friend class HexarotorSharedResource;
			friend class HexarotorTorqueObjective;
			friend class HexarotorInputConstraint;
		public:
			HexarotorPTPOptimization(UAVTG::UAVDyn::Hexarotor* UAV, const unsigned int orderOfBSpline = 5, const unsigned int numOfOptCP = 5, const unsigned int numOfSamples = 30)
				: PTPOptimization(UAV, orderOfBSpline, numOfOptCP, numOfSamples) {}
			~HexarotorPTPOptimization() {}

			virtual void makeObjectiveFunction();
			virtual void makeIneqConstraintFunction();
			virtual SharedResource* CreateSharedResource();
			
		};

		class HexarotorSharedResource : public SharedResource
		{
		public:
			HexarotorSharedResource(HexarotorPTPOptimization* PTPOptimizer) : SharedResource(PTPOptimizer) {}
			~HexarotorSharedResource() {}

			virtual void update(const irLib::irMath::VectorX& params);
		};

		class HexarotorTorqueObjective : public irLib::irMath::Function
		{
		public:
			HexarotorTorqueObjective(HexarotorPTPOptimization* optimizer) : _optimizer(optimizer) {}
			irLib::irMath::VectorX func(const irLib::irMath::VectorX& params) const;
			irLib::irMath::MatrixX Jacobian(const irLib::irMath::VectorX& params) const;
			HexarotorPTPOptimization* _optimizer;
		};

		class HexarotorInputConstraint : public irLib::irMath::Function
		{
		public:
			HexarotorInputConstraint(HexarotorPTPOptimization* optimizer) : _optimizer(optimizer) {}
			irLib::irMath::VectorX func(const irLib::irMath::VectorX& params) const;
			irLib::irMath::MatrixX Jacobian(const irLib::irMath::VectorX& params) const;
			HexarotorPTPOptimization* _optimizer;
		};


	}
}