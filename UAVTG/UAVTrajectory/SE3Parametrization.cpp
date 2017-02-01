#include "SE3Parametrization.h"

using namespace std;
using namespace irLib::irMath;
using namespace irLib::irDyn;

namespace UAVTG
{
	namespace UAVTrajectory
	{
		SE3Parametrization::SE3Parametrization() : SerialOpenChain()
		{
			unsigned int DOF = 6;
			for (unsigned int i = 0; i < DOF + 1; i++)
			{
				addLink(irLib::irDyn::LinkPtr(new irLib::irDyn::Link()));
			}

			for (unsigned int i = 0; i < 6; i++)
			{
				irLib::irDyn::JointPtr J = irLib::irDyn::JointPtr(new irLib::irDyn::Joint());
				irLib::irMath::Vector6 axis;
				axis.setZero();
				if (i == 0) axis << 0, 0, 0, 1, 0, 0;
				else if (i == 1) axis << 0, 0, 0, 0, 1, 0;
				else if (i == 2) axis << 0, 0, 0, 0, 0, 1;
				else if (i == 3) axis << 0, 0, 1, 0, 0, 0;
				else if (i == 4) axis << 0, 1, 0, 0, 0, 0;
				else if (i == 5) axis << 1, 0, 0, 0, 0, 0;
				J->setAxis(axis);
				addJoint(J, irLib::irMath::SE3(irLib::irMath::Vector3(0, 0, 0)), irLib::irMath::SE3(irLib::irMath::Vector3(0, 0, 0)));
			}

			irLib::irMath::se3 Vdot = irLib::irMath::se3::Zero();
			Vdot(5) = 9.8;
			completeAssembling(irLib::irMath::SE3(), irLib::irMath::se3::Zero(), Vdot);
		}
	}
}