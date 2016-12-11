#pragma once

#include <irDyn\SerialOpenChain.h>

namespace UAVDyn
{
	class SerialRobot : public irLib::irDyn::SerialOpenChain
	{
	public:
		SerialRobot() : SerialOpenChain() 
		{
			unsigned int DOF = 6;
			for (unsigned int i = 0; i < DOF + 1; i++)
			{
				addLink(irLib::irDyn::LinkPtr(new irLib::irDyn::Link()));
			}
			irLib::irDyn::JointPtr J = irLib::irDyn::JointPtr(new irLib::irDyn::Joint());
			irLib::irMath::Vector6 axis;
			axis << 0, 0, 0, 1, 0, 0;
			J->setAxis(axis);
			addJoint(J, irLib::irMath::SE3(irLib::irMath::Vector3(0, 0, 0)), irLib::irMath::SE3(irLib::irMath::Vector3(0, 0, 0)));
			axis << 0, 0, 0, 0, 1, 0;
			J->setAxis(axis);
			addJoint(J, irLib::irMath::SE3(irLib::irMath::Vector3(0, 0, 0)), irLib::irMath::SE3(irLib::irMath::Vector3(0, 0, 0)));
			axis << 0, 0, 0, 0, 0, 1;
			J->setAxis(axis);
			addJoint(J, irLib::irMath::SE3(irLib::irMath::Vector3(0, 0, 0)), irLib::irMath::SE3(irLib::irMath::Vector3(0, 0, 0)));
			axis << 0, 0, 1, 0, 0, 0;
			J->setAxis(axis);
			addJoint(J, irLib::irMath::SE3(irLib::irMath::Vector3(0, 0, 0)), irLib::irMath::SE3(irLib::irMath::Vector3(0, 0, 0)));
			axis << 0, 1, 0, 0, 0, 0;
			J->setAxis(axis);
			addJoint(J, irLib::irMath::SE3(irLib::irMath::Vector3(0, 0, 0)), irLib::irMath::SE3(irLib::irMath::Vector3(0, 0, 0)));
			axis << 1, 0, 0, 0, 0, 0;
			J->setAxis(axis);
			addJoint(J, irLib::irMath::SE3(irLib::irMath::Vector3(0, 0, 0)), irLib::irMath::SE3(irLib::irMath::Vector3(0, 0, 0)));
			completeAssembling();
		}

		~SerialRobot() {}
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}