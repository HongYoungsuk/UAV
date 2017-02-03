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
			_ParamDof = 6;
			//unsigned int DOF = 6;
			for (unsigned int i = 0; i < _ParamDof + 1; i++)
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
			//Vdot(5) = 9.8;
			completeAssembling(irLib::irMath::SE3(), irLib::irMath::se3::Zero(), Vdot);
			_state = makeState();
		}

		void SE3Parametrization::calculateT(ParamStatePtr & ParamState, UAVStatePtr & UAVState)
		{
			_state->setJointPos(ParamState->_q);
			solveForwardKinematics(_state);
			UAVState->_T = _state->getLinkSE3(_ParamDof);
		}

		void SE3Parametrization::calculateVelocityValues(ParamStatePtr & ParamState, UAVStatePtr & UAVState)
		{
			unsigned int dof = ParamState->_q.size();
			unsigned int pN = ParamState->_dqdp.cols(); ///< number of parameters
			_state->setJointPos(ParamState->_q);
			_state->setJointVel(ParamState->_qdot);
			_state->setJointAcc(ParamState->_qddot);
			
			for (unsigned int i = 0; i < dof; i++)
			{
				solveJointExponentialMapping(_state, i);
			}

			se3 V(_baseV), Vdot(_baseVdot);
			_state->getLinkState(0).setLinkVel(V);
			_state->getLinkState(0).setLinkAcc(Vdot);

			UAVState->_Vp.setZero();
			UAVState->_dVdp.setZero();
			UAVState->_dVdotdp.setZero();

			for (unsigned int i = 0; i < dof; i++)
			{
				LinkState& plink = _state->getLinkState(i + 1);
				LinkState& clink = _state->getLinkState(i);
				JointState& joint = _state->getJointState(i);

				const se3& screw = _joints[i].getScrew();
				
				// calculate V, Vdot
				V = SE3::InvAd(joint.getJointExp(), V) + screw * joint.getJointVel();
				Vdot = SE3::InvAd(joint.getJointExp(), Vdot) + SE3::ad(plink.getLinkVel(), screw * joint.getJointVel())
					+ screw * joint.getJointAcc();

				plink.setLinkVel(V);
				plink.setLinkAcc(Vdot);
				
				// calculate Vp
				UAVState->_Vp = SE3::InvAd(joint.getJointExp()) * UAVState->_Vp + screw * ParamState->_dqdp.row(i);

				// calculate dVdp
				UAVState->_dVdp = SE3::InvAd(joint.getJointExp()) * UAVState->_dVdp + 
					screw * ParamState->_dqdotdp.row(i) -
					SE3::ad(screw, plink.getLinkVel()) * ParamState->_dqdp.row(i);

				// calculate dVdotdp
				UAVState->_dVdotdp = SE3::InvAd(joint.getJointExp()) * UAVState->_dVdotdp +
					screw * ParamState->_dqddotdp.row(i) -
					SE3::ad(screw, plink.getLinkVel()) * ParamState->_dqdotdp.row(i) -
					SE3::ad(screw) * UAVState->_dVdp * joint.getJointVel() -
					(SE3::ad(screw) * SE3::InvAd(joint.getJointExp(), clink.getLinkAcc())) * ParamState->_dqdp.row(i);				
			}

			UAVState->_V = V;
			UAVState->_Vdot = Vdot;
		}

	}
}