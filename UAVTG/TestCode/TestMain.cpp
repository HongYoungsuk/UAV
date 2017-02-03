#include <iostream>
#include <conio.h>
#include <irMath\Constant.h>
#include <irDyn\SerialOpenChain.h>
#include <irRenderer\OSG_SimpleRender.h>

#include <UAVTrajectory\SE3Parametrization.h>
#include <UAVDyn\UAV.h>

using namespace std;
using namespace irLib::irMath;
using namespace irLib::irDyn;
using namespace irLib::irRenderer;
using namespace UAVTG::UAVDyn;
using namespace UAVTG::UAVTrajectory;

int main()
{	
	UAVTG::UAVDyn::Hexarotor hexarotor;
	UAVTG::UAVTrajectory::SE3Parametrization serialRobot;

	// Inverse Dynamic of UAV test
	StatePtr state = serialRobot.makeState();
	VectorX q(6);
	q << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
	state->setJointPos(q);
	serialRobot.solveForwardKinematics(state);
	SE3 T = state->getLinkSE3(6);
	se3 V, Vdot;
	V << 1, 1, 1, 1, 1, 1;
	Vdot << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
	//VectorX input = hexarotor.solveUAVInverseDynamics(T, V, Vdot);
	//cout << input << endl;

	// SE3Parametrization calculateT, calculateVelocityValues function Test
	ParamStatePtr paramState = ParamStatePtr(new ParamState(36));
	UAVStatePtr uavState = UAVStatePtr(new UAVState(36));
	VectorX qdot(6), qddot(6);
	q << 0.00409891711786755, 0.00409891711786755, 0.00683152852977926, 0, 0, 0;
	qdot << 0.0819783423573511, 0.0819783423573511, 0.136630570595585, 0, 0, 0;
	qddot << 1.09304456476468, 1.09304456476468, 1.82174094127447, 0, 0, 0;
	MatrixX dqdp(6, 36), dqdotdp(6, 36), dqddotdp(6, 36);
	dqdp.setZero(); dqdotdp.setZero(); dqddotdp.setZero();
	for (unsigned int i = 0; i < 6; i++)
	{
		dqdp(i, i * 6) = 0.00956413994169096;
		dqdotdp(i, i * 6) = 0.191282798833819;
		dqddotdp(i, i * 6) = 2.55043731778426;
	}
	paramState->_q = q; paramState->_qdot = qdot; paramState->_qddot = qddot;
	paramState->_dqdp = dqdp; paramState->_dqdotdp = dqdotdp; paramState->_dqddotdp = dqddotdp;
	serialRobot.calculateVelocityValues(paramState, uavState);
	//cout << "V" << endl << uavState->_V << endl << endl;
	//cout << "Vdot" << endl << uavState->_Vdot << endl << endl;
	//for (unsigned int i = 0; i < 36; i++)
	//	cout << uavState->_dVdotdp.col(i) << endl << endl;



	/*
	 Test code
	*/
	//SerialOpenChain robot;
	//unsigned int dof = 1;
	//for (unsigned int i = 0; i < dof + 1; i++)
	//{
	//	LinkPtr link_tmp = LinkPtr(new Link());
	//	link_tmp->addDrawingGeomtryInfo(std::shared_ptr<Box>(new Box(0.3, 0.3, 1.5)));
	//	robot.addLink(link_tmp);
	//}

	//for (unsigned int i = 0; i < dof; i++)
	//{
	//	JointPtr J = JointPtr(new Joint());
	//	Vector6 axis;
	//	axis << 0, 0, 0, 0, 1, 0;
	//	J->setAxis(axis);
	//	robot.addJoint(J, SE3(Vector3(0, 0, 1.0)), SE3(SO3::RotX(-PI_HALF), Vector3(0, 1.0, 0)));
	//	//robot.addJoint(JointPtr(new Joint()), SE3(Vector3(0, 0, 1.0)), SE3(SO3::RotX(-PI_HALF), Vector3(0, 1.0, 0)));
	//}
	//robot.completeAssembling();

	//// rendering
	//StatePtr state = robot.makeState();
	//VectorX q(1); q << 1.5;
	//state->setJointPos(q);
	//robot.solveForwardKinematics(state);

	//OSG_simpleRender renderer(std::shared_ptr<SerialOpenChain>(&robot), state, 600, 600);
	//renderer.getViewer().run();

	cout << "Program Complete" << endl;
	_getch();
	return 0;
}