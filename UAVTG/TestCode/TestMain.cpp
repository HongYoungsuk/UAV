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
	VectorX input = hexarotor.solveUAVInverseDynamics(T, V, Vdot);
	cout << input << endl;


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