#include <iostream>
#include <conio.h>
#include <UAVDyn\UAV.h>
#include <UAVTrajectory\HexarotorPTPOptimization.h>
#include <irRenderer\OSG_SimpleRender.h>

using namespace std;
using namespace irLib::irMath;
using namespace irLib::irDyn;
using namespace irLib::irRenderer;
using namespace UAVTG::UAVDyn;
using namespace UAVTG::UAVTrajectory;

int main()
{
	// setting variables
	unsigned int orderOfBSpline = 4;
	unsigned int numOfOptCP = 6;
	unsigned int numOfSamples = 100;
	Real tf = 5.0;
	MatrixX initialState(6, 3), finalState(6, 3);
	initialState.setZero(); finalState.setZero();
	finalState(0, 0) = 3; // x coordinate
	finalState(1, 0) = 3; // y coordinate
	finalState(2, 0) = 5; // z coordinate
	std::vector<Vector3> center;
	std::vector<Real> radius;
	center.push_back(Vector3(1.5, 1.5, 2.5)); radius.push_back(0.4);

	// optimization
	Hexarotor* uav = new Hexarotor();
	HexarotorPTPOptimization* hexarotorOptimizer = new HexarotorPTPOptimization(uav, orderOfBSpline, numOfOptCP, numOfSamples);
	hexarotorOptimizer->setInitialState(initialState);
	hexarotorOptimizer->setFinalState(finalState);
	hexarotorOptimizer->setTravelingTime(tf);
	for (unsigned int i = 0; i < center.size(); i++)
		hexarotorOptimizer->setSphereObstacleInequalityFun(center[i], radius[i]);
	hexarotorOptimizer->generateTrajectory();
	
	// rendering
	UAV_SimpleRender renderer(hexarotorOptimizer, 600, 600);
	renderer.getViewer().run();

	cout << "Program Complete" << endl;
	_getch();
	return 0;
}