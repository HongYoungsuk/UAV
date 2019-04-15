#include <iostream>
#include <conio.h>
#include <UAVDyn\UAV.h>
#include <UAVTrajectory\HexarotorPTPOptimization.h>
#include <irRenderer\OSG_SimpleRender.h>

#define SAVE_TRAJ

#ifdef SAVE_TRAJ
#include <fstream>
#endif

using namespace std;
using namespace irLib::irMath;
using namespace irLib::irDyn;
using namespace irLib::irRenderer;
using namespace UAVTG::UAVDyn;
using namespace UAVTG::UAVTrajectory;

void setFinalStateAndTf(Real& tf, MatrixX& finalState)
{
	//finalState(0.0) = 8; finalState(1, 0) = 8; finalState(2, 0) = 8; tf = 4;					// 1
	finalState(0.0) = 5; finalState(1, 0) = 5; finalState(2, 0) = 5; tf = 3.5;				// 2
	//finalState(0.0) = 4; finalState(1, 0) = 5; finalState(2, 0) = 6; tf = 3.5;				// 3
	//inalState(0.0) = 3; finalState(1, 0) = 3; finalState(2, 0) = 12; tf = 4;					// 4
	//finalState(0.0) = 4; finalState(1, 0) = 5; finalState(2, 0) = 8; tf = 3.5;				// 5
	//finalState(0.0) = 4; finalState(1, 0) = 5; finalState(2, 0) = 12; tf = 4;					// 6
	//finalState(0.0) = -2; finalState(1, 0) = -2; finalState(2, 0) = 3; tf = 3.5;				// 7
	//finalState(0.0) = 2; finalState(1, 0) = 2; finalState(2, 0) = 15; tf = 4;					// 8
	//finalState(0.0) = -4; finalState(1, 0) = -5; finalState(2, 0) = 6; tf = 3.5;				// 9
	//finalState(0.0) = 3; finalState(1, 0) = 0; finalState(2, 0) = 8; tf = 3.4;				// 10
	//finalState(0.0) = -4; finalState(1, 0) = -5; finalState(2, 0) = 12; tf = 4;				// 11
	//finalState(0.0) = 0; finalState(1, 0) = -3; finalState(2, 0) = 8; tf = 3.5;				// 12
}

int main()
{
	// setting variables
	unsigned int orderOfBSpline = 4;
	unsigned int numOfOptCP = 6;
	unsigned int numOfSamples = 100;
	Real tf = 5;	
	MatrixX initialState(6, 3), finalState(6, 3);
	initialState.setZero(); finalState.setZero();
	finalState(0, 0) = 12; // x coordinate
	finalState(1, 0) = 12; // y coordinate
	finalState(2, 0) = 12; // z coordinate

	setFinalStateAndTf(tf, finalState);

	std::vector<Vector3> center;
	std::vector<Real> radius;
	//center.push_back(Vector3(3.0/3.0, 3.0/3.0, 5.0/5.0)); radius.push_back(0.05);
	//center.push_back(Vector3(3.0 / 2.0, 3.0 / 2.0, 5.0 / 2.0)); radius.push_back(0.2);
	//center.push_back(Vector3(0.1, 0.1, 1)); radius.push_back(0.5);

	// optimization
	Hexarotor* uav = new Hexarotor();
	HexarotorPTPOptimization* hexarotorOptimizer = new HexarotorPTPOptimization(uav, orderOfBSpline, numOfOptCP, numOfSamples);
	hexarotorOptimizer->setInitialState(initialState);
	hexarotorOptimizer->setFinalState(finalState);
	hexarotorOptimizer->setTravelingTime(tf);
	for (unsigned int i = 0; i < center.size(); i++)
		hexarotorOptimizer->setSphereObstacleInequalityFun(center[i], radius[i]);
	hexarotorOptimizer->generateTrajectory();

	hexarotorOptimizer->checkAllInequalityConstraint();
	std::cout << "iteration : " << hexarotorOptimizer->_cntForEvaluation << std::endl;
	for (int i = 0; i < hexarotorOptimizer->_fvals.size(); i++)
	{
		//cout << "fval: " << hexarotorOptimizer->_fvals[i] << endl;
		cout << hexarotorOptimizer->_fvals[i] << endl;
	}

#ifdef SAVE_TRAJ
	ofstream fout_init, fout_final;
	fout_init.open("initialTrajectoryForPlot.txt");
	fout_final.open("finalTrajectoryForPlot.txt");
	for (int i = 0; i < hexarotorOptimizer->_initialTrajectory.size(); i++)
	{
		for (int j = 0; j < 6; j++)
			fout_init << hexarotorOptimizer->_initialTrajectory[i](j) << '\t';
		fout_init << endl;
	}
	for (int i = 0; i < hexarotorOptimizer->_finalTrajectory.size(); i++)
	{
		for (int j = 0; j < 6; j++)
			fout_final << hexarotorOptimizer->_finalTrajectory[i](j) << '\t';
		fout_final << endl;
	}
	fout_init.close();
	fout_final.close();
#endif

	
	// rendering
	UAV_SimpleRender renderer(hexarotorOptimizer, 600, 600);
	renderer.getViewer().run();

	cout << "Program Complete" << endl;
	_getch();
	return 0;
}