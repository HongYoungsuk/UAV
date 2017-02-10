#include <iostream>
#include <conio.h>
#include <UAVDyn\UAV.h>
#include <UAVTrajectory\HexarotorPTPOptimization.h>

using namespace std;
using namespace irLib::irMath;
using namespace irLib::irDyn;
using namespace UAVTG::UAVDyn;
using namespace UAVTG::UAVTrajectory;

int main()
{
	Hexarotor* uav = new Hexarotor();
	HexarotorPTPOptimization* hexarotorOptimizer = new HexarotorPTPOptimization(uav);


	cout << "Program Complete" << endl;
	_getch();
	return 0;
}