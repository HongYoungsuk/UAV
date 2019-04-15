#include "WayPointOptimization.h"

using namespace std;
using namespace irLib::irMath;
using namespace irLib::irDyn;
using namespace UAVTG::UAVDyn;
using namespace UAVTG::UAVTrajectory;

namespace UAVTG
{
	namespace UAVTrajectory
	{
		WayPointOptimization::WayPointOptimization(UAVTG::UAVDyn::UAVModel * UAV, WAYPOINTCONDITION waypointCondition, const unsigned int orderOfBSpline,
			const unsigned int numOfOptCP, const unsigned int numOfSamples)
			: _UAV(UAV), _waypointCondition(waypointCondition), _orderOfBSpline(orderOfBSpline), _numOfOptCP(numOfOptCP), _numOfSamples(numOfSamples)
		{
			_SE3Params = new SE3Parametrization();
			_constraintorder = 3;

			_numOfCP = _numOfOptCP + _constraintorder * 2;
			_numOfKnots = _numOfCP + _orderOfBSpline;
			_knots.resize(_numOfKnots);
			_si = 0;
			_sf = 1;

			LOGIF(_numOfCP > 2 * _orderOfBSpline, "WayPointOptimization::WayPointOptimization(). The number of control points is not enough.");

			_numOfNodes = 0;
			_numOfSectors = 0;


			// dimOfParams µµ ³Ö¾îÁà¾ßµÊ
			// tfvec µµ ¼ÂÆÃÇØÁà¾ßµÊ...
			// _totaltime µµ ¼ÂÆÃ
		}

		WayPointOptimization::~WayPointOptimization()
		{
			delete _SE3Params;
		}

		void WayPointOptimization::generateTrajectory()
		{
			makeBSplineKnots();
			LOG("Complete making B-Spline knots.");

			makeSectors();
			LOG("Complete making sectors.");



		}

		void WayPointOptimization::makeBSplineKnots()
		{
			for (unsigned int i = 0; i < _orderOfBSpline; i++)
			{
				_knots(i) = _si;
				_knots(_numOfKnots - i - 1) = _sf;
			}

			Real delta = _sf / (_numOfKnots - 2 * _orderOfBSpline + 1);
			for (unsigned int i = 0; i < _numOfKnots - 2 * _orderOfBSpline; i++)
				_knots(_orderOfBSpline + i) = delta * (i + 1);
		}

		void WayPointOptimization::makeSectors()
		{

		}

		WayPointSector::WayPointSector(WayPointOptimization* WPOptimizer, const Real tf)
			: _WPOptimizer(WPOptimizer), _tf(tf)
		{

		}

		WayPointSector::~WayPointSector(){}

	}
}