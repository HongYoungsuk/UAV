#pragma once

#include <irDyn\SerialOpenChain.h>

namespace UAVTG
{
	namespace UAVTrajectory
	{
		class SE3Parametrization;
		
		class SE3Parametrization : public irLib::irDyn::SerialOpenChain
		{
		public:
			SE3Parametrization();
			~SE3Parametrization() {}

			irLib::irMath::SE3& calculateT();

		private:
			irLib::irMath::SE3 _T;
			irLib::irMath::se3 _V;
			irLib::irMath::se3 _Vdot;



		};

	}

}