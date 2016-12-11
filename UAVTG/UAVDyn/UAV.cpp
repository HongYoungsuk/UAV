#include <iostream>
#include "UAV.h"

using namespace irLib::irMath;
using namespace irLib::irDyn;

namespace UAVDyn
{
	namespace UAV
	{
		Hexarotor::Hexarotor(const irLib::irMath::Real massb, const irLib::irMath::Real massr, const irLib::irMath::Real kv,
			const irLib::irMath::Real kt, const irLib::irMath::Real r) : UAVModel(massb, massr, kv, kt, r)
		{
			_umin.resize(dof);

		}
	}
}