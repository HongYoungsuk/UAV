#pragma once

#include "SerialRobot.h"
#include <irDyn\State.h>
#include <vector>

namespace UAVDyn
{
	namespace UAV
	{
		class UAVModel;
		class Hexarotor;

		class UAVModel
		{
		public:
			UAVModel(const irLib::irMath::Real massb, const irLib::irMath::Real massr, const irLib::irMath::Real kv,
				const irLib::irMath::Real kt, const irLib::irMath::Real r) : _massb(massb), _massr(massr), _kv(kv), _kt(kt), _r(r) {}
			virtual ~UAVModel() = 0;

		public:
			irLib::irMath::Real _massb;		// body mass
			irLib::irMath::Real _massr;		// rotor mass
			irLib::irMath::Real _kv;		// motor velocity constant
			irLib::irMath::Real _kt;		// motor constant
			irLib::irMath::Real _r;			// resistance
			irLib::irMath::VectorX _umin;	// input minimum values
			irLib::irMath::VectorX _umax;	// input maximum values
			
			irLib::irMath::Matrix6X _screw;	// rotor screw
			irLib::irMath::Matrix6 _Gb;		// body inertia matrix w.r.t body frame
			irLib::irMath::Matrix6 _GT;		// total inertia w.r.t body frame
			irLib::irMath::MatrixX _n;		// axis direction


			std::vector<irLib::irMath::Matrix4> _Tbi; // SE3 from body frame to each rotor frame
			std::vector<irLib::irMath::Matrix4> _Tib; // SE3 from each rotor frame to body frame
			std::vector<irLib::irMath::Matrix6> _Gi;  // rotor inertia matrix w.r.t each rotor frame
			std::vector<irLib::irMath::Matrix6> _Gib; // rotor inertai matrix w.r.t body frame
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};

		class Hexarotor : public UAVModel
		{
		public:
			Hexarotor(const irLib::irMath::Real massb, const irLib::irMath::Real massr, const irLib::irMath::Real kv,
				const irLib::irMath::Real kt, const irLib::irMath::Real r);
			~Hexarotor() {}

		private:
			const static int _dof = 6;
			
			SerialRobot _SE3;

			irLib::irMath::SE3 _bodyT;
			irLib::irMath::se3 _bodyV;
			irLib::irMath::se3 _bodyVdot;
			irLib::irMath::se3 _gravity;
			irLib::irDyn::StatePtr _state;


		public:

		};
	}
}