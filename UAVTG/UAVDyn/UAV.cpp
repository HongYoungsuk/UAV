#include "UAV.h"

using namespace std;
using namespace irLib::irMath;
using namespace irLib::irDyn;

namespace UAVTG
{
	namespace UAVDyn
	{
		Hexarotor::Hexarotor()
		{
			// initialization
			//_invDynUpToDate = 0;
			_dof = 6;			

			_massb = 1.3;
			_massr = 0.025;
			_l = 0.175;
			_Ixx = 8.1 * 1E-3;
			_Iyy = 8.1 * 1E-3;
			_Izz = 14.2 * 1E-3;
			_Ir = 4.9 * 1E-6;

			_kv = 920;
			_kt = 9.5493 / _kv;
			_r = 0.2;

			_umin.resize(_dof);
			_umax.resize(_dof);
			_screw.resize(6, _dof);
			_n.resize(3, _dof);
			_Tbi.resize(_dof);
			_Tib.resize(_dof);
			_Gi.resize(_dof);
			_Gib.resize(_dof);

			_K.resize(6, _dof);
			_C.resize(6, _dof);
			_Cinv.resize(6, _dof);

			// Set gravity
			_gravity.setZero();
			_gravity(5, 0) = -9.81;

			// Set screw & SE3
			_screw.setZero();
			_screw(2, 0) = -1;
			for (unsigned int i = 1; i < _dof; i++)
				_screw(2, i) = _screw(2, i - 1) * (-1);

			Vector3 p;
			p << _l, 0, 0;
			Real deg, alpha, beta;
			for (unsigned int i = 0; i < _dof; i++)
			{
				deg = DEG2RAD * (60 * (i + 1) - 30);
				alpha = DEG2RAD * 20;
				beta = DEG2RAD * 20;
				alpha = alpha * pow(-1, i);
				beta = beta * pow(-1, i);

				_Tbi[i].setRotation(SO3::RotZ(deg) * SO3::RotX(alpha) * SO3::RotY(beta));
				_Tbi[i].setPosition(SO3::RotZ(deg).matrix() * p);
				_Tib[i] = _Tbi[i].inverse();
			}

			// Set inertia
			_Gb(0, 0) = _Ixx; _Gb(1, 1) = _Iyy; _Gb(2, 2) = _Izz;
			_Gb(3, 3) = _massb; _Gb(4, 4) = _massb; _Gb(5, 5) = _massb;
			for (unsigned int i = 0; i < _dof; i++)
			{
				_Gi[i].setZero();
				_Gi[i](2, 2) = _Ir;
				_Gi[i](3, 3) = _massr;
				_Gi[i](4, 4) = _massr;
				_Gi[i](5, 5) = _massr;

				_Gib[i] = SE3::Ad(_Tib[i]).transpose() * _Gi[i] * SE3::Ad(_Tib[i]);
			}
			_GT = _Gb;
			for (unsigned int i = 0; i < _dof; i++)
				_GT = _GT + _Gib[i];
			_GTinv = (_GT.inverse());

			// Set rotor axis direction
			p << 0, 0, 1;
			for (unsigned int i = 0; i < _dof; i++)
				_n.col(i) = p * pow(-1, i + 1);

			// Set coefficient related to thrust
			_Kd.setIdentity();
			_Kl.setIdentity();
			_Kd = _Kd * 1.14 * 1E-7;
			_Kl = _Kl * 2.98 * 1E-6;

			for (unsigned int i = 0; i < _dof; i++)
			{
				_K.col(i).block(0, 0, 3, 1) = -_Kd * _n.col(i);
				_K.col(i).block(3, 0, 3, 1) = _Kl * _n.col(i) * pow(-1, i + 1);
			}
			for (unsigned int i = 0; i < _dof; i++)
				_C.col(i) = SE3::Ad(_Tib[i]).transpose() * _K.col(i);
			_Cinv = _C.inverse();

			// Set input min, max contraint
			_umin.setZero();
			_umax.setOnes();
			_umax = _umax * 1200.0 * 1200.0;
		}

		irLib::irMath::VectorX & Hexarotor::solveUAVInverseDynamics(irLib::irMath::SE3 & T, irLib::irMath::se3 & V, irLib::irMath::se3 & Vdot)
		{
			SE3 Tinv = T.inverse();
			_input = _Cinv * (_GT * Vdot - SE3::ad(V).transpose() * _GT * V - SE3::Ad(Tinv) * _gravity * _massb);
			return _input;
		}
	}
}