#pragma once

#include <irDyn\State.h>
#include <vector>
#include <iostream>

namespace UAVTG
{
	namespace UAVDyn
	{
		class UAVModel;
		class Hexarotor;

		class UAVModel
		{
		public:
			UAVModel() {}
			~UAVModel() {}
			virtual irLib::irMath::VectorX& solveUAVInverseDynamics(irLib::irMath::SE3&, irLib::irMath::se3&, irLib::irMath::se3&) = 0;
			virtual irLib::irMath::MatrixX solveUAVDiffInverseDynamics(irLib::irMath::SE3&, irLib::irMath::se3&, irLib::irMath::se3&, 
				irLib::irMath::MatrixX&, irLib::irMath::MatrixX&, irLib::irMath::MatrixX&) = 0;
		public:
			unsigned int _dof;

			irLib::irMath::Real _massb;		///< body mass, unit : kg
			irLib::irMath::Real _massr;		///< rotor mass, unit :  kg
			irLib::irMath::Real _l;			///< UAV length, unit : m
			irLib::irMath::Real _Ixx;		///< UAV body x-axis inertia
			irLib::irMath::Real _Iyy;		///< UAV body y-axis inertia
			irLib::irMath::Real _Izz;		///< UAV body z-axis inertia
			irLib::irMath::Real _Ir;		///< rotor z-axis inertia

			irLib::irMath::Real _kv;		///< motor velocity constant, unit : rmp/V
			irLib::irMath::Real _kt;		///< motor constant
			irLib::irMath::Real _r;			///< resistance

			irLib::irMath::VectorX _umin;	///< input minimum values
			irLib::irMath::VectorX _umax;	///< input maximum values

			irLib::irMath::Matrix6X _screw;	///< rotor screw
			irLib::irMath::MatrixX _n;		///< axis direction

			std::vector<irLib::irMath::SE3> _Tbi; ///< SE3 from body frame to each rotor frame
			std::vector<irLib::irMath::SE3> _Tib; ///< SE3 from each rotor frame to body frame

			std::vector<irLib::irMath::Matrix6, Eigen::aligned_allocator< irLib::irMath::Matrix6 >> _Gi;	///< rotor inertia matrix w.r.t each rotor frame
			std::vector<irLib::irMath::Matrix6, Eigen::aligned_allocator< irLib::irMath::Matrix6 >> _Gib;	///< rotor inertia matrix w.r.t body frame
			irLib::irMath::Matrix6 _Gb;																		///< body inertia matrix w.r.t body frame
			irLib::irMath::Matrix6 _GT;																		///< total inertia matirx w.r.t body frame
			irLib::irMath::Matrix6 _GTinv;																	///< inverse matrix of total inertia w.r.t body frame

			irLib::irMath::se3 _gravity;	///< gravity, we need gravity value for calculating inverse dynamics of UAV

			irLib::irMath::Matrix3 _Kd;		///< constant matrix related to thrust force
			irLib::irMath::Matrix3 _Kl;		///< constant matrix related to thrust force
			irLib::irMath::Matrix6X _K;		///< constant matrix related to thrust force w.r.t each rotor frame
			irLib::irMath::Matrix6X _C;		///< constant matrix related to thrust force w.r.t body frame
			irLib::irMath::Matrix6X _Cinv;	///< constant inverse matrix of _C

			irLib::irMath::VectorX _tau;	///< UAV input vector
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};

		class Hexarotor : public UAVModel
		{
		public:
			Hexarotor();
			~Hexarotor() {}
			/*
			 Inverse dynamic of UAV
			 Given T(SE3), V, and Vdot, solve motor input values
			*/
			virtual irLib::irMath::VectorX& solveUAVInverseDynamics(irLib::irMath::SE3&, irLib::irMath::se3&, irLib::irMath::se3&);

			/*
				Differential inverse dynamic of UAV
				Given T(SE3), V, Vdot, dVdp, dVdotdp, and Vp, solve gradient of motor input values w.r.t parameters
			*/
			virtual irLib::irMath::MatrixX solveUAVDiffInverseDynamics(irLib::irMath::SE3&, irLib::irMath::se3&, irLib::irMath::se3&,
				irLib::irMath::MatrixX&, irLib::irMath::MatrixX&, irLib::irMath::MatrixX&);
		public:
			
		};
	}
}