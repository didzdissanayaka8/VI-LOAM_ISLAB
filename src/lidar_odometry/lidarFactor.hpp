// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>



//Roll, Pitch, Yaw to quarternion //D
Eigen::Quaterniond euler2Quaternion( const double yaw,const double pitch, const double roll )
{

    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

    return q;
}


struct LidarEdgeFactor
{
	LidarEdgeFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_,
					Eigen::Vector3d last_point_b_, double s_)
		: curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_), s(s_) {}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{

		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> lpa{T(last_point_a.x()), T(last_point_a.y()), T(last_point_a.z())};
		Eigen::Matrix<T, 3, 1> lpb{T(last_point_b.x()), T(last_point_b.y()), T(last_point_b.z())};

		//Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
		q_last_curr = q_identity.slerp(T(s), q_last_curr);
		Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

		Eigen::Matrix<T, 3, 1> lp;
		lp = q_last_curr * cp + t_last_curr;

		Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
		Eigen::Matrix<T, 3, 1> de = lpa - lpb;

		residual[0] = nu.x() / de.norm();
		residual[1] = nu.y() / de.norm();
		residual[2] = nu.z() / de.norm();

		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_a_,
									   const Eigen::Vector3d last_point_b_, const double s_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarEdgeFactor, 3, 4, 3>(
			new LidarEdgeFactor(curr_point_, last_point_a_, last_point_b_, s_)));
	}

	Eigen::Vector3d curr_point, last_point_a, last_point_b;
	double s;
};

struct LidarPlaneFactor
{
	LidarPlaneFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
					 Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_, double s_)
		: curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_),
		  last_point_m(last_point_m_), s(s_)
	{
		ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
		ljm_norm.normalize();
	}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{

		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> lpj{T(last_point_j.x()), T(last_point_j.y()), T(last_point_j.z())};
		//Eigen::Matrix<T, 3, 1> lpl{T(last_point_l.x()), T(last_point_l.y()), T(last_point_l.z())};
		//Eigen::Matrix<T, 3, 1> lpm{T(last_point_m.x()), T(last_point_m.y()), T(last_point_m.z())};
		Eigen::Matrix<T, 3, 1> ljm{T(ljm_norm.x()), T(ljm_norm.y()), T(ljm_norm.z())};

		//Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
		q_last_curr = q_identity.slerp(T(s), q_last_curr);
		Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

		Eigen::Matrix<T, 3, 1> lp;
		lp = q_last_curr * cp + t_last_curr;

		residual[0] = (lp - lpj).dot(ljm);

		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_j_,
									   const Eigen::Vector3d last_point_l_, const Eigen::Vector3d last_point_m_,
									   const double s_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarPlaneFactor, 1, 4, 3>(
			new LidarPlaneFactor(curr_point_, last_point_j_, last_point_l_, last_point_m_, s_)));
	}

	Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
	Eigen::Vector3d ljm_norm;
	double s;
};

struct LidarPlaneNormFactor
{

	LidarPlaneNormFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_,
						 double negative_OA_dot_norm_) : curr_point(curr_point_), plane_unit_norm(plane_unit_norm_),
														 negative_OA_dot_norm(negative_OA_dot_norm_) {}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
		Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> point_w;
		point_w = q_w_curr * cp + t_w_curr;

		Eigen::Matrix<T, 3, 1> norm(T(plane_unit_norm.x()), T(plane_unit_norm.y()), T(plane_unit_norm.z()));
		residual[0] = norm.dot(point_w) + T(negative_OA_dot_norm);
		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d plane_unit_norm_,
									   const double negative_OA_dot_norm_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarPlaneNormFactor, 1, 4, 3>(
			new LidarPlaneNormFactor(curr_point_, plane_unit_norm_, negative_OA_dot_norm_)));
	}

	Eigen::Vector3d curr_point;
	Eigen::Vector3d plane_unit_norm;
	double negative_OA_dot_norm;
};


struct LidarDistanceFactor
{

	LidarDistanceFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d closed_point_) 
						: curr_point(curr_point_), closed_point(closed_point_){}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
		Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> point_w;
		point_w = q_w_curr * cp + t_w_curr;


		residual[0] = point_w.x() - T(closed_point.x());
		residual[1] = point_w.y() - T(closed_point.y());
		residual[2] = point_w.z() - T(closed_point.z());
		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d closed_point_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarDistanceFactor, 3, 4, 3>(
			new LidarDistanceFactor(curr_point_, closed_point_)));
	}

	Eigen::Vector3d curr_point;
	Eigen::Vector3d closed_point;
};



struct RelativeRTError
{
	RelativeRTError(double t_x, double t_y, double t_z, 
					double q_w, double q_x, double q_y, double q_z,
					double t_var, double q_var)
				  :t_x(t_x), t_y(t_y), t_z(t_z), 
				   q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z),
				   t_var(t_var), q_var(q_var){}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		

		Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
		Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
		Eigen::Quaternion<T>VO_q{T(q_w), T(q_x), T(q_y), T(q_z)};
		Eigen::Quaternion<T>error_q;

		residual[0] = (t_w_curr[0] - T(t_x)) / T(t_var);
		residual[1] = (t_w_curr[1] - T(t_y)) / T(t_var);
		residual[2] = (t_w_curr[2] - T(t_z)) / T(t_var);

		error_q=VO_q.inverse()*q_w_curr;

		residual[3] = T(2) * error_q.x() / T(q_var);
		residual[4] = T(2) * error_q.y() / T(q_var);
		residual[5] = T(2) * error_q.z() / T(q_var);

		return true;
	}

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
									   const double q_w, const double q_x, const double q_y, const double q_z,
									   const double t_var, const double q_var) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          RelativeRTError, 6, 4, 3>(
	          	new RelativeRTError(t_x, t_y, t_z, q_w, q_x, q_y, q_z, t_var, q_var)));
	}

	double t_x, t_y, t_z, t_norm;
	double q_w, q_x, q_y, q_z;
	double t_var, q_var;

};



/*struct GlobalRotationError
{
	GlobalRotationError(double q_w, double q_x, double q_y, double q_z,double q_var)
				  :q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z),
				   q_var(q_var){}

	template <typename T>
	bool operator()(const T *q, T *residual) const
	{
		

		Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
		Eigen::Quaternion<T>VO_q{T(q_w), T(q_x), T(q_y), T(q_z)};
		Eigen::Quaternion<T>error_q;

		error_q=VO_q.inverse()*q_w_curr;

		residual[0] = T(2) * error_q.x() / T(q_var);
		residual[1] = T(2) * error_q.y() / T(q_var);
		residual[2] = T(2) * error_q.z() / T(q_var);

		return true;
	}

	static ceres::CostFunction* Create(const double q_w, const double q_x, const double q_y, const double q_z,const double q_var) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          GlobalRotationError, 3, 4>(
	          	new GlobalRotationError(q_w, q_x, q_y, q_z, q_var)));
	}

	double q_w, q_x, q_y, q_z;
	double q_var;

};*/


struct GlobalRotationError
{
	GlobalRotationError(double q_w, double q_x, double q_y, double q_z,double q_var)
				  :q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z),
				   q_var(q_var){}

	template <typename T>
	bool operator()(const T *q, T *residual) const
	{		

		Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
		Eigen::Quaternion<T>VO_q{T(q_w), T(q_x), T(q_y), T(q_z)};
		Eigen::Quaternion<T>error_q;	
		
		//std::cout <<"-------VO_q[0]----- "<<VO_q.x()<< std::endl;
		//std::cout <<"-------VO_q[1]----- "<<VO_q.y()<< std::endl;
		//std::cout <<"-------VO_q[2]----- "<<VO_q.z()<< std::endl;
		//std::cout <<"-------VO_q[3]----- "<<VO_q.w()<< std::endl;


   		Eigen::Matrix<T, 3, 1> euler_param = q_w_curr.toRotationMatrix().eulerAngles(2, 1, 0);	
		Eigen::Matrix<T, 3, 1> euler = VO_q.toRotationMatrix().eulerAngles(2, 1, 0);	


		//Radians to Degrees
	 	const double PI = 3.14159265358979323846;
 		const T radians_to_degrees(180.0 / PI);

  		const T pitch(euler[2]  *radians_to_degrees);
  		const T roll(euler[1]  *radians_to_degrees);
  		const T yaw(euler_param[0]  *radians_to_degrees);

		Eigen::Matrix<T, 3, 1> rot(pitch ,roll ,yaw);

		//std::cout <<"-------pitch----- "<<pitch<< std::endl;
		//std::cout <<"-------roll----- "<<roll<< std::endl;
		//std::cout <<"-------yaw----- "<<yaw<< std::endl;

		T qR[4];
		T R[9];
		T R_new[9];

		ceres::EulerAnglesToRotationMatrix(rot.data(),3, R);

		//Change raw major to column major		
		R_new[0]=R[0];
		R_new[1]=R[3];
		R_new[2]=R[6];
		R_new[3]=R[1];
		R_new[4]=R[4];
		R_new[5]=R[7];
		R_new[6]=R[2];
		R_new[7]=R[5];
		R_new[8]=R[8];

		//std::cout <<"-------rot.data()[0]----- "<<rot.data()[0]<< std::endl;
		//std::cout <<"-------rot.data()[1]----- "<<rot.data()[1]<< std::endl;
		//std::cout <<"-------rot.data()[2]----- "<<rot.data()[2]<< std::endl;
		//std::cout <<"-------euler[1]----- "<<euler[1]<< std::endl;
		//std::cout <<"-------euler[2]----- "<<euler[2]<< std::endl;


		//ceres::AngleAxisToQuaternion(rot.data(), qR1);

		ceres::RotationMatrixToQuaternion(R_new,qR);

  		/*std::cout <<"-------R11----- "<<R[0]<< std::endl;
  		std::cout <<"-------R12----- "<<R[1]<< std::endl;
  		std::cout <<"-------R13----- "<<R[2]<< std::endl;
  		std::cout <<"-------R21----- "<<R[3]<< std::endl;
  		std::cout <<"-------R22----- "<<R[4]<< std::endl;
  		std::cout <<"-------R23----- "<<R[5]<< std::endl;
  		std::cout <<"-------R31----- "<<R[6]<< std::endl;
  		std::cout <<"-------R32----- "<<R[7]<< std::endl;
  		std::cout <<"-------R33----- "<<R[8]<< std::endl;

		std::cout <<"-------qR[1]----- "<<qR[1]<< std::endl;
		std::cout <<"-------qR[2]----- "<<qR[2]<< std::endl;
		std::cout <<"-------qR[3]----- "<<qR[3]<< std::endl;
		std::cout <<"-------qR[0]----- "<<qR[0]<< std::endl;*/

		VO_q.x()=qR[1];
		VO_q.y()=qR[2];
		VO_q.z()=qR[3];
		VO_q.w()=qR[0];

		//ceres::EulerAnglesToRotationMatrix(Cam,3, R);	
		//T R[9];
		//T Cam[3];
		//VO_q = euler2Quaternion(euler_param[0] ,euler[1] ,euler[2] );
		//Eigen::Vector3d vo(euler_param[0] ,euler[1] ,euler[2]);
    		//Eigen::AngleAxisd rollAngle(euler_param[0], Eigen::Vector3d::UnitX());
    		//Eigen::AngleAxisd pitchAngle(euler[1], Eigen::Vector3d::UnitY());
	    	//Eigen::AngleAxisd yawAngle(euler[2], Eigen::Vector3d::UnitZ());
	    	//Eigen::Quaterniond ddf = yawAngle * pitchAngle * rollAngle;


		error_q=VO_q.inverse()*q_w_curr;

		residual[0] = T(2) * error_q.x() / T(q_var);
		residual[1] = T(2) * error_q.y() / T(q_var);
		residual[2] = T(2) * error_q.z() / T(q_var);

		return true;
	}

	static ceres::CostFunction* Create(const double q_w, const double q_x, const double q_y, const double q_z,const double q_var) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          GlobalRotationError, 3, 4>(
	          	new GlobalRotationError(q_w, q_x, q_y, q_z, q_var)));
	}

	double q_w, q_x, q_y, q_z;
	double q_var;

};



