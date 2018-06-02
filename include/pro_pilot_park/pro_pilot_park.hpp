#pragma once
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
// #include <ypspur.h>
//
union IntAndFloat
{/*{{{*/
	int ival;
	float fval;
};/*}}}*/

class ProPilotPark
{
	public:
		ProPilotPark();
		void spin();

	private:
		// params
		double spur_vel_;
		double spur_angvel_;
		double spur_accel_;
		double spur_angaccel_;
		double angle_threshold_;
		double dist_threshold_;
		double garage_length_;
		double garage_length_tolerance_;
		double garage_position_tolerance_;
		double garage_angle_tolerance_;
		int min_pts_per_line_;
		double control_hz_ = 10.0;
		double scancallback_patience_ = 0.2;

		bool scan_recieved_ = false;
		sensor_msgs::LaserScan scan_;
		struct Point {
			Eigen::Vector2d pos;
			Eigen::Vector2d normal_vector;
			unsigned int index;
		};
		std::vector<Point> points_;
		struct Clust { int i_start; int i_end; };
		std::vector<Clust> clust_list_;
		struct Wall {
			Eigen::Vector2d mean;
			Eigen::Vector2d first_component;
			Eigen::Vector2d second_component;
			Eigen::Vector2d p1; // near side
			Eigen::Vector2d p2; // far side
		};
		std::vector<Wall> wall_list_;
		std::vector<Clust> wall_index_list_;
		struct ParkSpace {
			Eigen::Vector2d position;
			Eigen::Vector2d direction;
		};
		std::vector<ParkSpace> parkspace_list_;
		std::vector<ParkSpace> sample_list_;
		struct ParkCircle {
			Eigen::Vector2d position;
			double radius;
		};
		std::vector<ParkCircle> parkcircle_list_;
		std::vector<int> dist_error_index_;
		std::vector<int> angle_error_index_;
		ros::Time last_callback_time_;
		ros::Subscriber scan_sub_;
		ros::Publisher clustered_scan_pub_;
		ros::Publisher normal_vector_pub_;
		ros::Publisher dist_error_pc_pub_;
		ros::Publisher angle_error_pc_pub_;
		ros::Publisher walls_pub_;
		ros::Publisher ps_sphere_pub_;
		ros::Publisher ps_circle_pub_;
		ros::Publisher ps_text_pub_;
		ros::Publisher path_circle_pub_;
		ros::Publisher line_pub_;

		void scanCallback( const sensor_msgs::LaserScan::ConstPtr& msg );
		void clustering();
		void detectWalls();
		void detectParkSpace();
		void detectParkSpace( Wall& side1, Wall& back, Wall& side2 );
		void detectParkCircle();
		Eigen::Vector2d calcNormalVector( int i_start, int i_end );
		Eigen::Vector2d calcNormalVector( int i_start, int i_end, Eigen::Vector2d& mean );
		double angleError( Eigen::Vector2d& vec1, Eigen::Vector2d& vec2 );
		double distError( Eigen::Vector2d& vec1, Eigen::Vector2d& vec2 );
		void publishClusteredScan();
		void publishNormalVector();
		void publishDistError();
		void publishAngleError();
		void publishWalls();
		void publishParkSpace();
		void publishPSSphere();
		void publishPSCircle();
		void publishPSText();
		void publishPathCircle();
		void publishLine();

}; // class ProPilotPark
