#include <pro_pilot_park/pro_pilot_park.hpp>

ProPilotPark::ProPilotPark ()
{/*{{{*/
	ros::NodeHandle nh, private_nh("~");
	private_nh.param( "spur_vel", spur_vel_, 0.1 );
	private_nh.param( "spur_angvel", spur_angvel_, M_PI * 10.0 / 180.0 );
	private_nh.param( "spur_accel", spur_accel_, 0.5 );
	private_nh.param( "spur_angaccel", spur_angaccel_, M_PI * 180.0 / 180.0 );
	private_nh.param( "angle_threshold", angle_threshold_, M_PI * 10.0 / 180.0 );
	private_nh.param( "dist_threshold", dist_threshold_, 0.05 );
	private_nh.param( "min_pts_per_line", min_pts_per_line_, 10 );
	private_nh.param( "garage_length", garage_length_, 0.45 );
	private_nh.param( "garage_length_tolerance", garage_length_tolerance_, 0.3 );
	private_nh.param( "garage_position_tolerance", garage_position_tolerance_, 0.1 );
	private_nh.param( "garage_angle_tolerance", garage_angle_tolerance_, M_PI * 10.0 / 180.0 );

	scan_sub_ = nh.subscribe( "scan", 1, &ProPilotPark::scanCallback, this );
	point_sub_ = nh.subscribe( "clicked_point", 1, &ProPilotPark::pointCallback, this );
	clustered_scan_pub_ = nh.advertise<sensor_msgs::LaserScan> ( "clustered_scan", 1 );
	normal_vector_pub_ = nh.advertise<geometry_msgs::PoseArray>("normal_vector", 1);
	dist_error_pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("dist_error", 1);
	angle_error_pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("angle_error", 1);
	walls_pub_ = nh.advertise<visualization_msgs::Marker>("wall_markers", 1);
	ps_sphere_pub_ = nh.advertise<visualization_msgs::Marker>("parkspace_markers_sphere", 1);
	ps_circle_pub_ = nh.advertise<visualization_msgs::MarkerArray>("parkspace_markers_circle", 1);
	ps_text_pub_ = nh.advertise<visualization_msgs::MarkerArray>("parkspace_markers_text", 1);
	path_circle_pub_ = nh.advertise<visualization_msgs::MarkerArray>("parkspace_markers_path", 1);
	line_pub_ = nh.advertise<visualization_msgs::Marker>("parkspace_markers_line", 1);

	circle_size_ = garage_length_ * 0.4;

	Spur_init();
	Spur_set_vel ( spur_vel_ );
	Spur_set_angvel ( spur_angvel_ );
	Spur_set_accel ( spur_accel_ );
	Spur_set_angaccel ( spur_angaccel_ );
}/*}}}*/

void ProPilotPark::scanCallback( const sensor_msgs::LaserScan::ConstPtr& msg )
{/*{{{*/
	scan_ = *msg;
	scan_recieved_ = true;
	last_callback_time_ = ros::Time::now();
}/*}}}*/

void ProPilotPark::pointCallback( const geometry_msgs::PointStamped& msg )
{/*{{{*/
	point_recieved_ = true;
	point_ = msg;
	point_last_callback_time_ = ros::Time::now();
}/*}}}*/

void ProPilotPark::spin()
{/*{{{*/
	ros::Rate looprate( control_hz_ );

	while ( ros::ok() ) {

		// if no scan data, stop
		if ( ros::Time::now() - last_callback_time_ > ros::Duration( scancallback_patience_ ) ) {
			ROS_INFO( "em stop" );
			Spur_stop();
			ros::Duration( 1.0 ).sleep();
		}


		else if ( scan_recieved_ ) {
			Spur_free();
			clustering();
			detectWalls();
			detectParkSpace();
			detectParkCircle();
			publishClusteredScan();
			publishNormalVector();
			publishDistError();
			publishAngleError();
			publishWalls();
			publishParkSpace();
			publishPathCircle();
			publishLine();
			if ( point_recieved_ ) {
				park();
			}
			scan_recieved_ = false;
		}

		ros::spinOnce();
		looprate.sleep();
	}
}/*}}}*/

void ProPilotPark::park()
{/*{{{*/
	if ( ros::Time::now() - point_last_callback_time_ > ros::Duration( 4.0 ) ) {
		point_recieved_ = false;
		return;
	}

	if ( point_.header.frame_id != scan_.header.frame_id ) {
		ROS_INFO( "clicked point frame is not valid" );
		return;
	}

	Eigen::Vector2d clicked_point;
	clicked_point[0] = point_.point.x;
	clicked_point[1] = point_.point.y;

	for ( const auto parkspace : parkspace_list_ ) {
		double error = ( clicked_point - parkspace.position ).norm();
		if ( error < circle_size_ / 2.0 ) {
			doPark( parkspace );
		}
	}
}/*}}}*/

void ProPilotPark::doPark( ParkSpace ps )
{/*{{{*/
	Eigen::Vector2d target = ps.position + ps.direction * garage_length_ / 2.0;
	Eigen::Vector2d e;
	double offset = -0.15;
	e[0] =  ps.direction[1];
	e[1] = -ps.direction[0];
	int sign = 1;
	if ( 0.0 < target[1] ) {
		e *= -1;
		sign = -1;
	}
	ParkSpace sample;
	sample.position = target;
	sample.direction = parkspace_list_[0].direction;
	sample_list_.push_back( sample );
	sample.direction = e;
	sample_list_.push_back( sample );

	double kei2 = pow( e[0], 2 ) + pow( e[1] - sign, 2 ) - 4.0;
	double kei1 = e[0]  * ( target[0] - offset ) + ( e[1] - sign ) * target[1];
	double kei0 = pow( target[0] - offset, 2 ) + pow( target[1], 2 );

	double radius1 = ( - kei1 + sqrt( kei1 * kei1 - kei2 * kei0 ) ) / kei2;
	double radius2 = ( - kei1 - sqrt( kei1 * kei1 - kei2 * kei0 ) ) / kei2;
	double radius = std::max( fabs(radius1), fabs(radius2) );

	Eigen::Vector2d center1;
	center1[0] = offset;
	center1[1] = radius * sign;
	ROS_INFO( "radius = %f, sign = %d", radius, sign );
	Eigen::Vector2d center2 = target + radius * e;

	center1[0] -= offset;
	center2[0] -= offset;

	Eigen::Vector2d diff1 = center2 - center1;
	double angle1 = atan2( diff1[1] , diff1[0] );
	ROS_INFO( "ang = %f", angle1 * 180.0 / M_PI );
	Eigen::Vector2d diff2 = target - center2;
	double angle2 = atan2( diff2[1] , diff2[0] );

	Spur_set_vel ( spur_vel_ );
	Spur_set_angvel ( spur_angvel_ );
	Spur_set_accel ( spur_accel_ );
	Spur_set_angaccel ( spur_angaccel_ );


	Spur_set_pos_GL( 0.0, 0.0, 0.0 );
	Spur_circle_GL( center1[0], center1[1], sign * radius );
	while( ! Spur_over_line_GL( center1[0], center1[1], angle1 + sign * M_PI / 2.0 ) ) {
		usleep( 10000 );
	}

	Spur_set_vel ( -spur_vel_ );

	Spur_circle_GL( center2[0], center2[1], - sign * radius );
	while( ! Spur_over_line_GL( center2[0], center2[1], angle2 + sign * M_PI / 2.0 ) ) {
		usleep( 10000 );
	}

	Eigen::Vector2d goal = ps.position;
	goal[0] -= offset;
	double angle3 = atan2( ps.direction[1] , ps.direction[0] );

	Spur_line_GL( goal[0], goal[1], angle3 );
	while( ! Spur_over_line_GL( goal[0], goal[1], angle3 + M_PI ) ) {
		usleep( 10000 );
	}

	Spur_stop();

}/*}}}*/

void ProPilotPark::clustering()
{/*{{{*/
	points_.clear();
	for ( int range_index=0; range_index<scan_.ranges.size(); ++range_index ) {
		double range = scan_.ranges[range_index];
		if ( std::isnan( range ) || std::isinf( range ) ) continue;
		double angle = scan_.angle_min + range_index * scan_.angle_increment;
		Point point;
		point.pos.x() = range * cos( angle );
		point.pos.y() = range * sin( angle );
		if ( std::isnan( point.pos.x() ) || std::isinf( point.pos.x() ) ) continue;
		if ( std::isnan( point.pos.y() ) || std::isinf( point.pos.y() ) ) continue;
		// ROS_INFO( " pos : %f, %f", point.pos.x(), point.pos.y() );
		point.index = range_index;
		points_.push_back ( point );
	}

	int half_n = 9 / 2;
	for ( int point_index=0; point_index<points_.size(); ++point_index ) {
		points_[point_index].normal_vector = calcNormalVector( point_index-half_n, point_index+half_n );
	}

	dist_error_index_.clear();
	angle_error_index_.clear();
	clust_list_.clear();
	int i_start = 0;
	int points_per_cluster = 1;
	for ( int point_index=1; point_index<points_.size(); ++point_index ) {
		// connected
		if ( angleError( points_[point_index-1].normal_vector, points_[point_index].normal_vector ) < angle_threshold_
				&& distError( points_[point_index-1].pos, points_[point_index].pos ) < dist_threshold_ ) {
			++points_per_cluster;
		}

		// not connected
		else {
			if ( min_pts_per_line_ < points_per_cluster ) {
				Clust clust;
				clust.i_start = i_start;
				clust.i_end = point_index-1;
				clust_list_.push_back( clust );
			}
			i_start = point_index;
			points_per_cluster = 1;
		}
	} // end for
}/*}}}*/

void ProPilotPark::detectWalls()
{/*{{{*/
	wall_list_.clear();
	wall_index_list_.clear();
	Eigen::Vector2d diff1, diff2;
	for ( const auto& clust : clust_list_ ) {
		Wall wall;
		wall.second_component = calcNormalVector( clust.i_start, clust.i_end, wall.mean );
		wall.first_component[0] = -wall.second_component[1];
		wall.first_component[1] =  wall.second_component[0];
		if ( 0.0 < wall.mean.dot( wall.first_component ) ) wall.first_component *= -1;
		if ( 0.0 < wall.mean.dot( wall.second_component ) ) wall.second_component *= -1;
		diff1 = wall.mean - points_[clust.i_start].pos;
		diff2 = wall.mean - points_[clust.i_end].pos;
		double norm1 = wall.first_component.dot( diff1 );
		double norm2 = wall.first_component.dot( diff2 );
		if ( norm1 < norm2 ) {
			double tmp = norm1;
			norm1 = norm2;
			norm2 = tmp;
		}
		wall.p1 = wall.mean + norm1 * wall.first_component;
		wall.p2 = wall.mean + norm2 * wall.first_component;
		wall_list_.push_back( wall );
		wall_index_list_.push_back( clust );
	}
}/*}}}*/

void ProPilotPark::detectParkSpace( Wall& side1, Wall& back, Wall& side2 )
{/*{{{*/
	if ( garage_angle_tolerance_ < angleError( side1.first_component, back.second_component ) ) return;
	if ( garage_angle_tolerance_ < angleError( back.second_component, side2.first_component ) ) return;
	if ( garage_angle_tolerance_ < angleError( side2.first_component, side1.first_component ) ) return;
	if ( garage_length_ * garage_length_tolerance_< fabs( distError( side1.p1, side1.p2 ) - garage_length_ ) ) return;
	if ( garage_length_ * garage_length_tolerance_< fabs( distError( back.p1 , back.p2  ) - garage_length_ ) ) return;
	if ( garage_length_ * garage_length_tolerance_< fabs( distError( side2.p1, side2.p2 ) - garage_length_ ) ) return;
	if ( garage_length_ * garage_position_tolerance_< ( side1.mean - side2.mean ).norm() - garage_length_ ) return;
	if ( garage_length_ / 1.4142 * garage_position_tolerance_< ( side1.mean - back.mean ).norm() - garage_length_ / 1.4142) return;
	if ( garage_length_ / 1.4142 * garage_position_tolerance_< ( side2.mean - back.mean ).norm() - garage_length_ / 1.4142) return;

	ParkSpace ps;
	ps.direction = ( side1.first_component + side2.first_component ) / 2.0;
	ps.position = ( side1.p1 + side2.p1 ) / 2.0 - ps.direction * garage_length_ / 2.0;

	parkspace_list_.push_back( ps );
}/*}}}*/

void ProPilotPark::detectParkSpace()
{/*{{{*/
	parkspace_list_.clear();
	double angle_threshold = M_PI * 5.0 / 180.0;
	double dist_threshold = 0.1;
	for ( int i=2; i<wall_list_.size(); ++i ) {
		detectParkSpace( wall_list_[i-2], wall_list_[i-1], wall_list_[i] );
	}
}/*}}}*/

void ProPilotPark::detectParkCircle()
{/*{{{*/
	sample_list_.clear();
	parkcircle_list_.clear();
	if ( parkspace_list_.empty() ) return;

	Eigen::Vector2d target = parkspace_list_[0].position + parkspace_list_[0].direction * garage_length_ / 2.0;
	Eigen::Vector2d e;
	double offset = -0.15;
	e[0] =  parkspace_list_[0].direction[1];
	e[1] = -parkspace_list_[0].direction[0];
	int sign = 1;
	if ( 0.0 < target[1] ) {
		e *= -1;
		sign = -1;
	}
	ParkSpace sample;
	sample.position = target;
	sample.direction = parkspace_list_[0].direction;
	sample_list_.push_back( sample );
	sample.direction = e;
	sample_list_.push_back( sample );

	double kei2 = pow( e[0], 2 ) + pow( e[1] - sign, 2 ) - 4.0;
	double kei1 = e[0]  * ( target[0] - offset ) + ( e[1] - sign ) * target[1];
	double kei0 = pow( target[0] - offset, 2 ) + pow( target[1], 2 );

	double radius1 = ( - kei1 + sqrt( kei1 * kei1 - kei2 * kei0 ) ) / kei2;
	double radius2 = ( - kei1 - sqrt( kei1 * kei1 - kei2 * kei0 ) ) / kei2;
	double radius = std::max( fabs(radius1), fabs(radius2) );
	// double l2 = ( - kei1 - sqrt( kei1 * kei1 - kei2 * kei0 ) ) / kei2;
	// ROS_INFO( "radius = %f, %f", radius1, radius2 );
	ROS_INFO( "radius : %f", radius );
	ParkCircle pc;
	pc.radius = radius;
	pc.position = target + radius * e;
	parkcircle_list_.push_back( pc );
	pc.position[0] = offset;
	pc.position[1] = radius * sign;
	parkcircle_list_.push_back( pc );

}/*}}}*/

Eigen::Vector2d ProPilotPark::calcNormalVector( int i_start, int i_end )
{/*{{{*/
	Eigen::Vector2d mean;
	return calcNormalVector( i_start, i_end, mean );
}/*}}}*/

Eigen::Vector2d ProPilotPark::calcNormalVector( int i_start, int i_end, Eigen::Vector2d& mean )
{/*{{{*/
	// calc mean
	mean = Eigen::Vector2d::Zero(); 
	int n_points = 0;
	for ( int point_index=i_start; point_index<=i_end; ++point_index ) {
		if ( 0 <= point_index && point_index < points_.size() ) {
			// if ( std::isnan( points_[i].pos.x() ) || std::isinf( points_[i].pos.x() ) ) continue;
			// if ( std::isnan( points_[i].pos.y() ) || std::isinf( points_[i].pos.y() ) ) continue;
			mean += points_[point_index].pos;
			++n_points;
			// ROS_INFO( "current index : %d : %d", index, i );
			// ROS_INFO( "pos : %f , %f", points_[i].pos.x(), points_[i].pos.y() );
		}
	}
	if ( n_points == 0 ) {
		ROS_INFO( "No point" );
		return Eigen::Vector2d::Zero();
	}

	mean /= n_points;
	// ROS_INFO("n_points : %d, hn = %d, mean = %f, %f", n_points, hn, mean.x(), mean.y() );

	// calc variance
	Eigen::Vector2d err;
	Eigen::Vector3d var = Eigen::Vector3d::Zero();
	for ( int point_index=i_start; point_index<=i_end; ++point_index ) {
		if ( 0 <= point_index && point_index < points_.size() ) {
			err = points_[point_index].pos - mean;
			var[0] += err[0] * err[0];
			var[1] += err[0] * err[1];
			var[2] += err[1] * err[1];
		}
	}
	// ROS_INFO( "var : %f, %f, %f", var[0], var[1], var[2] );
	var /= n_points;

	Eigen::Matrix2d mat;
	mat << var[0], var[1], var[1], var[2];
	Eigen::EigenSolver<Eigen::Matrix2d> es( mat );
	auto eigenvalues = es.eigenvalues();
	auto eigenvectors = es.eigenvectors();
	Eigen::Vector2d first_component, second_component;
	if( eigenvalues[1].real() < eigenvalues[0].real() ) {
		first_component[0] = eigenvectors.col( 0 )[0].real();
		first_component[1] = eigenvectors.col( 0 )[1].real();
	}
	else {
		first_component[0] = eigenvectors.col( 1 )[0].real();
		first_component[1] = eigenvectors.col( 1 )[1].real();
	}
	first_component /= first_component.norm();
	second_component[0] = -first_component[1];
	second_component[1] =  first_component[0];

	return second_component;
}/*}}}*/

double ProPilotPark::angleError( Eigen::Vector2d& vec1, Eigen::Vector2d& vec2 )
{/*{{{*/
	double error = acos( vec1.dot( vec2 ) );
	if ( M_PI / 2 < error ) error -= M_PI;
	error = fabs( error );

	// if ( angle_threshold_ < error ) {
	// 	angle_error_index_.push_back( i );
	// }

	return error;
}/*}}}*/

double ProPilotPark::distError( Eigen::Vector2d& vec1, Eigen::Vector2d& vec2 )
{/*{{{*/
	double error = ( vec1 - vec2 ).norm();

	// if ( dist_threshold_ < error ) {
	// 	dist_error_index_.push_back( i );
	// }

	return error;
}/*}}}*/

void ProPilotPark::publishClusteredScan()
{/*{{{*/
	scan_.intensities.clear();
	scan_.intensities.assign( scan_.ranges.size(), 0.0 );
	float current_label = 0.0;
	// for ( const auto clust : clust_list_ ) {
	for ( const auto clust : wall_index_list_ ) {
		current_label += 1.0;
		for ( int i=points_[clust.i_start].index; i<=points_[clust.i_end].index; ++i ) {
			scan_.intensities[i] = current_label;
			// scan_.intensities[points_[i].index] = current_label;
		}
	}
	for ( int i=0; i<scan_.ranges.size(); ++i ) {
		if ( scan_.intensities[i] < 0.1 ) {
			scan_.ranges[i] = 0.0;
		}
	}

	clustered_scan_pub_.publish ( scan_ );
}/*}}}*/

void ProPilotPark::publishNormalVector()
{/*{{{*/
	geometry_msgs::PoseArray nv;
	geometry_msgs::Pose pose;
	geometry_msgs::Quaternion q;
	for ( const auto& point : points_ ) {
		pose.position.x = point.pos.x();
		pose.position.y = point.pos.y();
		// ROS_INFO( "pos = %f, %f", point.pos.x(), point.pos.y() );
		pose.position.z = 0.0;
		double direction = atan2( point.normal_vector.y(), point.normal_vector.x() );
		quaternionTFToMsg( tf::createQuaternionFromRPY( 0.0, 0.0, direction ), q );
		pose.orientation = q;
		nv.poses.push_back( pose );
	}

	static int seq = 0;
	nv.header = scan_.header;
	normal_vector_pub_.publish( nv );
}/*}}}*/

void ProPilotPark::publishDistError()
{/*{{{*/
	sensor_msgs::PointCloud2 pc;
	pc.header = scan_.header;

	sensor_msgs::PointField temp;
	temp.count = 1;
	temp.offset = 0;

	temp.name = "x";
	temp.datatype = sensor_msgs::PointField::FLOAT32;
	pc.fields.push_back( temp );
	temp.offset += 4;

	temp.name = "y";
	temp.datatype = sensor_msgs::PointField::FLOAT32;
	pc.fields.push_back( temp );
	temp.offset += 4;

	temp.name = "z";
	temp.datatype = sensor_msgs::PointField::FLOAT32;
	pc.fields.push_back( temp );
	temp.offset += 4;

	pc.is_bigendian = false;
	pc.is_dense = true;
	pc.point_step = 12;
	pc.height = 1;
	pc.width = 0;
	pc.row_step = pc.point_step * pc.width;

	IntAndFloat x, y, z;

	for ( int i=0; i<dist_error_index_.size(); ++i ) {
		int start_index = pc.data.size();

		++pc.width;
		pc.row_step = pc.point_step * pc.width;
		pc.data.resize( pc.row_step );

		x.fval = points_[dist_error_index_[i]].pos.x();
		y.fval = points_[dist_error_index_[i]].pos.y();
		z.fval = 0.0;
		pc.data[start_index]      = ( x.ival & 0x000000FF );
		pc.data[start_index + 1]  = ( x.ival & 0x0000FF00 ) >> 8;
		pc.data[start_index + 2]  = ( x.ival & 0x00FF0000 ) >> 16;
		pc.data[start_index + 3]  = ( x.ival & 0xFF000000 ) >> 24;
		pc.data[start_index + 4]  = ( y.ival & 0x000000FF );
		pc.data[start_index + 5]  = ( y.ival & 0x0000FF00 ) >> 8;
		pc.data[start_index + 6]  = ( y.ival & 0x00FF0000 ) >> 16;
		pc.data[start_index + 7]  = ( y.ival & 0xFF000000 ) >> 24;
		pc.data[start_index + 8]  = ( z.ival & 0x000000FF );
		pc.data[start_index + 9]  = ( z.ival & 0x0000FF00 ) >> 8;
		pc.data[start_index + 10] = ( z.ival & 0x00FF0000 ) >> 16;
		pc.data[start_index + 11] = ( z.ival & 0xFF000000 ) >> 24;
	}

	dist_error_pc_pub_.publish( pc );
}/*}}}*/

void ProPilotPark::publishAngleError()
{/*{{{*/
	sensor_msgs::PointCloud2 pc;
	pc.header = scan_.header;

	sensor_msgs::PointField temp;
	temp.count = 1;
	temp.offset = 0;

	temp.name = "x";
	temp.datatype = sensor_msgs::PointField::FLOAT32;
	pc.fields.push_back( temp );
	temp.offset += 4;

	temp.name = "y";
	temp.datatype = sensor_msgs::PointField::FLOAT32;
	pc.fields.push_back( temp );
	temp.offset += 4;

	temp.name = "z";
	temp.datatype = sensor_msgs::PointField::FLOAT32;
	pc.fields.push_back( temp );
	temp.offset += 4;

	pc.is_bigendian = false;
	pc.is_dense = true;
	pc.point_step = 12;
	pc.height = 1;
	pc.width = 0;
	pc.row_step = pc.point_step * pc.width;

	IntAndFloat x, y, z;

	for ( int i=0; i<angle_error_index_.size(); ++i ) {
		int start_index = pc.data.size();

		++pc.width;
		pc.row_step = pc.point_step * pc.width;
		pc.data.resize( pc.row_step );

		x.fval = points_[angle_error_index_[i]].pos.x();
		y.fval = points_[angle_error_index_[i]].pos.y();
		z.fval = 0.0;
		pc.data[start_index]      = ( x.ival & 0x000000FF );
		pc.data[start_index + 1]  = ( x.ival & 0x0000FF00 ) >> 8;
		pc.data[start_index + 2]  = ( x.ival & 0x00FF0000 ) >> 16;
		pc.data[start_index + 3]  = ( x.ival & 0xFF000000 ) >> 24;
		pc.data[start_index + 4]  = ( y.ival & 0x000000FF );
		pc.data[start_index + 5]  = ( y.ival & 0x0000FF00 ) >> 8;
		pc.data[start_index + 6]  = ( y.ival & 0x00FF0000 ) >> 16;
		pc.data[start_index + 7]  = ( y.ival & 0xFF000000 ) >> 24;
		pc.data[start_index + 8]  = ( z.ival & 0x000000FF );
		pc.data[start_index + 9]  = ( z.ival & 0x0000FF00 ) >> 8;
		pc.data[start_index + 10] = ( z.ival & 0x00FF0000 ) >> 16;
		pc.data[start_index + 11] = ( z.ival & 0xFF000000 ) >> 24;
	}

	angle_error_pc_pub_.publish( pc );
}/*}}}*/

void ProPilotPark::publishWalls()
{/*{{{*/
	visualization_msgs::Marker walls;
	walls.header = scan_.header;
	walls.type = visualization_msgs::Marker::LINE_LIST;
	walls.id = 0;
	walls.action = 3;  // delete
	walls_pub_.publish( walls ); // delete

	walls.action = visualization_msgs::Marker::ADD;

	walls.scale.x = 0.03;

	walls.color.a = 1.0;
	walls.color.r = 1.0;
	walls.color.g = 1.0;
	walls.color.b = 1.0;

	geometry_msgs::Point point;
	point.z = 0.0;

	for ( const auto wall : wall_list_ ) {
		point.x = wall.p1[0];
		point.y = wall.p1[1];
		walls.points.push_back( point );
		point.x = wall.p2[0];
		point.y = wall.p2[1];
		walls.points.push_back( point );
	}

	walls_pub_.publish( walls );
}/*}}}*/

void ProPilotPark::publishParkSpace()
{/*{{{*/
	publishPSSphere();
	publishPSCircle();
	publishPSText();
}/*}}}*/

void ProPilotPark::publishPSSphere()
{/*{{{*/
	visualization_msgs::Marker ps;
	ps.header = scan_.header;
	ps.type = visualization_msgs::Marker::LINE_LIST;
	ps.id = 0;
	ps.lifetime = ros::Duration( 2.0 * 1.0/control_hz_ );
	// ps.action = 3;  // delete
	// ps_sphere_pub_.publish( ps ); // delete

	ps.action = visualization_msgs::Marker::ADD;

	ps.scale.x = garage_length_ * 0.8;

	ps.color.a = 0.3;
	ps.color.r = 0.5;
	ps.color.g = 1.0;
	ps.color.b = 0.5;

	Eigen::Vector2d pos;
	geometry_msgs::Point point;
	point.z = 0.0;

	for ( const auto parkspace : parkspace_list_ ) {
		pos = parkspace.position + parkspace.direction * garage_length_ / 2.0;
		point.x = pos[0];
		point.y = pos[1];
		ps.points.push_back( point );
		pos = parkspace.position - parkspace.direction * garage_length_ / 2.0;
		point.x = pos[0];
		point.y = pos[1];
		ps.points.push_back( point );
	}

	ps_sphere_pub_.publish( ps );
}/*}}}*/

void ProPilotPark::publishLine()
{/*{{{*/
	visualization_msgs::Marker ps;
	ps.header = scan_.header;
	ps.type = visualization_msgs::Marker::LINE_LIST;
	ps.id = 0;
	ps.lifetime = ros::Duration( 2.0 * 1.0/control_hz_ );
	// ps.action = 3;  // delete
	// ps_sphere_pub_.publish( ps ); // delete

	ps.action = visualization_msgs::Marker::ADD;

	ps.scale.x = 0.01;

	ps.color.a = 0.3;
	ps.color.r = 0.5;
	ps.color.g = 1.0;
	ps.color.b = 0.5;

	Eigen::Vector2d pos;
	geometry_msgs::Point point;
	point.z = 0.0;

	for ( const auto el : sample_list_ ) {
		point.x = el.position[0];
		point.y = el.position[1];
		ps.points.push_back( point );
		point.x = el.position[0] + el.direction[0];
		point.y = el.position[1] + el.direction[1];
		ps.points.push_back( point );
	}

	line_pub_.publish( ps );
}/*}}}*/

void ProPilotPark::publishPSCircle()
{/*{{{*/
	visualization_msgs::Marker ps;
	ps.header = scan_.header;
	ps.type = visualization_msgs::Marker::CYLINDER;
	ps.id = 0;
	ps.lifetime = ros::Duration( 2.0 * 1.0/control_hz_ );

	ps.action = visualization_msgs::Marker::ADD;

	ps.scale.x = circle_size_;
	ps.scale.y = ps.scale.x;
	ps.scale.z = 0.01;

	ps.color.a = 1.0;
	ps.color.r = 0.5;
	ps.color.g = 0.5;
	ps.color.b = 1.0;

	visualization_msgs::MarkerArray ps_array;
	for ( const auto el : parkspace_list_ ) {
		static int id = 0;
		ps.pose.position.x = el.position[0];
		ps.pose.position.y = el.position[1];
		ps.pose.position.z = 0.01;
		ps.id = id++;
		ps_array.markers.push_back( ps );
	}

	ps_circle_pub_.publish( ps_array );
}/*}}}*/

void ProPilotPark::publishPSText()
{/*{{{*/
	visualization_msgs::Marker ps;
	ps.header = scan_.header;
	ps.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	ps.id = 0;
	ps.lifetime = ros::Duration( 2.0 * 1.0/control_hz_ );

	ps.action = visualization_msgs::Marker::ADD;
	ps.text = "P";

	ps.scale.x = garage_length_ * 4.0;
	ps.scale.y = ps.scale.x;
	ps.scale.z = 0.1;

	ps.color.a = 1.0;
	ps.color.r = 1.0;
	ps.color.g = 1.0;
	ps.color.b = 1.0;

	visualization_msgs::MarkerArray ps_array;
	// double offset_ratio = 0.03;
	// double offset_x = garage_length_ * offset_ratio;
	// double offset_y = garage_length_ * offset_ratio;
	for ( const auto el : parkspace_list_ ) {
		static int id = 0;
		ps.pose.position.x = el.position[0];
		ps.pose.position.y = el.position[1];
		ps.pose.position.z = 0.03;
		ps.id = id++;
		ps_array.markers.push_back( ps );
	}

	ps_text_pub_.publish( ps_array );
}/*}}}*/

void ProPilotPark::publishPathCircle()
{/*{{{*/
	if ( parkcircle_list_.empty() ) return;
	visualization_msgs::Marker ps;
	ps.header = scan_.header;
	ps.type = visualization_msgs::Marker::CYLINDER;
	ps.id = 0;
	ps.lifetime = ros::Duration( 2.0 * 1.0/control_hz_ );

	ps.action = visualization_msgs::Marker::ADD;

	ps.scale.x = 2 * parkcircle_list_[0].radius;
	ps.scale.y = ps.scale.x;
	ps.scale.z = 0.01;

	ps.color.a = 0.3;
	ps.color.r = 1.0;
	ps.color.g = 0.5;
	ps.color.b = 0.5;

	visualization_msgs::MarkerArray ps_array;
	for ( const auto el : parkcircle_list_ ) {
		static int id = 0;
		ps.pose.position.x = el.position[0];
		ps.pose.position.y = el.position[1];
		ps.pose.position.z = 0.0;
		ps.id = id++;
		ps_array.markers.push_back( ps );
	}

	path_circle_pub_.publish( ps_array );
}/*}}}*/
