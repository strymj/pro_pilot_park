#include <pro_pilot_park/pro_pilot_park.hpp>

int main ( int argc, char **argv )
{
	ros::init ( argc, argv, "pro_pilot_park" );
	
	ProPilotPark a;
	a.spin ();

	return 0;
}
