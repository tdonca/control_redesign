
#include <client/WorldStateClient.hpp>


using namespace client;


int main( int argc, char* argv[] ){
	
	ros::init(argc, argv, "client_test_node");
	ros::NodeHandle node;
	ros::AsyncSpinner spinner(4);
	spinner.start();
	
	
	WorldStateClient ws_client;
	
	// test below
	PlannerPart p;
	if( ws_client.getPartType("gear_part", p) ){
		p.printPart();
	}
	if( ws_client.getPartType("gasket_part", p) ){
		p.printPart();
	}
	if( ws_client.getPartType("piston_rod_part", p) ){
		p.printPart();
	}
	if( ws_client.getPartType("pulley_part", p) ){
		p.printPart();
	}
	if( ws_client.getPartType("disk_part", p) ){
		p.printPart();
	}
	
	
	ros::waitForShutdown();
	return 0;
}
