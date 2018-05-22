
#include <client/WorldStateClient.hpp>


using namespace client;


int main( int argc, char* argv[] ){
	
	ros::init(argc, argv, "client_test_node");
	ros::NodeHandle node;
	ros::AsyncSpinner spinner(4);
	spinner.start();
	
	
	WorldStateClient ws_client;
	
	// test below
	
	
	
	ros::waitForShutdown();
	return 0;
}
