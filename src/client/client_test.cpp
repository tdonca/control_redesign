
#include <client/WorldStateClient.hpp>


using namespace client;


int main( int argc, char* argv[] ){
	
	ros::init(argc, argv, "client_test_node");
	ros::NodeHandle node;
	ros::AsyncSpinner spinner(4);
	spinner.start();
	
	
	WorldStateClient ws_client;
	
	// test below
	PlannerPart gp;
	std::vector<PlannerPart> bp;
	
	if( ws_client.getGripperPart(gp) ){
		gp.printPart();
	}
	
	
	if( ws_client.getBoxParts(bp) ){
		for( int i = 0; i < bp.size(); ++i ){
			bp[i].printPart();
		}
	}
	
	
	ros::waitForShutdown();
	return 0;
}
