
#include <order/OrderScheduler.hpp>


using namespace order;


int main( int argc, char* argv[] ){
	
	ros::init(argc, argv, "order_server_node");
	ros::NodeHandle node;
	ros::AsyncSpinner spinner(4);
	spinner.start();
	
	
	OrderScheduler os;
	
	
	
	ros::waitForShutdown();
	return 0;
}
