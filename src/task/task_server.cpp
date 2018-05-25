
#include <task/TaskPlanner.hpp>


using namespace task;


int main( int argc, char* argv[] ){
	
	ros::init(argc, argv, "task_server_node");
	ros::NodeHandle node;
	ros::AsyncSpinner spinner(4);
	spinner.start();
	
	
	TaskPlanner os;
	
	
	
	ros::waitForShutdown();
	return 0;
}
