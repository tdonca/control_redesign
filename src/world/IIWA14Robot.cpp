#include <world/IIWA14Robot.hpp>
#include <ros/ros.h>
#include <world/StateGraph.hpp>

namespace world {
	
	std::string stateString(int state);
	
	
	std::string IIWA14Robot::getState(){
		
		return stateString(m_state);
	}
		
		
		
			
	bool IIWA14Robot::initialize( StateGraph & graph ){
		
		
		// define joint values for each state position
		
		graph.setStateJointValues( "BOX", std::vector<double>{-0.07,  0.00,  0.91,  0.00, -1.81,  0.00,  0.21,  0.00} );

		graph.setStateJointValues( "REMOVE", std::vector<double>{-0.07, -2.11,  0.91,  0.00, -1.81,  0.00,  0.21,  0.00} );

		//graph.setStateJointValues( "ASIDE", );
		
		graph.setStateJointValues( "BIN1", std::vector<double>{-1.80, -0.94, -1.65, -1.52, -0.95, -1.58, 1.44, 0.06} );

		graph.setStateJointValues( "BIN2", std::vector<double>{-1.00, -0.94, -1.65, -1.52, -0.95, -1.58, 1.44, 0.06} );

		graph.setStateJointValues( "BIN3", std::vector<double>{-0.20, -0.94, -1.65, -1.52, -0.95, -1.58, 1.44, 0.06} );

		graph.setStateJointValues( "BIN4", std::vector<double>{ 0.60, -0.94, -1.65, -1.52, -0.95, -1.58, 1.44, 0.06} );

		graph.setStateJointValues( "BIN5", std::vector<double>{ 1.40, -0.94, -1.65, -1.52, -0.95, -1.58, 1.44, 0.06} );

		graph.setStateJointValues( "FACEBIN1", std::vector<double>{-1.80, -2.11, -1.40, -1.52, -2.06, -1.58, 1.44, 0.06} );

		graph.setStateJointValues( "FACEBIN2", std::vector<double>{-1.00, -2.11, -1.40, -1.52, -2.06, -1.58, 1.44, 0.06} );

		graph.setStateJointValues( "FACEBIN3", std::vector<double>{-0.20, -2.11, -1.40, -1.52, -2.06, -1.58, 1.44, 0.06} );

		graph.setStateJointValues( "FACEBIN4", std::vector<double>{ 0.60, -2.11, -1.40, -1.52, -2.06, -1.58, 1.44, 0.06} );

		graph.setStateJointValues( "FACEBIN5", std::vector<double>{1.40, -2.11, -1.40, -1.52, -2.06, -1.58, 1.44, 0.06} );
		
		
		// move to init position (FACEBIN2)
		State starting;
		if( graph.findState( "FACEBIN2", starting ) ){
			ROS_INFO("Initializing robot to start position (IMPLEMENT!).");
			m_state = IN_TRANSITION;
			m_graph_state = "FACEBIN2";
			// wait for collision scene to load
			ros::Duration(0.5).sleep();
			m_move_group.setJointValueTarget( starting.joint_values );
			moveit::planning_interface::MoveGroupInterface::Plan plan;
			m_move_group.plan(plan);
			m_move_group.execute(plan);
		}
		else{
			ROS_ERROR("Graph error at robot setup!");
		}
		
		return true;
	}
	
	
	
	void IIWA14Robot::printRobot(){
		
		std::vector<double> j = getJoints();
		ROS_INFO(" ");
		ROS_INFO("--------%s Info ----------", getName().c_str());
		ROS_INFO("Joints: %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f", j[0], j[1], j[2], j[3], j[4], j[5], j[6], j[7]);
		ROS_INFO("State: %s", getState().c_str());	
		if( getState() == "IN_TRANSITION" ){
			ROS_INFO("Graph state: %s", getGraphState().c_str());
		}	
		ROS_INFO(" ");
		
	}
	
	std::string stateString(int state) {
		
		if( state == IIWA14Robot::INVALID ){
				return "INVALID";
		}
		else if( state == IIWA14Robot::AT_CONTAINER ){
				return "AT_CONTAINER";
		}
		else if( state == IIWA14Robot::IN_TRANSITION ){
				return "IN_TRANSITION";
		}
		else{
			return "Invalid state provided!";
		}
	}
	
}
