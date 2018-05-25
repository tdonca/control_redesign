#include <world/Part.hpp>
#include <world/Container.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace world {
	
	double getPartSize( std::string type );
	std::string stateToString( int state );
	void getRPY( const geometry_msgs::Pose  pose, double & r, double & p, double & y );
	
	
	void Part::setPlaced( Container* location ){
		
		ROS_INFO("Placing Part %s in %s.", m_name.c_str(), location->getName().c_str());
		m_location = location;
		
		m_state = PLACED;
	}
	
	
	
	void Part::setGrabbed( Container* location ){
		ROS_INFO("Grabbing Part %s into %s.", m_name.c_str(), location->getName().c_str());
		m_location = location;
		
		m_state = GRABBED;
	}
	
	
	
	void Part::setRemoved(){
		ROS_INFO("Removing Part %s from the world.", m_name.c_str());
		m_location = nullptr;
		
		m_state = REMOVED;
	}
	
	
	
	std::string Part::getState() const { 
		return stateToString(m_state); 
	}
	
	
	
	double Part::getSize() const {
		return getPartSize( m_type );
	}
	
	
	void Part::printPart() {
		
		double r, p, y;
		getRPY( getPose(), r, p, y );

		ROS_INFO("Name: %s", getName().c_str());
		ROS_INFO("Type: %s", getType().c_str());
		ROS_INFO("State: %s", getState().c_str());
		ROS_INFO("Container: %s", getLocation()->getName().c_str());
		ROS_INFO("Position: %.3f, %.3f, %.3f", getPose().position.x, getPose().position.y, getPose().position.z);
		ROS_INFO("Orientation: %.3f, %.3f, %.3f, %.3f", getPose().orientation.x, getPose().orientation.y, getPose().orientation.z, getPose().orientation.w);
		ROS_INFO("RollPitchYaw: %.3f, %.3f, %.3f", r, p, y);
		if( m_faulty ){
			ROS_INFO("Faulty: YES!");
		}
		ROS_INFO(" ");
	}
	
	
	void getRPY( const geometry_msgs::Pose  pose, double & r, double & p, double & y ){
		
		tf2::Quaternion arm_q;
		tf2::fromMsg( pose.orientation, arm_q );
		tf2::Matrix3x3(arm_q).getRPY( r, p, y );
		
	}
	
	
	double getPartSize( std::string type ){
		
		if( type == "gear_part" ){
			return .01;
		}
		else if( type == "piston_rod_part" ){
			return .01;
		}
		else if( type == "gasket_part" ){
			return .03;
		}
		else if( type == "disk_part" ){
			return .03;
		}
		else if( type == "pulley_part" ){
			return .08;
		}
		else{
			ROS_ERROR("Invalid Part type provided!");
			return 0;
		}
	}	
	
		
	std::string stateToString( int state ){
		
		if( state == Part::INVALID ){
				return "INVALID";
		}
		else if( state == Part::PLACED ){
				return "PLACED";
		}
		else if( state == Part::GRABBED ){
				return "GRABBED";
		}
		else if( state == Part::REMOVED ){
				return "REMOVED";
		}
		else{
			return "Invalid state provided!";
		}
	}
	
	
	
}







