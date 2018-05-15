#include <world/Part.hpp>




namespace world {
	
	double getPartSize( std::string type );
	std::string stateToString( int state );
	
	
	void Part::setPlaced( std::shared_ptr<Container> location ){
		ROS_INFO("Placing Part %s in %s:", m_name.c_str(), location->getName().c_str());
		
		ROS_INFO("Remove association with current location: %s", m_location->getName().c_str());
		m_location->removePart( m_name );
		
		
		ROS_INFO("Create association with new location: %s\n", location->getName().c_str());
		location->addPart( m_name );
		m_location = location;
		
		
		m_state = PLACED;
	}
	
	
	void Part::setGrabbed( std::shared_ptr<Container> location ){
		ROS_INFO("Grabbing Part %s:", m_name.c_str());
		
		ROS_INFO("Remove association with current location: %s", m_location->getName().c_str());
		m_location->removePart( m_name );
		
		ROS_INFO("Create association with robot EE: %s\n", location->getName().c_str());
		location->addPart( m_name );
		m_location = location;
		
		
		m_state = GRABBED;
	}
	
	
	void Part::setRemoved(){
		ROS_INFO("Removing Part %s:", m_name.c_str());
		
		ROS_INFO("Remove association with current location: %s", m_location->getName().c_str());
		m_location->removePart( m_name );
		
		
		ROS_INFO("Remove from the world.\n");
		m_location = none_container;
		
		
		m_state = REMOVED;
	}
	
	std::string Part::getState() const { 
		return stateToString(m_state); 
	}
	
	double Part::getSize() const {
		return getPartSize( m_type );
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







