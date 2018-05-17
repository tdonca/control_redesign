#include <world/Bin.hpp>
#include <ros/ros.h>
#include <world/Part.hpp>


namespace world {
	
	std::string Bin::getName() const { 
		
		return m_name; 
	}
	
	
	
			
	std::vector<Part> Bin::getParts() const {
		 
		return containerMapToVector( m_parts );
	}
			
	

			
			
	void Bin::addPart(  std::shared_ptr<Part> part_ptr ){
		
		// only add the part if it doesn't already exist
		if( m_parts[part_ptr->getName()] == nullptr ){
			
			// notify part of its container
			part_ptr->setPlaced(this);
			m_parts[part_ptr->getName()] = part_ptr;
		}
		else{
			ROS_ERROR("Cannot add the part to %s, it already exists!", getName().c_str());
		}
	}
			
			
			
			
			
	std::shared_ptr<Part> Bin::removePart( std::string part_name ){
		
		
		if( m_parts[part_name] != nullptr ){
			std::shared_ptr<Part> part = m_parts[part_name]; 
			m_parts.erase( part_name );
			return part;
		}
		else{
			ROS_ERROR("Cannot remove the part from %s, it does not exist!", getName().c_str());
			return nullptr;
		}
	}
	
	
	
	

	void Bin::printContainer(){
		
		std::vector< Part > parts = getParts();
		ROS_INFO("--------%s Info: %lu parts----------", getName().c_str(), parts.size());
		for( int i = 0; i < parts.size(); ++i ){
			ROS_INFO("Part %d", i+1);
			parts[i].printPart();
		}
		
		ROS_INFO(" ");
	}

	
	
}
