#include <world/Gripper.hpp>
#include <ros/ros.h>
#include <world/Part.hpp>


namespace world {

	std::string Gripper::getName() const { 
		
		return m_name; 
	}
			
		
			
				
	std::vector< Part > Gripper::getParts() const {
		
		std::vector< Part > p;
		if( m_part != nullptr ){
			p.push_back( *m_part );
		}
		
		return p; 
	}



	void Gripper::addPart( std::shared_ptr<Part> part_ptr ){
		
		if( m_part == nullptr ){
			
			part_ptr->setGrabbed(this);
			m_part = part_ptr;
		}
		else{
			ROS_ERROR("Cannot add the part to %s, another part exists: %s", getName().c_str(), m_part->getName().c_str());
		}
	}




	std::shared_ptr<Part> Gripper::removePart( std::string part_name ){
		
		if( m_part != nullptr ){
			if( m_part->getName() == part_name ){
				
				std::shared_ptr<Part> part = m_part; 
				m_part = nullptr;
				return part;
			}
			else{
				ROS_ERROR("Cannot remove, robot is holding a different part: %s", m_part->getName().c_str());
				return nullptr;
			}
		}
		else{
			ROS_ERROR("Cannot remove the part from %s, no part exists!", getName().c_str());
			return nullptr;
		}
	}

	
	void Gripper::printContainer(){
		
		std::vector< Part > parts = getParts();
		ROS_INFO("--------%s Info: %lu parts----------", getName().c_str(), parts.size());
		for( int i = 0; i < parts.size(); ++i ){
			ROS_INFO("Part %d", i+1);
			parts[i].printPart();
		}
		
		ROS_INFO(" ");
	}
	
	
	
}
