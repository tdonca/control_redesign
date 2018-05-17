#include <world/WorldState.hpp>



namespace world {
	
	std::string getTypeFromName( std::string name );
	std::vector<std::string> split(const char *str, char c = '_');
	
		
	bool WorldState::initializeWorld(){
		
		// Bins
		
		// Robot
		
		// Boxes
		
		return true;
		
	}
	
	
	bool WorldState::addPart( Part part, int bin ){
		
		ROS_INFO("Adding %s to bin %d.", part.getName().c_str(), bin);
		int binnum = bin-1;
		
		// only add part if it doesn't already exist
		if( m_parts[part.getType()][part.getName()].expired() ){
			
			// add part shared_ptr to bin
			std::shared_ptr<Part> part_ptr = std::make_shared<Part>(part);
			m_bins[binnum].addPart(part_ptr);
			// add part weak_ptr to world
			m_parts[part.getType()][part.getName()] = part_ptr;
		}
		else{
			ROS_ERROR("Could not add %s to the world, it already exists!", part.getName().c_str());
			return false;
		}
		
		return true;
	}
			
			
			
	bool WorldState::removePart( std::string name ){
		
		std::string type = getTypeFromName(name);
		
		// release pointer if it is still active
		if( m_parts[type][name].expired() ){
			ROS_ERROR("Could not remove %s from the world, it does not exist!", name.c_str());
		}
		else{
			ROS_ERROR("Could not remove %s from the world, it still exists somewhere!", name.c_str());
		}
		
		return false;
	}
	
	
	bool WorldState::addBox(Box box ){
		
		ROS_INFO("Adding box %s to the world.", box.getName().c_str());
		std::unique_ptr<Box> b = std::unique_ptr<Box>( new Box(box) );
		m_boxes.push_back( std::move(b) );
		ROS_INFO("There are now %lu boxes.", m_boxes.size());
		return true;
	}
			
	
	
	bool WorldState::removeBox(){
		
		if( m_boxes.size() > 0 ){
			ROS_INFO("Removing the farthest box %s from the world.", m_boxes.front()->getName().c_str());
			m_boxes.pop_front();
		}
		else{
			ROS_ERROR("There are no boxes in the world to remove!");
			return false;
		}
		
		return true;
	}
		
		
	
	bool WorldState::addSensor( Sensor* sensor ){
		
		
		return true;
	}
	
	bool WorldState::removeSensor( std::string name ){
	
		
		return true;
	}	
	
	
	
	bool WorldState::testFunction(){
		
		
		// create a few parts
		ROS_INFO("Create three parts");
		geometry_msgs::Pose pose;
		pose.position.x = 0.5;
		pose.position.y = 0.5;
		pose.position.z = 0.5;
		pose.orientation.x = 0.0;
		pose.orientation.y = 0.0;
		pose.orientation.z = 0.0;
		pose.orientation.w = 1.0;
		
		Part p1("gear_part_12", pose, &m_removed);
		Part p2("pulley_part_1", pose, &m_removed);
		Part p3("disk_part_99", pose, &m_removed);
		addPart(p1, 1);
		addPart(p2, 1);
		addPart(p3, 5);
		Box b1("BOX0");
		addBox(b1);
		ROS_INFO(" ");
		
		
		// print container info
		m_bins[0].printContainer();
		m_bins[4].printContainer();


		ROS_INFO("Move the parts around");
		m_gripper.addPart( m_bins[0].removePart("gear_part_12") );
		m_bins[0].printContainer();
		m_gripper.printContainer();
		
		
		
		m_boxes[0]->addPart( m_gripper.removePart("gear_part_12") );
		m_gripper.printContainer();
		m_boxes[0]->printContainer();
		
		
		
		m_gripper.addPart( m_boxes[0]->removePart("gear_part_12") );
		m_bins[2].addPart( m_gripper.removePart("gear_part_12") );
		m_bins[0].printContainer();
		m_bins[1].printContainer();
		m_bins[2].printContainer();
		m_bins[3].printContainer();
		m_bins[4].printContainer();
		m_gripper.printContainer();
		
		// submit the box
		removeBox();
		
	}
	
	
	
	
			
			
	std::string getTypeFromName( std::string name ){
		
		std::vector<std::string> result = split( name.c_str() ); 
		result.pop_back(); // remove ID number
		
		std::string type = "";
		for( int i = 0; i < result.size(); ++i ){
			type += result[i] + "_";
		}
		type.pop_back(); // remove trailing "_"
		
		return type;
	}
	
	
	std::vector<std::string> split(const char *str, char c){
    
		std::vector<std::string> result;
		do
		{
			const char *begin = str;

			while(*str != c && *str)
				str++;

			result.push_back(std::string(begin, str));
		} while (0 != *str++);

		return result;
	}	


}
