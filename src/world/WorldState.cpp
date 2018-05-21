#include <world/WorldState.hpp>
#include <algorithm>


namespace world {
	
	std::string getTypeFromName( std::string name );
	std::vector<std::string> split(const char *str, char c = '_');
	
		
	bool WorldState::initializeWorld(){
		
		// Bins
		
		// Robot
		
		// Boxes
		
		return true;
		
	}
	
	
	void WorldState::cb_updateParts(){
		
		// bins
		for( int i = 0; i < m_bins.size(); ++i ){
			
			std::vector<SensorPart> s_parts = m_bins[i].getSensor()->getVisibleParts();
			std::vector<Part> c_parts = m_bins[i].getParts();
			
			// every sensor part
			for( int k = 0; k < s_parts.size(); ++k ){
				
				//find the part in the container
				bool found = false;
				for( int p = 0; p < c_parts.size(); ++p ){
					if( c_parts[p].getName() == s_parts[k].name ){
						
						// update part pose
						if( m_bins[i].updatePartPose( c_parts[p].getName(), s_parts[k].pose ) ){
							ROS_INFO("Update %s pose", c_parts[p].getName().c_str());
						}
						else{
							ROS_ERROR("Error trying to update %s pose!", c_parts[p].getName().c_str());
						}
						
						found = true;
					}
				} 
				
				// add the part to the world and bin only if it does not exist in the world yet
				if( !found && m_parts[s_parts[k].name][s_parts[k].type].expired() ){
					
					Part new_part( s_parts[k].name, s_parts[k].pose, &m_removed );
					if( addNewPart( new_part, i+1 ) ){
						ROS_INFO("Added the new part %s to the world in %s", s_parts[k].name.c_str(), m_bins[i].getName().c_str());
					}
					else{
						ROS_ERROR("Error adding the new part %s to the world!", s_parts[k].name.c_str());
					}
				}
			}
		}
		
		
		// box
		if(m_boxes.size() > 0){
			std::vector<SensorPart> s_parts = m_boxes[0]->getSensor()->getVisibleParts();
			std::vector<Part> c_parts = m_boxes[0]->getParts();
			
			// every sensor part
			for( int k = 0; k < s_parts.size(); ++k ){
				
				//find the part in the container
				bool found = false;
				for( int p = 0; p < c_parts.size(); ++p ){
					if( c_parts[p].getName() == s_parts[k].name ){
						
						// update part pose
						if( m_boxes[0]->updatePartPose( c_parts[p].getName(), s_parts[k].pose ) ){
							ROS_INFO("Update %s pose", c_parts[p].getName().c_str());
						}
						else{
							ROS_ERROR("Error trying to update %s pose!", c_parts[p].getName().c_str());
						}
						
						found = true;
					}
				} 
				
		
				if( !found ){
					ROS_ERROR("Error updating %s from %s sensor, the part does not exist in the container!", s_parts[k].name.c_str(), m_boxes[0]->getName().c_str());
				}
			}
		}
		
		// gripper
		
		
		// only add new parts to the world in containers from sensors
		
		// update the poses of all visible PLACED parts
	}
	
	bool WorldState::addNewPart( Part part, int bin ){
		
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
		
		
	
	bool WorldState::addSensor( std::string sensor_name ){
		
		ROS_INFO("Adding sensor %s to the world.", sensor_name.c_str());
		std::unique_ptr<Sensor> s( new LogicalCameraSensor(sensor_name) );
		m_sensors[ sensor_name ] = std::move(s);
		ROS_INFO("There are now %lu sensors.", m_sensors.size());
		
		return true;
	}
	
	
	
	bool WorldState::removeSensor( std::string sensor_name ){
	
		if( m_sensors[sensor_name] != nullptr ){
			ROS_INFO("Removing %s from the world.", sensor_name.c_str());
			m_sensors[sensor_name] = nullptr;
		}
		else{
			ROS_ERROR("Could not remove %s, it does not exist!", sensor_name.c_str());
			return false;
		}

		return true;
	}	
	
	
	bool WorldState::addRobot( std::unique_ptr<Robot> robot ){
		
		if( m_robot == nullptr ){
			ROS_INFO("Adding robot %s to the world.", robot->getName().c_str());
			m_robot = std::move(robot);
		}
		else{
			ROS_ERROR("Cannot add the robot, one already exists!");
			return false;
		}
		
		return true;
	}
			
			
	bool WorldState::removeRobot( std::string robot_name ){
		
		if( robot_name == m_robot->getName() ){
			m_robot = nullptr;
		}
		else{
			ROS_ERROR("Cannot remove the robot %s, it does not exist.", robot_name.c_str());
			return false;
		}
		
		return true;
	}
			
			
	bool WorldState::testFunction(){
		
		
		// create pose
		ROS_INFO("Create three parts");
		geometry_msgs::Pose pose;
		pose.position.x = 0.5;
		pose.position.y = 0.5;
		pose.position.z = 0.5;
		pose.orientation.x = 0.0;
		pose.orientation.y = 0.0;
		pose.orientation.z = 0.0;
		pose.orientation.w = 1.0;
		
		
		// create parts
		Part p1("gear_part_12", pose, &m_removed);
		Part p2("pulley_part_1", pose, &m_removed);
		Part p3("disk_part_99", pose, &m_removed);
		addNewPart(p1, 1);
		addNewPart(p2, 1);
		addNewPart(p3, 5);
		Box b1("BOX0");
		addBox(b1);
		ROS_INFO(" ");
		m_bins[0].printContainer();
		m_bins[4].printContainer();
		
		
		// create sensors
		//~ std::unique_ptr<Sensor> s1 =  std::unique_ptr<Sensor>( new LogicalCameraSensor("logical_camera_1") );
		//~ std::unique_ptr<Sensor> s2 =  std::unique_ptr<Sensor>( new LogicalCameraSensor("logical_camera_2") );
		//~ std::unique_ptr<Sensor> s3 =  std::unique_ptr<Sensor>( new LogicalCameraSensor("logical_camera_3") );
		//~ std::unique_ptr<Sensor> s4 =  std::unique_ptr<Sensor>( new LogicalCameraSensor("logical_camera_4") );
		//~ std::unique_ptr<Sensor> s5 =  std::unique_ptr<Sensor>( new LogicalCameraSensor("logical_camera_5") );
		//~ std::unique_ptr<Sensor> s6 =  std::unique_ptr<Sensor>( new LogicalCameraSensor("logical_camera_6") );
		addSensor("logical_camera_1");
		addSensor("logical_camera_2");
		addSensor("logical_camera_3");
		addSensor("logical_camera_4");
		addSensor("logical_camera_5");
		addSensor("logical_camera_6");
		m_bins[0].connectSensor( m_sensors["logical_camera_1"].get() );
		m_bins[1].connectSensor( m_sensors["logical_camera_2"].get() );
		m_bins[2].connectSensor( m_sensors["logical_camera_3"].get() );
		m_bins[3].connectSensor( m_sensors["logical_camera_4"].get() );
		m_bins[4].connectSensor( m_sensors["logical_camera_5"].get() );
		m_boxes[0]->connectSensor( m_sensors["logical_camera_6"].get() );
		
		
		// create graph
		m_graph.initializeGraph();
		
		
		// create robot
		std::unique_ptr<Robot> r1 = std::unique_ptr<Robot>( new IIWA14Robot("iiwa14") );
		addRobot( std::move(r1) );
		m_robot->initialize(m_graph);
		m_robot->printRobot();
		
		
		// detect parts
		m_bins[0].getSensor()->addPart(p1); // sensor1
		m_bins[0].getSensor()->addPart(p2); // sensor1
		m_bins[4].getSensor()->addPart(p3); // sensor5
		for( int i = 0; i < m_bins.size(); ++i ){
			m_bins[i].getSensor()->printSensor();
		}
		m_boxes[0]->getSensor()->printSensor();
		
		
		// move parts around
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
		
		
		// test state graph
		std::vector< State > path;
		if( m_graph.findPath("BOX", "BIN1", path) ){
			ROS_INFO("Path found");
			for(int i = 0; i < path.size(); ++i){
				ROS_INFO("Pos %d: %s, %.2f %.2f ...", i+1, path[i].name.c_str(), path[i].joint_values[0], path[i].joint_values[1]);
			}
		}
		
		
		return true;
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
