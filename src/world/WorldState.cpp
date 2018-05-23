#include <world/WorldState.hpp>
#include <algorithm>


namespace world {
	
	
	
		
	bool WorldState::initializeWorld(){
		
		// topics
		m_tf_sub = m_node.subscribe( "tf", 100, &WorldState::cb_tfList, this);
		ros::Duration(0.5).sleep();
		
		// timers
		m_update_t = m_node.createTimer( ros::Duration(2.0), &WorldState::cb_updateParts, this );
		
		// Boxes
		Box b1("BOX0");
		addBox(b1);
		
		
		//~ // DEBUG -- REMOVE!!
			//~ Part part("gasket_part_63", geometry_msgs::Pose(), &m_removed);
			//~ addNewPartToBox(part);
		//~ // DEBUNG -- REMOVE!!
		
		
		// Sensors
		addCameraSensor("logical_camera_1");
		addCameraSensor("logical_camera_2");
		addCameraSensor("logical_camera_3");
		addCameraSensor("logical_camera_4");
		addCameraSensor("logical_camera_5");
		addCameraSensor("logical_camera_6");
		addQualitySensor("quality_control_sensor_1");
		m_bins[0].connectSensor( m_sensors["logical_camera_1"].get() );
		m_bins[1].connectSensor( m_sensors["logical_camera_2"].get() );
		m_bins[2].connectSensor( m_sensors["logical_camera_3"].get() );
		m_bins[3].connectSensor( m_sensors["logical_camera_4"].get() );
		m_bins[4].connectSensor( m_sensors["logical_camera_5"].get() );
		m_boxes[0]->connectSensor( m_sensors["logical_camera_6"].get() );
		m_sensors["logical_camera_1"]->start();
		m_sensors["logical_camera_2"]->start();
		m_sensors["logical_camera_3"]->start();
		m_sensors["logical_camera_4"]->start();
		m_sensors["logical_camera_5"]->start();
		m_sensors["logical_camera_6"]->start();
		m_sensors["quality_control_sensor_1"]->start();
		
		// Robot
		m_graph.initializeGraph();
		std::unique_ptr<Robot> r1 = std::unique_ptr<Robot>( new IIWA14Robot("iiwa14", m_move_group) );
		addRobot( std::move(r1) );
		m_robot->initialize(m_graph);
		
		// services
		m_find_part_type_srv = m_node.advertiseService( "find_part_type", &WorldState::sv_findPartType, this );
		m_release_part_srv = m_node.advertiseService( "release_part", &WorldState::sv_releasePart, this );
		
			
		return true;
	}
	
	
	void WorldState::cb_updateParts( const ros::TimerEvent & t ){
		ROS_INFO("+++++World Update+++++");
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
							//~ ROS_INFO("Update %s pose", c_parts[p].getName().c_str());
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
							//~ ROS_INFO("Update %s pose", c_parts[p].getName().c_str());
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
			
			
			
			// check for faulty parts
			std::vector<SensorPart> faulty_parts = m_sensors["quality_control_sensor_1"]->getVisibleParts();
			
			
			
			for( int i = 0; i < faulty_parts.size(); ++i ){
				ROS_ERROR("Faulty Part Detected: %s!", faulty_parts[i].name.c_str());
				bool found = false;
				
				// only mark the fauly part once
				if( std::find(m_faulty_parts.begin(), m_faulty_parts.end(), faulty_parts[i].id) != m_faulty_parts.end() ){
					continue;
				}
				
				// v MOVE TO A SEPARATE FUNCTION --------------------
				// find matching part id
				for( WorldPartsMap::iterator it = m_parts.begin(); it != m_parts.end(); ++it ){
					for( std::unordered_map<std::string, std::weak_ptr<Part> >::iterator it2 = it->second.begin(); it2 != it->second.end(); ++it2 ){
						
						std::string tmp_name = it2->first;
						if( getIDFromName(tmp_name) == faulty_parts[i].id ){
							// mark faulty
							it2->second.lock()->markFaulty();
							ROS_ERROR("Marked %s as faulty", it2->first.c_str());
							m_faulty_parts.push_back(getIDFromName(tmp_name)); 
							found = true;
							break;
						}
					}
					if(found){
						break;
					}
				}
				// ^ MOVE TO A SEPARATE FUNCTION ________________
				
				if(!found){
					ROS_ERROR("Could not mark %s faulty !!!", faulty_parts[i].name.c_str());
				}
			}	
		}
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
			
	
	bool WorldState::addNewPartToBox( Part part ){
		
		if(m_boxes.size() > 0){
			// only add part if it doesn't already exist
			if( m_parts[part.getType()][part.getName()].expired() ){
				
				// add part shared_ptr to bin
				std::shared_ptr<Part> part_ptr = std::make_shared<Part>(part);
				m_boxes[0]->addPart(part_ptr);
				// add part weak_ptr to world
				m_parts[part.getType()][part.getName()] = part_ptr;
			}
			else{
				ROS_ERROR("Could not add %s to the world, it already exists!", part.getName().c_str());
				return false;
			}
			
		}
		else{
			ROS_ERROR("Cannot add part to box, there are no boxes in the world!");
		}
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
		
		
	
	bool WorldState::addCameraSensor( std::string sensor_name ){
		
		ROS_INFO("Adding sensor %s to the world.", sensor_name.c_str());
		std::unique_ptr<Sensor> s( new LogicalCameraSensor(sensor_name, &m_tfBuf, m_tf_list) );
		m_sensors[ sensor_name ] = std::move(s);
		ROS_INFO("There are now %lu sensors.", m_sensors.size());
		
		return true;
	}
	
	bool WorldState::addQualitySensor( std::string sensor_name ){
		
		ROS_INFO("Adding sensor %s to the world.", sensor_name.c_str());
		std::unique_ptr<Sensor> s( new QualityControlSensor(sensor_name, &m_tfBuf, m_tf_list) );
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
		return true;
	}
	
	
	
	
		
	void WorldState::cb_tfList( const tf2_msgs::TFMessage::ConstPtr & tf_list ){
		
		// update internal list of TF frame names
		std::string pname;
		for( int i = 0; i < tf_list->transforms.size(); ++i ){
			pname = tf_list->transforms[i].child_frame_id;
			m_tf_list[pname].first = pname;
			m_tf_list[pname].second = tf_list->transforms[i].header.stamp;
		}
		
		// remove old frame names
		ros::Time now = ros::Time::now();
		for( TFMap::iterator it = m_tf_list.begin(); it != m_tf_list.end(); ++it ){
			if( now - it->second.second > ros::Duration(0.2) ){
				ROS_ERROR("Removed %s", it->first.c_str());
				m_tf_list.erase( it->first );
			}
		}
		
	}

		
	bool WorldState::sv_findPartType(control_redesign::FindPartType::Request & req, control_redesign::FindPartType::Response & rsp ){
		
		if( req.type != "gear_part" && req.type != "gasket_part" && req.type != "disk_part" && req.type != "pulley_part" && req.type != "piston_rod_part" ){
			rsp.success = false;
			rsp.message = "Did not provide a valid part type";
			return true;
		}
		else{
			
			for( std::unordered_map<std::string, std::weak_ptr<Part> >::iterator it = m_parts[req.type].begin(); it != m_parts[req.type].end(); ++it ){
				
				std::shared_ptr<Part> p = it->second.lock();
				if( p->isAvailable() && !p->isFaulty() ){
					rsp.name = p->getName();
					rsp.type = p->getType();
					rsp.id = getIDFromName(rsp.name);
					rsp.current_pose = p->getPose();
					ROS_INFO("Marking %s to be used in the next shipment.", rsp.name.c_str());
					p->markUsed();
					rsp.success = true;
					rsp.message = "Found the part: " + rsp.name;
					return true;
				}
			}
			
			rsp.success = false;
			rsp.message = "could not find any avilable " + req.type + " type";
			return true;
		}
	}
			
	
	bool WorldState::sv_releasePart( control_redesign::ReleasePart::Request & req, control_redesign::ReleasePart::Response & rsp ){
		
		std::string type = getTypeFromName(req.name);
		if( !m_parts[type][req.name].expired() ){
			m_parts[type][req.name].lock()->markAvailable();
			rsp.success = true;
			return true;
		}
		else{
			rsp.success = false;
			rsp.message = "Could not find the part: " + req.name;
			return true;	
		}
		
	}

}
