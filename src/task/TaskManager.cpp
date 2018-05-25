
#include <task/TaskManager.hpp>

namespace task {
	
	bool TaskManager::hasNextTask() const{
		
		ROS_INFO("Checking if there are any tasks queued.");
		if( m_tasks_q.size() > 0 ){
			return true;
		}
		else{
			ROS_INFO("No tasks left for this shipment.");
			return false;
		}
	}
			
	void TaskManager::executeNextTask(){
		
		ROS_INFO("Executing the next available task.");
		m_tasks_q.front()->execute();
		m_tasks_q.pop_front();
		
		
		ROS_INFO(" ");
	}
	
	
	
	// find parts and the order in which to get them
	// create AddPartTask or PutPartsBackTask
	void TaskManager::receiveNewShipment( const osrf_gear::Shipment & shipment ){
		
		std::vector<osrf_gear::Product> parts_needed = shipment.products;
		
		ROS_INFO("Parts Needed:");
		for(int i = 0; i < parts_needed.size(); ++i ){
			ROS_INFO("%s", parts_needed[i].type.c_str());
		}
		
		// check box parts for useable part
		bool put_back = false;
		std::vector<client::PlannerPart> bp;
		if( m_world_client.getBoxParts(bp) && bp.size() > 0 ){
			
			ROS_INFO("Parts in the box:");
			for( int i = 0; i < bp.size(); ++i ){
				ROS_INFO("%s", bp[i].name.c_str());
			}
			
			put_back = true;
			bool found = false;
			for( int i = 0; i < bp.size(); ++i ){
				for( int j = 0; j < parts_needed.size(); ++j ){
					if( bp[i].type == parts_needed[j].type ){
						ROS_INFO("One of the parts in the box is useable: %s, adding this one to the queue, and mark to put the others back.", bp[i].name.c_str());
						// add the part to queue
						found = true;
						bp[i].goal_pose = parts_needed[j].pose;
						m_world_client.markPartUsed(bp[i]);
						m_parts_q.push_back(bp[i]);
						parts_needed[j] = parts_needed.back();
						parts_needed.pop_back();
						bp[i] = bp.back();
						bp.pop_back();
						break;
					}
				}
				if(found) break;
			}	
		}
		
		// check gripper part for useable part
		client::PlannerPart gp;
		if( m_world_client.getGripperPart(gp) ){
			
			ROS_INFO("Gripper part: %s", gp.name.c_str());
			
			// put the gripper part back to free gripper for PutPartBackTask
			if( put_back ){
				bp.insert(bp.begin(), gp);
			}
			// no parts in the box, see if the held part is useable
			else{
				for( int i = 0; i < parts_needed.size(); ++i ){
					if( gp.type == parts_needed[i].type ){
						ROS_INFO("The gripper has a needed part %s, adding it to the queue.", gp.type.c_str());
						// add the part to queue
						gp.goal_pose = parts_needed[i].pose;
						m_world_client.markPartUsed(gp);
						m_parts_q.push_back(gp);
						parts_needed[i] = parts_needed.back();
						parts_needed.pop_back();
						break;
					}
				}
			}
			
		}
		
		
		// mark the rest to be put back in the bins
		if( put_back ){
			
			ROS_INFO("Parts to put back:");
			for( int i = 0; i < bp.size(); ++i ){
				ROS_INFO("%s", bp[i].name.c_str());
			}
			
			createPutPartsBackTask(bp);
		}
		
		
		ROS_INFO("Parts Needed after checking box and gripper:");
		for(int i = 0; i < parts_needed.size(); ++i ){
			ROS_INFO("%s", parts_needed[i].type.c_str());
		}
		
		
		// find the rest of the parts for the shipment
		ROS_INFO("Checking to see if all parts are avilable in the world...");
		for( int i = 0; i < parts_needed.size(); ++i ){
			
			client::PlannerPart p;
			if( m_world_client.getPartType( parts_needed[i].type, p ) ){
				ROS_INFO("Found part for order: %s", p.name.c_str());
				m_world_client.markPartUsed(p);
				m_parts_q.push_back(p);
			}
			else{
				ROS_ERROR("Could not find any available %s type! skipping this part...", parts_needed[i].type.c_str());
				continue;
			}	
		}
		ROS_INFO(" ");
		
		
		// create AddPartTask for the first part
		if(m_parts_q.size() > 0){
			createAddPartTask( m_parts_q.front() );
		}
		else{
			ROS_ERROR("No parts requested in this shipment, why??");
		}
			
	}



	void TaskManager::createAddPartTask( client::PlannerPart const & part ) {
		std::unique_ptr<Task> t = std::unique_ptr<Task>( new AddPartTask( part ) );
		m_tasks_q.push_back(std::move(t));
		ROS_INFO("Created an AddPartTask for %s.", part.name.c_str());
	}




	void TaskManager::createPutPartsBackTask( std::vector<client::PlannerPart> const box_parts ) {
		std::unique_ptr<Task> t = std::unique_ptr<Task>( new PutPartsBackTask(box_parts) );
		m_tasks_q.push_back(std::move(t));
		ROS_INFO("Created a PutPartBackTask.");
	}




	void TaskManager::createRemoveFaultyPartTask() {
		std::unique_ptr<Task> t = std::unique_ptr<Task>( new RemoveFaultyPartTask() );
		m_tasks_q.push_back(std::move(t));
		ROS_INFO("Created a RemoveFaultyPartTask.");
	}



	void TaskManager::createReplacePartTask() {
		
		std::unique_ptr<Task> t = std::unique_ptr<Task>( new ReplacePartTask() );
		m_tasks_q.push_back(std::move(t));
		ROS_INFO("Created a ReplacePartTask.");
	} 
	
	
	
	void TaskManager::createFinishShipmentTask() {
		std::unique_ptr<Task> t = std::unique_ptr<Task>( new FinishShipmentTask() );
		m_tasks_q.push_back(std::move(t));
		ROS_INFO("Created a FinishShipmentTask.");
	}


}
