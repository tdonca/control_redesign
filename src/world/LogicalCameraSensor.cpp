
#include <world/LogicalCameraSensor.hpp>


namespace world {
	
	
	
	bool LogicalCameraSensor::start(){
		
		// setup to detect parts
		
		return true;
	}
		
	void LogicalCameraSensor::addPart( Part part ){
		
		SensorPart p;
		p.name = part.getName();
		p.type = part.getType();
		p.pose = part.getPose();
		m_parts[p.name] = p;
	}
	
		
			
	std::vector<SensorPart> LogicalCameraSensor::getVisibleParts(){
		
		return sensorMapToVector( m_parts );
	}
	
	
	
	
	void LogicalCameraSensor::printSensor(){
		
		std::vector< SensorPart > parts = getVisibleParts();
		ROS_INFO("--------%s Info: %lu parts----------", getName().c_str(), parts.size());
		for( int i = 0; i < parts.size(); ++i ){
			ROS_INFO("Part %d", i+1);
			parts[i].printPart();
		}
		
		ROS_INFO(" ");
	}
	
} 
