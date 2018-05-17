#ifndef WORLD_SENSOR_LOGICALCAMERA
#define WORLD_SENSOR_LOGICALCAMERA

#include <world/Sensor.hpp>
#include <world/Part.hpp>


namespace world{
	
	
	class LogicalCameraSensor: public Sensor {
		
		public:
			
			LogicalCameraSensor()
			:	m_name()	
			{}
			
			virtual std::string getName() { return ""; };
			
			virtual std::vector<Part> getVisibleParts() { return std::vector<Part>(); };
			
			virtual ~LogicalCameraSensor() {};
		
		
		private:
		
			std::string m_name;
			
	};
	
	
}

#endif
