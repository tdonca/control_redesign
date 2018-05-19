#ifndef WORLD_SENSOR_LOGICALCAMERA
#define WORLD_SENSOR_LOGICALCAMERA

#include <world/Sensor.hpp>
#include <world/Part.hpp>

namespace world{
	
	
	class LogicalCameraSensor: public Sensor {
		
		public:
			
			LogicalCameraSensor( std::string name )
			:	m_name(name),
				m_parts()
			{}
			
			virtual std::string getName() { return m_name; }
			
			virtual bool start();
			
			virtual void addPart( Part part );
			
			virtual void removePart( std::string part_name ) { m_parts.erase(part_name); }
			
			virtual std::vector<SensorPart> getVisibleParts();
			
			virtual void printSensor();
			
			virtual ~LogicalCameraSensor() {};
		
		
		private:
		
			std::string m_name;
			SensorPartsMap m_parts;
	};
	
	
	
	
}

#endif
