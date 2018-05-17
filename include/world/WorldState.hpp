#ifndef WORLD_WORLDSTATE
#define WORLD_WORLDSTATE

#include <memory>
#include <deque>
#include <ros/ros.h>
#include <world/Container.hpp>
#include <world/Part.hpp>
#include <world/Gripper.hpp>
#include <world/Bin.hpp>
#include <world/Box.hpp>
#include <world/NoneContainer.hpp>
#include <world/Sensor.hpp>
#include <world/LogicalCameraSensor.hpp>

namespace world {
						//type					//name
	typedef std::map< std::string, std::map< std::string, std::weak_ptr<Part> > > WorldPartsMap;
						//name				
	typedef std::map< std::string, std::shared_ptr<Sensor> > SensorMap;
	
	class WorldState {
		
		public:
			
			WorldState() 
			:	m_bins{ Bin("BIN1"), Bin("BIN2"), Bin("BIN3"), Bin("BIN4"), Bin("BIN5") },
				m_boxes(),
				m_removed("None"),
				m_sensors(),
				m_gripper("ROBOT"),
				m_parts()
			{
				if( initializeWorld() ){
					ROS_INFO("Initialized the world.");
				}
				else{
					ROS_ERROR("World initialization failed!");
				}
			}
			
			bool addPart( Part part, int bin );
			
			bool removePart ( std::string part_name );
			
			bool addBox( Box box );
			
			bool removeBox();
			
			bool addSensor( Sensor* sensor );
			
			bool removeSensor( std::string sensor_name );
			
			bool testFunction();
			
		private:
		
			bool initializeWorld();
			
			std::vector<Bin> m_bins;
			std::deque< std::unique_ptr<Box> > m_boxes;
			NoneContainer m_removed;
			SensorMap m_sensors;
			Gripper m_gripper;
			WorldPartsMap m_parts;
	};
	
}

#endif
