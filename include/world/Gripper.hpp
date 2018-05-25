#ifndef WORLD_CONTAINER_GRIPPER
#define WORLD_CONTAINER_GRIPPER

#include <world/Container.hpp>

namespace world {
	
	
	class Gripper: public Container {
		
		public:
		
			Gripper( std::string name ) 
			:	m_name(name),
				m_part()
			{}
			
			virtual std::string getName() const;
			
			virtual std::vector< Part > getParts() const;
			
			virtual void addPart( std::shared_ptr<Part> part_ptr );
			
			virtual std::shared_ptr<Part> removePart( std::string part_name );
			
			virtual bool updatePartPose( std::string part_name, geometry_msgs::Pose pose ) { return false; }
			
			virtual bool connectSensor( Sensor* sensor ) { return false; }
			
			virtual Sensor* getSensor() { return nullptr; }
			
			virtual void printContainer();
			
			virtual ~Gripper() {}
			
		private:
		
			const std::string m_name;
			std::shared_ptr<Part> m_part;
			
			// Adapter for actual EE object
		
	};
	
}

#endif
