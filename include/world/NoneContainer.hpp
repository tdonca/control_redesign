#ifndef WORLD_CONTAINER_NONE
#define WORLD_CONTAINER_NONE

#include <world/Container.hpp>


namespace world {
	
	
	class NoneContainer: public Container {
		
		public:
		
			NoneContainer( std::string name ) 
			:	m_name("None"),
				m_parts()
			{}
			
			std::string getName() const { return m_name; }
			
			
			std::vector< Part > getParts() const {return m_parts; }
			
			void addPart( std::shared_ptr<Part> part_ptr ){}
			
			std::shared_ptr<Part> removePart( std::string part_name ){ return nullptr; }
			
			virtual bool connectSensor( Sensor* sensor ) { return false; }
			
			virtual Sensor* getSensor() { return nullptr; }
			
			void printContainer() {}
			
			~NoneContainer() {}
			
		private:
		
			const std::string m_name;
			std::vector<Part> m_parts;
		
	};
	
}

#endif
