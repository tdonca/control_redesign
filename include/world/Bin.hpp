#ifndef WORLD_CONTAINER_BIN
#define WORLD_CONTAINER_BIN

#include <world/Container.hpp>


namespace world {
	
	
	class Bin: public Container {
		
		public:
		
			Bin( std::string name ) 
			:	m_name(name),
				m_parts(),
				m_sensor()
			{}
			
			virtual std::string getName() const;
			
			virtual std::vector< Part > getParts() const;
			
			virtual void addPart( std::shared_ptr<Part> part_ptr );
			
			virtual std::shared_ptr<Part> removePart( std::string part_name );
			
			virtual bool connectSensor( Sensor* sensor );
			
			virtual Sensor* getSensor() { return m_sensor; }
			
			virtual void printContainer();
			
			virtual ~Bin() {}
			
		private:
		
			const std::string m_name;
			ContainerPartsMap m_parts;
			Sensor* m_sensor;
		
	};
}

#endif
