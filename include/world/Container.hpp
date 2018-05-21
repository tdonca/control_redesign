#ifndef WORLD_CONTAINER
#define WORLD_CONTAINER


#include <vector>
#include <string>
#include <memory>
#include <unordered_map>
#include <geometry_msgs/Pose.h>

namespace world {
	
	class Part;
	class Sensor;
	typedef std::unordered_map< std::string, std::shared_ptr<Part> > ContainerPartsMap;
	typedef std::unordered_map< std::string, Part > PartsMap;
	
	class Container {
		
		public:
			
			virtual std::string getName() const = 0;
			
			virtual std::vector< Part > getParts() const = 0;
			
			virtual void addPart( std::shared_ptr<Part> part_ptr ) = 0;
			
			virtual std::shared_ptr<Part> removePart( std::string part_name ) = 0;
			
			virtual bool updatePartPose( std::string part_name, geometry_msgs::Pose pose ) = 0;
			
			virtual bool connectSensor( Sensor* sensor ) = 0;
			
			virtual Sensor* getSensor() = 0;
			// move implementation to here so that there is less code duplication
			virtual void printContainer() = 0;
			
			virtual ~Container() {}
			
	};
	


	inline std::vector<Part> containerMapToVector( ContainerPartsMap parts ){
		
		std::vector<Part> p;
		for( ContainerPartsMap::const_iterator it = parts.begin(); it != parts.end(); ++it ){
			p.push_back( *(it->second) );
		}
		return p;
	}
	
	
}


#endif
