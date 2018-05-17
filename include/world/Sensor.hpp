#ifndef WORLD_SENSOR
#define WORLD_SENSOR


#include <string>
#include <vector>


namespace world {
	
	class Part;
	
	class Sensor {
		
		public:
			virtual std::string getName() = 0;
			
			virtual std::vector<Part> getVisibleParts() = 0;
			
			virtual ~Sensor() {};
		
	};
	
}


#endif
