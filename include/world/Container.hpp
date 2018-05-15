#ifndef WORLD_CONTAINER
#define WORLD_CONTAINER

#include <vector>
#include <string>

namespace world {
	
	class Container {
		
		public:
			
			virtual std::string getName() const = 0;
			
			virtual std::vector<std::string> getParts() const = 0;
			
			virtual void addPart( std::string part ) = 0;
			
			virtual void removePart( std::string part ) = 0;
			
			virtual ~Container() {};
			
	};
	
	
}



#endif
