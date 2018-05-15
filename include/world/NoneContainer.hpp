#ifndef WORLD_CONTAINER_NONE
#define WORLD_CONTAINER_NONE

#include <world/Container.hpp>

namespace world {
	
	class NoneContainer: public Container {
		
		public:
		
			NoneContainer( std::string name ) 
			:	m_name(name)
			{}
			
			std::string getName() const { return m_name; }
			
			std::vector<std::string> getParts() const {return m_parts; }
			
			void addPart( std::string part ){}
			
			void removePart( std::string part ){}
			
			~NoneContainer() {}
			
		private:
		
			const std::string m_name;
			std::vector<std::string> m_parts;
		
	};
	
}

#endif
