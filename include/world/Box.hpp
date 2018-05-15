#ifndef WORLD_CONTAINER_BOX
#define WORLD_CONTAINER_BOX

#include <world/Container.hpp>

namespace world {
	
	class Box: public Container {
		
		public:
		
			Box( std::string name ) 
			:	m_name(name)
			{}
			
			std::string getName() const { return m_name; }
			
			std::vector<std::string> getParts() const {return m_parts; }
			
			
			void addPart( std::string part ){
				m_parts.push_back(part);
			}
			
			
			
			void removePart( std::string part ){
				int ix = -1;
				// find
				for(int i = 0; i < m_parts.size(); ++i){
					if(m_parts[i] == part){
						ix = i;
					}
				} 
				if(ix != -1){
					//swap
					m_parts[ix] = m_parts.back();
					//pop
					m_parts.pop_back();
				}
				
			}
			
			~Box() {}
			
		private:
		
			const std::string m_name;
			std::vector<std::string> m_parts;
		
	};
}

#endif
