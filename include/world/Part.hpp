#ifndef WORLD_PART
#define WORLD_PART


#include <ros/ros.h>
#include <world/Container.hpp>
#include <world/NoneContainer.hpp>
#include <geometry_msgs/Pose.h>
#include <memory>

namespace world {
	
	
	
	static std::shared_ptr<Container> none_container = std::make_shared<NoneContainer>("None");
	
	double getPartSize( std::string );


	class Part {
		
		public:
			
			static const int INVALID = 0;
			static const int PLACED  = 1;
			static const int GRABBED = 2;
			static const int REMOVED = 3; 
			
			
			Part( std::string name, std::string type, geometry_msgs::Pose pose, int state = INVALID )
			:	m_name(name),
				m_type(type),
				m_size( getSize() ),
				m_pose(pose),
				m_location(none_container),
				m_state(state)
			{
			}
			

			void setPlaced( std::shared_ptr<Container> location );
			void setGrabbed( std::shared_ptr<Container> location );
			void setRemoved();
			
			
			std::string getName() const	{ return m_name; } 
			std::string getType() const	{ return m_type; }
			
			geometry_msgs::Pose getPose() const { return m_pose; }
			const std::shared_ptr<Container> getLocation() const { return m_location; }
			std::string getState() const;
			double getSize() const;
			
			
		
		private:
			
			std::string m_name;
			std::string m_type;
			double m_size;
			geometry_msgs::Pose m_pose;
			std::shared_ptr<Container> m_location;
			int m_state;
	};
	
	
	
	
	
	
	
}


#endif
