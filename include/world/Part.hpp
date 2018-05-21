#ifndef WORLD_PART
#define WORLD_PART


#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <memory>

namespace world {
	
	
	class Container;
	std::string getTypeFromName( std::string name );

	class Part {
		
		public:
			
			static const int INVALID = 0;
			static const int PLACED  = 1;
			static const int GRABBED = 2;
			static const int REMOVED = 3; 
			
			
			Part( std::string name, geometry_msgs::Pose pose, Container * none_container )
			:	m_name(name),
				m_type( getTypeFromName(name) ),
				m_size( getSize() ),
				m_pose(pose),
				m_location(none_container),
				m_state(INVALID),
				m_available(true),
				m_faulty(false)
			{
			}
			

			void setPlaced( Container* location );
			void setGrabbed( Container* location );
			void setRemoved();
			
			
			std::string getName() const	{ return m_name; } 
			std::string getType() const	{ return m_type; }
			
			geometry_msgs::Pose getPose() const { return m_pose; }
			const Container* getLocation() const { return m_location; }
			std::string getState() const;
			double getSize() const;
			
			void updatePose( geometry_msgs::Pose p) { m_pose = p; }
			
			void printPart();
			
		
		private:
			
			std::string m_name;
			std::string m_type;
			double m_size;
			geometry_msgs::Pose m_pose;
			Container* m_location;
			int m_state;
			bool m_available;
			bool m_faulty;
	};
	
	
	
	
	
	
	
}


#endif
