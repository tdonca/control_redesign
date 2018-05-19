#ifndef WORLD_SENSOR
#define WORLD_SENSOR


#include <string>
#include <vector>
#include <unordered_map>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>


namespace world {
	
	struct SensorPart;
	class Part;
	void getRPY( const geometry_msgs::Pose  pose, double & r, double & p, double & y );
	typedef std::unordered_map< std::string, SensorPart > SensorPartsMap;
	
	class Sensor {
		
		public:
			virtual std::string getName() = 0;
			
			virtual bool start() = 0;
			
			virtual void addPart( Part part ) = 0;
			
			virtual void removePart( std::string part_name ) = 0;
			
			virtual std::vector<SensorPart> getVisibleParts() = 0;
			
			virtual void printSensor() = 0;
			
			virtual ~Sensor() {};
		
	};
	
	
	
	struct SensorPart{
		
		std::string name;
		std::string type;
		geometry_msgs::Pose pose;
		
		void printPart(){
			double r, p, y;
			getRPY( pose, r, p, y );
			ROS_INFO("Name: %s", name.c_str());
			ROS_INFO("Type: %s", type.c_str());
			ROS_INFO("Position: %.3f, %.3f, %.3f", pose.position.x, pose.position.y, pose.position.z);
			ROS_INFO("Orientation: %.3f, %.3f, %.3f, %.3f", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
			ROS_INFO("RollPitchYaw: %.3f, %.3f, %.3f", r, p, y);
			ROS_INFO(" ");
		}
	};
	
	
	inline std::vector<SensorPart> sensorMapToVector( SensorPartsMap parts ){
		
		std::vector<SensorPart> p;
		for( SensorPartsMap::const_iterator it = parts.begin(); it != parts.end(); ++it ){
			p.push_back( it->second );
		}
		
		return p;
	}
	
}


#endif
