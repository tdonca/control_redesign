#ifndef WORLD_SENSOR_QUALITYCONTROL
#define WORLD_SENSOR_QUALITYCONTROL

#include <world/Sensor.hpp>
#include <world/Part.hpp>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>

namespace world{
	
	typedef std::map< std::string, std::pair<std::string, ros::Time> > TFMap;
	
	class QualityControlSensor: public Sensor {
		
		public:
			
			QualityControlSensor( std::string name, tf2_ros::Buffer* tfBuf, TFMap & tf_list )
			:	m_name(name),
				m_parts(),
				m_tf_list(tf_list),
				m_node(),
				m_update_t(),
				m_tfBuf(tfBuf)
			{
				
			}
			
			virtual std::string getName() { return m_name; }
			
			virtual bool start();
			
			virtual std::vector<SensorPart> getVisibleParts();
			
			virtual void printSensor();
			
			virtual ~QualityControlSensor() {};
		
		
		private:
			
			void addPart( SensorPart part );
			
			void removePart( std::string part_name );
			
			void cb_updateParts( const ros::TimerEvent & t );
			
		
			std::string m_name;
			SensorPartsMap m_parts;
			TFMap & m_tf_list;
			
			ros::NodeHandle m_node;
			ros::Timer m_update_t;
			
			tf2_ros::Buffer* m_tfBuf;
	};
	
	
	
	
}

#endif
