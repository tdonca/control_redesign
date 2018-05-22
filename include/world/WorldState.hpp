#ifndef WORLD_WORLDSTATE
#define WORLD_WORLDSTATE

#include <memory>
#include <deque>
#include <ros/ros.h>
#include <world/Container.hpp>
#include <world/Part.hpp>
#include <world/Gripper.hpp>
#include <world/Bin.hpp>
#include <world/Box.hpp>
#include <world/NoneContainer.hpp>
#include <world/Sensor.hpp>
#include <world/LogicalCameraSensor.hpp>
#include <world/QualityControlSensor.hpp>
#include <world/Robot.hpp>
#include <world/IIWA14Robot.hpp>
#include <world/StateGraph.hpp>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>


namespace world {
						//type					//name
	typedef std::unordered_map< std::string, std::unordered_map< std::string, std::weak_ptr<Part> > > WorldPartsMap;
						//name				
	typedef std::unordered_map< std::string, std::unique_ptr<Sensor> > SensorMap;
	typedef std::map< std::string, std::pair<std::string, ros::Time> > TFMap;
	
	class WorldState {
		
		public:
			
			WorldState() 
			:	m_bins{ Bin("BIN1"), Bin("BIN2"), Bin("BIN3"), Bin("BIN4"), Bin("BIN5") },
				m_boxes(),
				m_removed("None"),
				m_sensors(),
				m_gripper("Gripper"),
				m_parts(),
				m_robot(),
				m_tf_list(),
				m_node(),
				m_update_t(),
				m_tf_sub(),
				m_tfBuf(),
				m_tfListener(m_tfBuf),
				m_move_group("manipulator")
			{
				if( initializeWorld() ){
					ROS_INFO("Initialized the world.");
				}
				else{
					ROS_ERROR("World initialization failed!");
				}
			}
			
			bool addNewPart( Part part, int bin );
			
			bool addNewPartToBox( Part part );
			
			bool removePart ( std::string part_name );
			
			bool addBox( Box box );
			
			bool removeBox();
			
			bool addCameraSensor( std::string sensor_name );
			
			bool addQualitySensor( std::string sensor_name );
			
			bool removeSensor( std::string sensor_name );
			
			bool addRobot( std::unique_ptr<Robot> robot );
			
			bool removeRobot( std::string robot_name );
			
			bool testFunction();
			
			bool sv_gripperPart();
			
		private:
		
			bool initializeWorld();
			
			void cb_updateParts( const ros::TimerEvent & t );
			
			void cb_tfList( const tf2_msgs::TFMessage::ConstPtr & tf_list );
			
			
			std::vector<Bin> m_bins;
			std::deque< std::unique_ptr<Box> > m_boxes;
			NoneContainer m_removed;
			SensorMap m_sensors;
			Gripper m_gripper;
			WorldPartsMap m_parts;
			std::vector<std::string> m_faulty_parts; 
			std::unique_ptr<Robot> m_robot;
			StateGraph m_graph;
			TFMap m_tf_list;
			
			ros::NodeHandle m_node;
			ros::Timer m_update_t;
			ros::Subscriber m_tf_sub;
			
			tf2_ros::Buffer m_tfBuf;
			tf2_ros::TransformListener m_tfListener;
			
			moveit::planning_interface::MoveGroupInterface m_move_group;
	};
	
}

#endif
