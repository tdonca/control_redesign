
#include <ros/ros.h>
#include <world/Part.hpp>
#include <world/Container.hpp>
#include <world/Gripper.hpp>
#include <world/Bin.hpp>
#include <world/Box.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace world;

bool testPart();
void printPart( std::shared_ptr<Part> );
void printContainer( std::shared_ptr<Container> );
void getRPY( const geometry_msgs::Pose , double &, double &, double & );

int main( int argc, char* argv[] ){
	
	testPart();
	
	
}


bool testPart(){
	
	// create objects
	ROS_INFO("Create three parts and move them around:");
	geometry_msgs::Pose pose;
	pose.position.x = 0.5;
	pose.position.y = 0.5;
	pose.position.z = 0.5;
	pose.orientation.x = 0.0;
	pose.orientation.y = 0.0;
	pose.orientation.z = 0.0;
	pose.orientation.w = 1.0;
	std::shared_ptr<Container> bin1 = std::make_shared<Bin>("BIN1");
	std::shared_ptr<Container> bin2 = std::make_shared<Bin>("BIN2");
	std::shared_ptr<Container> box1 = std::make_shared<Box>("BOX1");
	std::shared_ptr<Container> robot = std::make_shared<Gripper>("GRIPPER");
	std::shared_ptr<Part> p1 = std::make_shared<Part>("piston_rod_part_73", "piston_rod_part", pose);
	p1->setPlaced(bin1);
	pose.position.y += 0.5;
	std::shared_ptr<Part> p2 = std::make_shared<Part>("gear_part_12", "gear_part", pose);
	p2->setPlaced(bin1);
	pose.position.x += 1.0;
	std::shared_ptr<Part> p3 = std::make_shared<Part>("pulley_part_2", "pulley_part", pose);
	p3->setPlaced(bin2);
	
	// print part info
	printPart(p1);
	printPart(p2);
	printPart(p3);
	
	// print container info
	printContainer(bin1);
	printContainer(bin2);
	
	
	
	// move parts around
	// move p1 from bin to box
	p1->setGrabbed(robot);
	printContainer(bin1);
	printContainer(robot);
	p1->setPlaced(box1);
	printContainer(robot);
	printContainer(box1);
	// remove p2 from world
	p2->setGrabbed(robot);
	p2->setRemoved();
	printContainer(robot);
	
	
	
	
	
	return true;
}


void printPart( std::shared_ptr<Part> part ) {
	
	double r, p, y;
	getRPY( part->getPose(), r, p, y );

	ROS_INFO("--Printing Part Info--------");
	ROS_INFO("Name: %s", part->getName().c_str());
	ROS_INFO("Type: %s", part->getType().c_str());
	ROS_INFO("State: %s", part->getState().c_str());
	ROS_INFO("Container: %s", part->getLocation()->getName().c_str());
	ROS_INFO("Position: %.3f, %.3f, %.3f", part->getPose().position.x, part->getPose().position.y, part->getPose().position.z);
	ROS_INFO("Orientation: %.3f, %.3f, %.3f, %.3f", part->getPose().orientation.x, part->getPose().orientation.y, part->getPose().orientation.z, part->getPose().orientation.w);
	ROS_INFO("RollPitchYaw: %.3f, %.3f, %.3f", r, p, y);
	ROS_INFO(" ");
}

void printContainer( std::shared_ptr<Container> c ){
	
	std::vector<std::string> parts = c->getParts();
	ROS_INFO("Name: %s, %lu parts", c->getName().c_str(), parts.size());
	for( int i = 0; i < parts.size(); ++i ){
		ROS_INFO("Part %d: %s", i+1, parts[i].c_str());
	}
	
	ROS_INFO(" ");
}

void getRPY( const geometry_msgs::Pose  pose, double & r, double & p, double & y ){
	
	tf2::Quaternion arm_q;
	tf2::fromMsg( pose.orientation, arm_q );
	tf2::Matrix3x3(arm_q).getRPY( r, p, y );
	
}
