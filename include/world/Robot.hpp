#ifndef WORLD_ROBOT
#define WORLD_ROBOT

#include <string>
#include <vector>

namespace world {
		
	class StateGraph;	
	
	class Robot {
		
		public:
		
			virtual std::string getName() = 0;
			
			virtual std::string getState() = 0;
			
			virtual std::string getGraphState() = 0;
			
			virtual std::vector<double> getJoints() = 0;
			
			virtual bool initialize( StateGraph & graph ) = 0;
			
			virtual void printRobot() = 0;
			
			virtual ~Robot() {}
	};
	
}


#endif
