#ifndef TASK_INTERFACE
#define TASK_INTERFACE


namespace task {
	
	
	class Task {
		
		public:
	
			virtual bool execute() = 0;
			
			virtual ~Task() {}
		
	};
	
}

#endif
