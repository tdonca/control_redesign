#ifndef WORLD_STATEGRAPH
#define ARIAC_STATEGRAPH

#include <boost/graph/adjacency_list.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/graph/breadth_first_search.hpp>


namespace world {
	
	struct State;
	typedef boost::adjacency_list< boost::vecS, boost::vecS, boost::undirectedS, State > Graph;
	typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
	typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
	typedef boost::graph_traits<Graph>::adjacency_iterator adj_iter;
	enum StateNum { BOX, REMOVE, FACEBIN1, FACEBIN2, FACEBIN3, FACEBIN4, FACEBIN5, BIN1, BIN2, BIN3, BIN4, BIN5, NUMSTATES };
	
	class StateGraph {
		
		public:
			
			StateGraph()
			:	m_graph(StateNum::NUMSTATES)
			{}
			
			bool initializeGraph();
			
			bool findPath( std::string init, std::string goal, std::vector< State > & path );
			
			bool findState( std::string state_name, State & state );
			
			bool setStateJointValues( std::string state, std::vector<double> jv );
			
			
		private:
		
			Graph m_graph;
			
		
	};
	
	
	struct State {
		
		std::string name;
		int state_num;
		bool at_container;
		std::vector<double> joint_values;
	};
	
}

#endif
