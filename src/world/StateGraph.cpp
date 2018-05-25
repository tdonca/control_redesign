#include <world/StateGraph.hpp>
#include <algorithm>
#include <ros/ros.h>

namespace world {
	
	
	int stateToLabel( std::string state );
	
	bool StateGraph::initializeGraph(){
		
		m_graph[StateNum::BOX].name = "BOX";
		m_graph[StateNum::BOX].state_num = StateNum::BOX;
		m_graph[StateNum::BOX].at_container = true;
		
		m_graph[StateNum::REMOVE].name = "REMOVE";
		m_graph[StateNum::REMOVE].state_num = StateNum::REMOVE;
		m_graph[StateNum::REMOVE].at_container = false;
		
		m_graph[StateNum::FACEBIN1].name = "FACEBIN1";
		m_graph[StateNum::FACEBIN1].state_num = StateNum::FACEBIN1;
		m_graph[StateNum::FACEBIN1].at_container = false;
		
		m_graph[StateNum::FACEBIN2].name = "FACEBIN2";
		m_graph[StateNum::FACEBIN2].state_num = StateNum::FACEBIN2;
		m_graph[StateNum::FACEBIN2].at_container = false;
		
		m_graph[StateNum::FACEBIN3].name = "FACEBIN3";
		m_graph[StateNum::FACEBIN3].state_num = StateNum::FACEBIN3;
		m_graph[StateNum::FACEBIN3].at_container = false;
		
		m_graph[StateNum::FACEBIN4].name = "FACEBIN4";
		m_graph[StateNum::FACEBIN4].state_num = StateNum::FACEBIN4;
		m_graph[StateNum::FACEBIN4].at_container = false;
		
		m_graph[StateNum::FACEBIN5].name = "FACEBIN5";
		m_graph[StateNum::FACEBIN5].state_num = StateNum::FACEBIN5;
		m_graph[StateNum::FACEBIN5].at_container = false;
		
		m_graph[StateNum::BIN1].name = "BIN1";
		m_graph[StateNum::BIN1].state_num = StateNum::BIN1;
		m_graph[StateNum::BIN1].at_container = true;
		
		m_graph[StateNum::BIN2].name = "BIN2";
		m_graph[StateNum::BIN2].state_num = StateNum::BIN2;
		m_graph[StateNum::BIN2].at_container = true;
		
		m_graph[StateNum::BIN3].name = "BIN3";
		m_graph[StateNum::BIN3].state_num = StateNum::BIN3;
		m_graph[StateNum::BIN3].at_container = true;
		
		m_graph[StateNum::BIN4].name = "BIN4";
		m_graph[StateNum::BIN4].state_num = StateNum::BIN4;
		m_graph[StateNum::BIN4].at_container = true;
		
		m_graph[StateNum::BIN5].name = "BIN5";
		m_graph[StateNum::BIN5].state_num = StateNum::BIN5;
		m_graph[StateNum::BIN5].at_container = true;
		
		
		
		// Add edges between States
		
		add_edge( BOX, 		REMOVE, 	m_graph );
		add_edge( FACEBIN2, BOX, 		m_graph );
		add_edge( FACEBIN2, REMOVE, 	m_graph );
		
		add_edge( FACEBIN1, FACEBIN2, 	m_graph );
		add_edge( FACEBIN2, FACEBIN3, 	m_graph );
		add_edge( FACEBIN3, FACEBIN4, 	m_graph );
		add_edge( FACEBIN4, FACEBIN5,	m_graph );
		
		add_edge( FACEBIN1, BIN1, 		m_graph );
		add_edge( FACEBIN2, BIN2, 		m_graph );
		add_edge( FACEBIN3, BIN3, 		m_graph );
		add_edge( FACEBIN4, BIN4, 		m_graph );
		add_edge( FACEBIN5, BIN5, 		m_graph );
		
		return true;
	}

	bool StateGraph::findPath( std::string init, std::string goal, std::vector< State > & path ){
		
		ROS_INFO("%s - %s", init.c_str(), goal.c_str());
		
		std::vector<Vertex> predecessor_map(StateNum::NUMSTATES);
		Vertex s = stateToLabel(init);
		Vertex t = stateToLabel(goal);
		if( s == -1 || t == -1){
			return false;
		}
		
		
		// same states
		if(  init == goal ){
			path.clear();
			path.push_back( m_graph[s] );
			return true;
		}
		
		
		// perform BFS and record predecessors
		predecessor_map[s] = s;
		boost::breadth_first_search( m_graph, s, boost::visitor( boost::make_bfs_visitor( boost::record_predecessors( &predecessor_map[0], boost::on_tree_edge() ) ) ) );
		
		ROS_INFO("goal name: %s", m_graph[t].name.c_str());
		path.clear();
		path.push_back( m_graph[t] );
		Vertex p = t;
		
		// backtrack through predecessors to record path
		while( predecessor_map[p] != p ){
			p = predecessor_map[p];
			path.push_back( m_graph[p] );
			ROS_INFO("name: %s", m_graph[p].name.c_str());
		}
		reverse( path.begin(), path.end() );
		
		return true;
	}



	bool StateGraph::findState( std::string state_name, State & state ){
		
		// iterate through vertices and find the state
		vertex_iter vi, vi_end;
		for( boost::tie(vi, vi_end) = vertices(m_graph); vi != vi_end; ++vi ){
			
			Vertex v = *vi;
			if( m_graph[v].name == state_name ){
				state = m_graph[v];
				return true;
			}
		}
		
		ROS_ERROR("Could not find the state: %s", state_name.c_str());
		return false;
	}


	bool StateGraph::setStateJointValues( std::string state, std::vector<double> jv ){
		
		if( jv.size() == 8 ){
			
			int l = stateToLabel(state);
			m_graph[l].joint_values = jv;
		}
		else{
			ROS_ERROR("Invalid joint values provided");
			return false;
		}
		
		return true;
	}
	
	
	int stateToLabel( std::string state ){
		
		if( state == "BOX" ){
			return StateNum::BOX;
		}
		else if( state == "REMOVE" ){
			return StateNum::REMOVE;
		}
		else if( state == "FACEBIN1" ){
			return StateNum::FACEBIN1;
		}
		else if( state == "FACEBIN2" ){
			return StateNum::FACEBIN2;
		}
		else if( state == "FACEBIN3" ){
			return StateNum::FACEBIN3;
		}
		else if( state == "FACEBIN4" ){
			return StateNum::FACEBIN4;
		}
		else if( state == "FACEBIN5" ){
			return StateNum::FACEBIN5;
		}
		else if( state == "BIN1" ){
			return StateNum::BIN1;
		}
		else if( state == "BIN2" ){
			return StateNum::BIN2;
		}
		else if( state == "BIN3" ){
			return StateNum::BIN3;
		}
		else if( state == "BIN4" ){
			return StateNum::BIN4;
		}
		else if( state == "BIN5" ){
			return StateNum::BIN5;
		}
		else{
			ROS_ERROR("Invalid state provided!");
			return -1;
		}
	}
	
	
}
