#ifndef ORDER_SCHEDULER
#define ORDER_SCHEDULER

#include <ros/ros.h>
#include <deque>
#include <osrf_gear/Order.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_redesign/FillShipmentAction.h>


namespace order {
	
	class OrderScheduler {
		
		public:
		
			OrderScheduler()
			:	m_node(),
				m_order_sub(),
				m_shipment_ac( m_node, "fill_shipment", true ),
				m_shipment_q()
				
			{
				m_shipment_ac.waitForServer();
				m_order_sub = m_node.subscribe( "ariac/orders", 1, &OrderScheduler::cb_newOrder, this );
				
				ROS_INFO("Order Scheduler initialized.");	
			}
			
			
			
		private:
			
			void cb_newOrder( const osrf_gear::Order::ConstPtr & msg );
			
			void acb_shipmentFinished( const actionlib::SimpleClientGoalState & state, const control_redesign::FillShipmentResultConstPtr & result );
			
			
			
			ros::NodeHandle m_node;
			ros::Subscriber m_order_sub;
			actionlib::SimpleActionClient<control_redesign::FillShipmentAction> m_shipment_ac;
			std::deque<osrf_gear::Shipment> m_shipment_q;
			
		
	};
	
	
}


#endif
