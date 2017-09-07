#include <ros/ros.h>
#include <ros/console.h>
#include <vector>
#include <string>
#include <algorithm>
#include <stdlib.h>
#include "agent_msgs/UtmGraph.h"
#include "agent_msgs/AgentCosts.h"

using std::vector ;

class UtmManager{
  public:
    UtmManager(ros::NodeHandle) ;
    ~UtmManager(){}
  private:
    // pub/sub handlers
    vector<ros::Subscriber> subAgents ; // subscribe to all agent cost messages
    ros::Publisher pubGraph ; // publish updated to UTM graph
    
    // callback function
    void costCallback(const agent_msgs::AgentCosts&) ;
    
    vector<int> agentIDs ;
    int numAgents ;
    bool initialised ;
    agent_msgs::UtmGraph graph ;
} ;

UtmManager::UtmManager(ros::NodeHandle nh): initialised(false){
  ros::param::get("/utm_agents/num_agents", numAgents);
  ROS_INFO_STREAM("Initialising UTM manager for " << numAgents << " agents...") ;
  
  // Initialise graph to UTM team size, assign callback function to all agent cost topics
  for (int i = 0; i < numAgents; i++){
    agentIDs.push_back(-1) ; // initialise to -1, used to check that all agents are loaded and activated
    graph.actual_traversal_costs.push_back(0.0) ;
    graph.policy_output_costs.push_back(0.0) ;
    graph.wait_to_enter.push_back(false) ;
    
    char buffer[50] ;
    sprintf(buffer,"/agent%i/costs",i) ;
    subAgents.push_back(nh.subscribe(buffer, 10, &UtmManager::costCallback, this)) ;
  }
  pubGraph = nh.advertise<agent_msgs::UtmGraph>("/utm_graph", 10, true) ;
}

void UtmManager::costCallback(const agent_msgs::AgentCosts& msg){
  if (agentIDs[msg.agentID] == -1){
    agentIDs[msg.agentID] = msg.agentID ;
    ROS_INFO_STREAM("Received initial costs from Agent " << agentIDs[msg.agentID]) ;
    
    bool init = true ;
    for (size_t i = 0; i < agentIDs.size(); i++){
      if (agentIDs[i] == -1){
        init = false ;
        break ;
      }
    }
    if (init){
      initialised = true ;
      ROS_INFO("All agents initialised in UTM graph") ;
    }
  }
  else{
    ROS_INFO_STREAM("Received new costs for Agent " << msg.agentID) ;
  }
  graph.actual_traversal_costs[msg.agentID] = msg.traversal_time ;
  graph.policy_output_costs[msg.agentID] = msg.policy_output ;
  graph.wait_to_enter[msg.agentID] = msg.wait ;
  
  if (initialised){ // only publish graph if all agents have been initialised
    pubGraph.publish(graph) ;
  }
}
