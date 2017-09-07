#include <ros/ros.h>
#include <ros/console.h>
#include <vector>
#include <string>
#include <algorithm>
#include <stdlib.h>
#include "agent_msgs/UtmGraph.h"
#include "agent_msgs/AgentMembership.h"
#include "agent_msgs/AgentCosts.h"
#include "utm_agents/NeuralNet.h"
#include "yaml-cpp/yaml.h"

using std::vector ;
using std::string ;
using std::min ;

class Agent{
  public:
    Agent(ros::NodeHandle) ;
    ~Agent(){delete agentPolicy ;}
  private:
    // pub/sub handlers
    vector<ros::Subscriber> subTraffic ; // subscribe to all membership messages from system traffic
    ros::Subscriber subGraph ; // subscribe to latest UTM cost graph
    ros::Publisher pubCosts ; // publish updated to UTM graph
    
    agent_msgs::UtmGraph graph ;
    agent_msgs::AgentCosts costs ;
    int agentID ;
    int numTraffic ;
    int linkCapacity ;
    double traversalTime ;
    double alpha ;
    double thresh = 0.00001 ;
    NeuralNet * agentPolicy ;
    vector<string> robotNames ;
    
    // Callback functions
    void graphCallback(const agent_msgs::UtmGraph&) ;
    void trafficCallback(const agent_msgs::AgentMembership&) ;
    
} ;

Agent::Agent(ros::NodeHandle nh){
  // Initialise pub/sub handlers
  subGraph = nh.subscribe("/utm_graph", 1, &Agent::graphCallback, this) ;
  pubCosts = nh.advertise<agent_msgs::AgentCosts>("costs", 1, true) ;
  
  // Read in traffic parameters
  ros::param::get("utm_agents/num_traffic", numTraffic) ;
  
  // Assign callback function to all membership topics
  for (int i = 0; i < numTraffic; i++){
    char buffer[50] ;
    sprintf(buffer,"/pioneer%i/membership",i+1) ;
    subTraffic.push_back(nh.subscribe(buffer, 10, &Agent::trafficCallback, this)) ;
  }
  
  // Read in agent parameters
  ros::param::get("utm_agents/agentID", agentID) ;
  string nnFile ;
  ros::param::get("/utm_agents/NNFile",nnFile) ;
  char paramBuffer[50] ;
  sprintf(paramBuffer,"/utm_agents/capacity/agent%i",agentID) ;
  ros::param::get(paramBuffer,linkCapacity) ;
  sprintf(paramBuffer,"/utm_agents/length/agent%i",agentID) ;
  ros::param::get(paramBuffer,traversalTime) ;
  
  // Read in corresponding agent NN policy
  char buffer[100] ;
  sprintf(buffer,"%s%i.csv",nnFile.c_str(),agentID) ;
  ROS_INFO_STREAM("Reading in agent policy from " << buffer) ;
  string config_file ;
  ros::param::get("/utm_agents/configFile",config_file) ;
  ROS_INFO_STREAM("Reading in multiagent team configuration from " << config_file) ;
  YAML::Node configs = YAML::LoadFile(config_file); 
  alpha = configs["constants"]["alpha"].as<double>();
  agentPolicy = new NeuralNet(configs["neuroevo"],0) ;
  agentPolicy->load(buffer) ;
  
  costs.agentID = agentID ;
  costs.traversal_time = traversalTime ;
  vector<double> x ;
  x.push_back(0.0) ;
  vector<double> y = (*agentPolicy)(x) ;
  costs.policy_output = y[0]*alpha ;
  costs.wait = false ;
  robotNames.clear() ;
  
  pubCosts.publish(costs) ; // publish initial values
  
  ROS_INFO_STREAM("Agent " << agentID << " initialised!") ;
}

void Agent::graphCallback(const agent_msgs::UtmGraph& msg){
  // Update values in stored graph variable
  // NOTE: used for copying values for other agents and checking that currently published values for
  // *this agent agree with internally stored value. If not in agreement, then republish with
  // correct values
  graph = msg ;
  bool repub = false ;
  
  if (abs(graph.actual_traversal_costs[agentID] - costs.traversal_time) > thresh){
    graph.actual_traversal_costs[agentID] = costs.traversal_time ;
    repub = true ;
  }
  if (abs(graph.policy_output_costs[agentID] - costs.policy_output) > thresh){
    graph.policy_output_costs[agentID] = costs.policy_output ;
    repub = true ;
  }
  if (graph.wait_to_enter[agentID] != costs.wait){
    graph.wait_to_enter[agentID] = costs.wait ;
    repub = true ;
  }
  
  if (repub){ // if inconsistency in *this agent's costs are discovered, republish latest costs
    pubCosts.publish(costs) ;
  }
}

void Agent::trafficCallback(const agent_msgs::AgentMembership& msg){
  bool agentChange = false ;
  bool robotFound = false ;
  for (size_t i = 0; i < robotNames.size(); i++){ // compare to robots current in *this agent's list
    if (msg.robot_name.compare(robotNames[i]) == 0){ // currently in list, will need to remove
      if (msg.parent != agentID){ // double check robot has moved to new agent
        robotNames.erase(robotNames.begin()+i) ; // remove robot from list of controlled robots
        agentChange = true ;
      }
      robotFound = true ;
      break ;
    }
  }
  if (!robotFound) { // not currently in list, will need to add
    if (msg.parent == agentID){ // double check robot has moved to *this agent
      robotNames.push_back(msg.robot_name) ;
      agentChange = true ;
    }
  }
  
  if (agentChange){
    int agentState = (int)robotNames.size() ; // total number of robots controlled by *this agent
    if (agentState >= linkCapacity){
      costs.wait = true ;
      ROS_INFO_STREAM("Agent " << agentID << " at capacity of " << agentState << " robots!") ;
    }
    else{
      if (costs.wait){
        ROS_INFO_STREAM("Agent " << agentID << " no longer full.") ;
      }
      costs.wait = false ;
    }
    
    vector<double> x ;
    x.push_back((double)agentState) ;
    vector<double> y = (*agentPolicy)(x) ;
    costs.policy_output = y[0]*alpha ;
    
    costs.traversal_time = traversalTime*((double)robotNames.size()+1.0) ;
    
    pubCosts.publish(costs) ;
  }
}
