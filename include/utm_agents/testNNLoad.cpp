#include <iostream>
#include <string>
#include <vector>

#include "NeuralNet.h"
#include "FileIn.h"
#include "yaml-cpp/yaml.h"

int main(){
  std::string config_file = "config.yaml";
  
	// Read some parameters from config file
  YAML::Node configs = YAML::LoadFile(config_file);
  size_t nPop = configs["neuroevo"]["popsize"].as<size_t>();
  double alpha = configs["constants"]["alpha"].as<double>();
  size_t nAgents = 18 ;
  
  std::vector<double> X ;

  for (size_t i = 0; i < nAgents; i++){
    NeuralNet nn(configs["neuroevo"],0) ;
    char buffer[500] ;
    sprintf(buffer,"/home/jchu2041/Documents/CPP/UTM_Project_Code/OSU_UTM_Project_Code/build/neural_nets/6_Sectors12/net_0_%i.csv",i) ;
    nn.load(buffer) ;
    for (size_t k = 0; k < 4; k++){
      X.clear() ;
      X.push_back((double)k) ;
      std::vector<double> Y = nn(X) ;
      std::cout << "NN(" << X[0] << "): " << Y[0]*alpha << ", " ;
    }
    std::cout << "\n" ;
  }
}
