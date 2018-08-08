#include <urdf/model.h>
#include "ros/ros.h"

#include <iostream>
#include <map>
#include <string>
#include <iterator>

#include <urdf_model/link.h>
#include <urdf_model/types.h>


int main(int argc, char** argv){

    ros::init(argc, argv, "my_parser");
    if (argc != 2){
      ROS_ERROR("Need a urdf file as argument");
      return -1;
    }
    std::string urdf_file = argv[1];

    urdf::Model model;
    if (!model.initFile(urdf_file)){
      ROS_ERROR("Failed to parse urdf file");
      return -1;
    }
    ROS_INFO("Successfully parsed urdf file");

    std::cout << "Printing link map" << std::endl;

    auto m = model.links_;  //Returns a map of links

    // Iterate through all links
    std::map<std::string, urdf::LinkSharedPtr>::iterator it = m.begin();
    while(it != m.end())
    {
      std::cout<<it->first<<" :: "<<it->second<<std::endl;
      it++;
    }
    std::cout << "Getting mesh filename example" << std::endl;
    auto linkPtr =  model.getLink("panda_link0");

        auto filepath =  boost::dynamic_pointer_cast<const urdf::Mesh>(linkPtr->visual->geometry)->filename;
        std::cout << filepath << std::endl;

        auto Mesh = linkPtr->visual->geometry->MESH;  //This should give you the actual mesh




//  ptr.get();
//    auto x = m.begin();
//    std::cout << std::to_string(x) << std::endl;
//    for (const auto &p : m) {
//      std::cout << "m[" << p.first << "] = " << p.second << '\n';
//    }
//    std::cout << model.links_ << std::endl;

    return 0;
}
