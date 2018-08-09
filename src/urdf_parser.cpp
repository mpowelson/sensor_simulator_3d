#include <urdf/model.h>
#include "ros/ros.h"

#include <iostream>
#include <map>
#include <string>
#include <iterator>

#include <urdf_model/link.h>
#include <urdf_model/types.h>


int main(int argc, char** argv){

  // ---------------- Parse URDF ----------------------
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


    // --------------------- Print link map example ---------------------
    std::cout << "Printing link map" << std::endl;
    auto m = model.links_;  //Returns a map of links
    // Iterate through all links
    std::map<std::string, urdf::LinkSharedPtr>::iterator it = m.begin();
    while(it != m.end())
    {
      std::cout  <<  it->first  << " :: " <<  it->second  <<  std::endl;
      it++;
    }

    // --------------------- Get filename example ---------------------
    std::cout << std::endl << "Getting mesh filename example" << std::endl;
    auto linkPtr =  model.getLink("panda_link0");
    auto filepath =  boost::dynamic_pointer_cast<const urdf::Mesh>(linkPtr->visual->geometry)->filename;
    std::cout << filepath << std::endl << std::endl;
    auto Mesh = linkPtr->visual->geometry->MESH;  //This should give you the actual mesh


    // --------------------- Get all filename example ---------------------
    std::cout << "Getting all mesh filename example" << std::endl;
    std::map<std::string, urdf::LinkSharedPtr>::iterator it2 = m.begin();
    while(it2 != m.end())  // Note that it looks like there is a vectorized method to this as well with getLinks
    {
      std::cout<<it2->first<<" :: "<<it2->second<< "   ";



      auto linkPtr =  model.getLink(it2->first);

      if ((linkPtr && linkPtr->visual && linkPtr->visual->geometry) && (linkPtr && linkPtr->visual))
      {

        auto tmp =  boost::dynamic_pointer_cast<const urdf::Mesh>(linkPtr->visual->geometry);
        auto tmp2 = linkPtr->visual;
        if (tmp){
        if (tmp2){


          // Get transform in link frame
          double x = tmp2->origin.position.x;
          double y = tmp2->origin.position.y;
          double z = tmp2->origin.position.z;
          double qx, qy, qz, qw;
          tmp2->origin.rotation.getQuaternion(qx,qy,qz,qw);

          //Get filepath
          auto filepath =  tmp->filename;
//          linkPtr->name;        This is also the name of the tf associated with this link
          std::cout << filepath << "   " << x << "   " << y << "   " << z << "   " << linkPtr->parent_joint->name;




        }}

      }
      std::cout <<std::endl;
      it2++;
    }

    // --------------------- Get all joints example ---------------------
    std::cout << " ------- Joints ----------- " << std::endl ;
    auto joints = model.joints_;
    std::map<std::string, urdf::JointSharedPtr>::iterator it3 = joints.begin();
    while(it3 != joints.end())
    {
      std::cout<<it3->first<<" :: "<<it3->second<< "   ";



      auto jointPtr =  model.getLink(it3->first);

      std::cout <<std::endl;
      it3++;
    }



    return 0;
}
