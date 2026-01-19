// Copyright (c) 2023.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <unordered_map>

#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include <gazebo_map_creator_interface/MapRequest.h>
#include <ignition/math/Vector3.hh>
#include <boost/gil.hpp>
#include <boost/gil/io/dynamic_io_new.hpp>
#include <boost/gil/extension/io/png/old.hpp>
#include <boost/shared_ptr.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <octomap/octomap.h>
#include <ros/ros.h>
#include <ros/spinner.h>

namespace gazebo
{

class GazeboMapCreator : public gazebo::SystemPlugin
{
  std::unique_ptr<ros::NodeHandle> ros_node_;
  ros::ServiceServer map_service_;
  std::unique_ptr<ros::AsyncSpinner> spinner_;
  physics::WorldPtr world_;

  /// To be notified once the world is created.
  gazebo::event::ConnectionPtr world_created_event_;

  public: void Load(int argc, char ** argv)
  {
    if (!ros::isInitialized()) {
      ros::init(argc, argv, "gazebo_map_creator", ros::init_options::NoSigintHandler);
    }

    ros_node_ = std::make_unique<ros::NodeHandle>();
    spinner_ = std::make_unique<ros::AsyncSpinner>(1);
    spinner_->start();

    // Get a callback when a world is created
    this->world_created_event_ = gazebo::event::Events::ConnectWorldCreated(
      std::bind(&GazeboMapCreator::OnWorldCreated, this, std::placeholders::_1));
 
  }

  public: void OnWorldCreated(const std::string & _world_name)
  {
      world_created_event_.reset();
      world_ = gazebo::physics::get_world(_world_name);

      // Create service for map creation request
      map_service_ = ros_node_->advertiseService(
        "/world/save_map",
        &GazeboMapCreator::OnMapCreate,
        this);

      ROS_INFO("gazebo_map_creator: service /world/save_map advertised");
  }
  
  public: bool OnMapCreate(gazebo_map_creator_interface::MapRequest::Request & req,
                           gazebo_map_creator_interface::MapRequest::Response & res)
  {
    ROS_INFO("gazebo_map_creator: received map request");

     // Calculate the size of the cubic area
    float size_x = req.upperleft.x - req.lowerright.x;
    float size_y = req.lowerright.y - req.upperleft.y;  // size_y to be -ve
    float size_z = req.upperleft.z - req.lowerright.z;
    
    // Check for coordinate validity
    if (size_x <= 0 || size_y >=0 || size_z <= 0 )
    {
        ROS_ERROR("gazebo_map_creator: invalid coordinates");
        res.success = false;
        return true;
    }

    const double resolution = (req.resolution > 0.0) ? req.resolution : 0.01;
    const double range_multiplier = (req.range_multiplier > 0.0) ? req.range_multiplier : 0.55;
    const bool skip_vertical_scan = (req.skip_vertical_scan != 0);

    int threshold_2d = req.threshold_2d;
    if (threshold_2d < 0) {
      threshold_2d = 0;
    } else if (threshold_2d > 255) {
      threshold_2d = 255;
    }

    const std::string filename = req.filename.empty() ? std::string("map") : req.filename;

    // Calculate the number of points in each dimension
    int num_points_x = static_cast<int>(std::abs(size_x) / resolution) + 1;
    int num_points_y = static_cast<int>(std::abs(size_y) / resolution) + 1;
    int num_points_z = static_cast<int>(std::abs(size_z) / resolution) + 1;
    
    // Calculate the step size in each dimension
    float step_x = size_x / num_points_x;
    float step_y = size_y / num_points_y;
    float step_z = size_z / num_points_z;   
 
    int dims = 6;

    if (skip_vertical_scan) 
    {
      num_points_z = 2;
      step_z = size_z;
      dims = 4;
    }

    std::cout << "-----------------" << std::endl << "Area Corners: (lower right, upper left)  (" <<
      req.lowerright.x << ", " << req.lowerright.y << ", " << req.lowerright.z << "), (" <<
      req.upperleft.x << ", " << req.upperleft.y << ", " << req.upperleft.z << ") " <<  std::endl <<
      "Area size : " << size_x << " x " << size_y << " x " << size_z << " (WxLxH)" << std::endl <<
      "Step size : " << step_x << ", " << step_y << ", " << step_z << " (stepx, stepy, stepz) " << std::endl <<
      "Resolution: (" << num_points_x << ", " <<  num_points_y << ", " << num_points_z << ") - "  << resolution << std::endl <<
      "Map Mode: " << (skip_vertical_scan ? "Partial Scan": "Full Scan")  << std::endl << "-----------------" << std::endl ;

    // Create ray object for collision detection
    gazebo::physics::PhysicsEnginePtr engine = world_->Physics();
    engine->InitForThread();
    gazebo::physics::RayShapePtr ray =
        boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
        engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

    // Pixed values near to 0 means occupied and 255 means empty
    boost::gil::gray8_pixel_t fill(255 - threshold_2d);
    boost::gil::gray8_pixel_t blank(255);
    boost::gil::gray8_image_t image(num_points_x, num_points_y);

    // Initially fill all area with empty pixel value
    boost::gil::fill_pixels(image._view, blank);

    // Create a point cloud object
    pcl::PointCloud<pcl::PointXYZ> cloud;
    std::string entityName;
    ignition::math::Vector3d start, end;
    double dx, dy, dz, dist;
    cloud.width    = num_points_x;
    cloud.height   = num_points_y;
        
    struct PointMask {
                int x, y, z;
            };
    PointMask directions[6] = {
                {-1, 0, 0}, // Left
                {1, 0, 0},  // Right
                {0, 1, 0},  // Front
                {0, -1, 0}, // Back
                {0, 0, 1},  // Top
                {0, 0, -1}  // Bottom
            };

    for (int x = 0; x < num_points_x; ++x) 
    {
        std::cout << "\rPercent complete: " << x * 100.0 / num_points_x << "%       " << std::flush;
        
        double cur_x = req.lowerright.x + x * step_x; 
        for (int y = 0; y < num_points_y; ++y)
        {
            double cur_y = req.upperleft.y + y * step_y;    
            
            ignition::math::Vector3d startV(cur_x, cur_y, req.lowerright.z);
            ignition::math::Vector3d endV(cur_x, cur_y, req.upperleft.z);
            
            // Detect collision upward until top of area.  z direction
            ray->SetPoints(startV, endV);
            ray->GetIntersection(dist, entityName);              
            if (!entityName.empty())
            { 
              image._view(x,y) = fill;                      
            }
           

            // Walk in z direction to check each point for any collision to it's neighbours
            for (int z = 0; z < num_points_z; ++z)
            {
                double cur_z = req.lowerright.z + z * step_z;
                ignition::math::Vector3d start(cur_x, cur_y, cur_z);

                // Check for right, left, front, back, top and bottom points for collision.
                // 2d mode checks for right, left, front and back.  see dims assignment.
                for(int i = 0; i < dims; ++i) {
                    dx =  directions[i].x * step_x * range_multiplier;
                    dy =  directions[i].y * step_y * range_multiplier;
                    dz =  directions[i].z * step_z * range_multiplier;
                    ignition::math::Vector3d end(cur_x + dx, cur_y + dy, cur_z + dz);
                    
                    ray->SetPoints(start, end);
                    ray->GetIntersection(dist, entityName);
                    if (!entityName.empty())
                    { 
                      // Collision found.  Push point to cloud and set in image
                      cloud.push_back(pcl::PointXYZ(cur_x, cur_y, cur_z));
                      image._view(x,y) = fill;
                      break;
                    }            
                }
            }
        }
    }
   
    std::cout << std::endl << "Completed calculations, writing to image" << std::endl;
    
    if (!filename.empty())
    { 
        // Save pcd file even if empty, so downstream tooling can rely on its existence.
        pcl::io::savePCDFileASCII (filename + ".pcd", cloud);

        if(cloud.size() > 0)
        {
          // Save octomap file
          octomap::OcTree octree(resolution);
          for (auto p:cloud.points)
              octree.updateNode(octomap::point3d(p.x, p.y, p.z), true );
          octree.updateInnerOccupancy();
          octree.writeBinary(filename + ".bt");
        }

        // Save png file
        boost::gil::gray8_view_t view = image._view;
        boost::gil::png_write_view(filename+".png", view); 

        // Save pgm file
        pgm_write_view(filename, view);

        // Write down yaml file for nav2 usage.
        std::unordered_map<std::string, std::string> yaml_dict;
        yaml_dict["image"] = filename + ".pgm";
        yaml_dict["mode"] = "trinary";
        yaml_dict["resolution"] = std::to_string(resolution);
        yaml_dict["origin"] =  "[" + std::to_string(req.lowerright.x) + std::string(", ") + std::to_string(req.lowerright.y) + std::string(", 0.0]");
        yaml_dict["negate"] = "0";
        yaml_dict["occupied_thresh"] = "0.95";  // hardcoding these values since we absolutely know occupied or free
        yaml_dict["free_thresh"] = "0.90";  

        std::ofstream outputFile(filename + ".yaml");
        if (outputFile.is_open()) {
            for (const auto& pair : yaml_dict) {
                outputFile << pair.first << ": " << pair.second << std::endl;
            }
            outputFile.close();
        } else {
            std::cout << "Unable to open yaml file for writing." << std::endl;
        }

        std::cout << "Output location: " << filename + "[.pcd, .bt, .pgm, .png, .yaml]" << std::endl;
    }

   
    std::cout << std::endl;
    res.success = true;
    return true;
  }


  public: void pgm_write_view(const std::string& filename, boost::gil::gray8_view_t& view)
  {
    // Write image to pgm file

    int h = view.height();
    int w = view.width();

    std::ofstream ofs;
    ofs.open(filename+".pgm");
    ofs << "P2" << '\n';          // grayscale
    ofs << w << ' ' << h << '\n'; // width and height
    ofs << 255 <<  '\n';          // max value
    for (int y = 0; y < h; ++y){
      for (int x = 0; x < w; ++x){
        // std::cout << (int)view(x, y)[0];
        ofs << (int)view(x, y)[0] << ' ';
      }
      ofs << '\n';
    }
    ofs.close();
  }
};

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(GazeboMapCreator)
}
