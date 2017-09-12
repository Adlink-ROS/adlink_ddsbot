/*
# Copyright 2017 ADLINK Technology, Inc.
# Developer: HaoChih, LIN (haochih.lin@adlinktech.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
*/

#include <adlink_ddsbot/swarm_layer.h>
#include <math.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ddsbot_swarm_layer::SwarmLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace ddsbot_swarm_layer
{
    void SwarmLayer::onInitialize()
    {
        ros::NodeHandle nh("~/" + name_), g_nh;
        current_ = true;
        first_time_ = true;
        robots_sub_ = nh.subscribe("/multi_robots", 1, &SwarmLayer::robotsCallback, this);
        // Dynamic recofigure
        server_ = new dynamic_reconfigure::Server<adlink_ddsbot::SwarmLayerConfig>(nh);
        f_ = boost::bind(&SwarmLayer::configure, this, _1, _2);
        server_->setCallback(f_);
    }
    
    void SwarmLayer::robotsCallback(const adlink_ddsbot::MultiRobots& robots) 
    {
        boost::recursive_mutex::scoped_lock lock(lock_);
        robots_list_ = robots;
    }

    double SwarmLayer::gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew)
    {
        double dx = x-x0, dy = y-y0;
        double h = sqrt(dx*dx+dy*dy);
        double angle = atan2(dy,dx);
        double mx = cos(angle-skew) * h;
        double my = sin(angle-skew) * h;
        double f1 = pow(mx, 2.0)/(2.0 * varx), 
               f2 = pow(my, 2.0)/(2.0 * vary);
        return A * exp(-(f1 + f2));
    }

    void SwarmLayer::updateBoundsFromRobots(double* min_x, double* min_y, double* max_x, double* max_y)
    {
        std::list<adlink_ddsbot::Robot>::iterator r_it;
        
        for(r_it = transformed_robots_.begin(); r_it != transformed_robots_.end(); ++r_it)
        {
            adlink_ddsbot::Robot robot = *r_it;
            double radius = robot.radius;
                          
            *min_x = std::min(*min_x, robot.transform.transform.translation.x - radius);
            *min_y = std::min(*min_y, robot.transform.transform.translation.y - radius);
            *max_x = std::max(*max_x, robot.transform.transform.translation.x + radius);
            *max_y = std::max(*max_y, robot.transform.transform.translation.y + radius);
              
        }
    }


    void SwarmLayer::updateBounds(double origin_x, double origin_y, double origin_z, double* min_x, double* min_y, double* max_x, double* max_y)
    {
        boost::recursive_mutex::scoped_lock lock(lock_);
        
        std::string global_frame = layered_costmap_->getGlobalFrameID();
        transformed_robots_.clear();
        
        for(unsigned int i=0; i<robots_list_.robots.size(); i++)
        {
            adlink_ddsbot::Robot& robot = robots_list_.robots[i];
            adlink_ddsbot::Robot robot_tmp = robot; // copy of the original one
            if(global_frame != robot.transform.header.frame_id)  
            {            
                try
                {
                    geometry_msgs::PointStamped pt_in, pt_out; // for tf::transformPoint()      
                    pt_in.point.x = robot.transform.transform.translation.x;
                    pt_in.point.y = robot.transform.transform.translation.y;
                    pt_in.point.z = robot.transform.transform.translation.z;
                    pt_in.header.frame_id = robot.transform.header.frame_id;
                    tf_.transformPoint(global_frame, pt_in, pt_out);
                    // update the transformed robot position
                    robot_tmp.transform.header.frame_id = global_frame;
                    robot_tmp.transform.transform.translation.x = pt_out.point.x;
                    robot_tmp.transform.transform.translation.y = pt_out.point.y;
                    robot_tmp.transform.transform.translation.z = pt_out.point.z;                  
                }
                catch(tf::LookupException& ex) 
                {
                  ROS_ERROR("No Transform available Error: %s\n", ex.what());
                  continue;
                }
                catch(tf::ConnectivityException& ex) 
                {
                  ROS_ERROR("Connectivity Error: %s\n", ex.what());
                  continue;
                }
                catch(tf::ExtrapolationException& ex)
                {
                  ROS_ERROR("Extrapolation Error: %s\n", ex.what());
                  continue;
                }
            }
            
            transformed_robots_.push_back(robot_tmp);
        }// end of for loop
        
        updateBoundsFromRobots(min_x, min_y, max_x, max_y);
        if(first_time_)
        {
            last_min_x_ = *min_x;
            last_min_y_ = *min_y;    
            last_max_x_ = *max_x;
            last_max_y_ = *max_y;    
            first_time_ = false;
        }
        else
        {
            double a = *min_x, b = *min_y, c = *max_x, d = *max_y;
            *min_x = std::min(last_min_x_, *min_x);
            *min_y = std::min(last_min_y_, *min_y);
            *max_x = std::max(last_max_x_, *max_x);
            *max_y = std::max(last_max_y_, *max_y);
            last_min_x_ = a;
            last_min_y_ = b;
            last_max_x_ = c;
            last_max_y_ = d;
        
        }
        
    }


    void SwarmLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        boost::recursive_mutex::scoped_lock lock(lock_);
        if(!enabled_) return;

        if( robots_list_.robots.size() == 0 )
          return;
        
        std::list<adlink_ddsbot::Robot>::iterator r_it;
        costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
        double res = costmap->getResolution();
        
        for(r_it = transformed_robots_.begin(); r_it != transformed_robots_.end(); ++r_it)
        {
            adlink_ddsbot::Robot robot = *r_it;
            double radius = robot.radius;
            
            unsigned int width  = int( (radius*2) / res ),
                         height = int( (radius*2) / res );
                          
            double center_Wx = robot.transform.transform.translation.x,
                   center_Wy = robot.transform.transform.translation.y;

            double origin_Wx, origin_Wy;
            origin_Wx = center_Wx - radius;
            origin_Wy = center_Wy - radius;

            int dx, dy;
            costmap->worldToMapNoBounds(origin_Wx, origin_Wy, dx, dy);
            int cx, cy;
            costmap->worldToMapNoBounds(center_Wx, center_Wy, cx, cy);

            int start_x=0, start_y=0, end_x=width, end_y=height;
            if(dx < 0)
                start_x = -dx;
            else if(dx + width > costmap->getSizeInCellsX())
                end_x = (int)costmap->getSizeInCellsX() - dx;

            if((int)(start_x+dx) < min_i)
                start_x = min_i - dx;
            if((int)(end_x+dx) > max_i)
                end_x = max_i - dx;

            if(dy < 0)
                start_y = -dy;
            else if(dy + height > costmap->getSizeInCellsY())
                end_y = (int)costmap->getSizeInCellsY() - dy;

            if((int)(start_y+dy) < min_j)
                start_y = min_j - dy;
            if((int)(end_y+dy) > max_j)
                end_y = max_j - dy;
            
            // label the footprint of robot on the costmap (lethal)
            for(int i=start_x;i<end_x;i++)
            {
                for(int j=start_y;j<end_y;j++)
                {
                    unsigned char old_cost = costmap->getCost(i+dx, j+dy);
                    if(old_cost == costmap_2d::NO_INFORMATION)
                        continue;

                    if( std::sqrt( (i+dx-cx)*(i+dx-cx) + (j+dy-cy)*(j+dy-cy) ) > (radius/res) )
                        continue;        
    
                    costmap->setCost(i+dx, j+dy, costmap_2d::LETHAL_OBSTACLE);
                }
            }    
        }
    }


    void SwarmLayer::configure(adlink_ddsbot::SwarmLayerConfig &config, uint32_t level)
    {
        enabled_ = config.enabled;
    }

}; // end of namespace
