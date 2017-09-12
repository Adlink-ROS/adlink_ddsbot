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

#ifndef SWARM_LAYER_H_
#define SWARM_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <adlink_ddsbot/MultiRobots.h> //custom msg
#include <geometry_msgs/PointStamped.h>
#include <dynamic_reconfigure/server.h>
#include <adlink_ddsbot/SwarmLayerConfig.h>
#include <boost/thread.hpp>

namespace ddsbot_swarm_layer
{
    class SwarmLayer : public costmap_2d::Layer
    {
        public:
            SwarmLayer() { layered_costmap_ = NULL; }

            virtual void onInitialize();
            virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
            virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

        protected:
            void robotsCallback(const adlink_ddsbot::MultiRobots& robots);
            void updateBoundsFromRobots(double* min_x, double* min_y, double* max_x, double* max_y);
            double gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew);
            void configure(adlink_ddsbot::SwarmLayerConfig &config, uint32_t level);
            dynamic_reconfigure::Server<adlink_ddsbot::SwarmLayerConfig>* server_;
            dynamic_reconfigure::Server<adlink_ddsbot::SwarmLayerConfig>::CallbackType f_;
            bool enabled_;

            ros::Subscriber robots_sub_;
            adlink_ddsbot::MultiRobots robots_list_;
            std::list<adlink_ddsbot::Robot> transformed_robots_;
            ros::Duration robots_keep_time_;
            boost::recursive_mutex lock_;
            tf::TransformListener tf_;
            bool first_time_;
            double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
    };
};


#endif

