/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

// #include <explore/costmap_client.h>

#include <functional>
#include <mutex>
#include <thread>
#include <string>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>

namespace MAP_PUB
{
    class map_pub
    {
        public:
            map_pub(ros::NodeHandle private_nh)
            {
                std::cout<<"enter map_pub"<<std::endl;
                node = private_nh;
                costmap_sub_1 = private_nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &map_pub::CostmapSubCallback1, this);
                costmap_pub_1 = private_nh.advertise<nav_msgs::OccupancyGrid>("/map_pub_cyclic",1);
                // local_costmap_pub_1 = private_nh.advertise<nav_msgs::OccupancyGrid>("/robot_1/local_map_pub",1);
                
                
                tf_thread_ = std::thread([this]() {
                    ros::Rate rate(1);
                    while (node.ok()) {
                        costmap_pub_1.publish(costmap_1);//global map
                        // local_costmap_pub_1.publish(local_costmap_1);//local map
                        rate.sleep();
                    }
                });
                
            }


        private:
            void CostmapSubCallback1(const nav_msgs::OccupancyGrid::ConstPtr& map)
            {
                ros::Time timestamp = map->header.stamp;
                getCostmap = true;
                std::cout << "enter costmapCb1" << std::endl;
                // std::cout << "test 1" << std::endl;
                std::lock_guard<std::mutex> lock(lock_costmap);
                costmap_1 = *map;
                // GetRobotpose("/robot_1/base_link",global_pose1, timestamp);
                // int a[3];
                // a[0]=(global_pose1.pose.position.x+global_costmap_size/2)*100000;
                // a[1]=(global_pose1.pose.position.y+global_costmap_size/2)*100000;
                // a[2]=costmap_1.info.resolution*100000;
                // robot1_index=(a[1]/a[2])*costmap_1.info.width+a[0]/a[2];

                // // local_costmap_1 = *map;
                // local_costmap_1.header=costmap_1.header;
                // // local_costmap_1.info.
                // local_costmap_1.info.resolution=costmap_1.info.resolution;
                // local_costmap_1.info.height=local_costmap_size;
                // local_costmap_1.info.width=local_costmap_size;
                // int robot1_start_index=robot1_index-local_costmap_size*local_costmap_size/2;
                // std::cout<<"robot1_start_index"<<robot1_start_index<<std::endl;
                // float local_costmap_position_x,local_costmap_position_y;
                // local_costmap_position_x = costmap_1.info.origin.position.x + a[0]/a[2] * costmap_1.info.resolution;
                // local_costmap_position_y = costmap_1.info.origin.position.y + a[1]/a[2] * costmap_1.info.resolution;
                // local_costmap_1.info.origin.position.x = local_costmap_position_x;
                // local_costmap_1.info.origin.position.y = local_costmap_position_y;
                // for(int i = 0 ; i<local_costmap_size ; i++)
                // {
                //     for(int j=0 ; j<local_costmap_size ; j++)
                //     {
                //         local_costmap_1.data.push_back(costmap_1.data.at(robot1_start_index+i*local_costmap_size+j));
                //     }
                // }
            // }
           
                //     {
                //         local_costmap_4.data.push_back(costmap_4.data.at(robot4_start_index+i*local_costmap_size+j));
                //     }
                // }
                
            }
            void GetRobotpose(std::string iden_frame_id,geometry_msgs::PoseStamped& global_pose, ros::Time timestamp)
            {
            tf::StampedTransform transform;
            geometry_msgs::PoseStamped iden_pose;
            iden_pose.header.frame_id = iden_frame_id;
            // iden_pose.header.frame_id = "robot_1/base_link";
            iden_pose.header.stamp = ros::Time::now(); 
            // std::cout << "Time: " << ros::Time::now() << std::endl;
            iden_pose.pose.orientation.w = 1;
            tf_listener_.waitForTransform("/map",iden_frame_id, ros::Time(0), ros::Duration(2.0));
            tf_listener_.lookupTransform( "/map",iden_frame_id, ros::Time(0), transform);
            // std::cout<<"坐标:"<<transform.getOrigin().x()<<","<<transform.getOrigin().y()<<","<<transform.getOrigin().z()<<std::endl;
            global_pose.pose.position.x=transform.getOrigin().x();
            global_pose.pose.position.y=transform.getOrigin().y();
            global_pose.pose.position.z=transform.getOrigin().z();
            // std::cout << "out" << std::endl;
            // tf_listener_.transformPose("/map", iden_pose, global_pose);
            }


            void mapToWorld(double mx, double my, double& wx, double& wy, const nav_msgs::OccupancyGrid gridmap_) {
                wx = gridmap_.info.origin.position.x + mx * gridmap_.info.resolution;
                wy = gridmap_.info.origin.position.y + my * gridmap_.info.resolution;
            }

            bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my, const nav_msgs::OccupancyGrid gridmap_) {
                double origin_x = gridmap_.info.origin.position.x, origin_y = gridmap_.info.origin.position.y;
                double resolution = gridmap_.info.resolution;

                if (wx < origin_x || wy < origin_y)
                    return false;

                mx = (wx - origin_x) / resolution;
                my = (wy - origin_y) / resolution;

                unsigned int sizecellx = gridmap_.info.width;
                unsigned int sizecelly = gridmap_.info.height;

                if (mx < sizecellx && my < sizecelly)
                    return true;

                return false;
            }

            void indexToCells(unsigned int index, unsigned int& x, unsigned int &y, const nav_msgs::OccupancyGrid gridmap_) {
                y = index / gridmap_.info.width;
                x = index - (y * gridmap_.info.width);

            }

            unsigned int getIndex(unsigned int x, unsigned int y, const nav_msgs::OccupancyGrid gridmap_){
                return y * gridmap_.info.width + x;
            }


        
            bool getCostmap;
            ros::Subscriber costmap_sub_1;
            ros::Subscriber costmap_sub_2;
            ros::Subscriber costmap_sub_3;
            ros::Subscriber costmap_sub_4;
            ros::Publisher costmap_pub_1;
            ros::Publisher costmap_pub_2;
            ros::Publisher costmap_pub_3;
            ros::Publisher costmap_pub_4;
            ros::Publisher local_costmap_pub_1;
            ros::Publisher local_costmap_pub_2;
            ros::Publisher local_costmap_pub_3;
            ros::Publisher local_costmap_pub_4;
            nav_msgs::OccupancyGrid costmap_1;
            nav_msgs::OccupancyGrid costmap_2;
            nav_msgs::OccupancyGrid costmap_3;
            nav_msgs::OccupancyGrid costmap_4;
            nav_msgs::OccupancyGrid local_costmap_1;
            nav_msgs::OccupancyGrid local_costmap_2;
            nav_msgs::OccupancyGrid local_costmap_3;
            nav_msgs::OccupancyGrid local_costmap_4;
            geometry_msgs::PoseStamped global_pose1;
            geometry_msgs::PoseStamped global_pose2;
            geometry_msgs::PoseStamped global_pose3;
            geometry_msgs::PoseStamped global_pose4;
            float local_costmap_size=20.0;
            float global_costmap_size=40.0;
            int robot1_index;
            int robot2_index;
            int robot3_index;
            int robot4_index;
            ros::Publisher map_pub_;
            std::mutex lock_costmap;
            ros::NodeHandle node;
            std::thread tf_thread_;
            tf::TransformListener tf_listener_;
            // protected:

    };
}
int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "map_pub");
    ros::NodeHandle private_nh("~");

    // tf2_ros::Buffer buffer(ros::Duration(10));
    // tf2_ros::TransformListener tf(buffer);

    std::cout<<"enter"<<std::endl;
    MAP_PUB::map_pub tb(private_nh);
    std::cout<<"exit"<<std::endl;

    ros::MultiThreadedSpinner spinner;
    spinner.spin();
    return 0;

}



