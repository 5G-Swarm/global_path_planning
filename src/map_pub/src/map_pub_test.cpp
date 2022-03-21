

#include <functional>
#include <mutex>
#include <thread>
#include <string>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>

// 终止：在这个地方，robot的局部地图会跟随tf坐标系转动，这样的话不方便裁剪地图，所以停止这个设想
namespace MAP_PUB
{
    class map_pub
    {
        public:
            map_pub(ros::NodeHandle private_nh);
        private:
            void CostmapSubCallback1(const nav_msgs::OccupancyGrid::ConstPtr& map);
            void CostmapSubCallback2(const nav_msgs::OccupancyGrid::ConstPtr& map);
            void CostmapSubCallback3(const nav_msgs::OccupancyGrid::ConstPtr& map);
            void CostmapSubCallback4(const nav_msgs::OccupancyGrid::ConstPtr& map);
            void GetRobotpose(std::string iden_frame_id,geometry_msgs::PoseStamped& global_pose, ros::Time timestamp);
            void track_map_1_pub();
            void track_map_2_pub();
            void track_map_3_pub();
            void track_map_4_pub();

        
            bool getCostmap;
            ros::Subscriber costmap_sub_1;
            ros::Subscriber costmap_sub_2;
            ros::Subscriber costmap_sub_3;
            ros::Subscriber costmap_sub_4;
            ros::Publisher costmap_pub_1;
            ros::Publisher costmap_pub_2;
            ros::Publisher costmap_pub_3;
            ros::Publisher costmap_pub_4;
            ros::Publisher costmap_track_pub_1;
            ros::Publisher costmap_track_pub_2;
            ros::Publisher costmap_track_pub_3;
            ros::Publisher costmap_track_pub_4;
            nav_msgs::OccupancyGrid costmap_1;
            nav_msgs::OccupancyGrid costmap_2;
            nav_msgs::OccupancyGrid costmap_3;
            nav_msgs::OccupancyGrid costmap_4;
            nav_msgs::OccupancyGrid track_costmap_1;
            nav_msgs::OccupancyGrid track_costmap_2;
            nav_msgs::OccupancyGrid track_costmap_3;
            nav_msgs::OccupancyGrid track_costmap_4;
            geometry_msgs::PoseStamped global_pose1;
            geometry_msgs::PoseStamped global_pose2;
            geometry_msgs::PoseStamped global_pose3;
            geometry_msgs::PoseStamped global_pose4;
            ros::Publisher map_pub_;
            std::mutex lock_costmap;
            ros::NodeHandle node;
            std::thread tf_thread_;
            tf::TransformListener tf_listener_;
            // int local_map_x;
            // int local_map_y;
            int robot1_index;
            int robot2_index;
            int robot3_index;
            int robot4_index;
            int robot1_track_index;
            int robot2_track_index;
            int robot3_track_index;
            int robot4_track_index;
            int width=20;//local_map_width
            int height=20;//local_map_height
            int black=100;
            int white=0;
            int grey=-1;
            // protected:
    };

    map_pub::map_pub(ros::NodeHandle private_nh)
        {
            std::cout<<"enter map_pub"<<std::endl;
            node = private_nh;
            costmap_sub_1 = private_nh.subscribe<nav_msgs::OccupancyGrid>("/robot_1/map", 1, &map_pub::CostmapSubCallback1, this);
            costmap_sub_2 = private_nh.subscribe<nav_msgs::OccupancyGrid>("/robot_2/map", 1, &map_pub::CostmapSubCallback2, this);
            costmap_sub_3 = private_nh.subscribe<nav_msgs::OccupancyGrid>("/robot_3/map", 1, &map_pub::CostmapSubCallback3, this);
            costmap_sub_4 = private_nh.subscribe<nav_msgs::OccupancyGrid>("/robot_4/map", 1, &map_pub::CostmapSubCallback4, this);
            costmap_pub_1 = private_nh.advertise<nav_msgs::OccupancyGrid>("/robot_1/map_pub_cyclic",1);
            costmap_pub_2 = private_nh.advertise<nav_msgs::OccupancyGrid>("/robot_2/map_pub_cyclic",1);
            costmap_pub_3 = private_nh.advertise<nav_msgs::OccupancyGrid>("/robot_3/map_pub_cyclic",1);
            costmap_pub_4 = private_nh.advertise<nav_msgs::OccupancyGrid>("/robot_4/map_pub_cyclic",1);
            costmap_track_pub_1 = private_nh.advertise<nav_msgs::OccupancyGrid>("/robot_1/track_map_pub",1);
            costmap_track_pub_2 = private_nh.advertise<nav_msgs::OccupancyGrid>("/robot_2/track_map_pubc",1);
            costmap_track_pub_3 = private_nh.advertise<nav_msgs::OccupancyGrid>("/robot_3/track_map_pub",1);
            costmap_track_pub_4 = private_nh.advertise<nav_msgs::OccupancyGrid>("/robot_4/track_map_pub",1);
            tf_thread_ = std::thread([this]() {
                ros::Rate rate(1.0);
                while (node.ok()) {
                    costmap_pub_1.publish(costmap_1);
                    costmap_pub_2.publish(costmap_2);
                    costmap_pub_3.publish(costmap_3);
                    costmap_pub_4.publish(costmap_4);
                    track_map_1_pub();
                    track_map_2_pub();
                    track_map_3_pub();
                    track_map_4_pub();
                    
                    rate.sleep();
                }
            });
            
        }
    void map_pub::GetRobotpose(std::string iden_frame_id,geometry_msgs::PoseStamped& global_pose, ros::Time timestamp)
            {
            tf::StampedTransform transform;
            geometry_msgs::PoseStamped iden_pose;
            iden_pose.header.frame_id = iden_frame_id;
            iden_pose.header.stamp = ros::Time::now(); 
            iden_pose.pose.orientation.w = 1;
            tf_listener_.waitForTransform("/map",iden_frame_id, ros::Time(0), ros::Duration(2.0));
            tf_listener_.lookupTransform( "/map",iden_frame_id, ros::Time(0), transform);
            global_pose.pose.position.x=transform.getOrigin().x();
            global_pose.pose.position.y=transform.getOrigin().y();
            global_pose.pose.position.z=transform.getOrigin().z();
            }
    void map_pub::CostmapSubCallback1(const nav_msgs::OccupancyGrid::ConstPtr& map)
        {
            ros::Time timestamp = map->header.stamp;
            getCostmap = true;
            std::cout << "enter costmapCb" << std::endl;
            // std::cout << "test 1" << std::endl;
            std::lock_guard<std::mutex> lock(lock_costmap);
            costmap_1 = *map;
            track_costmap_1.header.frame_id="robot_1/base_link";
            track_costmap_1.info.resolution=0.2;
            track_costmap_1.info.width=width;
            track_costmap_1.info.height=height;
            track_costmap_1.info.origin.position.x=-2;
            track_costmap_1.info.origin.position.y=-2;
            // for(int i=0;i<400;i++)
            //     local_costmap_1.data.push_back(0);
            GetRobotpose("/robot_1/base_link",global_pose1, timestamp);
            int a[3];
            a[0]=(global_pose1.pose.position.x+20)*100000;
            a[1]=(global_pose1.pose.position.y+20)*100000;
            a[2]=costmap_1.info.resolution*100000;
            robot1_index=(a[1]/a[2])*costmap_1.info.width+a[0]/a[2];


            //local map
            // robot1_local_index=robot1_index-costmap_1.info.width*height/2-width;
            // int x=local_costmap_1.info.height;
            // int k;
            // for(int i=0;i<height;i++)
            // {
            //     for(int j=0;j<width;j++)
            //     {
            //         std::cout<<local_costmap_1.data[robot1_local_index+i*x+j];

            //         // local_costmap_1.data.push_back(local_costmap_1.data[robot1_local_index+i*local_costmap_1.info.height+j]);
            //     }
            // }

            // global_map
            // for(int j=0;j<costmap_1.info.width/96;j++)
            //     {
            //         costmap_1.data[robot1_index-1+j]=white;
            //         costmap_1.data[robot1_index-1+costmap_1.info.width+j]=white;
            //         costmap_1.data[robot1_index-1-costmap_1.info.width+j]=white;
                    
            //         costmap_1.data[robot2_index-1+j]=white;
            //         costmap_1.data[robot2_index-1+costmap_2.info.width+j]=white;
            //         costmap_1.data[robot2_index-1-costmap_2.info.width+j]=white;

            //         costmap_1.data[robot3_index-1+j]=white;
            //         costmap_1.data[robot3_index-1+costmap_3.info.width+j]=white;
            //         costmap_1.data[robot3_index-1-costmap_3.info.width+j]=white;

            //         costmap_1.data[robot4_index-1+j]=white;
            //         costmap_1.data[robot4_index-1+costmap_4.info.width+j]=white;
            //         costmap_1.data[robot4_index-1-costmap_4.info.width+j]=white;
            //     }
                

            // local_costmap_1.data.push_back(100);
            // for(int i=1;i<400;i++)
            //     local_costmap_1.data.push_back(0);//终止：在这个地方，robot的局部地图会跟随tf坐标系转动，这样的话不方便裁剪地图，所以停止这个设想
        }
    void map_pub::CostmapSubCallback2(const nav_msgs::OccupancyGrid::ConstPtr& map)
        {
            ros::Time timestamp = map->header.stamp;
            getCostmap = true;
            std::cout << "enter costmapCb" << std::endl;
            // std::cout << "test 1" << std::endl;
            std::lock_guard<std::mutex> lock(lock_costmap);
            costmap_2 = *map;
            track_costmap_2.header.frame_id="robot_2/base_link";
            track_costmap_2.info.resolution=0.2;
            track_costmap_2.info.width=width;
            track_costmap_2.info.height=height;
            for(int i=0;i<400;i++)
                track_costmap_2.data.push_back(0);
            GetRobotpose("/robot_2/base_link",global_pose2, timestamp);
            int a[3];
            a[0]=(global_pose2.pose.position.x+20)*100000;
            a[1]=(global_pose2.pose.position.y+20)*100000;
            a[2]=costmap_2.info.resolution*100000;
            robot2_index=(a[1]/a[2])*costmap_2.info.width+a[0]/a[2];
            for(int j=0;j<costmap_2.info.width/96;j++)
                {
                    costmap_2.data[robot1_index-1+j]=white;
                    costmap_2.data[robot1_index-1+costmap_1.info.width+j]=white;
                    costmap_2.data[robot1_index-1-costmap_1.info.width+j]=white;
                    
                    costmap_2.data[robot2_index-1+j]=white;
                    costmap_2.data[robot2_index-1+costmap_2.info.width+j]=white;
                    costmap_2.data[robot2_index-1-costmap_2.info.width+j]=white;

                    costmap_2.data[robot3_index-1+j]=white;
                    costmap_2.data[robot3_index-1+costmap_3.info.width+j]=white;
                    costmap_2.data[robot3_index-1-costmap_3.info.width+j]=white;

                    costmap_2.data[robot4_index-1+j]=white;
                    costmap_2.data[robot4_index-1+costmap_4.info.width+j]=white;
                    costmap_2.data[robot4_index-1-costmap_4.info.width+j]=white;
                }

            }
    void map_pub::CostmapSubCallback3(const nav_msgs::OccupancyGrid::ConstPtr& map)
        {
            ros::Time timestamp = map->header.stamp;
            getCostmap = true;
            std::cout << "enter costmapCb" << std::endl;
            // std::cout << "test 1" << std::endl;
            costmap_3 = *map;
            track_costmap_3.header.frame_id="robot_3/base_link";
            track_costmap_3.info.resolution=0.2;
            track_costmap_3.info.width=width;
            track_costmap_3.info.height=height;
            GetRobotpose("/robot_3/base_link",global_pose3, timestamp);
            int a[3];
            a[0]=(global_pose3.pose.position.x+20)*100000;
            a[1]=(global_pose3.pose.position.y+20)*100000;
            a[2]=costmap_3.info.resolution*100000;
            robot3_index=(a[1]/a[2])*costmap_3.info.width+a[0]/a[2];
        }
    void map_pub::CostmapSubCallback4(const nav_msgs::OccupancyGrid::ConstPtr& map)
        {
            ros::Time timestamp = map->header.stamp;
            getCostmap = true;
            std::cout << "enter costmapCb1" << std::endl;
            std::lock_guard<std::mutex> lock(lock_costmap);
            costmap_4 = *map;
            track_costmap_4.header.frame_id="robot_4/base_link";
            track_costmap_4.info.resolution=0.2;
            track_costmap_4.info.width=width;
            track_costmap_4.info.height=height;
            int a[3];
            a[0]=(global_pose4.pose.position.x+20)*100000;
            a[1]=(global_pose4.pose.position.y+20)*100000;
            a[2]=costmap_4.info.resolution*100000;
            robot4_index=(a[1]/a[2])*costmap_4.info.width+a[0]/a[2];
        }

    void map_pub::track_map_1_pub()
    {
        costmap_track_pub_1.publish(track_costmap_1);
    }
    void map_pub::track_map_2_pub()
    {
        costmap_track_pub_2.publish(track_costmap_2);
    }
    void map_pub::track_map_3_pub()
    {
        costmap_track_pub_3.publish(track_costmap_3);
    }
    void map_pub::track_map_4_pub()
    {
        costmap_track_pub_4.publish(track_costmap_4);
    }

}
int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "map_pub");
    ros::NodeHandle private_nh("~");
    std::cout<<"enter"<<std::endl;
    MAP_PUB::map_pub tb(private_nh);
    std::cout<<"exit"<<std::endl;

    ros::MultiThreadedSpinner spinner;
    spinner.spin();
    return 0;

}



