#include <functional>
#include <mutex>
#include <thread>
#include <string>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "bspline.cpp"
#include <queue>
#include <unordered_map>
// #include <boost>
#include <time.h>
#include <fstream>
#include <algorithm>

#define INF INT_MAX
using namespace std;

namespace global_path_planning
{

    struct Node{
    float cost;
    int index;
    bool operator<(const Node &a) const {
        return cost < a.cost;
    }
    };
    class global_path_planning_
    {
        public:
            global_path_planning_(ros::NodeHandle private_nh)
            {
                getCostmap = false;
                NODE = private_nh;
                costmap_sub_global_ = private_nh.subscribe<nav_msgs::OccupancyGrid>("/map_pub_cyclic", 1, &global_path_planning_::CostmapSubCallback_global, this);
                goal_sub_global_ = private_nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &global_path_planning_::goal_callback, this);
                position_sub_global_ = private_nh.subscribe<nav_msgs::Odometry>("/base2map", 1, &global_path_planning_::position_callback, this);
                costmap_with_path_ = private_nh.advertise<nav_msgs::OccupancyGrid>("/costmap_with_path", 1);
                robot_position_pub_=private_nh.advertise<geometry_msgs::PoseStamped>("/robot_position", 1);
                global_path_pub_=private_nh.advertise<nav_msgs::Path>("/global_path", 1);

            }


        private:
            void position_callback(const nav_msgs::Odometry::ConstPtr& msg)
                {
                    nav_msgs::Odometry position=*msg;
                    double robot_position_x = position.pose.pose.position.x;
                    double robot_position_y = position.pose.pose.position.y;
                    // robot_index_x = global_costmap.info.width-position.pose.position.x-1;
                    robot_index_y = 741-20*robot_position_x;
                    robot_index_x = 1165+20*robot_position_y;
                    // cout<<"robot_index_x:"<<robot_index_x<<"  robot_index_y:"<<robot_index_y<<endl;
                    // double robot_index_y = position.pose.position.y;
                    // robot_index_x = position.pose.pose.position.x;
                    robot_index_y = global_costmap.info.height-robot_index_y-1;
                    
                    
                }
            void indextocell(int index, int &x, int &y)
            {
                x = index % global_costmap.info.width;
                y = index / global_costmap.info.width;
            }
            void maptoworld(int x, int y, double &wx, double &wy)
            {
                // wx = origin_x_ + (x + 0.5) * resolution_;
                // wy = origin_y_ + (y + 0.5) * resolution_;
                int u=x, v=global_costmap.info.height-y-1;
                wx = (741-v)/20;
                wy = (1165-u)/20;
            }
            bool isInBounds(int x, int y)
            {
                if( x < 0 || y < 0 || x >= global_costmap.info.height || y >= global_costmap.info.width)
                    return false;
                return true;
            }
            vector<int> get_neighbors(int current_cell)
            {   
                vector<int> neighborIndexes;
                
                for (int i = -1; i <= 1; i++)
                {
                    for (int j = -1; j <= 1; j++)
                    {
                        unsigned tmp1, tmp2;
                        // costmap_->indexToCells(current_cell, tmp1, tmp2);
                        tmp1 = current_cell%global_costmap.info.width;
                        tmp2 = current_cell/global_costmap.info.width;
                        int nextX = tmp1 + i;
                        int nextY = tmp2 + j;
                        // int nextIndex = costmap_->getIndex(nextX, nextY);
                        int nextIndex = nextX+nextY*global_costmap.info.width;

                        if(!( i == 0 && j == 0) && isInBounds(nextX, nextY) && OGM[nextIndex])
                        {
                            neighborIndexes.push_back(nextIndex);
                        }
                    }
                }
                return neighborIndexes;
            }
            
            double getMoveCost(int firstIndex, int secondIndex)
            {
                unsigned int tmp1, tmp2;
                // costmap_->indexToCells(firstIndex, tmp1, tmp2);
                tmp1 = firstIndex%global_costmap.info.width;
                tmp2 = firstIndex/global_costmap.info.width;
                int firstXCord = tmp1,firstYCord = tmp2;
                // costmap_->indexToCells(secondIndex, tmp1, tmp2);
                tmp1 = secondIndex%global_costmap.info.width;
                tmp2 = secondIndex/global_costmap.info.width;
                int secondXCord = tmp1, secondYCord = tmp2;
                
                int difference = abs(firstXCord - secondXCord) + abs(firstYCord - secondYCord);
                // Error checking
                if(difference != 1 && difference != 2){
                    ROS_ERROR("Astar global planner: Error in getMoveCost - difference not valid");
                    return 1.0;
                }
                if(difference == 1)
                    return 1.0;
                else
                    return 1.4;
            }
            double getHeuristic(int cell_index, int goal_index)
            {
                unsigned int tmp1, tmp2;
                tmp1 = cell_index%global_costmap.info.width;
                tmp2 = cell_index/global_costmap.info.width;
                int startX = tmp1, startY = tmp2;
                tmp1 = goal_index%global_costmap.info.width;
                tmp2 = goal_index/global_costmap.info.width;
                int goalX = tmp1, goalY = tmp2;
                
                return abs(goalY - startY) + abs(goalX - startX);
            }
            void find_closest_point(int index, nav_msgs::OccupancyGrid global_costmap, int &closest_idx)
            {
                int a[4]={1,(int)global_costmap.info.width,-1,-(int)global_costmap.info.width};
                std::unordered_map<int, int> visited, unvisited;
                std::queue<int> id;
                int current;
                if(global_costmap.data[index]==0)
                {
                    closest_idx=index;
                    return 0;
                }
                // if(occupancy_map.data[start_idx]<97)
                // {
                    id.push(index);
                    while (id.size()) 
                    {
                        current = id.front();
                        id.pop();
                        for(int i=0;i<4;i++)
                        {
                            int neighbor_id=current+a[i];
                            if((global_costmap.data[neighbor_id]==100)&&!(visited[neighbor_id]))
                            {
                                if(global_costmap.data[current]==0)
                                {
                                    closest_idx=current;
                                    id=queue<int>();
                                    break;
                                }
                                    id.push(neighbor_id);
                                    ++visited[neighbor_id];
                            }
                        }
                    }
                // }
            }
            bool path_planning(int start_index, int goal_index, nav_msgs::OccupancyGrid global_costmap, std::vector<int>& plan_result)
            {
                int map_size = global_costmap.data.size();
                vector<float> gCosts(map_size, INF);
                vector<int> cameFrom(map_size, -1);
                
                multiset<Node> priority_costs;
                
                gCosts[start_index] = 0;
                
                Node currentNode;
                currentNode.index = start_index;
                currentNode.cost = gCosts[start_index] + 0;
                priority_costs.insert(currentNode);
                vector<geometry_msgs::PoseStamped> plan;
                vector<geometry_msgs::PoseStamped> replan;
                
                plan.clear();
                replan.clear();
                std::cout<<"begin planning"<<std::endl;
                
                while(!priority_costs.empty())
                {
                    // Take the element from the top
                    currentNode = *priority_costs.begin();
                    //Delete the element from the top
                    priority_costs.erase(priority_costs.begin());
                    if (currentNode.index == goal_index){
                        break;
                    }
                    // Get neighbors
                    vector<int> neighborIndexes = get_neighbors(currentNode.index);
                    
                    for(int i = 0; i < neighborIndexes.size(); i++){
                        if(cameFrom[neighborIndexes[i]] == -1){
                        gCosts[neighborIndexes[i]] = gCosts[currentNode.index] + getMoveCost(currentNode.index, neighborIndexes[i]);
                        Node nextNode;
                        nextNode.index = neighborIndexes[i];
                        //nextNode.cost = gCosts[neighborIndexes[i]];    //Dijkstra Algorithm
                        nextNode.cost = gCosts[neighborIndexes[i]] + getHeuristic(neighborIndexes[i], goal_index);    //A* Algorithm
                        cameFrom[neighborIndexes[i]] = currentNode.index;
                        priority_costs.insert(nextNode);
                        }
                    }
                }
                
                if(cameFrom[goal_index] == -1){
                    cout << "Goal not reachable, failed making a global path." << endl;
                    return false;
                }
                
                if(start_index == goal_index)
                    return false;
                //Finding the best path
                vector<int> bestPath;
                currentNode.index = goal_index;
                while(currentNode.index != start_index){
                    bestPath.push_back(cameFrom[currentNode.index]);
                    currentNode.index = cameFrom[currentNode.index];
                }
                reverse(bestPath.begin(), bestPath.end());
                
                Path.clear();
                nav_msgs::Path Path_published;
                int path;
                geometry_msgs::PoseStamped this_pose_stamped;
                for(int i = 0; i < bestPath.size(); i=i+20){
                    path = bestPath[i];
                    Path.push_back(path);
                    this_pose_stamped.pose.position.x = path%global_costmap.info.width;
                    this_pose_stamped.pose.position.y = global_costmap.info.height-path/global_costmap.info.width-1;
                    // std::cout<<"u:"<<this_pose_stamped.pose.position.x<<" v:"<<this_pose_stamped.pose.position.y<<std::endl;
                    Path_published.poses.push_back(this_pose_stamped);
                    
                }
                global_path_pub_.publish(Path_published);

                cout << "/***********/" << "bestPath.size():" << bestPath.size() << "*****" <<"Path.size():" << Path.size() << endl;
               
                return true;
            
            }
            void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& goal_pose)
            {
                global_goal_pose=*goal_pose;
                // int a[3];//calculate the index of goal position;
                // a[0]=(global_goal_pose.pose.position.x)*100000;
                // a[1]=(global_goal_pose.pose.position.y)*100000;
                // a[2]=global_costmap.info.resolution*100000;
                // goal_index = (a[1]/a[2])*global_costmap.info.width+a[0]/a[2];
                goal_index = (global_costmap.info.height-global_goal_pose.pose.position.y-1)*global_costmap.info.width+global_goal_pose.pose.position.x;
                std::vector<int> path;
                path_planning(robot_index, goal_index, global_costmap, path);

            }
            void CostmapSubCallback_global(const nav_msgs::OccupancyGrid::ConstPtr& map)
            {
                ros::Time timestamp = map->header.stamp;
                global_costmap = *map;
                OGM.resize(global_costmap.data.size());
                for(int i=0;i<global_costmap.data.size();i++)
                {
                    if(global_costmap.data[i]==0)
                        OGM[i]=true;
                    else
                        OGM[i]=false;
                }

                
                // std::cout<<"global_costmap.size:"<<global_costmap.data.size()<<std::endl;
                // GetRobotpose("/PandarQT",global_local_pose, timestamp);
                // global_local_pose.pose.position.x=10.4849624634;//test map image_6.png
                // global_local_pose.pose.position.y=19.9478435516;
                global_local_pose.pose.position.x=58.9334449768;
                global_local_pose.pose.position.y=121.679786682;
                global_local_pose.pose.position.z=0;
                
                int mapHeight=global_costmap.info.height;
                int mapWidth=global_costmap.info.width;
                // int a[3];
                // a[0]=(global_local_pose.pose.position.x)*100000;
                // a[1]=(global_local_pose.pose.position.y)*100000;
                // a[2]=global_costmap.info.resolution*100000;
                // robot_index=(a[1]/a[2])*global_costmap.info.width+a[0]/a[2];
                robot_index = robot_index_y*global_costmap.info.width+robot_index_x;


                // std::cout<<"robot_index"<<robot_index<<std::endl;
                // std::cout<<"global_costmap.info.resolution"<<global_costmap.info.resolution<<std::endl;
                // std::cout<<"global_costmap.info.height"<<global_costmap.info.resolution<<std::endl;
                // std::cout<<"global_costmap.info.width"<<global_costmap.info.resolution<<std::endl;

                
                // temp_goal_index = goal_index;
                global_costmap.data[robot_index]=100;
                global_costmap.data[goal_index]=100;
                //wc:show the original path
                for(auto i:Path)
                {
                    global_costmap.data[i]=100;
                }
                
                //wc:show the smoothed path
                // for(auto i:Path_smooth)
                // {
                //     global_costmap.data[i]=-1;
                // }

                
                
                costmap_with_path_.publish(global_costmap);

            }


            
            void GetRobotpose(std::string iden_frame_id,geometry_msgs::PoseStamped& global_local_pose, ros::Time timestamp)
            {
            tf::StampedTransform transform;
            geometry_msgs::PoseStamped iden_pose;
            iden_pose.header.frame_id = iden_frame_id;
            iden_pose.header.stamp = ros::Time::now(); 
            iden_pose.pose.orientation.w = 1;
            tf_listener_.waitForTransform("/robot0/map",iden_frame_id, ros::Time(0), ros::Duration(2.0));
            tf_listener_.lookupTransform( "/robot0/map",iden_frame_id, ros::Time(0), transform);
            global_local_pose.pose.position.x=transform.getOrigin().x();
            global_local_pose.pose.position.y=transform.getOrigin().y();
            global_local_pose.pose.position.z=transform.getOrigin().z();
            }

            bool getCostmap;
            ros::NodeHandle NODE;
            ros::Subscriber costmap_sub_global_;
            ros::Subscriber goal_sub_global_;
            ros::Subscriber position_sub_global_;
            ros::Publisher global_path_pub_;
            ros::Publisher costmap_with_path_;
            ros::Publisher robot_position_pub_;
            nav_msgs::OccupancyGrid global_costmap;
            nav_msgs::OccupancyGrid local_costmap;
            // nav_msgs::Path Path_published;
            geometry_msgs::PoseStamped global_local_pose;
            geometry_msgs::PoseStamped global_goal_pose;
            geometry_msgs::PoseStamped closest_frontier_point;
            std::vector<bool> OGM;//check whether the cell is known
            vector<int> Path;
            vector<int> Path_smooth;
            int robot_index_x=0;
            int robot_index_y=0;
            std::mutex lock_costmap;
            std::thread tf_thread_;
            tf::TransformListener tf_listener_;
            int robot_index;
            int map_robot_index;
            int goal_index;
            int map_goal_index;
            int temp_goal_index;
            int seed;
            int unknown_digit=100;
            int known_digit=0;

            // protected:
    };
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_path_planning");
    ros::NodeHandle private_nh1("~");
    global_path_planning::global_path_planning_ ta(private_nh1);
    ros::MultiThreadedSpinner spinner;
    spinner.spin();
    return 0;
}