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
#include "KDTree/KDTree.hpp"
#include "dynamic_voronoi/dynamicvoronoi.h"

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
                costmap_with_path_ = private_nh.advertise<nav_msgs::OccupancyGrid>("/costmap_with_path", 1);
                robot_position_pub_=private_nh.advertise<geometry_msgs::PoseStamped>("/robot_position", 1);
                global_path_pub_ = private_nh.advertise<nav_msgs::Path>("/global_path", 1);
            }


        private:
            void find_closest_voronoi_point(int& robot_id, int& closest_voronoi_id, std::vector<bool> is_voronoi)
            {
                
                int a[4]={1,(int)global_costmap.info.width,-1,-(int)global_costmap.info.width};
                std::unordered_map<int, int> visited, unvisited;
                std::queue<int> id;
                int current;
                std::cout<<"start finding closest voronoi point"<<std::endl;
                if(is_voronoi[robot_id])
                {
                    closest_voronoi_id = robot_id;
                    std::cout<<"find closest voronoi point:"<<closest_voronoi_id<<std::endl;
                    return;
                }
                int count=10000;
                if(!is_voronoi[robot_id]&&count>0)
                {
                    count--;
                    id.push(robot_id);
                    while (id.size()) 
                    {
                        current = id.front();
                        id.pop();
                        for(int i=0;i<4;i++)
                        {
                            int neighbor_id=current+a[i];
                            // std::cout<<"neighbor_id:"<<neighbor_id<<std::endl;
                            if(!(visited[neighbor_id])&&neighbor_id>0&&neighbor_id<is_voronoi.size())
                            {
                                if(is_voronoi[current])
                                {
                                    closest_voronoi_id=current;
                                    std::cout<<"find closest voronoi point:"<<closest_voronoi_id<<std::endl;
                                    // id=queue<int>();
                                    return;
                                }
                                    id.push(neighbor_id);
                                    ++visited[neighbor_id];
                            }
                        }
                    }
                }
            }
            void CostmapSubCallback_global(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
            {
                
                global_costmap_for_voronoi = *map_msg;
                voronoi_sizeX = global_costmap_for_voronoi.info.width;
                voronoi_sizeY = global_costmap_for_voronoi.info.height;
                bool **map1=NULL;
                map1 = new bool*[voronoi_sizeX];

                for (int x=0; x<voronoi_sizeX; x++) 
                {
                    (map1)[x] = new bool[voronoi_sizeY];
                }
                for (int x=0; x<voronoi_sizeX; x++) 
                {
                    for (int y=0; y<voronoi_sizeY; y++) {
                        if (((int)global_costmap_for_voronoi.data[x+y*voronoi_sizeX]<100) && ((int)global_costmap_for_voronoi.data[x+y*voronoi_sizeX]>=0))
                            map1[x][y]=false; //cell is free
                        else map1[x][y] = true; // cell is occupied
                        }
                }
                if(!voronoi_generated)
                {
                    voronoi.initializeMap(voronoi_sizeX, voronoi_sizeY, map1);
                    voronoi.update(); // update distance map and Voronoi diagram
                    bool doPrune = false, doPruneAlternative = false;
                    if (doPrune) voronoi.prune();  // prune the Voronoi
                    if (doPruneAlternative) voronoi.updateAlternativePrunedDiagram();  // prune the Voronoi
                    voronoi_generated = true;
                }

                for(int i = 0; i < voronoi_sizeX ; i++)
                {
                    for(int j = 0; j < voronoi_sizeY ; j++)
                    {
                        if(voronoi.isVoronoi(i,j)==1)
                        {
                            global_costmap_for_voronoi.data[i+j*voronoi_sizeX]=50;
                        }
                    }
                }
                for (int x=0; x<voronoi_sizeX; x++) 
                {
                    delete [] map1[x];
                }
                delete [] map1;

                global_costmap = *map_msg;
                OGM.resize(global_costmap_for_voronoi.data.size());
                for(int i=0;i<global_costmap_for_voronoi.data.size();i++)
                {
                    if(global_costmap_for_voronoi.data[i]==50)
                        OGM[i]=true;
                    else
                        OGM[i]=false;
                }

                std::vector<int> path;
                std::vector<std::vector<double> > path_separted;
                if(goal_index!=0)
                {
                    path_planning(robot_index, goal_index, global_costmap, path);
                    // for(auto i:path)
                    // {
                    //     std::cout<<"path:"<<i<<std::endl;
                    //     global_costmap_for_voronoi.data[i]=100;
                    // }
                }
                for(int i=10; i<100; i++)
                {
                    global_costmap_for_voronoi.data[i+robot_index]=100;
                }
                int closest_robot_in_voronoi_idx=0;
                int closest_goal_in_vorono_idx=0;
                std::cout<<"robot_index:"<<robot_index<<std::endl;
                find_closest_voronoi_point(robot_index, closest_robot_in_voronoi_idx, OGM);
                if(goal_index!=0)
                    find_closest_voronoi_point(goal_index, closest_goal_in_vorono_idx, OGM);
                for(int i=10; i<100; i++)
                {
                    global_costmap_for_voronoi.data[i+closest_robot_in_voronoi_idx]=100;
                }
                if(goal_index!=0)
                {
                    path_planning(closest_robot_in_voronoi_idx, closest_goal_in_vorono_idx, global_costmap_for_voronoi, path);
                    if(!path.empty())
                    {
                        for(int i=0;i<path.size();i+=5)
                        {
                            std::vector<double> path_temp;
                            path_temp.push_back((double)(path[i]%voronoi_sizeX));
                            path_temp.push_back((double)(path[i]/voronoi_sizeX));
                            path_separted.push_back(path_temp);
                        }

                        nav_msgs::Path Path_published;
                        geometry_msgs::PoseStamped this_pose_stamped;
                        std::vector<int> refined_path = Path_smooth(path_separted);
                        refined_path.push_back(goal_index);
                        for(int i = 0; i < refined_path.size(); i++){
                            int temp_path_id=path[i];
                            Path.push_back(temp_path_id);
                            this_pose_stamped.pose.position.x = temp_path_id%global_costmap.info.width;
                            this_pose_stamped.pose.position.y = global_costmap.info.height-temp_path_id/global_costmap.info.width-1;
                            Path_published.header=global_costmap.header;
                            // std::cout<<"u:"<<this_pose_stamped.pose.position.x<<" v:"<<this_pose_stamped.pose.position.y<<std::endl;
                            Path_published.poses.push_back(this_pose_stamped);
                            
                        }
                        global_path_pub_.publish(Path_published);
                        
                        for(auto i: refined_path)
                        {
                            global_costmap_for_voronoi.data[i]=100;
                        }
                    }
                    
                    }
                    costmap_with_path_.publish(global_costmap_for_voronoi);
            }

            std::vector<int> Path_smooth(std::vector<std::vector<double> >& pair_path)
            {
                int refinded_path_size=pair_path.size();
                std::vector<std::vector<double> > refined_path;
                refined_path=pair_path;
                double tolerance_=0.5;
                double change=10000;
                int count=1000;
                double d1, d2;
                while(change>tolerance_&&count>0)
                {
                    count--;
                    change=0;
                    for(int i=1; i<refined_path.size()-1; i++)
                    {
                        for(int j=0; j<refined_path[i].size(); j++)
                        {
                            d1=weight_data * (refined_path[i][j] - pair_path[i][j]);
                            d2=weight_smooth * (refined_path[i-1][j]-refined_path[i+1][j]-2*refined_path[i][j]);
                            change = abs(d1+d2);
                            std::cout<<"change"<<change<<std::endl;
                            refined_path[i][j] += d1+d2;
                        }
                    }
                }
                std::vector<int> result;
                int temp=0;
                for(int i=0; i<refined_path.size(); i++)
                {
                    for(int j=0; j<refined_path[i].size(); j++)
                    {
                        std::cout<<"refined_path:"<<refined_path[i][j];
                    }
                    temp=(int)(refined_path[i][0]+refined_path[i][1]*global_costmap.info.width);
                    result.push_back(temp);
                    std::cout<<std::endl;
                }
                return result;
            }
            point_t get_kdtree_index(point_t point, KDTree tree)
            {
                point_t point_temp;
                point_temp.push_back(point[0]);
                point_temp.push_back(point[1]);
                point_t closest_point = tree.nearest_point(point_temp);
                return closest_point;
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

            bool path_planning(int start_index, int goal_index, nav_msgs::OccupancyGrid global_costmap, std::vector<int>& plan_result)
            {
                // ROS_INFO("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, 
                // goal.pose.position.x,goal.pose.position.y);
                // ros::Time time_1 = ros::Time::now();
                // double wx = start.pose.position.x;
                // double wy = start.pose.position.y;
                // unsigned int start_x, start_y;
                // costmap_->worldToMap(wx, wy, start_x, start_y);
                // int start_index = costmap_->getIndex(start_x, start_y);

                
                // wx = goal.pose.position.x;
                // wy = goal.pose.position.y;

                // unsigned int goal_x, goal_y;
                // costmap_->worldToMap(wx, wy, goal_x, goal_y);
                // int goal_index = costmap_->getIndex(goal_x, goal_y);
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
                std::cout<<"before: "<<Path.size()<<std::endl;
                Path.clear();
                int path;
                for(int i = 0; i < bestPath.size(); i=i+3){
                    path = bestPath[i];
                    Path.push_back(path);
                }
                std::cout<<"after: "<<Path.size()<<std::endl;
                cout << "/***********/" << "bestPath.size():" << bestPath.size() << "*****" <<"Path.size():" << Path.size() << endl;
                plan_result=Path;
                return true;
            
            }
            void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& goal_pose)
            {
                global_goal_pose=*goal_pose;
                std::cout<<"goal_data:"<<global_costmap.data.at(123)<<std::endl;
                goal_index = (global_costmap.info.height-global_goal_pose.pose.position.y-1)*global_costmap.info.width+global_goal_pose.pose.position.x;
                if(global_goal_pose.pose.orientation.x!=-9999&&global_goal_pose.pose.orientation.x!=-9999)
                    robot_index =(global_costmap.info.height-global_goal_pose.pose.orientation.y-1)*global_costmap.info.width+global_goal_pose.pose.orientation.x;

            }

            bool getCostmap;
            ros::NodeHandle NODE;
            ros::Subscriber costmap_sub_constructing_voronoi_;
            ros::Subscriber costmap_sub_global_;
            ros::Subscriber goal_sub_global_;
            ros::Subscriber position_sub_global_;
            ros::Publisher costmap_with_path_;
            ros::Publisher robot_position_pub_;
            ros::Publisher global_path_pub_;
            nav_msgs::OccupancyGrid global_costmap;
            nav_msgs::OccupancyGrid global_costmap_for_voronoi;
            nav_msgs::OccupancyGrid local_costmap;
            geometry_msgs::PoseStamped global_local_pose;
            geometry_msgs::PoseStamped global_goal_pose;
            geometry_msgs::PoseStamped closest_frontier_point;
            std::vector<bool> OGM;//check whether the cell is known
            vector<int> Path;
            vector<int> Path_smoothed;
            int robot_index_x=0;
            int robot_index_y=0;
            DynamicVoronoi voronoi;
            bool voronoi_generated=false;//generate voronoi once
            int voronoi_sizeX, voronoi_sizeY;
            double weight_data=0.005;//the two argument below is for path smooth
            double weight_smooth=0.0000001;

            std::mutex lock_costmap;
            std::thread tf_thread_;
            tf::TransformListener tf_listener_;
            int robot_index=8073724;
            int goal_index=0;
            int temp_goal_index;
            int unknown_digit=100;
            int known_digit=0;
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