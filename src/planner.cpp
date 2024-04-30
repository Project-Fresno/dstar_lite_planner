#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "Dstar.h"

using namespace std::chrono_literals;
using std::placeholders::_1;


class PathPlanner : public rclcpp::Node
{
  public:
    PathPlanner() : Node("path_planner")
    {
        path_publisher = this->create_publisher<nav_msgs::msg::Path>("path", 10);
        map_subscription = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 10, std::bind(&PathPlanner::map_callback, this, _1));
        goal_subscription=this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose",10,std::bind(&PathPlanner::goal_callback,this,_1));
        pose_subscription=this->create_subscription<nav_msgs::msg::Odometry>("odom",10,std::bind(&PathPlanner::pose_callback,this,_1));
        timer_ = this->create_wall_timer(30ms, std::bind(&PathPlanner::timer_callback, this));
    }

  private:

    Dstar dstar;

    nav_msgs::msg::Path path;
    list<state> mypath;
    std::vector<geometry_msgs::msg::PoseStamped> pose_array;
    geometry_msgs::msg::PoseStamped path_points;
    
    int start_row, start_col;
    int map_width, map_height;
    float resolution=0.1;
    std::vector<int8_t> map_data;

    float yaw;

    int threshold=80;
    bool once=false;

    bool flag_0=false;
    bool flag_1=false;
    int pose_update=10;

    float start_x, start_y;

    float goal_x, goal_y;
    

    void timer_callback()
    {   
        if(this->once==false){
            this->dstar.init(0,0,0,0);
            this->once=true;}

        if(this->flag_0==true){
            if(this->flag_1==true){
                this->dstar.updateStart(this->start_x / this->resolution, this->start_y / this->resolution);
                this->dstar.replan();

                this->pose_array.clear();

                this->mypath = this->dstar.getPath();

                for (state path_state : this->mypath){
                    this->path_points.pose.position.x=path_state.x * this->resolution;
                    this->path_points.pose.position.y=path_state.y * this->resolution;
                    this->pose_array.push_back(this->path_points);
                }
                
                RCLCPP_INFO(this->get_logger(), "Path updated");  
                }
        }

        this->path.header.frame_id="odom";
        this->path.poses = this->pose_array;
        path_publisher->publish(this->path);
    }

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
    {   
        RCLCPP_INFO(this->get_logger(), "Map generated"); 
        
        this->resolution=map->info.resolution;

        this->start_col=(map->info.origin.position.x / this->resolution);
        this->start_row=(map->info.origin.position.y / this->resolution);
            
        this->map_width = map->info.width;
        this->map_height = map->info.height;
        this->map_data=map->data;

        int count=0;

        this->flag_1=false;
               
        for(int i=0; i<this->map_height; i++){
            for(int j=0; j<this->map_width; j++){

                if(this->map_data[count]>this->threshold){

                            this->dstar.updateCell(j+this->start_col,  i+this->start_row, -1);
                    }

                count++;
            }
        }
   
        RCLCPP_INFO(this->get_logger(), "Map updated"); 
        this->flag_1=true; 
    
    }


    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr goal){

        RCLCPP_INFO(this->get_logger(), "Goal received");

        this->goal_x=goal->pose.position.x;
        this->goal_y=goal->pose.position.y;
        this->dstar.updateGoal(this->goal_x / this->resolution, this->goal_y / this->resolution);  

    }

    void pose_callback(const nav_msgs::msg::Odometry::SharedPtr pose){
        
        this->start_x=pose->pose.pose.position.x;
        this->start_y=pose->pose.pose.position.y;
        // double siny_cosp = 2 * (pose.pose.orientation.w * pose.pose.orientation.z + pose.pose.orientation.x * pose.pose.orientation.y);
        // double cosy_cosp = 1 - 2 * (pose.pose.orientation.y* pose.pose.orientation.y+ pose.pose.orientation.z * pose.pose.orientation.z);
        // this->yaw = std::atan2(siny_cosp, cosy_cosp);
        this->flag_0=true;
        
    }


    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscription;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
    
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanner>());
    rclcpp::shutdown();
    return 0;
}