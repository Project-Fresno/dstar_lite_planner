#include <chrono>
#include <functional>
#include <memory>
#include <vector>

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
        pose_subscription=this->create_subscription<nav_msgs::msg::Odometry>("odom",10,std::bind(&PathPlanner::initial_pose,this,_1));
        timer_ = this->create_wall_timer(50ms, std::bind(&PathPlanner::timer_callback, this));
    }

  private:

    Dstar dstar;

    nav_msgs::msg::Path path;
    list<state> mypath;
    std::vector<geometry_msgs::msg::PoseStamped> pose_array;
    geometry_msgs::msg::PoseStamped path_points;
    
    int start_row, start_col;
    int map_width, map_height;
    float resolution;
    std::vector<int8_t> map_data;

    int threshold=80;
    bool once=false;
    float inflation_m=0.1;
    int inflation;
    bool flag_0=false;
    bool flag_1=false;

    float start_x, start_y;

    float goal_x, goal_y;
    

    void timer_callback()
    {   
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

        this->inflation = this->inflation_m / this->resolution;
        RCLCPP_INFO(this->get_logger(), "Inflation: %i", this->inflation);

        if(this->once==false){
            this->dstar.init(0,0,0,0);
            this->once=true;}

        if(this->flag_0==true){
            if(this->flag_1==false){
                dstar.updateStart(this->start_x / this->resolution, this->start_y / this->resolution);
                this->flag_1=true;
            }
        }

        int count=0;
               
        for(int i=0; i<this->map_height; i++){
            for(int j=0; j<this->map_width; j++){

                if(this->map_data[count]>this->threshold){
                    for(int k=0; k<this->inflation; k++){
                        for(int l=0; l<this->inflation; l++){
                            this->dstar.updateCell(j+l+this->start_col,  i+k+this->start_row, -1);
                            this->dstar.updateCell(j-l+this->start_col,  i-k+this->start_row, -1);
                    }}}

                count++;
            }
        }
   

        this->dstar.replan();
        RCLCPP_INFO(this->get_logger(), "Planning done"); 

        this->pose_array.clear();

        this->mypath = this->dstar.getPath();

        for (state path_state : this->mypath){
            this->path_points.pose.position.x=path_state.x * this->resolution;
            this->path_points.pose.position.y=path_state.y * this->resolution;
            this->pose_array.push_back(this->path_points);
        }
        
        RCLCPP_INFO(this->get_logger(), "Path updated");   
    
    }


    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr goal){

        RCLCPP_INFO(this->get_logger(), "Goal received");

        this->goal_x=goal->pose.position.x;
        this->goal_y=goal->pose.position.y;
        this->dstar.updateGoal(this->goal_x / this->resolution, this->goal_y / this->resolution); 

        this->dstar.replan();
        RCLCPP_INFO(this->get_logger(), "Planning done"); 

        this->pose_array.clear();

        this->mypath = this->dstar.getPath();

        for (state path_state : this->mypath){
            this->path_points.pose.position.x=path_state.x * this->resolution;
            this->path_points.pose.position.y=path_state.y * this->resolution;
            this->pose_array.push_back(this->path_points);
        }
        
        RCLCPP_INFO(this->get_logger(), "Path updated");       

    }

    void initial_pose(const nav_msgs::msg::Odometry::SharedPtr pose){
        this->start_x=pose->pose.pose.position.x;
        this->start_y=pose->pose.pose.position.y;
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