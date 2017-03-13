#include <pluginlib/class_list_macros.h>
#include "arc_theta_star_global_planner.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace global_planner {

GlobalPlanner::GlobalPlanner ()
{

}

GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  initialized_ = false;
  initialize(name, costmap_ros);
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{ 
  if(!initialized_)
  {
    costmap_2d::Costmap2D* costmap_2d;
    unsigned int size_x;
    unsigned int size_y;
    sdm_project::Array1D col;

    // Initialize srv call
    ros::NodeHandle n;
    client_path_planner_ = n.serviceClient<sdm_project::arc_theta_star_get_plan>("arc_theta_star_get_plan");

    // Get the costmap_ from costmap_ros
    costmap_2d = costmap_ros->getCostmap();      
    
    // Get the size of the costmap in cells
    size_x = costmap_2d->getSizeInCellsX();
    size_y = costmap_2d->getSizeInCellsY();

    // Convert the costmap to a 2D array used by the path planner
    costmap_array_ = sdm_project::Array2D();
    for(int x_i=0;x_i<size_x;x_i++)
    {
      col = sdm_project::Array1D();
      for(int y_i=0;y_i<size_y;y_i++) 
      {
        col.dim2.push_back(costmap_2d->getCost(x_i, y_i));
      }      
      costmap_array_.dim1.push_back(col);
    }
  }
}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
                            const geometry_msgs::PoseStamped& goal,  
                            std::vector<geometry_msgs::PoseStamped>& plan )
{
  // Create service call
  sdm_project::arc_theta_star_get_plan srv_path_planner;
  
  // Update information for the service call
  srv_path_planner.request.map = costmap_array_;
  srv_path_planner.request.start = start.pose;
  srv_path_planner.request.goal = goal.pose;

  // Try to call the path planner service
  if (client_path_planner_.call(srv_path_planner))
  {
    ROS_INFO("Path Length (in cells):     %d", (int)srv_path_planner.response.path.size());
  }
  else
  {
    ROS_ERROR("Failed to call path planner service!");
    return 1;
  }
}



};