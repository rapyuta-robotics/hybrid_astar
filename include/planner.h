#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mbf_costmap_core/costmap_planner.h>
#include <nav_msgs/GetPlan.h>

#include "constants.h"
#include "helper.h"
#include "collisiondetection.h"
#include "dynamicvoronoi.h"
#include "algorithm.h"
#include "node3d.h"
#include "path.h"
#include "smoother.h"
#include "visualize.h"
#include "lookup.h"

namespace HybridAStar {
/*!
   \brief A class that creates the interface for the hybrid A* algorithm.

    It inherits from `mbf_costmap_core::CostmapPlanner` so that it can be used with Move Base Flex
*/
class Planner : public mbf_costmap_core::CostmapPlanner {
 public:
  /// The default constructor
  Planner();

    /**
     * @brief  Initialization function for the PlannerCore object
     * @param  name The name of this planner
     * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
     */
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /**
     * @brief Given a goal pose in the world, compute a plan
     * @param start The start pose
     * @param goal The goal pose
     * @param tolerance If the goal is obstructed, how many meters the planner can relax the constraint
     *        in x and y before failing
     * @param plan The plan... filled by the planner
     * @param cost The cost for the the plan
     * @param message Optional more detailed outcome as a string
     * @return Result code as described on GetPath action result:
     *         SUCCESS         = 0
     *         1..9 are reserved as plugin specific non-error results
     *         FAILURE         = 50  # Unspecified failure, only used for old, non-mfb_core based plugins
     *         CANCELED        = 51
     *         INVALID_START   = 52
     *         INVALID_GOAL    = 53
     *         NO_PATH_FOUND   = 54
     *         PAT_EXCEEDED    = 55
     *         EMPTY_PATH      = 56
     *         TF_ERROR        = 57
     *         NOT_INITIALIZED = 58
     *         INVALID_PLUGIN  = 59
     *         INTERNAL_ERROR  = 60
     *         71..99 are reserved as plugin specific errors
     */
    uint32_t makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance,
                      std::vector<geometry_msgs::PoseStamped>& plan, double& cost, std::string& message);

    bool cancel()
    {
      // not implemented
      return false;
    }

  /*!
     \brief Initializes the collision as well as heuristic lookup table
     \todo probably removed
  */
  void initializeLookups();

  /*!
     \brief Sets the map e.g. through a callback from a subscriber listening to map updates.
     \param map the map or occupancy occ_grid_
  */
  void setMap(const nav_msgs::OccupancyGrid& map);

  void fillOccupancyGrid();

  /*!
     \brief setInitialPose
     \param initial the start pose
  */
  void setInitialPose(const geometry_msgs::PoseWithCovarianceStamped& initial);
  void setStart(const geometry_msgs::PoseStamped& start);

  /*!
     \brief setGoal
     \param goal the goal pose
  */
  void setGoal(const geometry_msgs::PoseStamped& goal);

  /*!
     \brief The central function entry point making the necessary preparations to start the planning.
  */
  void computePath();

 private:
  /// The node handle
  ros::NodeHandle n;
  /// A publisher publishing the start position for RViz
  ros::Publisher pubStart;
  /// A subscriber for receiving map updates
  ros::Subscriber subMap;
  /// A subscriber for receiving goal updates
  ros::Subscriber subGoal;
  /// A subscriber for receiving start updates
  ros::Subscriber subStart;
  /// A listener that awaits transforms
  tf::TransformListener listener;
  /// A transform for moving start positions
  tf::StampedTransform transform;
  /// The path produced by the hybrid A* algorithm
  Path path;
  /// The smoother used for optimizing the path
  Smoother smoother;
  /// The path smoothed and ready for the controller
  Path smoothedPath = Path(true);
  /// The visualization used for search visualization
  Visualize visualization;
  /// The collission detection for testing specific configurations
  CollisionDetection configurationSpace;
  /// The voronoi diagram
  DynamicVoronoi voronoiDiagram;
  /// A pointer to the occ_grid_ the planner runs on
  nav_msgs::OccupancyGrid occ_grid_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  /// The start pose set through RViz
  geometry_msgs::PoseStamped start_;
  /// The goal pose set through RViz
  geometry_msgs::PoseStamped goal_;
  /// Flags for allowing the planner to plan
  bool initialized_ = false;
  bool validStart = false;
  bool validGoal = false;
  /// A lookup table for configurations of the vehicle and their spatial occupancy enumeration
  Constants::config collisionLookup[Constants::headings * Constants::positions];
  /// A lookup of analytical solutions (Dubin's paths)
  float* dubinsLookup = new float [Constants::headings * Constants::headings * Constants::dubinsWidth * Constants::dubinsWidth];
};
}
#endif // PLANNER_H
