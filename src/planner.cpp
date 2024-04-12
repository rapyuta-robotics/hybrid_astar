#include "planner.h"

#include <pluginlib/class_list_macros.h>

#include <mbf_msgs/GetPathAction.h>

//register this planner as a MBF's CostmapPlanner plugin
PLUGINLIB_EXPORT_CLASS(HybridAStar::Planner, mbf_costmap_core::CostmapPlanner)

using namespace HybridAStar;
//###################################################
//                                        CONSTRUCTOR
//###################################################
Planner::Planner() {
  // _____
  // TODOS
  //    initializeLookups();
  // Lookup::collisionLookup(collisionLookup);
  // ___________________
  // COLLISION DETECTION
  //    CollisionDetection configurationSpace;
  // _________________
  // TOPICS TO PUBLISH
  pubStart = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);

  // ___________________
  // TOPICS TO SUBSCRIBE
  if (Constants::manual) {
    subMap = n.subscribe("/map", 1, &Planner::setMap, this);
  } else {
    subMap = n.subscribe("/map", 1, &Planner::setMap, this);
  }

  subGoal = n.subscribe("/move_base_simple/goal", 1, &Planner::setGoal, this);
  subStart = n.subscribe("/initialpose", 1, &Planner::setInitialPose, this);
};

void Planner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  costmap_ros_ = costmap_ros;
  initialized_ = true;
}

uint32_t Planner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                 double tolerance, std::vector<geometry_msgs::PoseStamped>& plan, double& cost,
                                 std::string& message) {
//    boost::mutex::scoped_lock lock(mutex_);
    if (!initialized_) {
        message = "This planner has not been initialized yet, but it is being used, please call initialize() before use";
        ROS_ERROR_STREAM(message);
        return mbf_msgs::GetPathResult::NOT_INITIALIZED;
    }
    //fillOccupancyGrid();
    setStart(goal);
    setGoal(start);  // TODO creates paths from goal to start!!!
    computePath();
    plan = path.getPath().poses;
    return mbf_msgs::GetPathResult::SUCCESS;
}

//###################################################
//                                       LOOKUPTABLES
//###################################################
void Planner::initializeLookups() {
  if (Constants::dubinsLookup) {
    Lookup::dubinsLookup(dubinsLookup);
  }

  Lookup::collisionLookup(collisionLookup);
}

//###################################################
//                                                MAP
//###################################################
void Planner::fillOccupancyGrid()
{
  // Get the costmap data
  costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

  // Populate the OccupancyGrid message fields
  occ_grid_.header.stamp = ros::Time::now();
  occ_grid_.header.frame_id = costmap_ros_->getGlobalFrameID();
  occ_grid_.info.resolution = costmap->getResolution();
  occ_grid_.info.width = costmap->getSizeInCellsX();
  occ_grid_.info.height = costmap->getSizeInCellsY();
  occ_grid_.info.origin.position.x = costmap->getOriginX();
  occ_grid_.info.origin.position.y = costmap->getOriginY();
  occ_grid_.info.origin.orientation.w = 1.0; // Assuming no rotation

  // Convert costmap data to occupancy occ_grid_ data
  occ_grid_.data.resize(occ_grid_.info.width * occ_grid_.info.height);
  for (unsigned int y = 0; y < occ_grid_.info.height; ++y) {
    for (unsigned int x = 0; x < occ_grid_.info.width; ++x) {
      unsigned int costmap_index = costmap->getIndex(x, y);
      int cost = costmap->getCost(x, y);
      // Map the costmap values to occupancy occ_grid_ values (0-100)
      if (cost == costmap_2d::LETHAL_OBSTACLE)
        occ_grid_.data[y * occ_grid_.info.width + x] = 100;
//      else if (cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
//               cost == costmap_2d::NO_INFORMATION)
//        occ_grid_.data[y * occ_grid_.info.width + x] = -1;
      else
        occ_grid_.data[y * occ_grid_.info.width + x] = 0;
    }
  }

  setMap(occ_grid_);
}

void Planner::setMap(const nav_msgs::OccupancyGrid& map) {
  if (Constants::coutDEBUG) {
    std::cout << "I am seeing the map..." << std::endl;
  }

  occ_grid_ = map;
  //update the configuration space with the current map
  configurationSpace.updateGrid(&occ_grid_);
  //create array for Voronoi diagram
  ros::Time t0 = ros::Time::now();
  int height = occ_grid_.info.height;
  int width = occ_grid_.info.width;
  bool** binMap;
  binMap = new bool*[width];

  for (int x = 0; x < width; x++) { binMap[x] = new bool[height]; }

  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      binMap[x][y] = occ_grid_.data[y * width + x] ? true : false;
    }
  }

  voronoiDiagram.initializeMap(width, height, binMap);
  voronoiDiagram.update();
  voronoiDiagram.visualize();
  ros::Time t1 = ros::Time::now();
  ros::Duration d(t1 - t0);
  std::cout << "created Voronoi Diagram in ms: " << d * 1000 << std::endl;
  return;
  // plan if the switch is not set to manual and a transform is available
  if (!Constants::manual && listener.canTransform("/map", ros::Time(0), "/base_link", ros::Time(0), "/map", nullptr)) {

    listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

    // assign the values to start from base_link
    start_.pose.position.x = transform.getOrigin().x();
    start_.pose.position.y = transform.getOrigin().y();
    tf::quaternionTFToMsg(transform.getRotation(), start_.pose.orientation);

    if (occ_grid_.info.height >= start_.pose.position.y && start_.pose.position.y >= 0 &&
        occ_grid_.info.width >= start_.pose.position.x && start_.pose.position.x >= 0) {
      // set the start as valid and plan
      validStart = true;
    } else  {
      validStart = false;
    }

    computePath();
  }
}

//###################################################
//                                   INITIALIZE START
//###################################################
void Planner::setInitialPose(const geometry_msgs::PoseWithCovarianceStamped& initial) {
  geometry_msgs::PoseStamped start;
  start.pose = initial.pose.pose;
  start.header = initial.header;
  setStart(start);
}

void Planner::setStart(const geometry_msgs::PoseStamped& start) {
  float x = start.pose.position.x / Constants::cellSize;
  float y = start.pose.position.y / Constants::cellSize;
  float t = tf::getYaw(start.pose.orientation);
  // publish the start without covariance for rviz

  std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (occ_grid_.info.height >= y && y >= 0 && occ_grid_.info.width >= x && x >= 0) {
    validStart = true;
    start_ = start;

    if (Constants::manual) { computePath();}

    // publish start for RViz
    pubStart.publish(start);
  } else {
    std::cout << "invalid start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

//###################################################
//                                    INITIALIZE GOAL
//###################################################
void Planner::setGoal(const geometry_msgs::PoseStamped& goal) {
  // retrieving goal position
  float x = goal.pose.position.x / Constants::cellSize;
  float y = goal.pose.position.y / Constants::cellSize;
  float t = tf::getYaw(goal.pose.orientation);

  std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (occ_grid_.info.height >= y && y >= 0 && occ_grid_.info.width >= x && x >= 0) {
    validGoal = true;
    goal_ = goal;

    if (Constants::manual) { computePath();}

  } else {
    std::cout << "invalid goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

//###################################################
//                                      PLAN THE PATH
//###################################################
void Planner::computePath() {
  // if a start as well as goal are defined go ahead and plan
  if (validStart && validGoal) {

    // ___________________________
    // LISTS ALLOWCATED ROW MAJOR ORDER
    int width = occ_grid_.info.width;
    int height = occ_grid_.info.height;
    int depth = Constants::headings;
    int length = width * height * depth;
    // define list pointers and initialize lists
    Node3D* nodes3D = new Node3D[length]();
    Node2D* nodes2D = new Node2D[width * height]();

    // ________________________
    // retrieving goal position
    float x = goal_.pose.position.x / Constants::cellSize;
    float y = goal_.pose.position.y / Constants::cellSize;
    float t = tf::getYaw(goal_.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    const Node3D nGoal(x, y, t, 0, 0, nullptr);
    // __________
    // DEBUG GOAL
    //    const Node3D nGoal(155.349, 36.1969, 0.7615936, 0, 0, nullptr);


    // _________________________
    // retrieving start position
    x = start_.pose.position.x / Constants::cellSize;
    y = start_.pose.position.y / Constants::cellSize;
    t = tf::getYaw(start_.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    Node3D nStart(x, y, t, 0, 0, nullptr);
    // ___________
    // DEBUG START
    //    Node3D nStart(108.291, 30.1081, 0, 0, 0, nullptr);


    // ___________________________
    // START AND TIME THE PLANNING
    ros::Time t0 = ros::Time::now();

    // CLEAR THE VISUALIZATION
    visualization.clear();
    // CLEAR THE PATH
    path.clear();
    smoothedPath.clear();
    // FIND THE PATH
    Node3D* nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height, configurationSpace, dubinsLookup, visualization);
    // TRACE THE PATH
    smoother.tracePath(nSolution);
    // CREATE THE UPDATED PATH
    path.updatePath(smoother.getPath());
    // SMOOTH THE PATH
    smoother.smoothPath(voronoiDiagram);
    // CREATE THE UPDATED PATH
    smoothedPath.updatePath(smoother.getPath());
    ros::Time t1 = ros::Time::now();
    ros::Duration d(t1 - t0);
    std::cout << "TIME in ms: " << d * 1000 << std::endl;

    // _________________________________
    // PUBLISH THE RESULTS OF THE SEARCH
    ROS_ERROR_STREAM(path.getPath().poses.size());
    path.publishPath();
    path.publishPathNodes();
    path.publishPathVehicles();
    smoothedPath.publishPath();
    smoothedPath.publishPathNodes();
    smoothedPath.publishPathVehicles();
    visualization.publishNode3DCosts(nodes3D, width, height, depth);
    visualization.publishNode2DCosts(nodes2D, width, height);



    delete [] nodes3D;
    delete [] nodes2D;

  } else {
    std::cout << "missing goal or start" << std::endl;
  }
}
