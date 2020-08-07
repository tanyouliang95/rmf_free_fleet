#include <rclcpp/rclcpp.hpp>
#include <rmf_fleet_adapter/agv/RobotCommandHandle.hpp>
#include <rmf_fleet_adapter/agv/FleetUpdateHandle.hpp>
#include <rmf_fleet_adapter/agv/TrafficLight.hpp>
#include <rmf_fleet_adapter/agv/Waypoint.hpp>
#include <rmf_fleet_adapter/agv/Adapter.hpp>

#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/Profile.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>

#include <std_msgs/msg/string.hpp>

namespace rmf_free_fleet {

class traffic_light_control
  : public rmf_fleet_adapter::agv::TrafficLight::CommandHandle
{
public: 
  // constructor
  traffic_light_control()
  {};

  // Fleet Updater
  rmf_fleet_adapter::agv::TrafficLight::UpdateHandlePtr _path_updater;

protected:

  void receive_path_timing( std::size_t version,
        const std::vector<rclcpp::Time>& departure_timing,
        ProgressCallback progress_updater) override
  {
    // implementation
    std::cout<< " Received new path timing!" << std::endl;
  }

  void deadlock() override
  {
    // Damn!!! Sad :(
    std::cout<< " Damn deadlock!" << std::endl;
  }
};


//==============================================================================

// Example for full control is located in `rmf_core::fleetadapter::full_control`
class full_control
  : public rmf_fleet_adapter::agv::RobotCommandHandle
{

};

class read_only
{

};

//==============================================================================
std::vector<rmf_fleet_adapter::agv::Waypoint> make_path(
    const rmf_traffic::agv::Graph& graph,
    const std::vector<std::size_t>& wp_indices,
    const double orientation)
{
  std::vector<rmf_fleet_adapter::agv::Waypoint> path;
  for (auto index : wp_indices)
  {
    const auto& wp = graph.get_waypoint(index);
    Eigen::Vector3d p;
    p[2] = orientation;
    p.block<2,1>(0,0) = wp.get_location();
    path.emplace_back(wp.get_map_name(), p);
  }

  return path;
}

//==============================================================================
rmf_traffic::agv::Graph make_test_graph()
{
  const std::string test_map_name = "test_map";
  rmf_traffic::agv::Graph graph;
  graph.add_waypoint(test_map_name, {0.0, -10.0}); // 0
  graph.add_waypoint(test_map_name, {0.0, -5.0});  // 1
  graph.add_waypoint(test_map_name, {5.0, -5.0}).set_holding_point(true);  // 2
  graph.add_waypoint(test_map_name, {-10.0, 0.0}); // 3
  graph.add_waypoint(test_map_name, {-5.0, 0.0}); // 4
  graph.add_waypoint(test_map_name, {0.0, 0.0}); // 5
  graph.add_waypoint(test_map_name, {5.0, 0.0}); // 6
  graph.add_waypoint(test_map_name, {10.0, 0.0}); // 7
  graph.add_waypoint(test_map_name, {0.0, 5.0}); // 8
  graph.add_waypoint(test_map_name, {5.0, 5.0}).set_holding_point(true); // 9
  graph.add_waypoint(test_map_name, {0.0, 10.0}); // 10

  /*
   *                   10
   *                   |
   *                   |
   *                   8------9
   *                   |      |
   *                   |      |
   *     3------4------5------6------7
   *                   |      |
   *                   |      |
   *                   1------2
   *                   |
   *                   |
   *                   0
   **/

  auto add_bidir_lane = [&](const std::size_t w0, const std::size_t w1)
    {
      graph.add_lane(w0, w1);
      graph.add_lane(w1, w0);
    };

  add_bidir_lane(0, 1);  // 0   1
  add_bidir_lane(1, 2);  // 2   3
  add_bidir_lane(1, 5);  // 4   5
  add_bidir_lane(2, 6);  // 6   7
  add_bidir_lane(3, 4);  // 8   9
  add_bidir_lane(4, 5);  // 10 11
  add_bidir_lane(5, 6);  // 12 13
  add_bidir_lane(6, 7);  // 14 15
  add_bidir_lane(5, 8);  // 16 17
  add_bidir_lane(6, 9);  // 18 19
  add_bidir_lane(8, 9);  // 20 21
  add_bidir_lane(8, 10); // 22 23

  return graph;
}

//==============================================================================

// write a simple traffic light code
class free_fleet_server
{
public:
  free_fleet_server(const rmf_fleet_adapter::agv::AdapterPtr& adapter)
  {
    // something
    this->_node = adapter->node();
    this->get_parameters();
    
    // params
    _command  = std::make_shared<traffic_light_control>();
    rmf_traffic::Profile profile{
      rmf_traffic::geometry::make_final_convex<
        rmf_traffic::geometry::Circle>(1.0)
    };
    const rmf_traffic::agv::VehicleTraits traits{
      {0.7, 0.3}, {1.0, 0.45}, profile
    };

    // Traffic Light Control
    {
      std::cout<< " Add traffic light robot fleet!" << std::endl;

      // add a traffic light robot
      adapter->add_traffic_light(
        _command, "dummy_fleet_name", "dummy_bot_name", traits, profile,
        std::bind(this->updater_creator, _command, std::placeholders::_1)
      );
    }

    // create test subscriber to trigger task
    subscription_ = _node->create_subscription<std_msgs::msg::String>(
      "task_request", 10, std::bind(&free_fleet_server::mock_task_cb, this, std::placeholders::_1));    
  }

private:

  std::shared_ptr<rclcpp::Node> _node;
  std::shared_ptr<traffic_light_control> _command;

  // get node param 
  void get_parameters()
  {
    const std::string fleet_name = _node->declare_parameter(
          "fleet_name", std::string());
  }

  static void updater_creator(
    std::shared_ptr<traffic_light_control> cmd, 
    rmf_fleet_adapter::agv::TrafficLight::UpdateHandlePtr updater )
  {
    std::cout<<" Updater Created" << std::endl;
    cmd->_path_updater= std::move(updater);
  }

  void mock_task_cb(const std_msgs::msg::String::SharedPtr msg) const
  {
    std::cout << "I heard: " << msg->data.c_str() << std::endl;

    std::vector<rmf_fleet_adapter::agv::Waypoint> robot_path;
    const auto graph = make_test_graph();
    robot_path = make_path(graph, {0, 1, 5, 8, 10}, M_PI/2.0);

    // try update new path
    _command->_path_updater->update_path(robot_path);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

}

//==============================================================================

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // Create Fleet adapter node
  const auto adapter = rmf_fleet_adapter::agv::Adapter::make("free_fleet_server");
  if (!adapter)
    return 1;

  std::cout<< " Starting free fleet server!" << std::endl;
  rmf_free_fleet::free_fleet_server ff_server(adapter);
  std::cout<< " Done init, running free fleet server!" << std::endl;
  adapter->start().wait();

  rclcpp::shutdown();
}
