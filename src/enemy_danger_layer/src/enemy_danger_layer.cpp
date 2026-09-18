#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

namespace enemy_danger_layer
{

class EnemyDangerLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  EnemyDangerLayer() = default;

  // 1. Called once when Nav2 initializes costmaps
  void onInitialize() override
  {
    current_ = true;
    enabled_ = true;

    // Get global frame ('map') from parent layered costmap
    global_frame_ = layered_costmap_->getGlobalFrameID();

    // Declare configurable ROS parameters
    declareParameter("enemy_frame", rclcpp::ParameterValue("enemy_bot/base_link"));
    declareParameter("danger_radius", rclcpp::ParameterValue(2.0));

    auto node = node_.lock();
    if (node) {
      node->get_parameter(name_ + ".enemy_frame", enemy_frame_);
      node->get_parameter(name_ + ".danger_radius", danger_radius_);
    }
  }

  // 2. Pure Virtual Functions required by Nav2 Layer interface
  void reset() override
  {
    return;
  }

  bool isClearable() override
  {
    return false; // Prevents clearing services from wiping the dynamic zone
  }

  // 3. Tells Nav2 which region of the costmap needs recalculation
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double* min_x, double* min_y, double* max_x, double* max_y) override
  {
    geometry_msgs::msg::TransformStamped tf_enemy;
    try {
      // tf_ is provided automatically by base Layer class
      tf_enemy = tf_->lookupTransform(global_frame_, enemy_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      return; // Skip update if enemy TF is not broadcasted yet
    }

    double ex_x = tf_enemy.transform.translation.x;
    double ex_y = tf_enemy.transform.translation.y;

    // Expand dirty bounding box around enemy frame
    *min_x = std::min(*min_x, ex_x - danger_radius_);
    *min_y = std::min(*min_y, ex_y - danger_radius_);
    *max_x = std::max(*max_x, ex_x + danger_radius_);
    *max_y = std::max(*max_y, ex_y + danger_radius_);
  }

  // 4. Modifies costmap grid memory directly
  void updateCosts(
    nav2_costmap_2d::Costmap2D& master_grid,
    int min_i, int min_j, int max_i, int max_j) override
  {
    if (!enabled_) return;

    geometry_msgs::msg::TransformStamped tf_enemy;
    try {
      // Transform global coordinates into enemy local frame
      tf_enemy = tf_->lookupTransform(enemy_frame_, global_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      return;
    }

    // Loop through grid cells inside updated bounds
    for (int j = min_j; j < max_j; ++j) {
      for (int i = min_i; i < max_i; ++i) {
        double wx, wy;
        master_grid.mapToWorld(i, j, wx, wy);

        // Convert grid cell to enemy local coordinates
        geometry_msgs::msg::Point pt_world, pt_enemy;
        pt_world.x = wx; 
        pt_world.y = wy; 
        pt_world.z = 0.0;
        tf2::doTransform(pt_world, pt_enemy, tf_enemy);

        double dist = std::hypot(pt_enemy.x, pt_enemy.y);

        // Condition: Front 180° half-plane (pt_enemy.x > 0) within radius
        if (pt_enemy.x > 0.0 && dist <= danger_radius_) {
          master_grid.setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
        }
      }
    }
  }

private:
  std::string enemy_frame_{"enemy_bot/base_link"};
  double danger_radius_{2.0};
  std::string global_frame_{"map"};
};

} // namespace enemy_danger_layer

// Register class as pluginlib plugin
PLUGINLIB_EXPORT_CLASS(enemy_danger_layer::EnemyDangerLayer, nav2_costmap_2d::Layer)