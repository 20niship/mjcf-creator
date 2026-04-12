#include "mjcf/mjcf.hpp"
#include <iostream>

/**
 * @brief Example demonstrating the name utility functions
 * 
 * This example shows how to use the new utility functions to retrieve
 * all registered names in a MuJoCo model to avoid naming conflicts.
 */
int main() {
  std::cout << "=== Name Utilities Example ===" << std::endl << std::endl;

  // Create a MuJoCo model
  auto mujoco = std::make_shared<mjcf::Mujoco>("robot_model");
  auto worldbody = mujoco->worldbody_;
  auto asset = mujoco->asset_;
  auto actuator = mujoco->actuator_;

  // Add some bodies
  auto base_body = std::make_shared<mjcf::Body>();
  base_body->name = "base";
  worldbody->add_child(base_body);

  auto link1 = std::make_shared<mjcf::Body>();
  link1->name = "link1";
  base_body->add_child(link1);

  // Add geoms
  auto floor = mjcf::Geom::Plane("floor", {10.0, 10.0, 0.1});
  worldbody->add_child(floor);

  auto base_geom = mjcf::Geom::Box("base_geom", {0.1, 0.1, 0.2});
  base_body->add_child(base_geom);

  // Add joints
  auto joint1 = mjcf::Joint::Hinge("joint1", {0.0, 0.0, 1.0});
  link1->add_child(joint1);

  // Add assets
  auto texture = std::make_shared<mjcf::Texture>();
  texture->name = "grid_texture";
  texture->builtin = mjcf::TextureBuiltin::Checker;
  texture->type = mjcf::TextureType::TwoD;
  asset->add_child(texture);

  auto material = std::make_shared<mjcf::Material>();
  material->name = "grid_material";
  material->texture = "grid_texture";
  asset->add_child(material);

  // Add actuators
  auto motor = std::make_shared<mjcf::Motor>();
  motor->name = "motor1";
  motor->joint = "joint1";
  actuator->add_child(motor);

  std::cout << "Created model with various elements..." << std::endl << std::endl;

  // Get names by category
  std::cout << "=== Names by Category ===" << std::endl;
  
  auto body_names = mujoco->get_body_names();
  std::cout << "Bodies (" << body_names.size() << "): ";
  for(const auto& name : body_names) {
    std::cout << name << " ";
  }
  std::cout << std::endl;

  auto geom_names = mujoco->get_geom_names();
  std::cout << "Geoms (" << geom_names.size() << "): ";
  for(const auto& name : geom_names) {
    std::cout << name << " ";
  }
  std::cout << std::endl;

  auto joint_names = mujoco->get_joint_names();
  std::cout << "Joints (" << joint_names.size() << "): ";
  for(const auto& name : joint_names) {
    std::cout << name << " ";
  }
  std::cout << std::endl;

  auto asset_names = mujoco->get_asset_names();
  std::cout << "Assets (" << asset_names.size() << "): ";
  for(const auto& name : asset_names) {
    std::cout << name << " ";
  }
  std::cout << std::endl;

  auto actuator_names = mujoco->get_actuator_names();
  std::cout << "Actuators (" << actuator_names.size() << "): ";
  for(const auto& name : actuator_names) {
    std::cout << name << " ";
  }
  std::cout << std::endl;

  // Get all names at once
  std::cout << std::endl << "=== All Names ===" << std::endl;
  auto all_names = mujoco->get_all_names();
  std::cout << "Total unique names: " << all_names.size() << std::endl;
  std::cout << "Names: ";
  for(const auto& name : all_names) {
    std::cout << name << " ";
  }
  std::cout << std::endl;

  // Demonstrate name conflict checking
  std::cout << std::endl << "=== Name Conflict Checking ===" << std::endl;
  std::string proposed_name = "link1";
  if(all_names.find(proposed_name) != all_names.end()) {
    std::cout << "Warning: Name '" << proposed_name << "' is already in use!" << std::endl;
    std::cout << "Suggested alternatives: " << proposed_name << "_2, " << proposed_name << "_new" << std::endl;
  } else {
    std::cout << "Name '" << proposed_name << "' is available." << std::endl;
  }

  proposed_name = "link5";
  if(all_names.find(proposed_name) != all_names.end()) {
    std::cout << "Warning: Name '" << proposed_name << "' is already in use!" << std::endl;
  } else {
    std::cout << "Name '" << proposed_name << "' is available." << std::endl;
  }

  return 0;
}
