/**
 * @file multi_mjcf_example.cpp
 * @brief Example demonstrating the add_mjcf API for combining multiple MJCF files
 *
 * This example shows how to use the add_mjcf() method to combine multiple
 * MJCF files into a single MJCF model, allowing for complex scenes with
 * multiple robots or objects.
 */

#include "mjcf/mjcf.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>

int main() {
  std::cout << "=== Multi-MJCF Composition Example ===" << std::endl;

  // Create base scene
  auto scene = std::make_shared<mjcf::Mujoco>("multi_model_scene");
  scene->option_->timestep = 0.002;
  scene->option_->gravity  = {0.0, 0.0, -9.81};

  std::cout << "Created base MJCF scene" << std::endl;

  // Create first MJCF file: a simple robot arm
  std::string robot1_mjcf = R"(<?xml version="1.0"?>
<mujoco model="robot_arm">
  <asset>
    <texture name="arm_texture" type="2d" builtin="checker" rgb1="0.8 0.2 0.2" rgb2="0.9 0.3 0.3" width="256" height="256"/>
    <material name="arm_material" texture="arm_texture"/>
  </asset>
  <worldbody>
    <body name="base" pos="0 0 0.5">
      <geom name="base_geom" type="cylinder" size="0.1 0.05" material="arm_material"/>
      <body name="link1" pos="0 0 0.1">
        <joint name="joint1" type="hinge" axis="0 0 1"/>
        <geom name="link1_geom" type="capsule" size="0.04" fromto="0 0 0 0 0 0.3"/>
        <body name="link2" pos="0 0 0.3">
          <joint name="joint2" type="hinge" axis="1 0 0"/>
          <geom name="link2_geom" type="capsule" size="0.04" fromto="0 0 0 0 0 0.3"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="motor1" joint="joint1" gear="1"/>
    <motor name="motor2" joint="joint2" gear="1"/>
  </actuator>
</mujoco>)";

  std::string robot1_path = "/tmp/robot_arm.xml";
  std::ofstream robot1_file(robot1_path);
  robot1_file << robot1_mjcf;
  robot1_file.close();

  // Add first robot with prefix
  std::cout << "Adding first robot arm with prefix 'robot1'..." << std::endl;
  bool success = scene->add_mjcf(robot1_path, "robot1");
  if(success) {
    std::cout << "✓ Successfully added first robot arm" << std::endl;
  } else {
    std::cout << "✗ Failed to add first robot arm" << std::endl;
  }

  // Create second MJCF file: a simple box object
  std::string box_mjcf = R"(<?xml version="1.0"?>
<mujoco model="simple_box">
  <asset>
    <texture name="box_texture" type="2d" builtin="flat" rgb1="0.2 0.2 0.8" width="128" height="128"/>
    <material name="box_material" texture="box_texture"/>
  </asset>
  <worldbody>
    <body name="box" pos="1 0 0.5">
      <geom name="box_geom" type="box" size="0.2 0.2 0.2" material="box_material"/>
    </body>
  </worldbody>
</mujoco>)";

  std::string box_path = "/tmp/simple_box.xml";
  std::ofstream box_file(box_path);
  box_file << box_mjcf;
  box_file.close();

  // Add box without prefix
  std::cout << "Adding box object without prefix..." << std::endl;
  success = scene->add_mjcf(box_path, "");
  if(success) {
    std::cout << "✓ Successfully added box object" << std::endl;
  } else {
    std::cout << "✗ Failed to add box object" << std::endl;
  }

  // Add the same robot again with different prefix (simulating multiple robots)
  std::cout << "Adding second robot arm with prefix 'robot2'..." << std::endl;
  success = scene->add_mjcf(robot1_path, "robot2");
  if(success) {
    std::cout << "✓ Successfully added second robot arm" << std::endl;
  } else {
    std::cout << "✗ Failed to add second robot arm" << std::endl;
  }

  // Add a floor to the scene directly
  auto floor = std::make_shared<mjcf::Geom>();
  floor->name = "floor";
  floor->type = mjcf::GeomType::Plane;
  floor->size = {10.0, 10.0, 0.1};
  floor->rgba = {0.8, 0.8, 0.8, 1.0};
  scene->worldbody_->add_child(floor);

  // Generate the final MJCF scene
  std::cout << "Generating final MJCF scene..." << std::endl;
  std::string output_path = "/tmp/multi_mjcf_scene.xml";
  std::ofstream output_file(output_path);
  output_file << scene->get_xml_text();
  output_file.close();

  std::cout << "✓ Generated MJCF scene: " << output_path << std::endl;

  // Validate the result
  std::ifstream input_file(output_path);
  std::string content;
  std::string line;
  while(std::getline(input_file, line)) {
    content += line;
  }
  input_file.close();

  // Check for expected elements
  bool has_robot1_prefix = content.find("robot1_") != std::string::npos;
  bool has_robot2_prefix = content.find("robot2_") != std::string::npos;
  bool has_box_element   = content.find("box_geom") != std::string::npos;
  bool has_floor         = content.find("floor") != std::string::npos;
  bool has_actuators     = content.find("motor") != std::string::npos;

  std::cout << std::endl << "=== Validation Results ===" << std::endl;
  std::cout << "Contains robot1_ prefixes: " << (has_robot1_prefix ? "✓" : "✗") << std::endl;
  std::cout << "Contains robot2_ prefixes: " << (has_robot2_prefix ? "✓" : "✗") << std::endl;
  std::cout << "Contains box element: " << (has_box_element ? "✓" : "✗") << std::endl;
  std::cout << "Contains floor: " << (has_floor ? "✓" : "✗") << std::endl;
  std::cout << "Contains actuators: " << (has_actuators ? "✓" : "✗") << std::endl;

  // Clean up
  std::filesystem::remove(robot1_path);
  std::filesystem::remove(box_path);

  std::cout << std::endl << "=== Summary ===" << std::endl;
  std::cout << "The add_mjcf() API allows you to:" << std::endl;
  std::cout << "• Add multiple MJCF files to a single MJCF model" << std::endl;
  std::cout << "• Use name prefixes to avoid conflicts between multiple models" << std::endl;
  std::cout << "• Combine models from different sources in the same scene" << std::endl;
  std::cout << "• Maintain full control over simulation parameters" << std::endl;

  return 0;
}
