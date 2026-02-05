/**
 * @file multi_urdf_example.cpp
 * @brief Example demonstrating the new add_urdf API for combining multiple URDF files
 *
 * This example shows how to use the new add_urdf() method to combine multiple
 * URDF files into a single MJCF model, allowing for complex scenes with
 * multiple robots or objects.
 */

#include "mjcf/mjcf.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>

int main() {
  std::cout << "=== Multi-URDF MJCF Composition Example ===" << std::endl;

  auto scene = std::make_shared<mjcf::Mujoco>("multi_robot_scene");
  // auto option = std::make_shared<mjcf::Option>();
  // scene->add_child(option);
  // auto compiler             = std::make_shared<mjcf::Compiler>();
  // compiler->angle           = "radian";
  // compiler->coordinate      = "local";
  // compiler->inertiafromgeom = true;
  // scene->add_child(compiler);

  std::cout << "Created base MJCF scene with simulation settings" << std::endl;

  // Example 1: Add first robot with prefix to avoid naming conflicts
  std::string robot1_urdf = "/home/runner/work/mjcf-creator/mjcf-creator/misc/urdf2mjcf/tests/sample/robot.urdf";

  if(std::filesystem::exists(robot1_urdf)) {
    std::cout << "Adding first robot with prefix 'robot1'..." << std::endl;
    auto [body, joint] = scene->add_urdf(robot1_urdf, "robot1", false, {});
    if(body) {
      std::cout << "✓ Successfully added first robot with prefix 'robot1_'" << std::endl;
    } else {
      std::cout << "✗ Failed to add first robot" << std::endl;
    }
  } else {
    std::cout << "Skipping robot1 - URDF file not found at: " << robot1_urdf << std::endl;
  }

  // Example 2: Add the same robot again with different prefix (simulating multiple robots)
  if(std::filesystem::exists(robot1_urdf)) {
    std::cout << "Adding second instance with prefix 'robot2'..." << std::endl;
    auto [body, joint] = scene->add_urdf(robot1_urdf, "robot2");
    if(body) {
      std::cout << "✓ Successfully added second robot with prefix 'robot2_'" << std::endl;
    } else {
      std::cout << "✗ Failed to add second robot" << std::endl;
    }
  }

  // Example 3: Create and add a simple URDF without prefix
  std::cout << "Creating and adding a simple object without prefix..." << std::endl;

  std::string simple_urdf = R"(<?xml version="1.0"?>
<robot name="simple_box">
  <material name="blue_material">
    <color rgba="0.0 0.0 1.0 1.0"/>
  </material>
  
  <link name="box_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 0.3"/>
      </geometry>
      <material name="blue_material"/>
    </visual>
  </link>
</robot>)";

  std::string temp_urdf = "/tmp/simple_box.urdf";
  std::ofstream urdf_file(temp_urdf);
  urdf_file << simple_urdf;
  urdf_file.close();

  auto [body, joint] = scene->add_urdf(temp_urdf, "");
  if(body != nullptr) {
    std::cout << "✓ Successfully added simple box without prefix" << std::endl;
  } else {
    std::cout << "✗ Failed to add simple box" << std::endl;
  }

  // Generate the final MJCF scene
  std::cout << "Generating final MJCF scene..." << std::endl;
  std::string output_path = "/tmp/multi_robot_scene_example.mjcf";
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
  bool has_box_element   = content.find("box_link") != std::string::npos;
  bool has_blue_material = content.find("blue_material") != std::string::npos;

  std::cout << std::endl << "=== Validation Results ===" << std::endl;
  std::cout << "Contains robot1_ prefixes: " << (has_robot1_prefix ? "✓" : "✗") << std::endl;
  std::cout << "Contains robot2_ prefixes: " << (has_robot2_prefix ? "✓" : "✗") << std::endl;
  std::cout << "Contains box element: " << (has_box_element ? "✓" : "✗") << std::endl;
  std::cout << "Contains blue material: " << (has_blue_material ? "✓" : "✗") << std::endl;

  // Clean up
  std::filesystem::remove(temp_urdf);

  std::cout << std::endl << "=== Summary ===" << std::endl;
  std::cout << "The new add_urdf() API allows you to:" << std::endl;
  std::cout << "• Add multiple URDF files to a single MJCF model" << std::endl;
  std::cout << "• Use name prefixes to avoid conflicts between multiple robots" << std::endl;
  std::cout << "• Combine robots and geometric objects in the same scene" << std::endl;
  std::cout << "• Maintain full control over simulation parameters" << std::endl;

  return 0;
}
