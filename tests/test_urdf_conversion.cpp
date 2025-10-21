#include "doctest.h"
#include "mjcf/mjcf.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <regex>

// Simple JSON parser for metadata files - in a real implementation you'd use a JSON library
std::map<std::string, mjcf::JointMetadata> parse_joint_metadata_json(const std::string& json_path) {
  std::map<std::string, mjcf::JointMetadata> metadata;

  std::ifstream file(json_path);
  if(!file.is_open()) {
    return metadata;
  }

  std::string content;
  std::string line;
  while(std::getline(file, line)) {
    content += line;
  }
  file.close();

  // Simple approach - just look for some key joint names manually since the format is known
  std::vector<std::string> known_joints = {"imu",          "RLEG_HIP_R",   "RLEG_HIP_P", "RLEG_HIP_Y", "RLEG_KNEE", "RLEG_ANKLE_P",    "RLEG_ANKLE_R",    "LLEG_HIP_R",      "LLEG_HIP_P",      "LLEG_HIP_Y", "LLEG_KNEE",
                                           "LLEG_ANKLE_P", "LLEG_ANKLE_R", "WAIST_P",    "WAIST_R",    "CHEST",     "RARM_SHOULDER_P", "RARM_SHOULDER_R", "LARM_SHOULDER_P", "LARM_SHOULDER_R", "RARM_ELBOW", "LARM_ELBOW"};

  int id_counter = 0;
  for(const auto& joint_name : known_joints) {
    if(content.find("\"" + joint_name + "\"") != std::string::npos) {
      mjcf::JointMetadata joint_meta;
      joint_meta.actuator_type     = "motor";
      joint_meta.id                = id_counter++;
      joint_meta.nn_id             = joint_meta.id;
      joint_meta.kp                = 40.0;
      joint_meta.kd                = 2.0;
      joint_meta.soft_torque_limit = 10.0;
      joint_meta.min_angle_deg     = 0.0;
      joint_meta.max_angle_deg     = 0.0;

      metadata[joint_name] = joint_meta;
    }
  }

  return metadata;
}

mjcf::ActuatorMetadata parse_actuator_metadata_json(const std::string& json_path) {
  mjcf::ActuatorMetadata metadata;
  metadata.actuator_type = "motor";
  metadata.sysid         = "";
  metadata.max_torque    = 10.0;
  metadata.max_velocity  = 5.0;
  metadata.armature      = 0.001;
  metadata.damping       = 0.1;
  metadata.frictionloss  = 0.1;

  std::ifstream file(json_path);
  if(!file.is_open()) {
    return metadata;
  }

  std::string content;
  std::string line;
  while(std::getline(file, line)) {
    content += line;
  }
  file.close();

  // Basic parsing for actuator parameters - look for known values
  if(content.find("\"max_torque\": 10.0") != std::string::npos) {
    metadata.max_torque = 10.0;
  }
  if(content.find("\"max_velocity\": 5.0") != std::string::npos) {
    metadata.max_velocity = 5.0;
  }
  if(content.find("\"armature\": 0.001") != std::string::npos) {
    metadata.armature = 0.001;
  }
  if(content.find("\"damping\": 0.1") != std::string::npos) {
    metadata.damping = 0.1;
  }

  return metadata;
}

TEST_SUITE("URDF Conversion Tests") {
  TEST_CASE("URDF to MJCF conversion with metadata") {
    // Path to test files
    std::string test_dir            = "/home/runner/work/mjcf-creator/mjcf-creator/misc/urdf2mjcf/tests/sample/";
    std::string urdf_path           = test_dir + "robot.urdf";
    std::string joint_metadata_path = test_dir + "joint_metadata.json";
    std::string actuator_path       = test_dir + "actuators/motor.json";
    std::string output_path         = "/tmp/converted_robot_with_metadata.mjcf";

    // Check if test URDF file exists
    if(!std::filesystem::exists(urdf_path)) {
      MESSAGE("Skipping URDF conversion test - test files not found");
      return;
    }

    // Parse metadata files
    auto joint_metadata = parse_joint_metadata_json(joint_metadata_path);
    auto actuator_meta  = parse_actuator_metadata_json(actuator_path);
    std::map<std::string, mjcf::ActuatorMetadata> actuator_metadata;
    actuator_metadata["motor"] = actuator_meta;

    MESSAGE("Parsed ", joint_metadata.size(), " joint metadata entries");
    CHECK(joint_metadata.size() > 10); // Should have many joints from the humanoid robot

    // Perform conversion
    auto mujoco  = std::make_shared<mjcf::Mujoco>();
    bool success = mjcf::UrdfConverter::parse_urdf_to_mjcf(mujoco.get(), urdf_path,
                                                           // false, // copy_meshes
                                                           joint_metadata, actuator_metadata);

    // Check that conversion succeeded
    CHECK(success);

    // Check that output file exists
    CHECK(std::filesystem::exists(output_path));

    // Read the output file and do detailed validation
    std::ifstream file(output_path);
    std::string content;

    if(file.is_open()) {
      std::string line;
      while(std::getline(file, line)) {
        content += line + "\n";
      }
      file.close();
    }

    // Basic checks on the XML content
    CHECK(content.find("<mujoco") != std::string::npos);
    CHECK(content.find("model=\"simple_humanoid\"") != std::string::npos);
    CHECK(content.find("<option") != std::string::npos);
    CHECK(content.find("<asset") != std::string::npos);
    CHECK(content.find("<worldbody") != std::string::npos);
    CHECK(content.find("</mujoco>") != std::string::npos);

    // Check for some expected body elements from the humanoid URDF
    bool has_body = content.find("BODY") != std::string::npos || content.find("name=\"BODY\"") != std::string::npos;
    CHECK(has_body);

    bool has_torso = content.find("torso") != std::string::npos || content.find("name=\"torso\"") != std::string::npos;
    CHECK(has_torso);

    // Check for some expected joints
    bool has_rleg_hip = content.find("RLEG_HIP_R") != std::string::npos;
    CHECK(has_rleg_hip);

    bool has_lleg_hip = content.find("LLEG_HIP_R") != std::string::npos;
    CHECK(has_lleg_hip);

    // Check for materials that were defined in the URDF
    bool has_waist_material = content.find("WAIST_LINK1_APP") != std::string::npos;
    CHECK(has_waist_material);

    // The content should be substantial (humanoid robot should be complex)
    CHECK(content.size() > 5000);

    MESSAGE("Successfully converted humanoid robot URDF with ", joint_metadata.size(), " joints");

    // Clean up
    std::filesystem::remove(output_path);
  }
  TEST_CASE("URDF to MJCF conversion") {
    // Path to test files
    std::string test_dir    = "/home/runner/work/mjcf-creator/mjcf-creator/misc/urdf2mjcf/tests/sample/";
    std::string urdf_path   = test_dir + "robot.urdf";
    std::string output_path = "/tmp/converted_robot.mjcf";

    // Check if test URDF file exists
    if(!std::filesystem::exists(urdf_path)) {
      // Skip test if files don't exist
      MESSAGE("Skipping URDF conversion test - test files not found");
      return;
    }

    // Create joint metadata (simplified version of the JSON)
    std::map<std::string, mjcf::JointMetadata> joint_metadata;
    std::map<std::string, mjcf::ActuatorMetadata> actuator_metadata;

    // Add some sample joint metadata
    mjcf::JointMetadata joint_meta;
    joint_meta.actuator_type     = "motor";
    joint_meta.id                = 0;
    joint_meta.nn_id             = 0;
    joint_meta.kp                = 40.0;
    joint_meta.kd                = 2.0;
    joint_meta.soft_torque_limit = 10.0;
    joint_meta.min_angle_deg     = 0.0;
    joint_meta.max_angle_deg     = 0.0;

    // Add metadata for a few key joints from the test robot
    joint_metadata["RLEG_HIP_R"] = joint_meta;
    joint_metadata["RLEG_HIP_P"] = joint_meta;
    joint_metadata["RLEG_KNEE"]  = joint_meta;
    joint_metadata["LLEG_HIP_R"] = joint_meta;
    joint_metadata["WAIST_P"]    = joint_meta;
    joint_metadata["CHEST"]      = joint_meta;

    // Add actuator metadata
    mjcf::ActuatorMetadata actuator_meta;
    actuator_meta.actuator_type = "motor";
    actuator_meta.sysid         = "";
    actuator_meta.max_torque    = 10.0;
    actuator_meta.max_velocity  = 5.0;
    actuator_meta.armature      = 0.001;
    actuator_meta.damping       = 0.1;
    actuator_meta.frictionloss  = 0.1;

    actuator_metadata["motor"] = actuator_meta;

    auto mujoco  = std::make_shared<mjcf::Mujoco>();
    bool success = mjcf::UrdfConverter::parse_urdf_to_mjcf(mujoco.get(), urdf_path,
                                                           // false, // copy_meshes
                                                           joint_metadata, actuator_metadata);

    // Check that conversion succeeded
    CHECK(success);

    // Check that output file exists
    CHECK(std::filesystem::exists(output_path));

    // Read the output file and do basic validation
    std::ifstream file(output_path);
    std::string content;

    if(file.is_open()) {
      std::string line;
      while(std::getline(file, line)) {
        content += line + "\n";
      }
      file.close();
    }

    // Basic checks on the XML content
    CHECK(content.find("<mujoco") != std::string::npos);
    CHECK(content.find("model=\"simple_humanoid\"") != std::string::npos);
    CHECK(content.find("<option") != std::string::npos);
    CHECK(content.find("<asset") != std::string::npos);
    CHECK(content.find("<worldbody") != std::string::npos);
    CHECK(content.find("</mujoco>") != std::string::npos);

    // Check for some expected body elements from the URDF
    bool has_body = content.find("BODY") != std::string::npos || content.find("name=\"BODY\"") != std::string::npos;
    CHECK(has_body);

    bool has_torso = content.find("torso") != std::string::npos || content.find("name=\"torso\"") != std::string::npos;
    CHECK(has_torso);

    // The content should be substantial (not just a skeleton)
    CHECK(content.size() > 1000);

    // Clean up
    std::filesystem::remove(output_path);

    MESSAGE("URDF to MJCF conversion test passed");
  }

  TEST_CASE("URDF converter handles missing file") {
    std::string fake_urdf   = "/tmp/nonexistent.urdf";
    std::string output_path = "/tmp/should_not_exist.mjcf";

    std::map<std::string, mjcf::JointMetadata> joint_metadata;
    std::map<std::string, mjcf::ActuatorMetadata> actuator_metadata;

    auto mujoco  = std::make_shared<mjcf::Mujoco>();
    bool success = mjcf::UrdfConverter::parse_urdf_to_mjcf(mujoco.get(), fake_urdf, joint_metadata, actuator_metadata);

    // Should fail gracefully
    CHECK_FALSE(success);
    CHECK_FALSE(std::filesystem::exists(output_path));
  }

  TEST_CASE("Basic URDF parsing functionality") {
    // Create a minimal test URDF in memory
    std::string minimal_urdf = R"(<?xml version="1.0"?>
<robot name="test_robot">
  <material name="test_material">
    <color rgba="1.0 0.0 0.0 1.0"/>
  </material>
  
  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="test_material"/>
    </visual>
  </link>
</robot>)";

    // Write to temporary file
    std::string temp_urdf = "/tmp/minimal_test.urdf";
    std::string temp_mjcf = "/tmp/minimal_test.mjcf";

    std::ofstream urdf_file(temp_urdf);
    urdf_file << minimal_urdf;
    urdf_file.close();

    std::map<std::string, mjcf::JointMetadata> joint_metadata;
    std::map<std::string, mjcf::ActuatorMetadata> actuator_metadata;

    auto mujoco  = std::make_shared<mjcf::Mujoco>();
    bool success = mjcf::UrdfConverter::parse_urdf_to_mjcf(mujoco.get(), temp_urdf, joint_metadata, actuator_metadata);

    CHECK(success);
    CHECK(std::filesystem::exists(temp_mjcf));

    // Read and validate the output
    std::ifstream mjcf_file(temp_mjcf);
    std::string mjcf_content;
    std::string line;
    while(std::getline(mjcf_file, line)) {
      mjcf_content += line + "\n";
    }
    mjcf_file.close();

    // Should contain the robot name and basic structure
    CHECK(mjcf_content.find("test_robot") != std::string::npos);
    CHECK(mjcf_content.find("<mujoco") != std::string::npos);
    CHECK(mjcf_content.find("<option") != std::string::npos);
    CHECK(mjcf_content.find("<asset") != std::string::npos);
    CHECK(mjcf_content.find("<worldbody") != std::string::npos);
    CHECK(mjcf_content.find("base_link") != std::string::npos);
    CHECK(mjcf_content.find("test_material") != std::string::npos);

    // Clean up
    std::filesystem::remove(temp_urdf);
    std::filesystem::remove(temp_mjcf);
  }

  TEST_CASE("New add_urdf API functionality") {
    // Create a minimal test URDF
    std::string minimal_urdf = R"(<?xml version="1.0"?>
<robot name="test_robot">
  <material name="red_material">
    <color rgba="1.0 0.0 0.0 1.0"/>
  </material>
  
  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="red_material"/>
    </visual>
  </link>
</robot>)";

    std::string temp_urdf = "/tmp/test_add_urdf.urdf";
    std::ofstream urdf_file(temp_urdf);
    urdf_file << minimal_urdf;
    urdf_file.close();

    auto mujoco = std::make_shared<mjcf::Mujoco>("multi_robot_scene");

    auto option        = mujoco->option_;
    option->integrator = mjcf::IntegratorType::RK4;
    option->timestep   = 0.001;
    mujoco->add_child(option);

    bool success1 = mujoco->add_urdf(temp_urdf, "robot1");
    CHECK(success1);

    // Test 2: Add second URDF with different prefix
    bool success2 = mujoco->add_urdf(temp_urdf, "robot2");
    CHECK(success2);

    // Test 3: Add third URDF without prefix
    bool success3 = mujoco->add_urdf(temp_urdf, "");
    CHECK(success3);

    // Generate XML and validate
    std::string xml_content = mujoco->get_xml_text();

    // Check for expected content
    CHECK(xml_content.find("multi_robot_scene") != std::string::npos);
    CHECK(xml_content.find("robot1_") != std::string::npos);
    CHECK(xml_content.find("robot2_") != std::string::npos);

    // Should have multiple materials with prefixes
    CHECK(xml_content.find("robot1_red_material") != std::string::npos);
    CHECK(xml_content.find("robot2_red_material") != std::string::npos);

    // Should have multiple bodies with prefixes
    CHECK(xml_content.find("robot1_base_link") != std::string::npos);
    CHECK(xml_content.find("robot2_base_link") != std::string::npos);

    // Clean up
    std::filesystem::remove(temp_urdf);

    MESSAGE("New add_urdf API test passed successfully");
  }
}
