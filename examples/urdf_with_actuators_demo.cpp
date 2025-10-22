#include "mjcf/mjcf.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>

/**
 * @brief Demonstrates converting URDF to MJCF with actuators generated from joint metadata
 * 
 * This example shows how to:
 * 1. Create a URDF file with multiple joints
 * 2. Define joint metadata with actuator types and control parameters
 * 3. Convert URDF to MJCF with automatic actuator generation
 */
void create_urdf_with_actuators_demo() {
  std::cout << "Creating URDF with actuators demo..." << std::endl;

  // Create a sample URDF with a simple robot arm
  std::string robot_urdf = R"(<?xml version="1.0"?>
<robot name="demo_robot_arm">
  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 0.1"/>
      </geometry>
    </visual>
  </link>
  
  <link name="shoulder_link">
    <inertial>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </visual>
  </link>
  
  <link name="elbow_link">
    <inertial>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <mass value="0.7"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.2"/>
      </geometry>
    </visual>
  </link>
  
  <link name="wrist_link">
    <inertial>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <mass value="0.3"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.06"/>
      </geometry>
    </visual>
  </link>
  
  <joint name="shoulder_yaw" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="50" velocity="2.0"/>
  </joint>
  
  <joint name="shoulder_pitch" type="revolute">
    <parent link="shoulder_link"/>
    <child link="elbow_link"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="40" velocity="1.5"/>
  </joint>
  
  <joint name="elbow_pitch" type="revolute">
    <parent link="elbow_link"/>
    <child link="wrist_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="30" velocity="2.0"/>
  </joint>
</robot>)";

  // Save URDF to temporary file
  std::string urdf_path = "demo_robot_arm.urdf";
  std::ofstream urdf_file(urdf_path);
  urdf_file << robot_urdf;
  urdf_file.close();

  // Create joint metadata with actuator configurations
  std::map<std::string, mjcf::JointMetadata> joint_metadata;

  // Configure shoulder_yaw with a motor actuator
  mjcf::JointMetadata shoulder_yaw_meta;
  shoulder_yaw_meta.actuator_type     = "motor";
  shoulder_yaw_meta.soft_torque_limit = 50.0;
  shoulder_yaw_meta.id                = 0;
  shoulder_yaw_meta.nn_id             = 0;
  joint_metadata["shoulder_yaw"]      = shoulder_yaw_meta;

  // Configure shoulder_pitch with a position actuator (PD control)
  mjcf::JointMetadata shoulder_pitch_meta;
  shoulder_pitch_meta.actuator_type     = "position";
  shoulder_pitch_meta.kp                = 100.0; // Position gain
  shoulder_pitch_meta.kd                = 5.0;   // Damping gain
  shoulder_pitch_meta.soft_torque_limit = 40.0;
  shoulder_pitch_meta.id                = 1;
  shoulder_pitch_meta.nn_id             = 1;
  joint_metadata["shoulder_pitch"]      = shoulder_pitch_meta;

  // Configure elbow_pitch with a position actuator
  mjcf::JointMetadata elbow_pitch_meta;
  elbow_pitch_meta.actuator_type     = "position";
  elbow_pitch_meta.kp                = 80.0;
  elbow_pitch_meta.kd                = 4.0;
  elbow_pitch_meta.soft_torque_limit = 30.0;
  elbow_pitch_meta.id                = 2;
  elbow_pitch_meta.nn_id             = 2;
  joint_metadata["elbow_pitch"]      = elbow_pitch_meta;

  // Create actuator metadata (optional, for future extensions)
  std::map<std::string, mjcf::ActuatorMetadata> actuator_metadata;

  // Create MJCF model and convert URDF with actuators
  auto mujoco            = std::make_shared<mjcf::Mujoco>("robot_arm_with_actuators");
  mujoco->option_->timestep   = 0.002;
  mujoco->option_->integrator = mjcf::IntegratorType::RK4;

  std::cout << "Converting URDF to MJCF with actuators..." << std::endl;
  bool success = mujoco->add_urdf(urdf_path, "", false, joint_metadata, actuator_metadata);

  if(!success) {
    std::cerr << "Failed to convert URDF" << std::endl;
    return;
  }

  // Save the generated MJCF
  std::string output_file = "robot_arm_with_actuators.xml";
  std::ofstream output(output_file);
  output << mujoco->get_xml_text();
  output.close();

  std::cout << "Successfully generated MJCF with actuators: " << output_file << std::endl;

  // Print summary
  std::string xml = mujoco->get_xml_text();
  int actuator_count = 0;
  size_t pos         = 0;
  while((pos = xml.find("<motor", pos)) != std::string::npos || (pos = xml.find("<position", pos)) != std::string::npos) {
    actuator_count++;
    pos++;
  }

  std::cout << "\nSummary:" << std::endl;
  std::cout << "- Created 3 joints from URDF" << std::endl;
  std::cout << "- Generated 3 actuators from joint metadata:" << std::endl;
  std::cout << "  * shoulder_yaw: Motor actuator (torque limit: 50 Nm)" << std::endl;
  std::cout << "  * shoulder_pitch: Position actuator (kp=100, kd=5, torque limit: 40 Nm)" << std::endl;
  std::cout << "  * elbow_pitch: Position actuator (kp=80, kd=4, torque limit: 30 Nm)" << std::endl;
}

int main() {
  create_urdf_with_actuators_demo();
  return 0;
}
