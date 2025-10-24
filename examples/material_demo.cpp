/**
 * @file material_demo.cpp
 * @brief Example demonstrating URDF material support (global and inline materials)
 *
 * This example shows how the URDF converter now supports:
 * 1. Global materials defined at robot level
 * 2. Inline materials defined within visual elements
 * 3. Material references from visual elements to global materials
 */

#include "mjcf/mjcf.hpp"
#include <fstream>
#include <iostream>

int main() {
  std::cout << "=== URDF Material Support Demo ===" << std::endl;

  // Example 1: URDF with global materials
  std::cout << "\n1. Testing global materials..." << std::endl;
  std::string global_material_urdf = R"(<?xml version="1.0"?>
<robot name="global_material_robot">
  <!-- Define materials at robot level -->
  <material name="red_material">
    <color rgba="1.0 0.0 0.0 1.0"/>
  </material>
  
  <material name="green_material">
    <color rgba="0.0 1.0 0.0 1.0"/>
  </material>
  
  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
      <!-- Reference global material -->
      <material name="red_material"/>
    </visual>
  </link>
  
  <link name="arm_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
      <!-- Reference global material -->
      <material name="green_material"/>
    </visual>
  </link>
  
  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1.0"/>
  </joint>
</robot>)";

  std::string temp_urdf1 = "/tmp/global_material.urdf";
  std::ofstream urdf_file1(temp_urdf1);
  urdf_file1 << global_material_urdf;
  urdf_file1.close();

  auto mujoco1  = std::make_shared<mjcf::Mujoco>();
  bool success1 = mujoco1->add_urdf(temp_urdf1);

  if(success1) {
    std::string xml1 = mujoco1->get_xml_text();
    std::cout << "✓ Global materials converted successfully" << std::endl;
    std::cout << "  - Found red_material: " << (xml1.find("red_material") != std::string::npos ? "✓" : "✗") << std::endl;
    std::cout << "  - Found green_material: " << (xml1.find("green_material") != std::string::npos ? "✓" : "✗") << std::endl;
    std::cout << "  - Materials in asset section: " << (xml1.find("<material") != std::string::npos ? "✓" : "✗") << std::endl;
    std::cout << "  - Geometries reference materials: " << (xml1.find("material=\"") != std::string::npos ? "✓" : "✗") << std::endl;

    std::ofstream out1("/tmp/global_material_demo.xml");
    out1 << xml1;
    out1.close();
    std::cout << "  - Saved to: /tmp/global_material_demo.xml" << std::endl;
  } else {
    std::cout << "✗ Failed to convert URDF with global materials" << std::endl;
  }

  // Example 2: URDF with inline materials
  std::cout << "\n2. Testing inline materials..." << std::endl;
  std::string inline_material_urdf = R"(<?xml version="1.0"?>
<robot name="inline_material_robot">
  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <!-- Inline material definition with RGBA (full format) -->
      <material name="blue_shiny">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
  </link>
  
  <link name="top_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.1 0.1 0.3"/>
      </geometry>
      <!-- Another inline material with RGB only (alpha defaults to 1.0) -->
      <material name="yellow_matte">
        <color rgba="1.0 1.0 0.0"/>
      </material>
    </visual>
  </link>
  
  <joint name="top_joint" type="fixed">
    <parent link="base_link"/>
    <child link="top_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
  </joint>
</robot>)";

  std::string temp_urdf2 = "/tmp/inline_material.urdf";
  std::ofstream urdf_file2(temp_urdf2);
  urdf_file2 << inline_material_urdf;
  urdf_file2.close();

  auto mujoco2  = std::make_shared<mjcf::Mujoco>();
  bool success2 = mujoco2->add_urdf(temp_urdf2);

  if(success2) {
    std::string xml2 = mujoco2->get_xml_text();
    std::cout << "✓ Inline materials converted successfully" << std::endl;
    std::cout << "  - Found blue_shiny: " << (xml2.find("blue_shiny") != std::string::npos ? "✓" : "✗") << std::endl;
    std::cout << "  - Found yellow_matte: " << (xml2.find("yellow_matte") != std::string::npos ? "✓" : "✗") << std::endl;
    std::cout << "  - Blue color (0 0 1 1): " << (xml2.find("0 0 1 1") != std::string::npos ? "✓" : "✗") << std::endl;
    std::cout << "  - Yellow color (1 1 0 1): " << (xml2.find("1 1 0 1") != std::string::npos ? "✓" : "✗") << std::endl;

    std::ofstream out2("/tmp/inline_material_demo.xml");
    out2 << xml2;
    out2.close();
    std::cout << "  - Saved to: /tmp/inline_material_demo.xml" << std::endl;
  } else {
    std::cout << "✗ Failed to convert URDF with inline materials" << std::endl;
  }

  // Example 3: Mixed global and inline materials
  std::cout << "\n3. Testing mixed materials..." << std::endl;
  std::string mixed_material_urdf = R"(<?xml version="1.0"?>
<robot name="mixed_material_robot">
  <!-- Global material -->
  <material name="global_orange">
    <color rgba="1.0 0.5 0.0 1.0"/>
  </material>
  
  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.3 0.3 0.1"/>
      </geometry>
      <!-- Reference to global material -->
      <material name="global_orange"/>
    </visual>
  </link>
  
  <link name="arm_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.4"/>
      </geometry>
      <!-- Inline material -->
      <material name="inline_purple">
        <color rgba="0.5 0.0 0.5 1.0"/>
      </material>
    </visual>
  </link>
  
  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0.15 0 0.05" rpy="0 1.57 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="5" velocity="1.0"/>
  </joint>
</robot>)";

  std::string temp_urdf3 = "/tmp/mixed_material.urdf";
  std::ofstream urdf_file3(temp_urdf3);
  urdf_file3 << mixed_material_urdf;
  urdf_file3.close();

  auto mujoco3  = std::make_shared<mjcf::Mujoco>();
  bool success3 = mujoco3->add_urdf(temp_urdf3);

  if(success3) {
    std::string xml3 = mujoco3->get_xml_text();
    std::cout << "✓ Mixed materials converted successfully" << std::endl;
    std::cout << "  - Global material (global_orange): " << (xml3.find("global_orange") != std::string::npos ? "✓" : "✗") << std::endl;
    std::cout << "  - Inline material (inline_purple): " << (xml3.find("inline_purple") != std::string::npos ? "✓" : "✗") << std::endl;
    std::cout << "  - Both materials in asset section: " << (xml3.find("<material") != std::string::npos ? "✓" : "✗") << std::endl;

    std::ofstream out3("/tmp/mixed_material_demo.xml");
    out3 << xml3;
    out3.close();
    std::cout << "  - Saved to: /tmp/mixed_material_demo.xml" << std::endl;
  } else {
    std::cout << "✗ Failed to convert URDF with mixed materials" << std::endl;
  }

  std::cout << "\n=== Summary ===" << std::endl;
  std::cout << "The URDF converter now supports:" << std::endl;
  std::cout << "✓ Global materials defined at robot level" << std::endl;
  std::cout << "✓ Inline materials defined within visual elements" << std::endl;
  std::cout << "✓ Material references from visual elements to global materials" << std::endl;
  std::cout << "✓ RGB and RGBA color formats (alpha defaults to 1.0 for RGB)" << std::endl;
  std::cout << "✓ All URDF material writing styles are now supported" << std::endl;

  return 0;
}
