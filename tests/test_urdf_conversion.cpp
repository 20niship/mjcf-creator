#include "doctest.h"
#include "mjcf/mjcf.hpp"
#include <filesystem>
#include <fstream>
#include <tinyxml2.h>

TEST_SUITE("URDF Conversion Tests") {
  TEST_CASE("URDF to MJCF conversion with metadata") {
    // Path to test files
    std::string test_dir            = "../tests/";
    std::string urdf_path           = test_dir + "robot.urdf";
    std::string joint_metadata_path = test_dir + "metadata.xml";
    std::string actuator_path       = test_dir + "actuators/motor.json";

    // Check if test URDF file exists
    if(!std::filesystem::exists(urdf_path)) {
      MESSAGE("Skipping URDF conversion test - test files not found");
      return;
    }

    // Perform conversion
    auto mujoco  = std::make_shared<mjcf::Mujoco>();
    bool success = mujoco->add_urdf(urdf_path);
    CHECK(success);
    std::string content = mujoco->get_xml_text();

    CHECK(content.find("<mujoco") != std::string::npos);
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
    CHECK(content.size() > 5000);
  }

  TEST_CASE("URDF to MJCF conversion") {
    std::string urdf_path = "../tests/robot.urdf";
    auto mujoco  = std::make_shared<mjcf::Mujoco>();
    auto success = mujoco->add_urdf(urdf_path);
    CHECK(success);
    auto content = mujoco->get_xml_text();

    CHECK(content.find("<mujoco") != std::string::npos);
    CHECK(content.find("<option") != std::string::npos);
    CHECK(content.find("<asset") != std::string::npos);
    CHECK(content.find("<worldbody") != std::string::npos);
    CHECK(content.find("</mujoco>") != std::string::npos);

    // Check for some expected body elements from the URDF
    bool has_body = content.find("BODY") != std::string::npos || content.find("name=\"BODY\"") != std::string::npos;
    CHECK(has_body);

    bool has_torso = content.find("torso") != std::string::npos || content.find("name=\"torso\"") != std::string::npos;
    CHECK(has_torso);
    CHECK(content.size() > 1000);
  }

  TEST_CASE("URDF converter handles missing file") {
    std::string fake_urdf = "/tmp/nonexistent.urdf";
    auto mujoco           = std::make_shared<mjcf::Mujoco>();
    bool success          = mujoco->add_urdf(fake_urdf);
    CHECK_FALSE(success);
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

    std::string temp_urdf = "minimal_test.urdf";

    std::ofstream urdf_file(temp_urdf);
    urdf_file << minimal_urdf;
    urdf_file.close();

    auto mujoco  = std::make_shared<mjcf::Mujoco>();
    bool success = mujoco->add_urdf(temp_urdf);

    CHECK(success);

    auto mjcf_content = mujoco->get_xml_text();
    CHECK(mjcf_content.find("<mujoco") != std::string::npos);
    CHECK(mjcf_content.find("<option") != std::string::npos);
    CHECK(mjcf_content.find("<asset") != std::string::npos);
    CHECK(mjcf_content.find("<worldbody") != std::string::npos);
    CHECK(mjcf_content.find("base_link") != std::string::npos);
    CHECK(mjcf_content.find("test_material") != std::string::npos);
    CHECK(mjcf_content.find("box") != std::string::npos);
  }

  TEST_CASE("URDF with inline material definitions") {
    // Create a URDF with inline material definitions
    std::string inline_material_urdf = R"(<?xml version="1.0"?>
<robot name="material_test_robot">
  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
      <material name="inline_blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
  </link>
  
  <link name="link_with_inline_material_rgb">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="inline_green">
        <color rgba="0.0 1.0 0.0"/>
      </material>
    </visual>
  </link>
  
  <joint name="joint1" type="fixed">
    <parent link="base_link"/>
    <child link="link_with_inline_material_rgb"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
  </joint>
</robot>)";

    std::string temp_urdf = "inline_material_test.urdf";
    std::ofstream urdf_file(temp_urdf);
    urdf_file << inline_material_urdf;
    urdf_file.close();

    auto mujoco  = std::make_shared<mjcf::Mujoco>();
    bool success = mujoco->add_urdf(temp_urdf);

    CHECK(success);

    std::string xml_content = mujoco->get_xml_text();

    // Check that inline materials were created in the asset section
    CHECK(xml_content.find("<material") != std::string::npos);
    CHECK(xml_content.find("inline_blue") != std::string::npos);
    CHECK(xml_content.find("inline_green") != std::string::npos);

    // Check that the materials have correct RGBA values
    CHECK(xml_content.find("0 0 1 1") != std::string::npos); // Blue color
    CHECK(xml_content.find("0 1 0 1") != std::string::npos); // Green color (alpha defaulted to 1)

    // Check that geometries reference the materials
    CHECK(xml_content.find("material=\"inline_blue\"") != std::string::npos);
    CHECK(xml_content.find("material=\"inline_green\"") != std::string::npos);
  }

  TEST_CASE("URDF with global and referenced materials") {
    // Create a URDF with global materials and references
    std::string global_material_urdf = R"(<?xml version="1.0"?>
<robot name="global_material_robot">
  <material name="global_red">
    <color rgba="1.0 0.0 0.0 1.0"/>
  </material>
  
  <material name="global_yellow">
    <color rgba="1.0 1.0 0.0 1.0"/>
  </material>
  
  <link name="link1">
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
      <material name="global_red"/>
    </visual>
  </link>
  
  <link name="link2">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="global_yellow"/>
    </visual>
  </link>
  
  <joint name="joint1" type="fixed">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>
</robot>)";

    std::string temp_urdf = "global_material_test.urdf";
    std::ofstream urdf_file(temp_urdf);
    urdf_file << global_material_urdf;
    urdf_file.close();

    auto mujoco  = std::make_shared<mjcf::Mujoco>();
    bool success = mujoco->add_urdf(temp_urdf);

    CHECK(success);

    std::string xml_content = mujoco->get_xml_text();

    // Check that global materials were created in the asset section
    CHECK(xml_content.find("<material") != std::string::npos);
    CHECK(xml_content.find("global_red") != std::string::npos);
    CHECK(xml_content.find("global_yellow") != std::string::npos);

    // Check that geometries reference the materials
    CHECK(xml_content.find("material=\"global_red\"") != std::string::npos);
    CHECK(xml_content.find("material=\"global_yellow\"") != std::string::npos);
  }

  TEST_CASE("URDF with mesh geometry support") {
    std::string urdf_path = "../tests/robot_with_mesh.urdf";

    // Check if test file exists
    if(!std::filesystem::exists(urdf_path)) {
      MESSAGE("Skipping mesh test - test file not found");
      return;
    }

    auto mujoco  = std::make_shared<mjcf::Mujoco>();
    bool success = mujoco->add_urdf(urdf_path);

    CHECK(success);

    std::string xml_content = mujoco->get_xml_text();

    // Check for basic MJCF structure
    CHECK(xml_content.find("<mujoco") != std::string::npos);
    CHECK(xml_content.find("<asset") != std::string::npos);
    CHECK(xml_content.find("<worldbody") != std::string::npos);

    // Check that mesh assets were created
    CHECK(xml_content.find("<mesh") != std::string::npos);
    CHECK(xml_content.find("name=\"base_link_base\"") != std::string::npos);
    CHECK(xml_content.find("name=\"link1_link1\"") != std::string::npos);
    CHECK(xml_content.find("file=\"base.stl\"") != std::string::npos);
    CHECK(xml_content.find("file=\"link1.obj\"") != std::string::npos);

    // Check that mesh geometries reference the correct mesh names
    CHECK(xml_content.find("type=\"mesh\"") != std::string::npos);
    CHECK(xml_content.find("mesh=\"base_link_base\"") != std::string::npos);
    CHECK(xml_content.find("mesh=\"link1_link1\"") != std::string::npos);

    // Check that scale attributes are preserved
    CHECK(xml_content.find("scale=\"0.5 0.5 0.5\"") != std::string::npos);

    // Check that box geometry is still processed correctly
    CHECK(xml_content.find("type=\"box\"") != std::string::npos);
    CHECK(xml_content.find("link2") != std::string::npos);

    // Verify no unexpected sphere geoms were created
    // Count mesh and box geoms vs sphere geoms
    size_t mesh_count   = 0;
    size_t box_count    = 0;
    size_t sphere_count = 0;

    size_t pos = 0;
    while((pos = xml_content.find("type=\"mesh\"", pos)) != std::string::npos) {
      mesh_count++;
      pos += 11; // Length of "type=\"mesh\""
    }

    pos = 0;
    while((pos = xml_content.find("type=\"box\"", pos)) != std::string::npos) {
      box_count++;
      pos += 10; // Length of "type=\"box\""
    }

    pos = 0;
    while((pos = xml_content.find("type=\"sphere\"", pos)) != std::string::npos) {
      sphere_count++;
      pos += 13; // Length of "type=\"sphere\""
    }

    // We expect 2 mesh geoms (base_link and link1) and 1 box geom (link2)
    // No sphere geoms should be created from the URDF
    CHECK(mesh_count == 2);
    CHECK(box_count == 1);
    CHECK(sphere_count == 0);
  }


  // TEST_CASE("multiple-urdf") {
  //   // Create a minimal test URDF
  //   std::string minimal_urdf = R"(<?xml version="1.0"?>
  // <robot name="test_robot">
  // <material name="red_material">
  //   <color rgba="1.0 0.0 0.0 1.0"/>
  // </material>

  // <link name="base_link">
  //   <inertial>
  //     <origin xyz="0 0 0" rpy="0 0 0"/>
  //     <mass value="1.0"/>
  //     <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
  //   </inertial>
  //   <visual>
  //     <origin xyz="0 0 0" rpy="0 0 0"/>
  //     <geometry>
  //       <box size="0.1 0.1 0.1"/>
  //     </geometry>
  //     <material name="red_material"/>
  //   </visual>
  // </link>
  // </robot>)";

  //   std::string temp_urdf = "test_add_urdf.urdf";
  //   std::ofstream urdf_file(temp_urdf);
  //   urdf_file << minimal_urdf;
  //   urdf_file.close();

  //   auto mujoco = std::make_shared<mjcf::Mujoco>("multi_robot_scene");

  //   auto option        = mujoco->option_;
  //   option->integrator = mjcf::IntegratorType::RK4;
  //   option->timestep   = 0.001;
  //   mujoco->add_child(option);

  //   bool success1 = mujoco->add_urdf(temp_urdf, "robot1");
  //   CHECK(success1);

  //   bool success2 = mujoco->add_urdf(temp_urdf, "robot2");
  //   CHECK(success2);

  //   bool success3 = mujoco->add_urdf(temp_urdf, "");
  //   CHECK(success3);

  //   // Generate XML and validate
  //   std::string xml_content = mujoco->get_xml_text();

  //   // Check for expected content
  //   CHECK(xml_content.find("multi_robot_scene") != std::string::npos);
  //   CHECK(xml_content.find("robot1_") != std::string::npos);
  //   CHECK(xml_content.find("robot2_") != std::string::npos);

  //   // Should have multiple materials with prefixes
  //   CHECK(xml_content.find("robot1_red_material") != std::string::npos);
  //   CHECK(xml_content.find("robot2_red_material") != std::string::npos);

  //   // Should have multiple bodies with prefixes
  //   CHECK(xml_content.find("robot1_base_link") != std::string::npos);
  //   CHECK(xml_content.find("robot2_base_link") != std::string::npos);
  // }
}
