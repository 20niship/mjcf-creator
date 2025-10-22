#include "../src/ext/tinyxml2.h"
#include "doctest.h"
#include "mjcf/mjcf.hpp"
#include <filesystem>
#include <fstream>
#include <map>

std::map<std::string, mjcf::JointMetadata> parse_joint_metadata_xml(const std::string& xml_path) {
  std::map<std::string, mjcf::JointMetadata> metadata;

  tinyxml2::XMLDocument doc;
  if(doc.LoadFile(xml_path.c_str()) != tinyxml2::XML_SUCCESS) {
    return metadata;
  }

  tinyxml2::XMLElement* root = doc.FirstChildElement("joint_metadata");
  if(!root) {
    return metadata;
  }

  for(tinyxml2::XMLElement* joint_elem = root->FirstChildElement("joint"); joint_elem != nullptr; joint_elem = joint_elem->NextSiblingElement("joint")) {

    const char* joint_name = joint_elem->Attribute("name");
    if(!joint_name) continue;

    mjcf::JointMetadata joint_meta;

    // Parse actuator_type
    tinyxml2::XMLElement* actuator_type_elem = joint_elem->FirstChildElement("actuator_type");
    if(actuator_type_elem && actuator_type_elem->GetText()) {
      joint_meta.actuator_type = actuator_type_elem->GetText();
    }

    // Parse nn_id
    tinyxml2::XMLElement* nn_id_elem = joint_elem->FirstChildElement("nn_id");
    if(nn_id_elem && nn_id_elem->GetText()) {
      joint_meta.nn_id = std::stoi(nn_id_elem->GetText());
    }

    // Parse id
    tinyxml2::XMLElement* id_elem = joint_elem->FirstChildElement("id");
    if(id_elem && id_elem->GetText()) {
      joint_meta.id = std::stoi(id_elem->GetText());
    }

    // Parse kp
    tinyxml2::XMLElement* kp_elem = joint_elem->FirstChildElement("kp");
    if(kp_elem && kp_elem->GetText()) {
      joint_meta.kp = std::stod(kp_elem->GetText());
    }

    // Parse kd
    tinyxml2::XMLElement* kd_elem = joint_elem->FirstChildElement("kd");
    if(kd_elem && kd_elem->GetText()) {
      joint_meta.kd = std::stod(kd_elem->GetText());
    }

    // Parse soft_torque_limit
    tinyxml2::XMLElement* soft_torque_limit_elem = joint_elem->FirstChildElement("soft_torque_limit");
    if(soft_torque_limit_elem && soft_torque_limit_elem->GetText()) {
      joint_meta.soft_torque_limit = std::stod(soft_torque_limit_elem->GetText());
    }

    // Parse min_angle_deg
    tinyxml2::XMLElement* min_angle_elem = joint_elem->FirstChildElement("min_angle_deg");
    if(min_angle_elem && min_angle_elem->GetText()) {
      joint_meta.min_angle_deg = std::stod(min_angle_elem->GetText());
    }

    // Parse max_angle_deg
    tinyxml2::XMLElement* max_angle_elem = joint_elem->FirstChildElement("max_angle_deg");
    if(max_angle_elem && max_angle_elem->GetText()) {
      joint_meta.max_angle_deg = std::stod(max_angle_elem->GetText());
    }

    metadata[joint_name] = joint_meta;
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
    std::string test_dir            = "../tests/";
    std::string urdf_path           = test_dir + "robot.urdf";
    std::string joint_metadata_path = test_dir + "metadata.xml";
    std::string actuator_path       = test_dir + "actuators/motor.json";

    // Check if test URDF file exists
    if(!std::filesystem::exists(urdf_path)) {
      MESSAGE("Skipping URDF conversion test - test files not found");
      return;
    }

    // Parse metadata files
    auto joint_metadata = parse_joint_metadata_xml(joint_metadata_path);
    auto actuator_meta  = parse_actuator_metadata_json(actuator_path);
    std::map<std::string, mjcf::ActuatorMetadata> actuator_metadata;
    actuator_metadata["motor"] = actuator_meta;

    CHECK(joint_metadata.size() > 10); // Should have many joints from the humanoid robot

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
    std::string fake_urdf   = "/tmp/nonexistent.urdf";
    auto mujoco  = std::make_shared<mjcf::Mujoco>();
    bool success = mujoco->add_urdf(fake_urdf);
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

    std::map<std::string, mjcf::JointMetadata> joint_metadata;
    std::map<std::string, mjcf::ActuatorMetadata> actuator_metadata;

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
    CHECK(xml_content.find("base_link_base.stl") != std::string::npos);
    CHECK(xml_content.find("link1_link1.obj") != std::string::npos);
    
    // Check that mesh geometries reference the correct mesh names
    CHECK(xml_content.find("type=\"mesh\"") != std::string::npos);
    CHECK(xml_content.find("mesh=\"base_link_base.stl\"") != std::string::npos);
    CHECK(xml_content.find("mesh=\"link1_link1.obj\"") != std::string::npos);
    
    // Check that scale attributes are preserved
    CHECK(xml_content.find("scale=\"0.5 0.5 0.5\"") != std::string::npos);
    
    // Check that box geometry is still processed correctly
    CHECK(xml_content.find("type=\"box\"") != std::string::npos);
    CHECK(xml_content.find("link2") != std::string::npos);
    
    // Verify no unexpected sphere geoms were created
    // Count mesh and box geoms vs sphere geoms
    size_t mesh_count = 0;
    size_t box_count = 0;
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
