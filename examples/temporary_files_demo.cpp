/**
 * @file temporary_files_demo.cpp
 * @brief Example demonstrating temporary file tracking functionality
 *
 * This example shows how to use the new temporary file tracking feature
 * to manage temporary mesh files created during URDF to MJCF conversion.
 */

#include "mjcf/mjcf.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>

int main() {
  std::cout << "=== Temporary File Tracking Demo ===" << std::endl << std::endl;

  // Create a simple test URDF with mesh reference
  std::string test_urdf_content = R"(<?xml version="1.0"?>
<robot name="demo_robot">
  <material name="red">
    <color rgba="1.0 0.0 0.0 1.0"/>
  </material>
  
  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="demo_mesh.stl"/>
      </geometry>
      <material name="red"/>
    </visual>
  </link>
</robot>)";

  // Create temporary test files
  std::filesystem::path test_dir = std::filesystem::temp_directory_path() / "mjcf_demo";
  std::filesystem::create_directories(test_dir);
  
  std::string urdf_path = (test_dir / "demo_robot.urdf").string();
  std::string mesh_path = (test_dir / "demo_mesh.stl").string();
  
  // Write test URDF
  {
    std::ofstream urdf_file(urdf_path);
    urdf_file << test_urdf_content;
  }
  
  // Write dummy mesh file
  {
    std::ofstream mesh_file(mesh_path);
    mesh_file << "STL mesh content would go here...";
  }

  std::cout << "Created test URDF and mesh files in: " << test_dir << std::endl;

  // Example 1: Without copy_meshes (no temporary files)
  std::cout << "\n--- Example 1: add_urdf without copy_meshes ---" << std::endl;
  {
    auto mujoco = std::make_shared<mjcf::Mujoco>("demo_model");
    mujoco->add_urdf(urdf_path, "", false);
    
    const auto& temp_files = mujoco->get_temporary_files();
    std::cout << "Temporary files created: " << temp_files.size() << std::endl;
    std::cout << "No temporary files were created (copy_meshes=false)" << std::endl;
  }

  // Example 2: With copy_meshes (creates temporary files)
  std::cout << "\n--- Example 2: add_urdf with copy_meshes ---" << std::endl;
  {
    auto mujoco = std::make_shared<mjcf::Mujoco>("demo_model");
    mujoco->add_urdf(urdf_path, "", true);
    
    const auto& temp_files = mujoco->get_temporary_files();
    std::cout << "Temporary files created: " << temp_files.size() << std::endl;
    
    for (const auto& file : temp_files) {
      std::cout << "  - " << file << std::endl;
      if (std::filesystem::exists(file)) {
        std::cout << "    [Exists, size: " << std::filesystem::file_size(file) << " bytes]" << std::endl;
      }
    }
  }

  // Example 3: Multiple URDF imports with cleanup
  std::cout << "\n--- Example 3: Multiple URDF imports with cleanup ---" << std::endl;
  {
    auto mujoco = std::make_shared<mjcf::Mujoco>("multi_robot_scene");
    
    // Add first robot
    std::cout << "Adding first robot with copy_meshes..." << std::endl;
    mujoco->add_urdf(urdf_path, "robot1", true);
    size_t count_after_first = mujoco->get_temporary_files().size();
    std::cout << "  Temporary files after first add: " << count_after_first << std::endl;
    
    // Add second robot (simulating another URDF)
    std::cout << "Adding second robot with copy_meshes..." << std::endl;
    mujoco->add_urdf(urdf_path, "robot2", true);
    size_t count_after_second = mujoco->get_temporary_files().size();
    std::cout << "  Temporary files after second add: " << count_after_second << std::endl;
    
    // Generate MJCF
    std::string mjcf_output = mujoco->get_xml_text();
    std::cout << "\nGenerated MJCF (size: " << mjcf_output.size() << " bytes)" << std::endl;
    
    // At this point, you would load the MJCF into MuJoCo
    std::cout << "\n[Simulating: MJCF loaded into MuJoCo...]" << std::endl;
    
    // Now clean up the temporary files
    std::cout << "Cleaning up temporary files..." << std::endl;
    size_t deleted_count = mujoco->clear_temporary_files();
    std::cout << "  Successfully deleted " << deleted_count << " temporary file(s)" << std::endl;
    
    // Verify cleanup
    size_t remaining = mujoco->get_temporary_files().size();
    std::cout << "  Remaining temporary files: " << remaining << std::endl;
  }

  // Example 4: Manual temporary file tracking
  std::cout << "\n--- Example 4: Manual temporary file tracking ---" << std::endl;
  {
    auto mujoco = std::make_shared<mjcf::Mujoco>("custom_model");
    
    // You can manually track files if you create them yourself
    std::string custom_temp = (test_dir / "my_custom_temp_file.txt").string();
    {
      std::ofstream temp_file(custom_temp);
      temp_file << "This is a manually tracked temporary file";
    }
    
    // Add it to the tracking list
    mujoco->add_temporary_file(custom_temp);
    std::cout << "Added custom temporary file: " << custom_temp << std::endl;
    
    const auto& temp_files = mujoco->get_temporary_files();
    std::cout << "Total tracked files: " << temp_files.size() << std::endl;
    
    // Clean up
    size_t deleted = mujoco->clear_temporary_files();
    std::cout << "Deleted " << deleted << " file(s)" << std::endl;
  }

  // Clean up test directory
  std::cout << "\n--- Cleaning up test directory ---" << std::endl;
  std::filesystem::remove_all(test_dir);
  std::cout << "Test directory removed: " << test_dir << std::endl;

  std::cout << "\n=== Demo Complete ===" << std::endl;
  std::cout << "\nKey takeaways:" << std::endl;
  std::cout << "1. Use get_temporary_files() to retrieve list of temporary files" << std::endl;
  std::cout << "2. Use clear_temporary_files() to delete all tracked files" << std::endl;
  std::cout << "3. Files are automatically tracked when copy_meshes=true" << std::endl;
  std::cout << "4. You can manually track files with add_temporary_file()" << std::endl;

  return 0;
}
