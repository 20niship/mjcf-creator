#include "doctest.h"
#include "mjcf/mjcf.hpp"
#include <filesystem>
#include <fstream>

TEST_SUITE("Temporary File Tracking Tests") {
  TEST_CASE("Track temporary files from copy_meshes") {
    // Create a test URDF with a mesh reference
    std::string test_urdf_content = R"(<?xml version="1.0"?>
<robot name="test_robot">
  <material name="test_material">
    <color rgba="1.0 0.0 0.0 1.0"/>
  </material>
  
  <link name="test_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="test_mesh.stl"/>
      </geometry>
      <material name="test_material"/>
    </visual>
  </link>
</robot>)";

    // Create temporary test files
    std::filesystem::path test_dir = std::filesystem::temp_directory_path() / "mjcf_test_temp_files";
    std::filesystem::create_directories(test_dir);
    
    std::string urdf_path = (test_dir / "test_robot.urdf").string();
    std::string mesh_path = (test_dir / "test_mesh.stl").string();
    
    // Write test URDF
    {
      std::ofstream urdf_file(urdf_path);
      urdf_file << test_urdf_content;
    }
    
    // Write dummy mesh file
    {
      std::ofstream mesh_file(mesh_path);
      mesh_file << "dummy mesh content";
    }

    SUBCASE("get_temporary_files returns empty list initially") {
      auto mujoco = std::make_shared<mjcf::Mujoco>("test_model");
      CHECK(mujoco->get_temporary_files().empty());
    }

    SUBCASE("add_urdf without copy_meshes doesn't track files") {
      auto mujoco = std::make_shared<mjcf::Mujoco>("test_model");
      auto [body, joint] = mujoco->add_urdf(urdf_path);
      CHECK(body != nullptr);
      CHECK(joint != nullptr);
      CHECK(mujoco->get_temporary_files().empty());
    }

    SUBCASE("add_urdf with copy_meshes tracks copied files") {
      auto mujoco = std::make_shared<mjcf::Mujoco>("test_model");
      auto [body, joint] = mujoco->add_urdf(urdf_path);
      CHECK(body != nullptr);
      CHECK(joint != nullptr);
      
      const auto& temp_files = mujoco->get_temporary_files();
      CHECK_FALSE(temp_files.empty());
      
      // Check that tracked files actually exist
      for (const auto& file : temp_files) {
        CHECK(std::filesystem::exists(file));
      }
    }

    SUBCASE("clear_temporary_files deletes tracked files") {
      auto mujoco = std::make_shared<mjcf::Mujoco>("test_model");
      auto [body, joint] = mujoco->add_urdf(urdf_path, "", true);
      CHECK(body != nullptr);
      CHECK(joint != nullptr);
      
      const auto& temp_files = mujoco->get_temporary_files();
      CHECK_FALSE(temp_files.empty());
      
      // Store paths to check later
      std::vector<std::string> files_to_check = temp_files;
      
      // Clear temporary files
      size_t deleted_count = mujoco->clear_temporary_files();
      CHECK(deleted_count > 0);
      CHECK(deleted_count == files_to_check.size());
      
      // Check that list is now empty
      CHECK(mujoco->get_temporary_files().empty());
      
      // Check that files were actually deleted
      for (const auto& file : files_to_check) {
        CHECK_FALSE(std::filesystem::exists(file));
      }
    }

    SUBCASE("add_temporary_file manually tracks files") {
      auto mujoco = std::make_shared<mjcf::Mujoco>("test_model");
      
      std::string manual_temp_file = (test_dir / "manual_temp.txt").string();
      {
        std::ofstream temp_file(manual_temp_file);
        temp_file << "manual temporary file";
      }
      
      mujoco->add_temporary_file(manual_temp_file);
      
      const auto& temp_files = mujoco->get_temporary_files();
      CHECK(temp_files.size() == 1);
      CHECK(temp_files[0] == manual_temp_file);
      
      // Clear it
      size_t deleted = mujoco->clear_temporary_files();
      CHECK(deleted == 1);
      CHECK_FALSE(std::filesystem::exists(manual_temp_file));
    }

    SUBCASE("multiple add_urdf calls accumulate temporary files") {
      auto mujoco = std::make_shared<mjcf::Mujoco>("test_model");
      
      // Add URDF twice with copy_meshes
      auto [body1, joint1] = mujoco->add_urdf(urdf_path, "robot1", true);
      CHECK(body1 != nullptr);
      CHECK(joint1 != nullptr);
      size_t count_after_first = mujoco->get_temporary_files().size();
      CHECK(count_after_first > 0);
      
      auto [body2, joint2] = mujoco->add_urdf(urdf_path, "robot2", true);
      CHECK(body2 != nullptr);
      CHECK(joint2 != nullptr);
      size_t count_after_second = mujoco->get_temporary_files().size();
      CHECK(count_after_second >= count_after_first);
      
      // Clean up
      mujoco->clear_temporary_files();
    }

    // Cleanup test directory
    std::filesystem::remove_all(test_dir);
  }

  TEST_CASE("clear_temporary_files handles non-existent files gracefully") {
    auto mujoco = std::make_shared<mjcf::Mujoco>("test_model");
    
    // Add a file path that doesn't exist
    std::filesystem::path temp_dir = std::filesystem::temp_directory_path();
    mujoco->add_temporary_file((temp_dir / "nonexistent_file_12345.xyz").string());
    
    // This should not crash
    size_t deleted = mujoco->clear_temporary_files();
    CHECK(deleted == 0); // File didn't exist, so couldn't delete it
    CHECK(mujoco->get_temporary_files().empty()); // List should still be cleared
  }
}
