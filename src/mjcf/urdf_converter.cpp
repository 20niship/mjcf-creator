#include "urdf_converter.hpp"
#include "../ext/tinyxml2.h"
#include "actuator_elements.hpp"
#include "asset_elements.hpp"
#include "body_elements.hpp"
#include "core_elements.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <set>
#include <sstream>

namespace mjcf {

using namespace detail;

using namespace tinyxml2;

inline std::string read_file(const std::string& filepath) {
  std::ifstream file(filepath);
  if(!file.is_open()) {
    throw std::runtime_error("Cannot open file: " + filepath);
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

// Helper function to parse space-separated values
std::vector<double> parse_space_separated_values(const std::string& str) {
  std::vector<double> values;
  std::stringstream ss(str);
  std::string token;
  while(std::getline(ss, token, ' ')) {
    if(!token.empty()) {
      values.push_back(std::stod(token));
    }
  }
  return values;
}

/**
 * これはTanaka-sanのコードを参考にした
 */
std::vector<double> rpy_to_quat(const std::vector<double>& rpy) {
  if(rpy.size() != 3) {
    return {1.0, 0.0, 0.0, 0.0};
  }

  double roll  = rpy[0];
  double pitch = rpy[1];
  double yaw   = rpy[2];
  double cy    = cos(yaw * 0.5);
  double sy    = sin(yaw * 0.5);
  double cp    = cos(pitch * 0.5);
  double sp    = sin(pitch * 0.5);
  double cr    = cos(roll * 0.5);
  double sr    = sin(roll * 0.5);
  double w     = cr * cp * cy + sr * sp * sy;
  double x     = sr * cp * cy - cr * sp * sy;
  double y     = cr * sp * cy + sr * cp * sy;
  double z     = cr * cp * sy - sr * sp * cy;
  return {w, x, y, z};
}

bool UrdfConverter::parse_urdf_to_mjcf(Mujoco* mujoco, const std::string& urdf_path, const std::map<std::string, JointMetadata>& joint_metadata, const std::map<std::string, ActuatorMetadata>& actuator_metadata, bool copy_meshes, const std::string& output_dir) {

  const std::string urdf_content = read_file(urdf_path);
  XMLDocument doc;
  if(doc.Parse(urdf_content.c_str()) != XML_SUCCESS) {
    printf("Failed to parse URDF file: %s\n", urdf_path.c_str());
    return false;
  }

  XMLElement* robot = doc.FirstChildElement("robot");
  if(!robot) {
    std::cerr << "No <robot> element found in URDF file: " << urdf_path << std::endl;
    printf("[mjcf::parse_urdf_to_mjcf] No <robot> element found in urdf file %s \n", urdf_path.c_str());
    return false;
  }

  // Get robot name
  const char* robot_name = robot->Attribute("name");
  std::string model_name = robot_name ? robot_name : "converted_robot";

  for(XMLElement* material = robot->FirstChildElement("material"); material; material = material->NextSiblingElement("material")) {
    const char* mat_name = material->Attribute("name");
    if(!mat_name) continue;
    if(std::string(mat_name) == "") continue;
    if(mujoco->has_material(mat_name)) continue;

    auto mjcf_material  = std::make_shared<Material>();
    mjcf_material->name = mat_name;

    XMLElement* color = material->FirstChildElement("color");
    if(color) {
      const char* rgba = color->Attribute("rgba");
      if(rgba) {
        auto rgba_values = parse_space_separated_values(rgba);
        if(rgba_values.size() >= 3) {
          mjcf_material->rgba = {rgba_values[0], rgba_values[1], rgba_values[2], rgba_values.size() > 3 ? rgba_values[3] : 1.0};
        }
      }
    }
    mujoco->add_asset(mjcf_material);
  }

  auto worldbody = mujoco->worldbody_;

  std::map<std::string, std::shared_ptr<Body>> link_to_body;

  for(XMLElement* link = robot->FirstChildElement("link"); link; link = link->NextSiblingElement("link")) {
    const char* link_name = link->Attribute("name");
    if(!link_name) continue;

    auto body  = std::make_shared<Body>();
    body->name = link_name;

    // Process inertial properties
    XMLElement* inertial = link->FirstChildElement("inertial");
    if(inertial) {
      auto inertial_elem = std::make_shared<Inertial>();

      XMLElement* mass_elem = inertial->FirstChildElement("mass");
      if(mass_elem) {
        double mass         = mass_elem->DoubleAttribute("value");
        inertial_elem->mass = mass;
      }

      XMLElement* origin = inertial->FirstChildElement("origin");
      if(origin) {
        const char* xyz = origin->Attribute("xyz");
        if(xyz) {
          auto pos = parse_space_separated_values(xyz);
          if(pos.size() >= 3) inertial_elem->pos = {pos[0], pos[1], pos[2]};
        }
      }
      body->add_child(inertial_elem);
    }

    for(XMLElement* visual = link->FirstChildElement("visual"); visual; visual = visual->NextSiblingElement("visual")) {
      auto geom = std::make_shared<Geom>();

      XMLElement* origin = visual->FirstChildElement("origin");
      if(origin) {
        const char* xyz = origin->Attribute("xyz");
        if(xyz) {
          auto pos = parse_space_separated_values(xyz);
          if(pos.size() >= 3) geom->pos = {pos[0], pos[1], pos[2]};
        }

        const char* rpy = origin->Attribute("rpy");
        if(rpy) {
          auto rpy_values = parse_space_separated_values(rpy);
          if(rpy_values.size() >= 3) {
            auto quat = rpy_to_quat(rpy_values);
            if(quat.size() >= 4) geom->quat = {quat[0], quat[1], quat[2], quat[3]};
          }
        }
      }

      XMLElement* geometry = visual->FirstChildElement("geometry");
      if(geometry) {
        XMLElement* box      = geometry->FirstChildElement("box");
        XMLElement* cylinder = geometry->FirstChildElement("cylinder");
        XMLElement* sphere   = geometry->FirstChildElement("sphere");
        XMLElement* mesh     = geometry->FirstChildElement("mesh");

        bool geometry_found = false;

        if(box) {
          geom->type       = GeomType::Box;
          const char* size = box->Attribute("size");
          if(size) {
            auto sizes = parse_space_separated_values(size);
            if(sizes.size() >= 3) {
              geom->size = {sizes[0] / 2, sizes[1] / 2, sizes[2] / 2}; // MJCF uses half-sizes
            }
          }
          geometry_found = true;
        } else if(cylinder) {
          geom->type     = GeomType::Cylinder;
          double radius  = cylinder->DoubleAttribute("radius");
          double length  = cylinder->DoubleAttribute("length");
          geom->size     = {radius, length / 2, 0.001}; // MJCF uses half-length
          geometry_found = true;
        } else if(sphere) {
          geom->type     = GeomType::Sphere;
          double radius  = sphere->DoubleAttribute("radius");
          geom->size     = {radius, 0.001, 0.001};
          geometry_found = true;
        } else if(mesh) {
          geom->type = GeomType::Mesh;

          const char* filename = mesh->Attribute("filename");
          if(filename) {
            std::string mesh_filename   = filename;
            std::string final_mesh_path = mesh_filename;

            // Remove ROS package:// prefix if present
            const std::string package_prefix = "package://";
            if(mesh_filename.find(package_prefix) == 0) {
              mesh_filename = mesh_filename.substr(package_prefix.length());
            }

            if(copy_meshes) {
              // Resolve the full path to the mesh file
              std::string source_mesh_path = resolve_mesh_path(urdf_path, mesh_filename);

              if(std::filesystem::exists(source_mesh_path)) {
                // Generate hash-based filename
                std::string hash_filename  = generate_hash_filename(source_mesh_path);
                std::string dest_mesh_path = (output_dir.empty() ? "./" : (output_dir + "/")) + hash_filename;

                // Copy the mesh file
                if(copy_mesh_file(source_mesh_path, dest_mesh_path)) {
                  final_mesh_path = hash_filename; // Use the hash-based filename
                } else {
                  final_mesh_path = mesh_filename; // Fallback to original
                }
              } else {
                std::cerr << "Mesh file not found: " << source_mesh_path << std::endl;
                final_mesh_path = mesh_filename; // Fallback to original
              }
            } else {
              // Extract just the filename after the last slash
              size_t last_slash = mesh_filename.find_last_of("/\\");
              if(last_slash != std::string::npos) {
                mesh_filename = mesh_filename.substr(last_slash + 1);
              }
              final_mesh_path = mesh_filename;
            }

            // Create a unique mesh name based on link name and mesh filename
            std::string mesh_name = std::string(link_name) + "_" + std::filesystem::path(final_mesh_path).stem().string();

            // Create mesh asset
            auto mesh_asset  = std::make_shared<Mesh>();
            mesh_asset->name = mesh_name;
            mesh_asset->file = final_mesh_path;

            // Parse scale attribute if present
            const char* scale_attr = mesh->Attribute("scale");
            if(scale_attr) {
              auto scale_values = parse_space_separated_values(scale_attr);
              if(scale_values.size() >= 3) {
                mesh_asset->scale = {scale_values[0], scale_values[1], scale_values[2]};
              } else if(scale_values.size() == 1) {
                // Uniform scale
                mesh_asset->scale = {scale_values[0], scale_values[0], scale_values[0]};
              }
            }

            mujoco->add_asset(mesh_asset);
            geom->mesh = mesh_name;
          }
          geometry_found = true;
        }

        // Only add the geom if a valid geometry type was found
        if(geometry_found) {
          body->add_child(geom);
        }
      } else {
        // No geometry element found, don't add the geom
        continue;
      }

      XMLElement* material = visual->FirstChildElement("material");
      if(material) {
        const char* mat_name = material->Attribute("name");
        if(mat_name == nullptr) continue;
        if(std::string(mat_name) == "") continue;
        geom->material = mat_name;

        if(mujoco->has_material(mat_name)) continue;
        XMLElement* color = material->FirstChildElement("color");
        if(color == nullptr) continue;
        const char* rgba = color->Attribute("rgba");
        if(rgba == nullptr) continue;
        auto mjcf_material  = std::make_shared<Material>();
        mjcf_material->name = mat_name;

        auto rgba_values = parse_space_separated_values(rgba);
        if(rgba_values.size() >= 3) //
          mjcf_material->rgba = {rgba_values[0], rgba_values[1], rgba_values[2], rgba_values.size() > 3 ? rgba_values[3] : 1.0};
        mujoco->add_asset(mjcf_material);
      }
    }
#if 0
    for(XMLElement* collision = link->FirstChildElement("collision"); collision; collision = collision->NextSiblingElement("collision")) {
      auto geom         = std::make_shared<Geom>();
      geom->contype     = 1;
      geom->conaffinity = 1;
      XMLElement* origin = collision->FirstChildElement("origin");
      if(origin) {
        const char* xyz = origin->Attribute("xyz");
        if(xyz) {
          auto pos = parse_space_separated_values(xyz);
          if(pos.size() >= 3) {
            geom->pos = {pos[0], pos[1], pos[2]};
          }
        }
      }
      XMLElement* geometry = collision->FirstChildElement("geometry");
      if(geometry) {
        XMLElement* box = geometry->FirstChildElement("box");
        if(box) {
          geom->type       = GeomType::Box;
          const char* size = box->Attribute("size");
          if(size) {
            auto sizes = parse_space_separated_values(size);
            if(sizes.size() >= 3) {
              geom->size = {sizes[0] / 2, sizes[1] / 2, sizes[2] / 2};
            }
          }
        }
      }
      body->add_child(geom);
    }
#endif
    link_to_body[link_name] = body;
  }

  std::set<std::string> child_links;
  for(XMLElement* joint = robot->FirstChildElement("joint"); joint; joint = joint->NextSiblingElement("joint")) {
    XMLElement* child = joint->FirstChildElement("child");
    if(child) {
      const char* child_link = child->Attribute("link");
      if(child_link) {
        child_links.insert(child_link);
      }
    }
  }

  // Add root body to worldbody
  for(const auto& [link_name, body] : link_to_body) {
    if(child_links.find(link_name) == child_links.end()) {
      // This is a root link
      worldbody->add_child(body);
      break; // For simplicity, take the first root link
    }
  }

  // Process joints and create parent-child relationships
  for(XMLElement* joint = robot->FirstChildElement("joint"); joint != nullptr; joint = joint->NextSiblingElement("joint")) {
    const char* joint_name  = joint->Attribute("name");
    const char* joint_type_ = joint->Attribute("type");
    if(joint_name == nullptr || joint_type_ == nullptr) continue;
    const std::string joint_type = joint_type_;

    XMLElement* parent = joint->FirstChildElement("parent");
    XMLElement* child  = joint->FirstChildElement("child");
    if(!parent || !child) continue;

    const char* parent_link = parent->Attribute("link");
    const char* child_link  = child->Attribute("link");
    if(!parent_link || !child_link) continue;

    auto parent_body = link_to_body[parent_link];
    auto child_body  = link_to_body[child_link];
    if(!parent_body || !child_body) continue;

    // Set joint position and orientation
    XMLElement* origin = joint->FirstChildElement("origin");
    if(origin) {
      const char* xyz = origin->Attribute("xyz");
      if(xyz) {
        auto pos = parse_space_separated_values(xyz);
        if(pos.size() >= 3) child_body->pos = {pos[0], pos[1], pos[2]};
      }

      const char* rpy = origin->Attribute("rpy");
      if(rpy) {
        auto rpy_values = parse_space_separated_values(rpy);
        if(rpy_values.size() >= 3) {
          auto quat = rpy_to_quat(rpy_values);
          if(quat.size() >= 4) {
            child_body->quat = {quat[0], quat[1], quat[2], quat[3]};
          }
        }
      }
    }

    parent_body->add_child(child_body);

    if(joint_type != "fixed") {
      auto mjcf_joint  = std::make_shared<Joint>();
      mjcf_joint->name = joint_name;

      if(joint_type == "revolute" || joint_type == "continuous") {
        mjcf_joint->type = JointType::Hinge;
      } else if(joint_type == "prismatic") {
        mjcf_joint->type = JointType::Slide;
      }

      XMLElement* axis = joint->FirstChildElement("axis");
      if(axis) {
        const char* xyz = axis->Attribute("xyz");
        if(xyz) {
          auto axis_values = parse_space_separated_values(xyz);
          if(axis_values.size() >= 3) mjcf_joint->axis = {axis_values[0], axis_values[1], axis_values[2]};
        }
      }

      XMLElement* limit = joint->FirstChildElement("limit");
      if(limit) {
        double lower      = limit->DoubleAttribute("lower");
        double upper      = limit->DoubleAttribute("upper");
        mjcf_joint->range = {lower, upper};
      }

      child_body->add_child(mjcf_joint);

      if(mjcf_joint->type == JointType::Hinge) {
        auto ac         = Position::Create(joint_name);
        ac->name        = joint_name;
        ac->ctrllimited = false;
        ac->kp          = 100.0;
        ac->kv          = 10.0;
        // ac->gear        = {100, 0, 0, 0, 0, 0};
        mujoco->actuator_->add_child(ac);
      }
    }
  }
  return true;
}

void UrdfConverter::merge_asset_elements(const Asset& source_asset, Asset& target_asset, const std::string& name_prefix) {
  auto add_prefix = [&name_prefix](const std::string& name) -> std::string {
    if(name_prefix.empty()) {
      return name;
    }
    return name_prefix + "_" + name;
  };

  // Get existing names to avoid conflicts
  std::set<std::string> existing_names;
  for(const auto& child : target_asset.get_children()) {
    if(auto named_elem = std::dynamic_pointer_cast<Material>(child)) {
      if(auto name_attr = named_elem->get_attribute_public("name")) {
        existing_names.insert(std::get<std::string>(*name_attr));
      }
    } else if(auto tex_elem = std::dynamic_pointer_cast<Texture>(child)) {
      if(auto name_attr = tex_elem->get_attribute_public("name")) {
        existing_names.insert(std::get<std::string>(*name_attr));
      }
    }
  }

  // Clone and add asset children with prefix
  for(const auto& child : source_asset.get_children()) {
    auto cloned_child = clone_element_with_prefix(child, name_prefix, existing_names);
    if(cloned_child) {
      target_asset.add_child(cloned_child);
    }
  }
}

void UrdfConverter::merge_worldbody_elements(const Worldbody& source_worldbody, Worldbody& target_worldbody, const std::string& name_prefix) {
  // Skip default elements (light, floor) and only merge robot bodies
  for(const auto& child : source_worldbody.get_children()) {
    // Skip light and floor elements
    if(auto geom = std::dynamic_pointer_cast<Geom>(child)) {
      if(auto name_attr = geom->get_attribute_public("name")) {
        std::string name = std::get<std::string>(*name_attr);
        if(name == "floor" || name.find("floor") != std::string::npos) {
          continue; // Skip floor
        }
      }
    } else if(auto light = std::dynamic_pointer_cast<Light>(child)) {
      continue; // Skip lights
    }

    // Clone and add non-default elements with prefix
    std::set<std::string> existing_names; // Could be enhanced to track existing names
    auto cloned_child = clone_element_with_prefix(child, name_prefix, existing_names);
    if(cloned_child) {
      target_worldbody.add_child(cloned_child);
    }
  }
}

std::shared_ptr<Element> UrdfConverter::clone_element_with_prefix(const std::shared_ptr<Element>& element, const std::string& name_prefix, const std::set<std::string>& existing_names) {
  // This is a simplified cloning approach
  // In a full implementation, you'd want a proper clone method for each element type

  if(auto material = std::dynamic_pointer_cast<Material>(element)) {
    auto cloned = std::make_shared<Material>();
    clone_element_attributes(*material, *cloned, name_prefix);
    return cloned;
  } else if(auto texture = std::dynamic_pointer_cast<Texture>(element)) {
    auto cloned = std::make_shared<Texture>();
    clone_element_attributes(*texture, *cloned, name_prefix);
    return cloned;
  } else if(auto body = std::dynamic_pointer_cast<Body>(element)) {
    auto cloned = std::make_shared<Body>();
    clone_element_attributes(*body, *cloned, name_prefix);

    // Recursively clone children
    for(const auto& child : body->get_children()) {
      auto cloned_child = clone_element_with_prefix(child, name_prefix, existing_names);
      if(cloned_child) {
        cloned->add_child(cloned_child);
      }
    }
    return cloned;
  } else if(auto geom = std::dynamic_pointer_cast<Geom>(element)) {
    auto cloned = std::make_shared<Geom>();
    clone_element_attributes(*geom, *cloned, name_prefix);
    return cloned;
  } else if(auto joint = std::dynamic_pointer_cast<Joint>(element)) {
    auto cloned = std::make_shared<Joint>();
    clone_element_attributes(*joint, *cloned, name_prefix);
    return cloned;
  } else if(auto inertial = std::dynamic_pointer_cast<Inertial>(element)) {
    auto cloned = std::make_shared<Inertial>();
    clone_element_attributes(*inertial, *cloned, ""); // Don't prefix inertial
    return cloned;
  }

  // For unknown types, return nullptr
  return nullptr;
}

void UrdfConverter::clone_element_attributes(const Element& source, Element& target, const std::string& name_prefix) {
  // This is a simplified attribute cloning - would need proper implementation
  // For now, we'll need to access attributes directly (this is a limitation)
  // In a full implementation, Element would need a method to get all attributes

  // Add prefix to name attribute if it exists and prefix is provided
  auto add_prefix = [&name_prefix](const std::string& name) -> std::string {
    if(name_prefix.empty()) {
      return name;
    }
    return name_prefix + "_" + name;
  };

  // This is a placeholder - in practice you'd need to implement proper attribute copying
  // For the specific case of name attributes, we handle this in the concrete implementations

  // For now, we'll handle the most common case manually
  if(auto name_attr = source.get_attribute_public("name")) {
    std::string original_name = std::get<std::string>(*name_attr);
    target.set_attribute_public("name", add_prefix(original_name));
  }
}

std::string UrdfConverter::generate_hash_filename(const std::string& original_path) {
  // Get the parent directory path for hashing
  std::filesystem::path path(original_path);
  std::string parent_dir = path.parent_path().string();

  // Generate hash of the parent directory path
  std::hash<std::string> hasher;
  size_t hash_value = hasher(parent_dir);

  // Convert hash to hex string and take first 5 characters
  std::stringstream ss;
  ss << std::hex << hash_value;
  std::string hash_str = ss.str();
  if(hash_str.length() > 5) {
    hash_str = hash_str.substr(0, 5);
  }

  // Get original filename and extension
  std::string filename = path.filename().string();

  // Combine hash prefix with original filename
  return hash_str + "_" + filename;
}

bool UrdfConverter::copy_mesh_file(const std::string& source_path, const std::string& dest_path) {
  try {
    // Create output directory if it doesn't exist
    std::filesystem::path dest_file_path(dest_path);
    std::filesystem::create_directories(dest_file_path.parent_path());

    std::filesystem::copy_file(source_path, dest_path, std::filesystem::copy_options::overwrite_existing);
    return true;
  } catch(const std::filesystem::filesystem_error& e) {
    std::cerr << "Filesystem error: " << e.what() << std::endl;
    return false;
  } catch(const std::exception& e) {
    std::cerr << "Error copying file: " << e.what() << std::endl;
    return false;
  }
}

std::string UrdfConverter::resolve_mesh_path(const std::string& urdf_path, const std::string& mesh_filename) {
  std::filesystem::path urdf_dir = std::filesystem::path(urdf_path).parent_path();
  std::string mesh_path          = mesh_filename;

  // Remove ROS package:// prefix if present
  const std::string package_prefix = "package://";
  if(mesh_path.find(package_prefix) == 0) {
    mesh_path = mesh_path.substr(package_prefix.length());
  }

  // If it's a relative path, make it relative to URDF directory
  std::filesystem::path full_mesh_path;
  if(std::filesystem::path(mesh_path).is_absolute()) {
    full_mesh_path = mesh_path;
  } else {
    full_mesh_path = urdf_dir / mesh_path;
  }

  return full_mesh_path.string();
}

} // namespace mjcf
