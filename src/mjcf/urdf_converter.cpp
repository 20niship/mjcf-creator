#include "urdf_converter.hpp"
#include "../ext/tinyxml2.h"
#include "asset_elements.hpp"
#include "body_elements.hpp"
#include "core_elements.hpp"
#include <cmath>
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

std::vector<double> rpy_to_quat(const std::vector<double>& rpy) {
  if(rpy.size() != 3) {
    return {1.0, 0.0, 0.0, 0.0};
  }

  double roll  = rpy[0] / 2.0;
  double pitch = rpy[1] / 2.0;
  double yaw   = rpy[2] / 2.0;

  double cr = cos(roll);
  double sr = sin(roll);
  double cp = cos(pitch);
  double sp = sin(pitch);
  double cy = cos(yaw);
  double sy = sin(yaw);

  double w = cr * cp * cy + sr * sp * sy;
  double x = sr * cp * cy - cr * sp * sy;
  double y = cr * sp * cy + sr * cp * sy;
  double z = cr * cp * sy - sr * sp * cy;

  return {w, x, y, z};
}

JointMetadata JointMetadata::from_json(const std::string& json_str) {
  JointMetadata metadata;
  metadata.actuator_type     = "motor";
  metadata.nn_id             = 0;
  metadata.id                = 0;
  metadata.kp                = 40.0;
  metadata.kd                = 2.0;
  metadata.soft_torque_limit = 10.0;
  metadata.min_angle_deg     = 0.0;
  metadata.max_angle_deg     = 0.0;
  return metadata;
}

ActuatorMetadata ActuatorMetadata::from_json(const std::string& json_str) {
  ActuatorMetadata metadata;
  metadata.actuator_type = "motor";
  metadata.sysid         = "";
  metadata.max_torque    = 10.0;
  metadata.max_velocity  = 5.0;
  metadata.armature      = 0.001;
  metadata.damping       = 0.1;
  metadata.frictionloss  = 0.1;
  return metadata;
}

bool UrdfConverter::parse_urdf_to_mjcf(Mujoco* mujoco, const std::string& urdf_path, const std::map<std::string, JointMetadata>& joint_metadata, const std::map<std::string, ActuatorMetadata>& actuator_metadata) {
  const std::string urdf_content = read_file(urdf_path);
  XMLDocument doc;
  if(doc.Parse(urdf_content.c_str()) != XML_SUCCESS) {
    std::cerr << "Failed to parse URDF file: " << urdf_path << std::endl;
    return false;
  }

  XMLElement* robot = doc.FirstChildElement("robot");
  if(!robot) {
    std::cerr << "No <robot> element found in URDF file: " << urdf_path << std::endl;
    return false;
  }

  // Get robot name
  const char* robot_name = robot->Attribute("name");
  std::string model_name = robot_name ? robot_name : "converted_robot";

  // Add default textures and materials for visualization
  auto tex_grid     = std::make_shared<Texture>();
  tex_grid->name    = "grid";
  tex_grid->type    = TextureType::TwoD;
  tex_grid->builtin = TextureBuiltin::Checker;
  tex_grid->rgb1    = {0.1, 0.2, 0.3};
  tex_grid->rgb2    = {0.2, 0.3, 0.4};
  tex_grid->width   = 300;
  tex_grid->height  = 300;
  mujoco->add_asset(tex_grid);

  auto mat_grid         = std::make_shared<Material>();
  mat_grid->name        = "grid";
  mat_grid->texture     = "grid";
  mat_grid->reflectance = 0.2;
  mujoco->add_asset(mat_grid);

  for(XMLElement* material = robot->FirstChildElement("material"); material; material = material->NextSiblingElement("material")) {
    const char* mat_name = material->Attribute("name");
    if(!mat_name) continue;

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

  auto floor_geom      = std::make_shared<Geom>();
  floor_geom->name     = "floor";
  floor_geom->material = "grid";
  floor_geom->pos      = {0, 0, 0};
  floor_geom->size     = {40.0, 40.0, 40.0};
  floor_geom->type     = GeomType::Plane;
  worldbody->add_child(floor_geom);

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
          if(pos.size() >= 3) {
            inertial_elem->pos = {pos[0], pos[1], pos[2]};
          }
        }
      }

      body->add_child(inertial_elem);
    }

    // Process visual elements
    for(XMLElement* visual = link->FirstChildElement("visual"); visual; visual = visual->NextSiblingElement("visual")) {
      auto geom = std::make_shared<Geom>();

      XMLElement* origin = visual->FirstChildElement("origin");
      if(origin) {
        const char* xyz = origin->Attribute("xyz");
        if(xyz) {
          auto pos = parse_space_separated_values(xyz);
          if(pos.size() >= 3) {
            geom->pos = {pos[0], pos[1], pos[2]};
          }
        }

        const char* rpy = origin->Attribute("rpy");
        if(rpy) {
          auto rpy_values = parse_space_separated_values(rpy);
          if(rpy_values.size() >= 3) {
            auto quat = rpy_to_quat(rpy_values);
            if(quat.size() >= 4) {
              geom->quat = {quat[0], quat[1], quat[2], quat[3]};
            }
          }
        }
      }

      XMLElement* geometry = visual->FirstChildElement("geometry");
      if(geometry) {
        XMLElement* box      = geometry->FirstChildElement("box");
        XMLElement* cylinder = geometry->FirstChildElement("cylinder");
        XMLElement* sphere   = geometry->FirstChildElement("sphere");

        if(box) {
          geom->type       = GeomType::Box;
          const char* size = box->Attribute("size");
          if(size) {
            auto sizes = parse_space_separated_values(size);
            if(sizes.size() >= 3) {
              geom->size = {sizes[0] / 2, sizes[1] / 2, sizes[2] / 2}; // MJCF uses half-sizes
            }
          }
        } else if(cylinder) {
          geom->type    = GeomType::Cylinder;
          double radius = cylinder->DoubleAttribute("radius");
          double length = cylinder->DoubleAttribute("length");
          geom->size    = {radius, length / 2, 0}; // MJCF uses half-length
        } else if(sphere) {
          geom->type    = GeomType::Sphere;
          double radius = sphere->DoubleAttribute("radius");
          geom->size    = {radius, 0, 0};
        }
      }

      XMLElement* material = visual->FirstChildElement("material");
      if(material) {
        const char* mat_name = material->Attribute("name");
        if(mat_name) geom->material = mat_name;
      }
      body->add_child(geom);
    }

    // Process collision elements
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
    link_to_body[link_name] = body;
  }

  // Find root link (link without parent joint)
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
  for(XMLElement* joint = robot->FirstChildElement("joint"); joint; joint = joint->NextSiblingElement("joint")) {
    const char* joint_name = joint->Attribute("name");
    const char* joint_type = joint->Attribute("type");
    if(!joint_name || !joint_type) continue;

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
        if(pos.size() >= 3) {
          child_body->pos = {pos[0], pos[1], pos[2]};
        }
      }
    }

    // Add joint if not fixed
    if(std::string(joint_type) != "fixed") {
      auto mjcf_joint  = std::make_shared<Joint>();
      mjcf_joint->name = joint_name;

      if(std::string(joint_type) == "revolute" || std::string(joint_type) == "continuous") {
        mjcf_joint->type = JointType::Hinge;
      } else if(std::string(joint_type) == "prismatic") {
        mjcf_joint->type = JointType::Slide;
      }

      XMLElement* axis = joint->FirstChildElement("axis");
      if(axis) {
        const char* xyz = axis->Attribute("xyz");
        if(xyz) {
          auto axis_values = parse_space_separated_values(xyz);
          if(axis_values.size() >= 3) {
            mjcf_joint->axis = {axis_values[0], axis_values[1], axis_values[2]};
          }
        }
      }

      XMLElement* limit = joint->FirstChildElement("limit");
      if(limit) {
        double lower      = limit->DoubleAttribute("lower");
        double upper      = limit->DoubleAttribute("upper");
        mjcf_joint->range = {lower, upper};
      }

      child_body->add_child(mjcf_joint);
    }
    parent_body->add_child(child_body);
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

} // namespace mjcf
