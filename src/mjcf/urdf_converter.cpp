#include "urdf_converter.hpp"
#include "actuator_elements.hpp"
#include "asset_elements.hpp"
#include "body_elements.hpp"
#include "core_elements.hpp"
#include "sensor_elements.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <set>
#include <sstream>
#include <unordered_map>
#ifndef _WIN32
#include <unistd.h>  // getpid (並列プロセスで mesh コピー名を一意化するため)
#else
#include <process.h> // _getpid
#endif
#if __has_include(<tinyxml2/tinyxml2.h>)
#include <tinyxml2/tinyxml2.h>
#else
#include <tinyxml2.h>
#endif

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

template <typename T> using Shr = std::shared_ptr<T>;
using Str                       = std::string;

std::tuple<Shr<mjcf::Body>, Shr<mjcf::Joint>> UrdfConverter::parse_urdf_to_mjcf(Mujoco* mujoco, const Str& urdf_path, const Arr3& pos, const std::vector<Shr<BaseActuator>>& actuator_metadata, bool copy_meshes, const std::string& output_dir, bool use_collision_tag_only,
                                                                                const std::string& name_prefix) {
  auto add_pfx = [&name_prefix](const std::string& nm) -> std::string {
    if(name_prefix.empty() || nm.empty()) return nm;
    return name_prefix + "_" + nm;
  };
  const std::string urdf_content = read_file(urdf_path);
  XMLDocument doc;
  if(doc.Parse(urdf_content.c_str()) != XML_SUCCESS) {
    printf("Failed to parse URDF file: %s\n", urdf_path.c_str());
    return {nullptr, nullptr};
  }

  XMLElement* robot = doc.FirstChildElement("robot");
  if(!robot) {
    std::cerr << "No <robot> element found in URDF file: " << urdf_path << std::endl;
    printf("[mjcf::parse_urdf_to_mjcf] No <robot> element found in urdf file %s \n", urdf_path.c_str());
    return {nullptr, nullptr};
  }

  const char* robot_name = robot->Attribute("name");
  std::string model_name = (robot_name != nullptr) ? robot_name : "converted_robot";

  for(XMLElement* material = robot->FirstChildElement("material"); material; material = material->NextSiblingElement("material")) {
    const char* mat_name = material->Attribute("name");
    if(mat_name == nullptr) continue;
    if(std::string(mat_name) == "") continue;
    if(mujoco->has_material(mat_name)) continue;

    auto mjcf_material  = std::make_shared<Material>();
    mjcf_material->name = mat_name;

    XMLElement* color = material->FirstChildElement("color");
    if(color != nullptr) {
      const char* rgba = color->Attribute("rgba");
      if(rgba != nullptr) {
        auto rgba_values = parse_space_separated_values(rgba);
        if(rgba_values.size() >= 3) //
          mjcf_material->rgba = {rgba_values[0], rgba_values[1], rgba_values[2], rgba_values.size() > 3 ? rgba_values[3] : 1.0};
      }
    }
    mujoco->add_asset(mjcf_material);
  }

  auto worldbody = mujoco->worldbody_;

  // <gazebo reference="link_name"> からジオメトリ接触パラメータを収集
  struct GazeboGeomParams {
    double mu1                   = -1.0;
    double mu2                   = -1.0;
    int condim                   = -1; // -1 = 未指定 (デフォルト継承)
    int contype                  = -1; // -1 = 未指定 (デフォルト継承)
    int conaffinity              = -1; // -1 = 未指定 (デフォルト継承)
    std::array<double, 2> solref = {0.02, 1.0};
    std::array<double, 3> solimp = {0.9, 0.95, 0.001};
    bool has_solref() const { return solref[0] != 0.02 || solref[1] != 1.0; }
    bool has_solimp() const { return solimp[0] != 0.9 || solimp[1] != 0.95 || solimp[2] != 0.001; }
  };
  std::unordered_map<std::string, GazeboGeomParams> gazebo_geom_map;
  for(XMLElement* gazebo = robot->FirstChildElement("gazebo"); gazebo; gazebo = gazebo->NextSiblingElement("gazebo")) {
    const char* reference = gazebo->Attribute("reference");
    if(reference == nullptr) continue;

    GazeboGeomParams params;

    const auto mu1_elem = gazebo->FirstChildElement("mu1");
    const auto mu2_elem = gazebo->FirstChildElement("mu2");
    // 属性形式 `<mu1 value="1.5"/>` とテキストコンテント形式 `<mu1>1.5</mu1>` (Gazebo 標準) の双方をサポートする
    if(mu1_elem != nullptr) {
      params.mu1 = mu1_elem->DoubleAttribute("value", -1.0);
      if(params.mu1 < 0.0) params.mu1 = mu1_elem->DoubleText(-1.0);
    }
    if(mu2_elem != nullptr) {
      params.mu2 = mu2_elem->DoubleAttribute("value", -1.0);
      if(params.mu2 < 0.0) params.mu2 = mu2_elem->DoubleText(-1.0);
    }

    // solref: "timeconst ratio" の2値
    const auto solref_elem = gazebo->FirstChildElement("solref");
    if(solref_elem != nullptr && solref_elem->GetText()) {
      auto vals = parse_space_separated_values(solref_elem->GetText());
      if(vals.size() >= 2) params.solref = {vals[0], vals[1]};
    }

    // solimp: "dmin dmax width" の3値
    const auto solimp_elem = gazebo->FirstChildElement("solimp");
    if(solimp_elem != nullptr && solimp_elem->GetText()) {
      auto vals = parse_space_separated_values(solimp_elem->GetText());
      if(vals.size() >= 3) params.solimp = {vals[0], vals[1], vals[2]};
    }

    const auto condim_elem = gazebo->FirstChildElement("condim");
    if(condim_elem != nullptr && condim_elem->GetText()) {
      params.condim = std::stoi(condim_elem->GetText());
    }

    const auto contype_elem = gazebo->FirstChildElement("contype");
    if(contype_elem != nullptr && contype_elem->GetText()) {
      params.contype = std::stoi(contype_elem->GetText());
    }

    const auto conaffinity_elem = gazebo->FirstChildElement("conaffinity");
    if(conaffinity_elem != nullptr && conaffinity_elem->GetText()) {
      params.conaffinity = std::stoi(conaffinity_elem->GetText());
    }

    if(params.mu1 >= 0.0 || params.mu2 >= 0.0 || params.condim >= 0 || params.contype >= 0 || params.conaffinity >= 0 || params.has_solref() || params.has_solimp()) gazebo_geom_map[reference] = params;
  }

  // Gazeboプラグインからセンサー情報を収集
  struct UrdfSensorInfo {
    enum class Type { ForceTorque, IMU, Touch };
    Type type;
    std::string name;
    std::string joint_name; // FTセンサー: 対象joint名
    std::string link_name;  // IMU/Touch: 対象link名
  };
  std::vector<UrdfSensorInfo> detected_sensors;

  for(XMLElement* gazebo = robot->FirstChildElement("gazebo"); gazebo; gazebo = gazebo->NextSiblingElement("gazebo")) {
    // <gazebo><plugin ...> 形式
    for(XMLElement* plugin = gazebo->FirstChildElement("plugin"); plugin; plugin = plugin->NextSiblingElement("plugin")) {
      const char* filename = plugin->Attribute("filename");
      if(filename == nullptr) continue;
      std::string fn = filename;

      if(fn.find("ft_sensor") != std::string::npos) {
        XMLElement* joint_name_elem = plugin->FirstChildElement("jointName");
        if(joint_name_elem == nullptr || joint_name_elem->GetText() == nullptr) continue;
        const char* plugin_name = plugin->Attribute("name");
        detected_sensors.push_back({UrdfSensorInfo::Type::ForceTorque, plugin_name ? plugin_name : "ft_sensor", joint_name_elem->GetText(), ""});
      } else if(fn.find("imu") != std::string::npos) {
        XMLElement* frame_elem = plugin->FirstChildElement("frameName");
        if(frame_elem == nullptr || frame_elem->GetText() == nullptr) continue;
        const char* plugin_name = plugin->Attribute("name");
        detected_sensors.push_back({UrdfSensorInfo::Type::IMU, plugin_name ? plugin_name : "imu_sensor", "", frame_elem->GetText()});
      }
    }

    // <gazebo reference="link_name"><sensor type="imu"|"contact"> 形式
    const char* ref = gazebo->Attribute("reference");
    if(ref != nullptr) {
      for(XMLElement* sensor_elem = gazebo->FirstChildElement("sensor"); sensor_elem; sensor_elem = sensor_elem->NextSiblingElement("sensor")) {
        const char* stype = sensor_elem->Attribute("type");
        if(stype == nullptr) continue;
        const std::string stype_str = stype;
        const char* sname           = sensor_elem->Attribute("name");
        if(stype_str == "imu") {
          detected_sensors.push_back({UrdfSensorInfo::Type::IMU, sname ? sname : "imu_sensor", "", ref});
        } else if(stype_str == "contact") {
          // 接触センサーは対象linkにsiteを生成し、touchセンサーで参照する
          detected_sensors.push_back({UrdfSensorInfo::Type::Touch, sname ? sname : "touch_sensor", "", ref});
        }
      }
    }
  }

  std::map<std::string, std::shared_ptr<Body>> link_to_body;

  for(XMLElement* link = robot->FirstChildElement("link"); link; link = link->NextSiblingElement("link")) {
    const char* link_name = link->Attribute("name");
    if(!link_name) continue;

    auto body  = std::make_shared<Body>();
    body->name = add_pfx(link_name);

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

      // Parse inertia tensor (6 DOF: ixx, ixy, ixz, iyy, iyz, izz from URDF)
      XMLElement* inertia_elem = inertial->FirstChildElement("inertia");
      if(inertia_elem) {
        double ixx = inertia_elem->DoubleAttribute("ixx");
        double ixy = inertia_elem->DoubleAttribute("ixy");
        double ixz = inertia_elem->DoubleAttribute("ixz");
        double iyy = inertia_elem->DoubleAttribute("iyy");
        double iyz = inertia_elem->DoubleAttribute("iyz");
        double izz = inertia_elem->DoubleAttribute("izz");
        // MuJoCo fullinertia order: [ixx, iyy, izz, ixy, ixz, iyz]
        inertial_elem->fullinertia = {ixx, iyy, izz, ixy, ixz, iyz};
      }
      body->add_child(inertial_elem);
    }

    // Check if collision elements exist
    XMLElement* first_collision = link->FirstChildElement("collision");
    bool has_collision          = (first_collision != nullptr);

    // Process collision elements if they exist, otherwise fallback to visual (unless use_collision_tag_only is true)
    if(!has_collision && use_collision_tag_only) {
      // Skip geometry processing when collision tag is missing and use_collision_tag_only is true
    } else {
      XMLElement* geom_source = has_collision ? first_collision : link->FirstChildElement("visual");
      const char* source_type = has_collision ? "collision" : "visual";

      for(XMLElement* elem = geom_source; elem; elem = elem->NextSiblingElement(source_type)) {
        auto geom = std::make_shared<Geom>();

        XMLElement* origin = elem->FirstChildElement("origin");
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

        XMLElement* geometry = elem->FirstChildElement("geometry");
        if(geometry) {
          XMLElement* box      = geometry->FirstChildElement("box");
          XMLElement* cylinder = geometry->FirstChildElement("cylinder");
          XMLElement* capsule  = geometry->FirstChildElement("capsule");
          XMLElement* sphere   = geometry->FirstChildElement("sphere");
          XMLElement* mesh     = geometry->FirstChildElement("mesh");

          bool geometry_found = false;

          if(box) {
            geom->type       = GeomType::Box;
            const char* size = box->Attribute("size");
            if(size) {
              auto sizes = parse_space_separated_values(size);
              if(sizes.size() >= 3) geom->size = {sizes[0] / 2, sizes[1] / 2, sizes[2] / 2}; // MJCF uses half-sizes
            }
            geometry_found = true;
          } else if(cylinder) {
            geom->type     = GeomType::Cylinder;
            double radius  = cylinder->DoubleAttribute("radius");
            double length  = cylinder->DoubleAttribute("length");
            geom->size     = {radius, length / 2, 0.001}; // MJCF uses half-length
            geometry_found = true;
          } else if(capsule) {
            geom->type     = GeomType::Capsule;
            double radius  = capsule->DoubleAttribute("radius");
            double length  = capsule->DoubleAttribute("length");
            geom->size     = {radius, length / 2, 0.001};
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
                    mujoco->add_temporary_file(dest_mesh_path);
                  } else {
                    final_mesh_path = mesh_filename; // Fallback to original
                  }
                } else {
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

              auto mesh_name   = add_pfx(std::string(link_name) + "_" + std::filesystem::path(final_mesh_path).stem().string());
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

          if(geometry_found) {
            // Gazebo接触パラメータ（mu1/mu2/condim/contype/conaffinity/solref/solimp）を適用
            auto geom_it = gazebo_geom_map.find(link_name);
            if(geom_it != gazebo_geom_map.end()) {
              const auto& gp = geom_it->second;
              if(gp.mu1 >= 0.0) geom->friction[0] = gp.mu1;
              if(gp.mu2 >= 0.0) geom->friction[1] = gp.mu2;
              if(gp.condim >= 0) geom->condim = gp.condim;
              if(gp.contype >= 0) geom->contype = gp.contype;
              if(gp.conaffinity >= 0) geom->conaffinity = gp.conaffinity;
              if(gp.has_solref()) geom->solref = gp.solref;
              if(gp.has_solimp()) geom->solimp = gp.solimp;
            }
            body->add_child(geom);
          }
        } else {
          // No geometry element found, don't add the geom
          continue;
        }

        // Process material only from visual elements (not collision)
        if(!has_collision) {
          XMLElement* material = elem->FirstChildElement("material");
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
      }
    }
    link_to_body[link_name] = body;
  }

  // joint名→child link名のマップ（センサー用）
  std::map<std::string, std::string> joint_to_child_link;
  for(XMLElement* joint = robot->FirstChildElement("joint"); joint; joint = joint->NextSiblingElement("joint")) {
    const char* jname      = joint->Attribute("name");
    XMLElement* child_elem = joint->FirstChildElement("child");
    if(jname != nullptr && child_elem != nullptr) {
      const char* child_link = child_elem->Attribute("link");
      if(child_link != nullptr) joint_to_child_link[jname] = child_link;
    }
  }

  // <joint>内の<mujoco>ブロックからMuJoCo専用パラメータを事前収集
  struct MujocoJointParams {
    double stiffness                  = -1.0;
    double armature                   = -1.0;
    std::array<double, 2> solreflimit = {0.0, 0.0};
    bool has_solreflimit              = false;
  };
  std::map<std::string, MujocoJointParams> mujoco_joint_map;
  for(XMLElement* joint = robot->FirstChildElement("joint"); joint; joint = joint->NextSiblingElement("joint")) {
    const char* jname        = joint->Attribute("name");
    XMLElement* mujoco_block = joint->FirstChildElement("mujoco");
    if(jname == nullptr || mujoco_block == nullptr) continue;

    MujocoJointParams mp;

    XMLElement* stiffness_elem = mujoco_block->FirstChildElement("stiffness");
    if(stiffness_elem && stiffness_elem->GetText()) mp.stiffness = std::stod(stiffness_elem->GetText());

    XMLElement* armature_elem = mujoco_block->FirstChildElement("armature");
    if(armature_elem && armature_elem->GetText()) mp.armature = std::stod(armature_elem->GetText());

    XMLElement* solreflimit_elem = mujoco_block->FirstChildElement("solreflimit");
    if(solreflimit_elem && solreflimit_elem->GetText()) {
      auto vals = parse_space_separated_values(solreflimit_elem->GetText());
      if(vals.size() >= 2) {
        mp.solreflimit     = {vals[0], vals[1]};
        mp.has_solreflimit = true;
      }
    }

    mujoco_joint_map[jname] = mp;
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
  std::shared_ptr<Body> root_body = nullptr;
  for(const auto& [link_name, body] : link_to_body) {
    if(child_links.find(link_name) == child_links.end()) {
      body->pos = pos;
      worldbody->add_child(body);
      root_body = body;
      break; // For simplicity, take the first root link
    }
  }

  // Process joints and create parent-child relationships
  std::shared_ptr<Joint> first_joint = nullptr;
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

    if(joint_type != "fixed" and joint_type != "floating" and joint_type != "planar") {
      // if(joint_type == "prismatic") continue;

      auto mjcf_joint   = std::make_shared<Joint>();
      mjcf_joint->name  = add_pfx(joint_name);
      mjcf_joint->range = {-1e10, 1e10};

      if(joint_type == "continuous") {
        mjcf_joint->type    = JointType::Hinge;
        mjcf_joint->limited = false;
      } else if(joint_type == "revolute" || joint_type == "continuous") {
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
      double lower      = -1e10;
      double upper      = 1e10;
      if(limit) {
        lower = limit->DoubleAttribute("lower");
        upper = limit->DoubleAttribute("upper");
        if(upper > lower) mjcf_joint->range = {lower, upper};
      }

      // Parse joint dynamics for damping and friction
      XMLElement* dynamics = joint->FirstChildElement("dynamics");
      if(dynamics) {
        double damping  = dynamics->DoubleAttribute("damping", -1.0);
        double friction = dynamics->DoubleAttribute("friction", -1.0);

        // Only apply non-negative values (damping and friction cannot be negative)
        if(damping >= 0.0) mjcf_joint->damping = damping;
        if(friction >= 0.0) mjcf_joint->frictionloss = friction;
      }

      // <mujoco>ブロックのMuJoCo専用パラメータを適用
      auto mj_it = mujoco_joint_map.find(joint_name);
      if(mj_it != mujoco_joint_map.end()) {
        const auto& mjp = mj_it->second;
        if(mjp.stiffness >= 0.0) mjcf_joint->stiffness = mjp.stiffness;
        if(mjp.armature >= 0.0) mjcf_joint->armature = mjp.armature;
        if(mjp.has_solreflimit) mjcf_joint->solreflimit = mjp.solreflimit;
      }

      child_body->add_child(mjcf_joint);

      // Store the first joint found
      if(first_joint == nullptr) {
        first_joint = mjcf_joint;
      }

      auto filtered_ = std::find_if(actuator_metadata.begin(), actuator_metadata.end(),            //
                                    [&joint_name](const std::shared_ptr<BaseActuator>& actuator) { //
                                      return actuator->joint == joint_name;
                                    });

      auto empty_ = std::find_if(actuator_metadata.begin(), actuator_metadata.end(), //
                                 [](const std::shared_ptr<BaseActuator>& actuator) { //
                                   return actuator->joint == "";
                                 });

      // 多ロボット時は actuator の name と joint 参照に prefix を入れて mjcf の name 衝突を避ける。
      const std::string pfx_joint_name = add_pfx(joint_name);

      // metadata から関係するパラメータを抽出して prefix 付き actuator にコピーするヘルパ。
      auto clone_with_prefix = [&pfx_joint_name](const std::shared_ptr<BaseActuator>& src) -> std::shared_ptr<BaseActuator> {
        std::shared_ptr<BaseActuator> dst;
        if(auto p = std::dynamic_pointer_cast<Position>(src)) {
          auto cloned = std::make_shared<Position>();
          cloned->kp  = p->kp;
          cloned->kv  = p->kv;
          dst         = cloned;
        } else if(std::dynamic_pointer_cast<Motor>(src)) {
          dst = std::make_shared<Motor>();
        } else if(auto v = std::dynamic_pointer_cast<Velocity>(src)) {
          auto cloned = std::make_shared<Velocity>();
          cloned->kv  = v->kv;
          dst         = cloned;
        } else {
          return nullptr; // 未知の型は対応していないので処理をスキップ
        }
        // 共通パラメータをコピー
        dst->class_        = src->class_;
        dst->group         = src->group;
        dst->ctrllimited   = src->ctrllimited;
        dst->forcelimited  = src->forcelimited;
        dst->ctrlrange     = src->ctrlrange;
        dst->forcerange    = src->forcerange;
        dst->lengthrange   = src->lengthrange;
        dst->gear          = src->gear;
        dst->cranklength   = src->cranklength;
        dst->jointinparent = src->jointinparent;
        dst->tendon        = src->tendon;
        dst->cranksite     = src->cranksite;
        dst->site          = src->site;
        dst->refsite       = src->refsite;
        dst->user          = src->user;
        // name/joint のみ prefix 付きを書き込む (元 metadata は不変)
        dst->name  = pfx_joint_name;
        dst->joint = pfx_joint_name;
        return dst;
      };

      if(filtered_ != actuator_metadata.end()) {
        for(const auto& child : actuator_metadata) {
          if(child->joint == joint_name) {
            auto pfx_act = clone_with_prefix(child);
            if(pfx_act) mujoco->actuator_->add_child(pfx_act);
          }
        }
      } else if(empty_ != actuator_metadata.end()) {
        auto pfx_act = clone_with_prefix(*empty_);
        if(pfx_act) mujoco->actuator_->add_child(pfx_act);
      } else {

        if(mjcf_joint->type == JointType::Hinge) {
          auto ac         = Position::Create(pfx_joint_name);
          ac->name        = pfx_joint_name;
          ac->ctrllimited = false;
          ac->kp          = 100.0;
          ac->kv          = 10.0;
          if(limit != nullptr && lower < upper) {
            ac->ctrlrange   = {lower, upper};
            ac->ctrllimited = true;
          }
          // ac->gear        = {100, 0, 0, 0, 0, 0};
          mujoco->actuator_->add_child(ac);
        } else if(mjcf_joint->type == JointType::Slide) {
          auto ac         = Position::Create(pfx_joint_name);
          ac->name        = pfx_joint_name;
          ac->ctrllimited = false;
          ac->kp          = 10.0;
          ac->kv          = 1.0;
          mujoco->actuator_->add_child(ac);
        } else if(mjcf_joint->type == JointType::Ball) {
          auto ac         = Position::Create(pfx_joint_name);
          ac->name        = pfx_joint_name;
          ac->ctrllimited = false;
          ac->kp          = 100.0;
          ac->kv          = 10.0;
          mujoco->actuator_->add_child(ac);
        }
      }
    }
  }

  // 検出したセンサーに対してSiteとMJCFセンサーを生成
  for(const auto& sensor_info : detected_sensors) {
    std::shared_ptr<Body> target_body = nullptr;

    if(sensor_info.type == UrdfSensorInfo::Type::ForceTorque) {
      // joint名からchild bodyを取得
      auto it = joint_to_child_link.find(sensor_info.joint_name);
      if(it != joint_to_child_link.end()) {
        auto body_it = link_to_body.find(it->second);
        if(body_it != link_to_body.end()) target_body = body_it->second;
      }
    } else {
      // IMU / Touch はlink名から直接bodyを取得
      auto it = link_to_body.find(sensor_info.link_name);
      if(it != link_to_body.end()) target_body = it->second;
    }

    if(target_body == nullptr) continue;

    std::string site_name = add_pfx(sensor_info.name + "_site");
    auto site             = std::make_shared<Site>();
    site->name            = site_name;
    target_body->add_child(site);

    if(sensor_info.type == UrdfSensorInfo::Type::ForceTorque) {
      auto force_sensor  = std::make_shared<Force>();
      force_sensor->name = add_pfx(sensor_info.name + "_force");
      force_sensor->site = site_name;
      mujoco->sensor_->add_child(force_sensor);

      auto torque_sensor  = std::make_shared<Torque>();
      torque_sensor->name = add_pfx(sensor_info.name + "_torque");
      torque_sensor->site = site_name;
      mujoco->sensor_->add_child(torque_sensor);

    } else if(sensor_info.type == UrdfSensorInfo::Type::IMU) {
      auto gyro_sensor  = std::make_shared<Gyro>();
      gyro_sensor->name = add_pfx(sensor_info.name + "_gyro");
      gyro_sensor->site = site_name;
      mujoco->sensor_->add_child(gyro_sensor);

      auto acc_sensor  = std::make_shared<Accelerometer>();
      acc_sensor->name = add_pfx(sensor_info.name + "_accelerometer");
      acc_sensor->site = site_name;
      mujoco->sensor_->add_child(acc_sensor);
    } else { // Touch
      // 対象siteの接触力ノルムを測定するtouchセンサーを生成
      auto touch  = std::make_shared<Touch>();
      touch->name = add_pfx(sensor_info.name);
      touch->site = site_name;
      mujoco->sensor_->add_child(touch);
    }
  }

  return {root_body, first_joint};
}

void UrdfConverter::merge_asset_elements(const Asset& source_asset, Asset& target_asset, const std::string& name_prefix) {
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
  std::string filename = path.filename().string();
  // mesh は共有 cwd/cache に同名コピーされ並列プロセスで奪い合う (書込中を別プロセスが読むと
  // empty/truncated になりモデルが壊れる)。PID を混ぜて一意化する。コピー先 (dest_mesh_path) と
  // MJCF 参照 (final_mesh_path) は共にこの戻り値を使うため整合する。
#ifndef _WIN32
  const long pid = static_cast<long>(getpid());
#else
  const long pid = static_cast<long>(_getpid());
#endif
  return hash_str + "_" + std::to_string(pid) + "_" + filename;
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
