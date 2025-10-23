#pragma once

#include <map>
#include <memory>
#include <set>
#include <string>

namespace mjcf {

class Element;
class Mujoco;
namespace detail {
class Asset;
class Worldbody;
} // namespace detail

struct JointMetadata {
  std::string actuator_type;
  int nn_id;
  int id;
  double kp;
  double kd;
  double soft_torque_limit;
  double min_angle_deg;
  double max_angle_deg;
};

struct ActuatorMetadata {
  std::string actuator_type;
  std::string sysid;
  double max_torque;
  double max_velocity;
  double armature;
  double damping;
  double frictionloss;
};

class UrdfConverter {
public:
  static bool parse_urdf_to_mjcf( //
    Mujoco* mujoco,
    const std::string& urdf_path,                                    //
    const std::map<std::string, JointMetadata>& joint_metadata,      //
    const std::map<std::string, ActuatorMetadata>& actuator_metadata, //
    bool copy_meshes = false,                                        //
    const std::string& output_dir = ""                               //
  );

  static void merge_asset_elements(const detail::Asset& source_asset, detail::Asset& target_asset, const std::string& name_prefix);
  static void merge_worldbody_elements(const detail::Worldbody& source_worldbody, detail::Worldbody& target_worldbody, const std::string& name_prefix);
  static std::shared_ptr<Element> clone_element_with_prefix(const std::shared_ptr<Element>& element, const std::string& name_prefix, const std::set<std::string>& existing_names);
  static void clone_element_attributes(const Element& source, Element& target, const std::string& name_prefix);

private:
  static std::string generate_hash_filename(const std::string& original_path);
  static bool copy_mesh_file(const std::string& source_path, const std::string& dest_path);
  static std::string resolve_mesh_path(const std::string& urdf_path, const std::string& mesh_filename);
};

} // namespace mjcf
