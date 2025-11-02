#pragma once

#include "element.hpp"

#include <memory>
#include <set>
#include <string>
#include <unordered_map>

namespace mjcf {

class BaseActuator;
class Element;
class Mujoco;
namespace detail {
class Asset;
class Worldbody;
} // namespace detail

class UrdfConverter {
public:
  static bool parse_urdf_to_mjcf( //
    Mujoco* mujoco,
    const std::string& urdf_path,                                                                             //
    const Arr3& pos, const std::unordered_map<std::string, std::shared_ptr<BaseActuator>>& actuator_metadata, //
    bool copy_meshes              = false,                                                                    //
    const std::string& output_dir = ""                                                                        //
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
