#include "core_elements.hpp"
#include "body_elements.hpp"
#include "actuator_elements.hpp"
#include "sensor_elements.hpp"
#include "urdf_converter.hpp"
#include <filesystem>
#include <iostream>

namespace mjcf {


void Mujoco::set_xml_attrib() const {
  if(!model.empty()) this->set_attribute("model", model);
}

void* Mujoco::write_xml_element(void* doc_ptr, void* parent_ptr) const {
  auto elm = this->write_xml_element_base(doc_ptr, parent_ptr);
  compiler_->write_xml_element(doc_ptr, elm);
  option_->write_xml_element(doc_ptr, elm);
  asset_->write_xml_element(doc_ptr, elm);
  worldbody_->write_xml_element(doc_ptr, elm);
  actuator_->write_xml_element(doc_ptr, elm);
  sensor_->write_xml_element(doc_ptr, elm);
  contact_->write_xml_element(doc_ptr, elm);
  equality_->write_xml_element(doc_ptr, elm);
  tendon_->write_xml_element(doc_ptr, elm);
  default_->write_xml_element(doc_ptr, elm);
  for(const auto& child : this->get_children())
    if(child) child->write_xml_element(doc_ptr, elm);
  return elm;
}

std::tuple<std::shared_ptr<mjcf::Body>, std::shared_ptr<mjcf::Joint>> Mujoco::add_urdf(const std::string& urdf_path, [[maybe_unused]] const std::string& name_prefix, bool copy_meshes, const std::vector<std::shared_ptr<BaseActuator>>& actuator_metadata, const Arr3& pos) {
  if(!std::filesystem::exists(urdf_path)) {
    printf("URDF file does not exist: %s\n", urdf_path.c_str());
    return {nullptr, nullptr};
  }

  return UrdfConverter::parse_urdf_to_mjcf(this, urdf_path, pos, actuator_metadata, copy_meshes);
}

size_t Mujoco::clear_temporary_files() {
  size_t deleted_count = 0;
  for(const auto& filepath : temporary_files_) {
    try {
      if(std::filesystem::exists(filepath)) {
        std::filesystem::remove(filepath);
        deleted_count++;
      }
    } catch(const std::filesystem::filesystem_error& e) {
      std::cerr << "Failed to delete temporary file: " << filepath << " - " << e.what() << std::endl;
    }
  }
  temporary_files_.clear();
  return deleted_count;
}

namespace detail {

void Compiler::set_xml_attrib() const {
  if(angle != AngleUnit::Degree) this->set_attribute("angle", to_string(angle));
  if(coordinate != CoordinateType::Local) this->set_attribute("coordinate", to_string(coordinate));
  if(inertiafromgeom) this->set_attribute("inertiafromgeom", inertiafromgeom);
  if(autolimits) this->set_attribute("autolimits", autolimits);
}

bool Compiler::is_default_value(const std::string& name, const AttributeValue& value) const {
  if(name == "angle" && std::get<std::string>(value) == to_string(AngleUnit::Degree)) return true;
  if(name == "coordinate" && std::get<std::string>(value) == to_string(CoordinateType::Local)) return true;
  if(name == "inertiafromgeom" && !std::get<bool>(value)) return true;
  if(name == "autolimits" && !std::get<bool>(value)) return true;
  return false;
}

struct Flag final : Element {
  Flag() = default;
  [[nodiscard]] std::string element_name() const override { return "flag"; }
};


void Option::set_xml_attrib() const {
  if(integrator != IntegratorType::Euler) this->set_attribute("integrator", to_string(integrator));
  if(timestep != 0.002) this->set_attribute("timestep", timestep);
  this->set_attribute("gravity", std::vector<double>(gravity.begin(), gravity.end()));
  if(viscosity != 0.0) this->set_attribute("viscosity", viscosity);
  if(cone != "pyramidal") this->set_attribute("cone", cone);
  if(multi_ccd) {
    auto f = std::make_shared<Flag>();
    f->set_attribute_public("multiccd", "enable");
    auto c = const_cast<Option*>(this);
    c->add_child(f);
  }
  if(solver != SolverType::Newton) this->set_attribute("solver", to_string(solver));
  if(iterations != 100) this->set_attribute("iterations", iterations);
  if(noslip_iterations > 0) this->set_attribute("noslip_iterations", noslip_iterations);
  if(noslip_tolerance > 0) this->set_attribute("noslip_tolerance", noslip_tolerance);
  if(impratio != 5.0) this->set_attribute("impratio", impratio);
  if(tolerance != 1e-8) this->set_attribute("tolerance", tolerance);
}

bool Option::is_default_value(const std::string& name, const AttributeValue& value) const {
  if(name == "integrator" && std::get<std::string>(value) == to_string(IntegratorType::Euler)) return true;
  if(name == "timestep" && std::get<double>(value) == 0.002) return true;
  if(name == "viscosity" && std::get<double>(value) == 0.0) return true;
  return false;
}

Default::Default(const std::string& class_name) : class_(class_name) {}

void Default::set_xml_attrib() const {
  if(!class_.empty()) this->set_attribute("class", class_);
}

bool Default::is_default_value([[maybe_unused]] const std::string& name, [[maybe_unused]] const AttributeValue& value) const {
  return false; // 設定されている場合はclassを常に含める
}

} // namespace detail

Size::Size() = default;

void Size::set_xml_attrib() const {
  // Only set non-default values
  if(njmax != 0) this->set_attribute("njmax", njmax);
  if(nconmax != 0) this->set_attribute("nconmax", nconmax);
  if(nstack != 0) this->set_attribute("nstack", nstack);
  if(nuserdata != 0) this->set_attribute("nuserdata", nuserdata);
}

bool Size::is_default_value([[maybe_unused]] const std::string& name, [[maybe_unused]] const AttributeValue& value) const {
  return false; // サイズパラメータは通常明示的に設定される
}

// Helper function to recursively collect names from element tree
namespace {
template<typename T>
void collect_names_recursive(const Element* element, std::set<std::string>& names) {
  if(!element) return;
  
  // Try to cast to the target type and get name directly
  auto typed_element = dynamic_cast<const T*>(element);
  if(typed_element) {
    // Access the name member variable directly
    // This uses compile-time check to see if the type has a name member
    if constexpr (requires { typed_element->name; }) {
      if(!typed_element->name.empty()) {
        names.insert(typed_element->name);
      }
    }
  }
  
  // Recursively process children
  for(const auto& child : element->get_children()) {
    collect_names_recursive<T>(child.get(), names);
  }
}

// Helper to collect names from asset container elements
void collect_asset_names(const Element* container, std::set<std::string>& names) {
  if(!container) return;
  
  for(const auto& child : container->get_children()) {
    // Try each asset type
    if(auto texture = dynamic_cast<const Texture*>(child.get())) {
      if(!texture->name.empty()) names.insert(texture->name);
    } else if(auto material = dynamic_cast<const Material*>(child.get())) {
      if(!material->name.empty()) names.insert(material->name);
    } else if(auto mesh = dynamic_cast<const Mesh*>(child.get())) {
      if(!mesh->name.empty()) names.insert(mesh->name);
    } else if(auto hfield = dynamic_cast<const Hfield*>(child.get())) {
      if(!hfield->name.empty()) names.insert(hfield->name);
    }
  }
}

// Helper to collect sensor names
void collect_sensor_names(const Element* container, std::set<std::string>& names) {
  if(!container) return;
  
  for(const auto& child : container->get_children()) {
    if(auto sensor = dynamic_cast<const BaseSensor*>(child.get())) {
      if(!sensor->name.empty()) names.insert(sensor->name);
    }
  }
}

// Helper to collect actuator names
void collect_actuator_names(const Element* container, std::set<std::string>& names) {
  if(!container) return;
  
  for(const auto& child : container->get_children()) {
    if(auto actuator = dynamic_cast<const BaseActuator*>(child.get())) {
      if(!actuator->name.empty()) names.insert(actuator->name);
    }
  }
}
} // anonymous namespace

std::set<std::string> Mujoco::get_body_names() const {
  std::set<std::string> names;
  if(worldbody_) {
    collect_names_recursive<Body>(worldbody_.get(), names);
  }
  return names;
}

std::set<std::string> Mujoco::get_geom_names() const {
  std::set<std::string> names;
  if(worldbody_) {
    collect_names_recursive<Geom>(worldbody_.get(), names);
  }
  return names;
}

std::set<std::string> Mujoco::get_joint_names() const {
  std::set<std::string> names;
  if(worldbody_) {
    collect_names_recursive<Joint>(worldbody_.get(), names);
  }
  return names;
}

std::set<std::string> Mujoco::get_site_names() const {
  std::set<std::string> names;
  if(worldbody_) {
    collect_names_recursive<Site>(worldbody_.get(), names);
  }
  return names;
}

std::set<std::string> Mujoco::get_camera_names() const {
  std::set<std::string> names;
  if(worldbody_) {
    collect_names_recursive<Camera>(worldbody_.get(), names);
  }
  return names;
}

std::set<std::string> Mujoco::get_light_names() const {
  std::set<std::string> names;
  if(worldbody_) {
    collect_names_recursive<Light>(worldbody_.get(), names);
  }
  return names;
}

std::set<std::string> Mujoco::get_asset_names() const {
  std::set<std::string> names;
  if(asset_) {
    collect_asset_names(asset_.get(), names);
  }
  return names;
}

std::set<std::string> Mujoco::get_sensor_names() const {
  std::set<std::string> names;
  if(sensor_) {
    collect_sensor_names(sensor_.get(), names);
  }
  return names;
}

std::set<std::string> Mujoco::get_actuator_names() const {
  std::set<std::string> names;
  if(actuator_) {
    collect_actuator_names(actuator_.get(), names);
  }
  return names;
}

std::set<std::string> Mujoco::get_all_names() const {
  std::set<std::string> names;
  
  // Collect from worldbody (bodies, geoms, joints, sites, cameras, lights)
  auto body_names = get_body_names();
  names.insert(body_names.begin(), body_names.end());
  
  auto geom_names = get_geom_names();
  names.insert(geom_names.begin(), geom_names.end());
  
  auto joint_names = get_joint_names();
  names.insert(joint_names.begin(), joint_names.end());
  
  auto site_names = get_site_names();
  names.insert(site_names.begin(), site_names.end());
  
  auto camera_names = get_camera_names();
  names.insert(camera_names.begin(), camera_names.end());
  
  auto light_names = get_light_names();
  names.insert(light_names.begin(), light_names.end());
  
  // Collect from assets
  auto asset_names = get_asset_names();
  names.insert(asset_names.begin(), asset_names.end());
  
  // Collect from sensors
  auto sensor_names = get_sensor_names();
  names.insert(sensor_names.begin(), sensor_names.end());
  
  // Collect from actuators
  auto actuator_names = get_actuator_names();
  names.insert(actuator_names.begin(), actuator_names.end());
  
  return names;
}

} // namespace mjcf
