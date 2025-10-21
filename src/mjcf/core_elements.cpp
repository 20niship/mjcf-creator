#include "core_elements.hpp"
#include "urdf_converter.hpp"

namespace mjcf {


void Mujoco::set_xml_attrib() const {
  auto* mutable_this = const_cast<Mujoco*>(this);
  if(!model.empty()) mutable_this->set_attribute("model", model);
}

bool Mujoco::from_xml(const std::string& xml_str) { return false; }

void* Mujoco::write_xml_element(void* doc_ptr, void* parent_ptr) const {
  auto elm = this->write_xml_element_base(doc_ptr, parent_ptr);
  compiler_->write_xml_element(doc_ptr, elm);
  option_->write_xml_element(doc_ptr, elm);
  asset_->write_xml_element(doc_ptr, elm);
  worldbody_->write_xml_element(doc_ptr, elm);
  actuator_->write_xml_element(doc_ptr, elm);
  sensor_->write_xml_element(doc_ptr, elm);
  equality_->write_xml_element(doc_ptr, elm);
  tendon_->write_xml_element(doc_ptr, elm);
  for(const auto& child : this->get_children())
    if(child) child->write_xml_element(doc_ptr, elm);
  return elm;
}

bool Mujoco::add_urdf(const std::string& urdf_path, const std::string& name_prefix, bool copy_meshes, const std::map<std::string, JointMetadata>& joint_metadata, const std::map<std::string, ActuatorMetadata>& actuator_metadata) {
  return UrdfConverter::parse_urdf_to_mjcf(this, urdf_path, joint_metadata, actuator_metadata);
}

namespace detail {

void Compiler::set_xml_attrib() const {
  auto* mutable_this = const_cast<Compiler*>(this);
  mutable_this->set_attribute("angle", to_string(angle));
  if(coordinate != CoordinateType::Local) mutable_this->set_attribute("coordinate", to_string(coordinate));
  if(inertiafromgeom) mutable_this->set_attribute("inertiafromgeom", inertiafromgeom);
  if(autolimits) mutable_this->set_attribute("autolimits", autolimits);
}

bool Compiler::from_xml(const std::string& xml_str) { return false; }

bool Compiler::is_default_value(const std::string& name, const AttributeValue& value) const {
  if(name == "angle" && std::get<std::string>(value) == to_string(AngleUnit::Degree)) return true;
  if(name == "coordinate" && std::get<std::string>(value) == to_string(CoordinateType::Local)) return true;
  if(name == "inertiafromgeom" && !std::get<bool>(value)) return true;
  if(name == "autolimits" && !std::get<bool>(value)) return true;
  return false;
}

void Option::set_xml_attrib() const {
  auto* mutable_this = const_cast<Option*>(this);
  if(integrator != IntegratorType::Euler) mutable_this->set_attribute("integrator", to_string(integrator));
  if(timestep != 0.002) mutable_this->set_attribute("timestep", timestep);
  mutable_this->set_attribute("gravity", std::vector<double>(gravity.begin(), gravity.end()));
  if(viscosity != 0.0) mutable_this->set_attribute("viscosity", viscosity);
}

bool Option::from_xml(const std::string& xml_str) { return false; }

bool Option::is_default_value(const std::string& name, const AttributeValue& value) const {
  if(name == "integrator" && std::get<std::string>(value) == to_string(IntegratorType::Euler)) return true;
  if(name == "timestep" && std::get<double>(value) == 0.002) return true;
  if(name == "viscosity" && std::get<double>(value) == 0.0) return true;
  return false;
}
} // namespace detail

Size::Size() = default;

void Size::set_xml_attrib() const {
  // Set attributes from public members before generating XML
  auto* mutable_this = const_cast<Size*>(this);

  // Only set non-default values
  if(njmax != 0) mutable_this->set_attribute("njmax", njmax);
  if(nconmax != 0) mutable_this->set_attribute("nconmax", nconmax);
  if(nstack != 0) mutable_this->set_attribute("nstack", nstack);
  if(nuserdata != 0) mutable_this->set_attribute("nuserdata", nuserdata);
}

bool Size::from_xml(const std::string& xml_str) { return false; }

bool Size::is_default_value(const std::string& name, const AttributeValue& value) const {
  return false; // サイズパラメータは通常明示的に設定される
}

Visual::Visual()       = default;
Statistic::Statistic() = default;

Default::Default(const std::string& class_name) : class_(class_name) {}

void Default::set_xml_attrib() const {
  auto* mutable_this = const_cast<Default*>(this);
  if(!class_.empty()) mutable_this->set_attribute("class", class_);
}

bool Default::from_xml(const std::string& xml_str) { return false; }

bool Default::is_default_value(const std::string& name, const AttributeValue& value) const {
  return false; // 設定されている場合はclassを常に含める
}

} // namespace mjcf
