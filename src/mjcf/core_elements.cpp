#include "core_elements.hpp"
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

} // namespace mjcf
