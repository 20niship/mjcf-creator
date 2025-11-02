#include "core_elements.hpp"
#include "urdf_converter.hpp"
#include "asset_elements.hpp"
#include "body_elements.hpp"
#include "actuator_elements.hpp"
#include "sensor_elements.hpp"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <tinyxml2.h>

namespace mjcf {


void Mujoco::set_xml_attrib() const {
  if(!model.empty()) this->set_attribute("model", model);
}

bool Mujoco::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

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

bool Mujoco::add_urdf(const std::string& urdf_path, const std::string& name_prefix, bool copy_meshes, const std::unordered_map<std::string, std::shared_ptr<BaseActuator>>& actuator_metadata, const Arr3& pos) {
  if(!std::filesystem::exists(urdf_path)) {
    printf("URDF file does not exist: %s\n", urdf_path.c_str());
    return false;
  }

  return UrdfConverter::parse_urdf_to_mjcf(this, urdf_path, pos, actuator_metadata, copy_meshes);
}

namespace {
// Helper function to copy all XML attributes from an XML element to an Element object
void copy_xml_attributes_to_element(const tinyxml2::XMLElement* xml_elem, Element& element) {
  if(!xml_elem) return;
  
  const tinyxml2::XMLAttribute* attr = xml_elem->FirstAttribute();
  while(attr) {
    std::string attr_name = attr->Name();
    std::string attr_value = attr->Value();
    
    // Try to determine the type and convert appropriately
    // For simplicity, we'll store everything as strings and let the element handle conversion
    element.set_attribute_public(attr_name, attr_value);
    
    attr = attr->Next();
  }
}

// Helper function to recursively parse and create element hierarchy
std::shared_ptr<Element> parse_xml_element(const tinyxml2::XMLElement* xml_elem);

std::shared_ptr<Element> parse_xml_element(const tinyxml2::XMLElement* xml_elem) {
  if(!xml_elem) return nullptr;
  
  std::string elem_name = xml_elem->Name();
  std::shared_ptr<Element> element;
  
  // Create the appropriate element type based on the tag name
  if(elem_name == "body") {
    element = std::make_shared<Body>();
  } else if(elem_name == "geom") {
    element = std::make_shared<Geom>();
  } else if(elem_name == "joint") {
    element = std::make_shared<Joint>();
  } else if(elem_name == "light") {
    element = std::make_shared<Light>();
  } else if(elem_name == "camera") {
    element = std::make_shared<Camera>();
  } else if(elem_name == "site") {
    element = std::make_shared<Site>();
  } else if(elem_name == "inertial") {
    element = std::make_shared<Inertial>();
  } else if(elem_name == "texture") {
    element = std::make_shared<Texture>();
  } else if(elem_name == "material") {
    element = std::make_shared<Material>();
  } else if(elem_name == "mesh") {
    element = std::make_shared<Mesh>();
  } else if(elem_name == "motor") {
    element = std::make_shared<Motor>();
  } else if(elem_name == "position") {
    element = std::make_shared<Position>();
  } else if(elem_name == "velocity") {
    element = std::make_shared<Velocity>();
  } else if(elem_name == "touch") {
    element = std::make_shared<Touch>();
  } else if(elem_name == "force") {
    element = std::make_shared<Force>();
  } else if(elem_name == "torque") {
    element = std::make_shared<Torque>();
  }
  // Add more element types as needed
  
  if(!element) {
    // Unknown element type, skip it
    return nullptr;
  }
  
  // Copy all attributes
  copy_xml_attributes_to_element(xml_elem, *element);
  
  // Recursively parse and add children
  for(const tinyxml2::XMLElement* child = xml_elem->FirstChildElement(); child; child = child->NextSiblingElement()) {
    auto child_element = parse_xml_element(child);
    if(child_element) {
      element->add_child(child_element);
    }
  }
  
  return element;
}
} // anonymous namespace

bool Mujoco::add_mjcf(const std::string& mjcf_path, const std::string& name_prefix) {
  if(!std::filesystem::exists(mjcf_path)) {
    printf("MJCF file does not exist: %s\n", mjcf_path.c_str());
    return false;
  }

  // Read the MJCF file
  std::ifstream file(mjcf_path);
  if(!file.is_open()) {
    printf("Failed to open MJCF file: %s\n", mjcf_path.c_str());
    return false;
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  std::string mjcf_content = buffer.str();
  file.close();

  // Parse the MJCF XML
  tinyxml2::XMLDocument doc;
  if(doc.Parse(mjcf_content.c_str()) != tinyxml2::XML_SUCCESS) {
    printf("Failed to parse MJCF file: %s\n", mjcf_path.c_str());
    return false;
  }

  // Get the root mujoco element
  tinyxml2::XMLElement* root = doc.FirstChildElement("mujoco");
  if(!root) {
    printf("No mujoco root element found in file: %s\n", mjcf_path.c_str());
    return false;
  }

  // Create temporary containers for parsed elements
  auto temp_mujoco = std::make_shared<Mujoco>();

  // Parse asset elements
  tinyxml2::XMLElement* asset_elem = root->FirstChildElement("asset");
  if(asset_elem) {
    for(tinyxml2::XMLElement* child = asset_elem->FirstChildElement(); child; child = child->NextSiblingElement()) {
      auto element = parse_xml_element(child);
      if(element) {
        temp_mujoco->asset_->add_child(element);
      }
    }
  }

  // Parse worldbody elements
  tinyxml2::XMLElement* worldbody_elem = root->FirstChildElement("worldbody");
  if(worldbody_elem) {
    for(tinyxml2::XMLElement* child = worldbody_elem->FirstChildElement(); child; child = child->NextSiblingElement()) {
      auto element = parse_xml_element(child);
      if(element) {
        temp_mujoco->worldbody_->add_child(element);
      }
    }
  }

  // Parse actuator elements
  tinyxml2::XMLElement* actuator_elem = root->FirstChildElement("actuator");
  if(actuator_elem) {
    for(tinyxml2::XMLElement* child = actuator_elem->FirstChildElement(); child; child = child->NextSiblingElement()) {
      auto element = parse_xml_element(child);
      if(element) {
        temp_mujoco->actuator_->add_child(element);
      }
    }
  }

  // Parse sensor elements
  tinyxml2::XMLElement* sensor_elem = root->FirstChildElement("sensor");
  if(sensor_elem) {
    for(tinyxml2::XMLElement* child = sensor_elem->FirstChildElement(); child; child = child->NextSiblingElement()) {
      auto element = parse_xml_element(child);
      if(element) {
        temp_mujoco->sensor_->add_child(element);
      }
    }
  }

  // Merge the parsed elements into this Mujoco instance using existing merge functions
  UrdfConverter::merge_asset_elements(*temp_mujoco->asset_, *this->asset_, name_prefix);
  UrdfConverter::merge_worldbody_elements(*temp_mujoco->worldbody_, *this->worldbody_, name_prefix);

  // Merge actuators
  for(const auto& child : temp_mujoco->actuator_->get_children()) {
    auto cloned = UrdfConverter::clone_element_with_prefix(child, name_prefix, {});
    if(cloned) {
      this->actuator_->add_child(cloned);
    }
  }

  // Merge sensors
  for(const auto& child : temp_mujoco->sensor_->get_children()) {
    auto cloned = UrdfConverter::clone_element_with_prefix(child, name_prefix, {});
    if(cloned) {
      this->sensor_->add_child(cloned);
    }
  }

  return true;
}

namespace detail {

void Compiler::set_xml_attrib() const {
  if(angle != AngleUnit::Degree) this->set_attribute("angle", to_string(angle));
  if(coordinate != CoordinateType::Local) this->set_attribute("coordinate", to_string(coordinate));
  if(inertiafromgeom) this->set_attribute("inertiafromgeom", inertiafromgeom);
  if(autolimits) this->set_attribute("autolimits", autolimits);
}

bool Compiler::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool Compiler::is_default_value(const std::string& name, const AttributeValue& value) const {
  if(name == "angle" && std::get<std::string>(value) == to_string(AngleUnit::Degree)) return true;
  if(name == "coordinate" && std::get<std::string>(value) == to_string(CoordinateType::Local)) return true;
  if(name == "inertiafromgeom" && !std::get<bool>(value)) return true;
  if(name == "autolimits" && !std::get<bool>(value)) return true;
  return false;
}

void Option::set_xml_attrib() const {
  if(integrator != IntegratorType::Euler) this->set_attribute("integrator", to_string(integrator));
  if(timestep != 0.002) this->set_attribute("timestep", timestep);
  this->set_attribute("gravity", std::vector<double>(gravity.begin(), gravity.end()));
  if(viscosity != 0.0) this->set_attribute("viscosity", viscosity);
  if(cone != "pyramidal") this->set_attribute("cone", cone);
}

bool Option::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

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

bool Default::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

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

bool Size::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool Size::is_default_value([[maybe_unused]] const std::string& name, [[maybe_unused]] const AttributeValue& value) const {
  return false; // サイズパラメータは通常明示的に設定される
}

} // namespace mjcf
