#include "actuator_elements.hpp"

namespace mjcf {

// BaseActuator implementation
BaseActuator::BaseActuator(const std::string& element_name) : element_name_(element_name) {}

void BaseActuator::set_xml_attrib() const {
  // Set attributes from public members before generating XML
  auto* mutable_this = const_cast<BaseActuator*>(this);

  // Only set non-default values
  if(!name.empty()) mutable_this->set_attribute("name", name);
  if(!class_.empty()) mutable_this->set_attribute("class", class_);
  if(group != 0) mutable_this->set_attribute("group", group);
  if(ctrllimited) mutable_this->set_attribute("ctrllimited", ctrllimited);
  if(forcelimited) mutable_this->set_attribute("forcelimited", forcelimited);

  // For arrays, check if they're non-default
  if(ctrlrange != std::array<double, 2>{0.0, 0.0}) {
    mutable_this->set_attribute("ctrlrange", std::vector<double>(ctrlrange.begin(), ctrlrange.end()));
  }
  if(forcerange != std::array<double, 2>{0.0, 0.0}) {
    mutable_this->set_attribute("forcerange", std::vector<double>(forcerange.begin(), forcerange.end()));
  }
  if(lengthrange != std::array<double, 2>{0.0, 0.0}) {
    mutable_this->set_attribute("lengthrange", std::vector<double>(lengthrange.begin(), lengthrange.end()));
  }
  mutable_this->set_attribute("gear", std::vector<double>(gear.begin(), gear.end()));

  if(cranklength != 0.0) mutable_this->set_attribute("cranklength", cranklength);
  if(!joint.empty()) mutable_this->set_attribute("joint", joint);
  if(!jointinparent.empty()) mutable_this->set_attribute("jointinparent", jointinparent);
  if(!tendon.empty()) mutable_this->set_attribute("tendon", tendon);
  if(!cranksite.empty()) mutable_this->set_attribute("cranksite", cranksite);
  if(!site.empty()) mutable_this->set_attribute("site", site);
  if(!refsite.empty()) mutable_this->set_attribute("refsite", refsite);

  if(user != std::array<double, 3>{0.0, 0.0, 0.0}) {
    mutable_this->set_attribute("user", std::vector<double>(user.begin(), user.end()));
  }
}

bool BaseActuator::is_default_value(const std::string& name, const AttributeValue& value) const {
  if(name == "group" && std::get<int>(value) == 0) return true;
  if(name == "ctrllimited" && !std::get<bool>(value)) return true;
  if(name == "forcelimited" && !std::get<bool>(value)) return true;
  if(name == "cranklength" && std::get<double>(value) == 0.0) return true;
  return false;
}

// Motor implementation
Motor::Motor() : BaseActuator("motor") {}

bool Motor::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseActuator::is_default_value(name, value); }
void Motor::set_xml_attrib() const { BaseActuator::set_xml_attrib(); }

Position::Position() : BaseActuator("position") {}

void Position::set_xml_attrib() const {
  // Set attributes from public members before generating XML
  auto* mutable_this = const_cast<Position*>(this);

  // Set base class attributes first
  BaseActuator::set_xml_attrib();

  // Only set non-default values specific to Position
  if(kp != 0.0) {
    mutable_this->set_attribute("kp", kp);
  }
  if(kv != 0.0) {
    mutable_this->set_attribute("kv", kv);
  }
}

bool Position::from_xml(const std::string& xml_str) {
  // Default implementation - could be extended to parse XML
  return false;
}

bool Position::is_default_value(const std::string& name, const AttributeValue& value) const {
  if(name == "kp" && std::get<double>(value) == 0.0) return true;
  if(name == "kv" && std::get<double>(value) == 0.0) return true;
  return BaseActuator::is_default_value(name, value);
}

// Velocity implementation
Velocity::Velocity() : BaseActuator("velocity") {}

void Velocity::set_xml_attrib() const {
  // Set base class attributes first
  BaseActuator::set_xml_attrib();

  // Set attributes from public members before generating XML
  auto* mutable_this = const_cast<Velocity*>(this);

  // Only set non-default values specific to Velocity
  if(kv != 0.0) {
    mutable_this->set_attribute("kv", kv);
  }
}

bool Velocity::from_xml(const std::string& xml_str) {
  // Default implementation - could be extended to parse XML
  return false;
}

bool Velocity::is_default_value(const std::string& name, const AttributeValue& value) const {
  if(name == "kv" && std::get<double>(value) == 0.0) return true;
  return BaseActuator::is_default_value(name, value);
}

// Cylinder implementation
Cylinder::Cylinder() : BaseActuator("cylinder") {}

void Cylinder::set_xml_attrib() const {
  // Set base class attributes first
  BaseActuator::set_xml_attrib();

  // Set attributes from public members before generating XML
  auto* mutable_this = const_cast<Cylinder*>(this);

  // Only set non-default values
  if(timeconst != 0.0) mutable_this->set_attribute("timeconst", timeconst);
  if(area != 0.0) mutable_this->set_attribute("area", area);
  if(diameter != 0.0) mutable_this->set_attribute("diameter", diameter);
  if(bias != std::array<double, 3>{0.0, 0.0, 0.0}) {
    mutable_this->set_attribute("bias", std::vector<double>(bias.begin(), bias.end()));
  }
}

bool Cylinder::from_xml(const std::string& xml_str) { return false; }

bool Cylinder::is_default_value(const std::string& name, const AttributeValue& value) const {
  if(name == "timeconst" && std::get<double>(value) == 0.0) return true;
  if(name == "area" && std::get<double>(value) == 0.0) return true;
  if(name == "diameter" && std::get<double>(value) == 0.0) return true;
  return BaseActuator::is_default_value(name, value);
}

// Muscle implementation
Muscle::Muscle() : BaseActuator("muscle") {}

void Muscle::set_xml_attrib() const {
  // Set base class attributes first
  BaseActuator::set_xml_attrib();

  // Set attributes from public members before generating XML
  auto* mutable_this = const_cast<Muscle*>(this);

  // Only set non-default values
  if(timeconst != std::array<double, 2>{0.0, 0.0}) {
    mutable_this->set_attribute("timeconst", std::vector<double>(timeconst.begin(), timeconst.end()));
  }
  if(range != std::array<double, 2>{0.0, 0.0}) {
    mutable_this->set_attribute("range", std::vector<double>(range.begin(), range.end()));
  }
  if(force != 0.0) mutable_this->set_attribute("force", force);
  if(scale != 0.0) mutable_this->set_attribute("scale", scale);
  if(lmin != 0.0) mutable_this->set_attribute("lmin", lmin);
  if(lmax != 0.0) mutable_this->set_attribute("lmax", lmax);
  if(vmax != 0.0) mutable_this->set_attribute("vmax", vmax);
  if(fpmax != 0.0) mutable_this->set_attribute("fpmax", fpmax);
  if(fvmax != 0.0) mutable_this->set_attribute("fvmax", fvmax);
}

bool Muscle::from_xml(const std::string& xml_str) { return false; }

bool Muscle::is_default_value(const std::string& name, const AttributeValue& value) const {
  if(name == "force" && std::get<double>(value) == 0.0) return true;
  if(name == "scale" && std::get<double>(value) == 0.0) return true;
  if(name == "lmin" && std::get<double>(value) == 0.0) return true;
  if(name == "lmax" && std::get<double>(value) == 0.0) return true;
  if(name == "vmax" && std::get<double>(value) == 0.0) return true;
  if(name == "fpmax" && std::get<double>(value) == 0.0) return true;
  if(name == "fvmax" && std::get<double>(value) == 0.0) return true;
  return BaseActuator::is_default_value(name, value);
}

// TendonActuator implementation
TendonActuator::TendonActuator() : BaseActuator("tendon") {}

bool TendonActuator::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseActuator::is_default_value(name, value); }

// General implementation
General::General() : BaseActuator("general") {}

void General::set_xml_attrib() const {
  // Set base class attributes first
  BaseActuator::set_xml_attrib();

  // Set attributes from public members before generating XML
  auto* mutable_this = const_cast<General*>(this);

  // Only set non-default values specific to General
  if(dyntype != "none") {
    mutable_this->set_attribute("dyntype", dyntype);
  }
  if(gaintype != "fixed") {
    mutable_this->set_attribute("gaintype", gaintype);
  }
  if(biastype != "none") {
    mutable_this->set_attribute("biastype", biastype);
  }

  // For arrays, check if they're non-default
  if(dynprm != std::array<double, 10>{1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}) {
    mutable_this->set_attribute("dynprm", std::vector<double>(dynprm.begin(), dynprm.end()));
  }
  if(gainprm != std::array<double, 10>{1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}) {
    mutable_this->set_attribute("gainprm", std::vector<double>(gainprm.begin(), gainprm.end()));
  }
  if(biasprm != std::array<double, 10>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}) {
    mutable_this->set_attribute("biasprm", std::vector<double>(biasprm.begin(), biasprm.end()));
  }
}

bool General::from_xml(const std::string& xml_str) {
  // Default implementation - could be extended to parse XML
  return false;
}

bool General::is_default_value(const std::string& name, const AttributeValue& value) const {
  if(name == "dyntype" && std::get<std::string>(value) == "none") return true;
  if(name == "gaintype" && std::get<std::string>(value) == "fixed") return true;
  if(name == "biastype" && std::get<std::string>(value) == "none") return true;
  return BaseActuator::is_default_value(name, value);
}

} // namespace mjcf
