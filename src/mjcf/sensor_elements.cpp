#include "sensor_elements.hpp"

namespace mjcf {

BaseSensor::BaseSensor(const std::string& element_name) : element_name_(element_name) {}

void BaseSensor::set_xml_attrib() const {

  // Only set non-default values
  if(!name.empty()) this->set_attribute("name", name);
  if(noise != 0.0) this->set_attribute("noise", noise);
  if(cutoff != 0.0) this->set_attribute("cutoff", cutoff);
  if(user != std::array<double, 3>{0.0, 0.0, 0.0}) {
    this->set_attribute("user", std::vector<double>(user.begin(), user.end()));
  }
}

bool BaseSensor::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool BaseSensor::is_default_value(const std::string& name, const AttributeValue& value) const {
  if(name == "noise" && std::get<double>(value) == 0.0) return true;
  if(name == "cutoff" && std::get<double>(value) == 0.0) return true;
  return false;
}

JointPos::JointPos() : BaseSensor("jointpos") {}

void JointPos::set_xml_attrib() const {
  // Set base class attributes first
  BaseSensor::set_xml_attrib();

  if(!joint.empty()) this->set_attribute("joint", joint);
}

bool JointPos::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool JointPos::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// JointVel implementation
JointVel::JointVel() : BaseSensor("jointvel") {}

void JointVel::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!joint.empty()) this->set_attribute("joint", joint);
}

bool JointVel::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool JointVel::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// TendonPos implementation
TendonPos::TendonPos() : BaseSensor("tendonpos") {}

void TendonPos::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!tendon.empty()) this->set_attribute("tendon", tendon);
}

bool TendonPos::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool TendonPos::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// TendonVel implementation
TendonVel::TendonVel() : BaseSensor("tendonvel") {}

void TendonVel::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!tendon.empty()) this->set_attribute("tendon", tendon);
}

bool TendonVel::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool TendonVel::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// ActuatorPos implementation
ActuatorPos::ActuatorPos() : BaseSensor("actuatorpos") {}

void ActuatorPos::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!actuator.empty()) this->set_attribute("actuator", actuator);
}

bool ActuatorPos::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool ActuatorPos::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// ActuatorVel implementation
ActuatorVel::ActuatorVel() : BaseSensor("actuatorvel") {}

void ActuatorVel::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!actuator.empty()) this->set_attribute("actuator", actuator);
}

bool ActuatorVel::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool ActuatorVel::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// ActuatorFrc implementation
ActuatorFrc::ActuatorFrc() : BaseSensor("actuatorfrc") {}

void ActuatorFrc::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!actuator.empty()) this->set_attribute("actuator", actuator);
}

bool ActuatorFrc::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool ActuatorFrc::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// BallQuat implementation
BallQuat::BallQuat() : BaseSensor("ballquat") {}

void BallQuat::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!joint.empty()) this->set_attribute("joint", joint);
}

bool BallQuat::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool BallQuat::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// BallAngVel implementation
BallAngVel::BallAngVel() : BaseSensor("ballangvel") {}

void BallAngVel::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!joint.empty()) this->set_attribute("joint", joint);
}

bool BallAngVel::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool BallAngVel::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// SitePos implementation
SitePos::SitePos() : BaseSensor("framepos") {}

void SitePos::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(reftype != "body") this->set_attribute("reftype", reftype);
  if(objtype != "body") this->set_attribute("objtype", objtype);
  if(!objname.empty()) this->set_attribute("objname", objname);
}

bool SitePos::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool SitePos::is_default_value(const std::string& name, const AttributeValue& value) const {
  if(name == "reftype" && std::get<std::string>(value) == "body") return true;
  if(name == "objtype" && std::get<std::string>(value) == "body") return true;
  return BaseSensor::is_default_value(name, value);
}

// SiteQuat implementation
SiteQuat::SiteQuat() : BaseSensor("framequat") {}

void SiteQuat::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(reftype != "body") this->set_attribute("reftype", reftype);
  if(objtype != "body") this->set_attribute("objtype", objtype);
  if(!objname.empty()) this->set_attribute("objname", objname);
}

bool SiteQuat::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool SiteQuat::is_default_value(const std::string& name, const AttributeValue& value) const {
  if(name == "reftype" && std::get<std::string>(value) == "body") return true;
  if(name == "objtype" && std::get<std::string>(value) == "body") return true;
  return BaseSensor::is_default_value(name, value);
}

// SiteLinVel implementation
SiteLinVel::SiteLinVel() : BaseSensor("framelinvel") {}

void SiteLinVel::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(reftype != "body") this->set_attribute("reftype", reftype);
  if(objtype != "body") this->set_attribute("objtype", objtype);
  if(!objname.empty()) this->set_attribute("objname", objname);
}

bool SiteLinVel::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool SiteLinVel::is_default_value(const std::string& name, const AttributeValue& value) const {
  if(name == "reftype" && std::get<std::string>(value) == "body") return true;
  if(name == "objtype" && std::get<std::string>(value) == "body") return true;
  return BaseSensor::is_default_value(name, value);
}

// SiteAngVel implementation
SiteAngVel::SiteAngVel() : BaseSensor("frameangvel") {}

void SiteAngVel::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(reftype != "body") this->set_attribute("reftype", reftype);
  if(objtype != "body") this->set_attribute("objtype", objtype);
  if(!objname.empty()) this->set_attribute("objname", objname);
}

bool SiteAngVel::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool SiteAngVel::is_default_value(const std::string& name, const AttributeValue& value) const {
  if(name == "reftype" && std::get<std::string>(value) == "body") return true;
  if(name == "objtype" && std::get<std::string>(value) == "body") return true;
  return BaseSensor::is_default_value(name, value);
}

// SubtreeCom implementation
SubtreeCom::SubtreeCom() : BaseSensor("subtreecom") {}

void SubtreeCom::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!body.empty()) this->set_attribute("body", body);
}

bool SubtreeCom::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool SubtreeCom::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// SubtreeLinVel implementation
SubtreeLinVel::SubtreeLinVel() : BaseSensor("subtreelinvel") {}

void SubtreeLinVel::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!body.empty()) this->set_attribute("body", body);
}

bool SubtreeLinVel::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool SubtreeLinVel::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// SubtreeAngMom implementation
SubtreeAngMom::SubtreeAngMom() : BaseSensor("subtreeangmom") {}

void SubtreeAngMom::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!body.empty()) this->set_attribute("body", body);
}

bool SubtreeAngMom::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool SubtreeAngMom::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// Gyro implementation
Gyro::Gyro() : BaseSensor("gyro") {}

void Gyro::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!site.empty()) this->set_attribute("site", site);
}

bool Gyro::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool Gyro::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// Accelerometer implementation
Accelerometer::Accelerometer() : BaseSensor("accelerometer") {}

void Accelerometer::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!site.empty()) this->set_attribute("site", site);
}

bool Accelerometer::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool Accelerometer::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// Magnetometer implementation
Magnetometer::Magnetometer() : BaseSensor("magnetometer") {}

void Magnetometer::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!site.empty()) this->set_attribute("site", site);
}

bool Magnetometer::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool Magnetometer::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// Rangefinder implementation
Rangefinder::Rangefinder() : BaseSensor("rangefinder") {}

void Rangefinder::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!site.empty()) this->set_attribute("site", site);
}

bool Rangefinder::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool Rangefinder::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// Force implementation
Force::Force() : BaseSensor("force") {}

void Force::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!site.empty()) this->set_attribute("site", site);
}

bool Force::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool Force::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// Torque implementation
Torque::Torque() : BaseSensor("torque") {}

void Torque::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!site.empty()) this->set_attribute("site", site);
}

bool Torque::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool Torque::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// Touch implementation
Touch::Touch() : BaseSensor("touch") {}

void Touch::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!site.empty()) this->set_attribute("site", site);
}

bool Touch::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool Touch::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

} // namespace mjcf
