#include "sensor_elements.hpp"

namespace mjcf {

BaseSensor::BaseSensor(const std::string& element_name) : element_name_(element_name) {}

void BaseSensor::set_xml_attrib() const {

  // Only set non-default values
  if(!name.empty()) this->set_attribute("name", name);
  if(noise != 0.0) this->set_attribute("noise", noise);
  if(cutoff != 0.0) this->set_attribute("cutoff", cutoff);
  if(nsample != 0) this->set_attribute("nsample", nsample);
  if(!interp.empty()) this->set_attribute("interp", interp);
  if(delay != 0.0) this->set_attribute("delay", delay);
  if(interval != 0.0) this->set_attribute("interval", interval);
  if(user != std::array<double, 3>{0.0, 0.0, 0.0}) {
    this->set_attribute("user", std::vector<double>(user.begin(), user.end()));
  }
}


bool BaseSensor::is_default_value(const std::string& name, const AttributeValue& value) const {
  if(name == "noise" && std::get<double>(value) == 0.0) return true;
  if(name == "cutoff" && std::get<double>(value) == 0.0) return true;
  if(name == "nsample" && std::get<int>(value) == 0) return true;
  if(name == "interp" && std::get<std::string>(value).empty()) return true;
  if(name == "delay" && std::get<double>(value) == 0.0) return true;
  if(name == "interval" && std::get<double>(value) == 0.0) return true;
  return false;
}

JointPos::JointPos() : BaseSensor("jointpos") {}

void JointPos::set_xml_attrib() const {
  // Set base class attributes first
  BaseSensor::set_xml_attrib();

  if(!joint.empty()) this->set_attribute("joint", joint);
}


bool JointPos::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// JointVel implementation
JointVel::JointVel() : BaseSensor("jointvel") {}

void JointVel::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!joint.empty()) this->set_attribute("joint", joint);
}


bool JointVel::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// TendonPos implementation
TendonPos::TendonPos() : BaseSensor("tendonpos") {}

void TendonPos::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!tendon.empty()) this->set_attribute("tendon", tendon);
}


bool TendonPos::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// TendonVel implementation
TendonVel::TendonVel() : BaseSensor("tendonvel") {}

void TendonVel::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!tendon.empty()) this->set_attribute("tendon", tendon);
}


bool TendonVel::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// ActuatorPos implementation
ActuatorPos::ActuatorPos() : BaseSensor("actuatorpos") {}

void ActuatorPos::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!actuator.empty()) this->set_attribute("actuator", actuator);
}


bool ActuatorPos::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// ActuatorVel implementation
ActuatorVel::ActuatorVel() : BaseSensor("actuatorvel") {}

void ActuatorVel::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!actuator.empty()) this->set_attribute("actuator", actuator);
}


bool ActuatorVel::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// ActuatorFrc implementation
ActuatorFrc::ActuatorFrc() : BaseSensor("actuatorfrc") {}

void ActuatorFrc::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!actuator.empty()) this->set_attribute("actuator", actuator);
}


bool ActuatorFrc::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// BallQuat implementation
BallQuat::BallQuat() : BaseSensor("ballquat") {}

void BallQuat::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!joint.empty()) this->set_attribute("joint", joint);
}


bool BallQuat::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// BallAngVel implementation
BallAngVel::BallAngVel() : BaseSensor("ballangvel") {}

void BallAngVel::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!joint.empty()) this->set_attribute("joint", joint);
}


bool BallAngVel::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// SitePos implementation
SitePos::SitePos() : BaseSensor("framepos") {}

void SitePos::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(reftype != "body") this->set_attribute("reftype", reftype);
  if(objtype != "body") this->set_attribute("objtype", objtype);
  if(!objname.empty()) this->set_attribute("objname", objname);
  if(!refname.empty()) this->set_attribute("refname", refname);
}


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
  if(!refname.empty()) this->set_attribute("refname", refname);
}


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
  if(!refname.empty()) this->set_attribute("refname", refname);
}


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
  if(!refname.empty()) this->set_attribute("refname", refname);
}


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


bool SubtreeCom::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// SubtreeLinVel implementation
SubtreeLinVel::SubtreeLinVel() : BaseSensor("subtreelinvel") {}

void SubtreeLinVel::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!body.empty()) this->set_attribute("body", body);
}


bool SubtreeLinVel::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// SubtreeAngMom implementation
SubtreeAngMom::SubtreeAngMom() : BaseSensor("subtreeangmom") {}

void SubtreeAngMom::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!body.empty()) this->set_attribute("body", body);
}


bool SubtreeAngMom::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// Gyro implementation
Gyro::Gyro() : BaseSensor("gyro") {}

void Gyro::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!site.empty()) this->set_attribute("site", site);
}


bool Gyro::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// Accelerometer implementation
Accelerometer::Accelerometer() : BaseSensor("accelerometer") {}

void Accelerometer::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!site.empty()) this->set_attribute("site", site);
}


bool Accelerometer::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// Magnetometer implementation
Magnetometer::Magnetometer() : BaseSensor("magnetometer") {}

void Magnetometer::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!site.empty()) this->set_attribute("site", site);
}


bool Magnetometer::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// Rangefinder implementation
Rangefinder::Rangefinder() : BaseSensor("rangefinder") {}

void Rangefinder::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!site.empty()) this->set_attribute("site", site);
}


bool Rangefinder::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// Force implementation
Force::Force() : BaseSensor("force") {}

void Force::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!site.empty()) this->set_attribute("site", site);
}


bool Force::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// Torque implementation
Torque::Torque() : BaseSensor("torque") {}

void Torque::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!site.empty()) this->set_attribute("site", site);
}


bool Torque::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// Touch implementation
Touch::Touch() : BaseSensor("touch") {}

void Touch::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!site.empty()) this->set_attribute("site", site);
}


bool Touch::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// ===== フェーズ1: 優先度高 =====

// Velocimeter implementation
Velocimeter::Velocimeter() : BaseSensor("velocimeter") {}

void Velocimeter::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!site.empty()) this->set_attribute("site", site);
}

bool Velocimeter::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// Clock implementation
Clock::Clock() : BaseSensor("clock") {}

void Clock::set_xml_attrib() const { BaseSensor::set_xml_attrib(); }

bool Clock::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// FrameLinAcc implementation
FrameLinAcc::FrameLinAcc() : BaseSensor("framelinacc") {}

void FrameLinAcc::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(objtype != "body") this->set_attribute("objtype", objtype);
  if(!objname.empty()) this->set_attribute("objname", objname);
}

bool FrameLinAcc::is_default_value(const std::string& name, const AttributeValue& value) const {
  if(name == "objtype" && std::get<std::string>(value) == "body") return true;
  return BaseSensor::is_default_value(name, value);
}

// FrameAngAcc implementation
FrameAngAcc::FrameAngAcc() : BaseSensor("frameangacc") {}

void FrameAngAcc::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(objtype != "body") this->set_attribute("objtype", objtype);
  if(!objname.empty()) this->set_attribute("objname", objname);
}

bool FrameAngAcc::is_default_value(const std::string& name, const AttributeValue& value) const {
  if(name == "objtype" && std::get<std::string>(value) == "body") return true;
  return BaseSensor::is_default_value(name, value);
}

// PotentialEnergy implementation
PotentialEnergy::PotentialEnergy() : BaseSensor("e_potential") {}

void PotentialEnergy::set_xml_attrib() const { BaseSensor::set_xml_attrib(); }

bool PotentialEnergy::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// KineticEnergy implementation
KineticEnergy::KineticEnergy() : BaseSensor("e_kinetic") {}

void KineticEnergy::set_xml_attrib() const { BaseSensor::set_xml_attrib(); }

bool KineticEnergy::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// JointActuatorFrc implementation
JointActuatorFrc::JointActuatorFrc() : BaseSensor("jointactuatorfrc") {}

void JointActuatorFrc::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!joint.empty()) this->set_attribute("joint", joint);
}

bool JointActuatorFrc::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// TendonActuatorFrc implementation
TendonActuatorFrc::TendonActuatorFrc() : BaseSensor("tendonactuatorfrc") {}

void TendonActuatorFrc::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!tendon.empty()) this->set_attribute("tendon", tendon);
}

bool TendonActuatorFrc::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// ===== フェーズ2: 優先度中 =====

// JointLimitPos implementation
JointLimitPos::JointLimitPos() : BaseSensor("jointlimitpos") {}

void JointLimitPos::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!joint.empty()) this->set_attribute("joint", joint);
}

bool JointLimitPos::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// JointLimitVel implementation
JointLimitVel::JointLimitVel() : BaseSensor("jointlimitvel") {}

void JointLimitVel::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!joint.empty()) this->set_attribute("joint", joint);
}

bool JointLimitVel::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// JointLimitFrc implementation
JointLimitFrc::JointLimitFrc() : BaseSensor("jointlimitfrc") {}

void JointLimitFrc::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!joint.empty()) this->set_attribute("joint", joint);
}

bool JointLimitFrc::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// TendonLimitPos implementation
TendonLimitPos::TendonLimitPos() : BaseSensor("tendonlimitpos") {}

void TendonLimitPos::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!tendon.empty()) this->set_attribute("tendon", tendon);
}

bool TendonLimitPos::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// TendonLimitVel implementation
TendonLimitVel::TendonLimitVel() : BaseSensor("tendonlimitvel") {}

void TendonLimitVel::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!tendon.empty()) this->set_attribute("tendon", tendon);
}

bool TendonLimitVel::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// TendonLimitFrc implementation
TendonLimitFrc::TendonLimitFrc() : BaseSensor("tendonlimitfrc") {}

void TendonLimitFrc::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!tendon.empty()) this->set_attribute("tendon", tendon);
}

bool TendonLimitFrc::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// Distance implementation
Distance::Distance() : BaseSensor("distance") {}

void Distance::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  // geom pair優先
  if(!geom1.empty()) this->set_attribute("geom1", geom1);
  if(!geom2.empty()) this->set_attribute("geom2", geom2);

  // body pair (geom未設定時)
  if(geom1.empty() && !body1.empty()) this->set_attribute("body1", body1);
  if(geom2.empty() && !body2.empty()) this->set_attribute("body2", body2);
}

bool Distance::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// Normal implementation
Normal::Normal() : BaseSensor("normal") {}

void Normal::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  // geom pair優先
  if(!geom1.empty()) this->set_attribute("geom1", geom1);
  if(!geom2.empty()) this->set_attribute("geom2", geom2);

  // body pair (geom未設定時)
  if(geom1.empty() && !body1.empty()) this->set_attribute("body1", body1);
  if(geom2.empty() && !body2.empty()) this->set_attribute("body2", body2);
}

bool Normal::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// FromTo implementation
FromTo::FromTo() : BaseSensor("fromto") {}

void FromTo::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  // geom pair優先
  if(!geom1.empty()) this->set_attribute("geom1", geom1);
  if(!geom2.empty()) this->set_attribute("geom2", geom2);

  // body pair (geom未設定時)
  if(geom1.empty() && !body1.empty()) this->set_attribute("body1", body1);
  if(geom2.empty() && !body2.empty()) this->set_attribute("body2", body2);
}

bool FromTo::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// Contact implementation
Contact::Contact() : BaseSensor("contact") {}

void Contact::set_xml_attrib() const { BaseSensor::set_xml_attrib(); }

bool Contact::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

// Tactile implementation
Tactile::Tactile() : BaseSensor("tactile") {}

void Tactile::set_xml_attrib() const {
  BaseSensor::set_xml_attrib();

  if(!geom.empty()) this->set_attribute("geom", geom);
}

bool Tactile::is_default_value(const std::string& name, const AttributeValue& value) const { return BaseSensor::is_default_value(name, value); }

} // namespace mjcf
