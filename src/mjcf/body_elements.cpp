#include "body_elements.hpp"

namespace mjcf {

// Helper function to sanitize body names
std::string sanitize_body_name(const std::string& name) {
  if(name == "world") {
    return "world.001";
  }
  return name;
}

Body::Body() = default;

void Body::set_xml_attrib() const {
  // Only set non-default values
  // Sanitize the name to avoid reserved names like "world"
  if(!name.empty()) {
    std::string sanitized_name = sanitize_body_name(name);
    this->set_attribute("name", sanitized_name);
  }
  if(pos != Arr3{0.0, 0.0, 0.0}) {
    this->set_attribute("pos", std::vector<double>(pos.begin(), pos.end()));
  }
  if(quat != std::array<double, 4>{1.0, 0.0, 0.0, 0.0}) {
    this->set_attribute("quat", std::vector<double>(quat.begin(), quat.end()));
  }
  if(axisangle != std::array<double, 4>{0.0, 0.0, 0.0, 0.0}) {
    this->set_attribute("axisangle", std::vector<double>(axisangle.begin(), axisangle.end()));
  }
  if(euler != Arr3{0.0, 0.0, 0.0}) {
    this->set_attribute("euler", std::vector<double>(euler.begin(), euler.end()));
  }
  if(xyaxes != std::array<double, 6>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}) {
    this->set_attribute("xyaxes", std::vector<double>(xyaxes.begin(), xyaxes.end()));
  }
  if(zaxis != Arr3{0.0, 0.0, 0.0}) {
    this->set_attribute("zaxis", std::vector<double>(zaxis.begin(), zaxis.end()));
  }
  if(mocap) this->set_attribute("mocap", mocap);
  if(!childclass.empty()) this->set_attribute("childclass", childclass);
}

bool Body::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool Body::is_default_value(const std::string& name, const AttributeValue& value) const {
  if(name == "mocap" && !std::get<bool>(value)) return true;
  return false;
}

Geom::Geom() = default;

void Geom::set_xml_attrib() const {
  // Only set non-default values
  if(!name.empty()) this->set_attribute("name", name);
  if(!class_.empty()) this->set_attribute("class", class_);
  this->set_attribute("type", to_string(type));
  if(size != Arr3{0.0, 0.0, 0.0}) {
    this->set_attribute("size", std::vector<double>(size.begin(), size.end()));
  }
  if(pos != Arr3{0.0, 0.0, 0.0}) {
    this->set_attribute("pos", std::vector<double>(pos.begin(), pos.end()));
  }
  if(quat != std::array<double, 4>{1.0, 0.0, 0.0, 0.0}) {
    this->set_attribute("quat", std::vector<double>(quat.begin(), quat.end()));
  }
  if(rgba != std::array<double, 4>{0.5, 0.5, 0.5, 1.0}) {
    this->set_attribute("rgba", std::vector<double>(rgba.begin(), rgba.end()));
  }
  if(!material.empty()) this->set_attribute("material", material);
  if(contype != 1) this->set_attribute("contype", contype);
  if(conaffinity != 1) this->set_attribute("conaffinity", conaffinity);
  if(condim != 3) this->set_attribute("condim", condim);
  if(group != 0) this->set_attribute("group", group);
  if(priority != 0) this->set_attribute("priority", priority);
  if(friction != Arr3{1.0, 0.005, 0.0001}) {
    this->set_attribute("friction", std::vector<double>(friction.begin(), friction.end()));
  }
  if(solmix != 1.0) this->set_attribute("solmix", solmix);
  if(solref != std::array<double, 2>{0.02, 1.0}) {
    this->set_attribute("solref", std::vector<double>(solref.begin(), solref.end()));
  }
  if(solimp != std::array<double, 3>{0.9, 0.95, 0.001}) { 
    this->set_attribute("solimp", std::vector<double>(solimp.begin(), solimp.end()));
  }
  if(margin != 0.0) this->set_attribute("margin", margin);
  if(gap != 0.0) this->set_attribute("gap", gap);
  if(fromto != std::array<double, 6>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}) {
    this->set_attribute("fromto", std::vector<double>(fromto.begin(), fromto.end()));
  }
  if(axisangle != std::array<double, 4>{0.0, 0.0, 0.0, 0.0}) {
    this->set_attribute("axisangle", std::vector<double>(axisangle.begin(), axisangle.end()));
  }
  if(euler != Arr3{0.0, 0.0, 0.0}) {
    this->set_attribute("euler", std::vector<double>(euler.begin(), euler.end()));
  }
  if(xyaxes != std::array<double, 6>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}) {
    this->set_attribute("xyaxes", std::vector<double>(xyaxes.begin(), xyaxes.end()));
  }
  if(zaxis != Arr3{0.0, 0.0, 0.0}) {
    this->set_attribute("zaxis", std::vector<double>(zaxis.begin(), zaxis.end()));
  }
  if(!hfield.empty()) this->set_attribute("hfield", hfield);
  if(!mesh.empty()) this->set_attribute("mesh", mesh);
  if(fitscale != 1.0) this->set_attribute("fitscale", fitscale);
}

bool Geom::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool Geom::is_default_value(const std::string& name, const AttributeValue& value) const {
  // Only filter out true defaults, not common values that should be shown
  if(name == "margin" && std::get<double>(value) == 0.0) return true;
  if(name == "gap" && std::get<double>(value) == 0.0) return true;
  if(name == "fitscale" && std::get<double>(value) == 1.0) return true;
  // Don't filter out common contact and geometry settings
  return false;
}

// Light implementation
Light::Light(bool directional_, const Arr3& pos_, const Arr3& dir_, const Arr3& diffuse_) : directional(directional_), pos(pos_), dir(dir_), diffuse(diffuse_) {}

void Light::set_xml_attrib() const {

  // Only set non-default values
  if(!name.empty()) this->set_attribute("name", name);
  if(!class_.empty()) this->set_attribute("class", class_);
  if(directional) this->set_attribute("directional", directional);
  if(!castshadow) this->set_attribute("castshadow", castshadow);
  if(!active) this->set_attribute("active", active);
  if(pos != Arr3{0.0, 0.0, 0.0}) {
    this->set_attribute("pos", std::vector<double>(pos.begin(), pos.end()));
  }
  if(dir != Arr3{0.0, 0.0, -1.0}) {
    this->set_attribute("dir", std::vector<double>(dir.begin(), dir.end()));
  }
  if(attenuation != Arr3{1.0, 0.0, 0.0}) {
    this->set_attribute("attenuation", std::vector<double>(attenuation.begin(), attenuation.end()));
  }
  if(cutoff != 45.0) this->set_attribute("cutoff", cutoff);
  if(exponent != 10.0) this->set_attribute("exponent", exponent);
  if(ambient != Arr3{0.0, 0.0, 0.0}) {
    this->set_attribute("ambient", std::vector<double>(ambient.begin(), ambient.end()));
  }
  if(diffuse != Arr3{0.7, 0.7, 0.7}) {
    this->set_attribute("diffuse", std::vector<double>(diffuse.begin(), diffuse.end()));
  }
  if(specular != Arr3{0.3, 0.3, 0.3}) {
    this->set_attribute("specular", std::vector<double>(specular.begin(), specular.end()));
  }
  if(mode != LightMode::Fixed) this->set_attribute("mode", to_string(mode));
}

bool Light::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool Light::is_default_value(const std::string& name, const AttributeValue& value) const {
  // Only filter actual defaults that are not commonly set
  if(name == "cutoff" && std::get<double>(value) == 45.0) return true;
  if(name == "exponent" && std::get<double>(value) == 10.0) return true;
  // Don't filter common settings
  return false;
}

Camera::Camera() = default;

void Camera::set_xml_attrib() const {

  // Only set non-default values
  if(!name.empty()) this->set_attribute("name", name);
  if(!class_.empty()) this->set_attribute("class", class_);
  if(mode != CameraMode::Fixed) this->set_attribute("mode", to_string(mode));
  if(!target.empty()) this->set_attribute("target", target);
  if(pos != Arr3{0.0, 0.0, 0.0}) {
    this->set_attribute("pos", std::vector<double>(pos.begin(), pos.end()));
  }
  if(quat != std::array<double, 4>{1.0, 0.0, 0.0, 0.0}) {
    this->set_attribute("quat", std::vector<double>(quat.begin(), quat.end()));
  }
  if(axisangle != std::array<double, 4>{0.0, 0.0, 0.0, 0.0}) {
    this->set_attribute("axisangle", std::vector<double>(axisangle.begin(), axisangle.end()));
  }
  if(euler != Arr3{0.0, 0.0, 0.0}) {
    this->set_attribute("euler", std::vector<double>(euler.begin(), euler.end()));
  }
  if(xyaxes != std::array<double, 6>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}) {
    this->set_attribute("xyaxes", std::vector<double>(xyaxes.begin(), xyaxes.end()));
  }
  if(zaxis != Arr3{0.0, 0.0, 0.0}) {
    this->set_attribute("zaxis", std::vector<double>(zaxis.begin(), zaxis.end()));
  }
  if(fovy != 45.0) this->set_attribute("fovy", fovy);
  if(ipd != 0.068) this->set_attribute("ipd", ipd);
}

bool Camera::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool Camera::is_default_value(const std::string& name, const AttributeValue& value) const {
  if(name == "mode" && std::get<std::string>(value) == to_string(CameraMode::Fixed)) return true;
  if(name == "fovy" && std::get<double>(value) == 45.0) return true;
  if(name == "ipd" && std::get<double>(value) == 0.068) return true;
  return false;
}

// Site implementation
Site::Site() = default;

void Site::set_xml_attrib() const {

  // Only set non-default values
  if(!name.empty()) this->set_attribute("name", name);
  if(!class_.empty()) this->set_attribute("class", class_);
  if(type != SiteType::Sphere) this->set_attribute("type", to_string(type));
  if(size != Arr3{0.005, 0.005, 0.005}) {
    this->set_attribute("size", std::vector<double>(size.begin(), size.end()));
  }
  if(pos != Arr3{0.0, 0.0, 0.0}) {
    this->set_attribute("pos", std::vector<double>(pos.begin(), pos.end()));
  }
  if(quat != std::array<double, 4>{1.0, 0.0, 0.0, 0.0}) {
    this->set_attribute("quat", std::vector<double>(quat.begin(), quat.end()));
  }
  if(axisangle != std::array<double, 4>{0.0, 0.0, 0.0, 0.0}) {
    this->set_attribute("axisangle", std::vector<double>(axisangle.begin(), axisangle.end()));
  }
  if(euler != Arr3{0.0, 0.0, 0.0}) {
    this->set_attribute("euler", std::vector<double>(euler.begin(), euler.end()));
  }
  if(xyaxes != std::array<double, 6>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}) {
    this->set_attribute("xyaxes", std::vector<double>(xyaxes.begin(), xyaxes.end()));
  }
  if(zaxis != Arr3{0.0, 0.0, 0.0}) {
    this->set_attribute("zaxis", std::vector<double>(zaxis.begin(), zaxis.end()));
  }
  if(rgba != std::array<double, 4>{0.5, 0.5, 0.5, 1.0}) {
    this->set_attribute("rgba", std::vector<double>(rgba.begin(), rgba.end()));
  }
  if(!material.empty()) this->set_attribute("material", material);
  if(group != 0) this->set_attribute("group", group);
  if(fromto != std::array<double, 6>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}) {
    this->set_attribute("fromto", std::vector<double>(fromto.begin(), fromto.end()));
  }
}

bool Site::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool Site::is_default_value(const std::string& name, const AttributeValue& value) const {
  // Only filter truly unnecessary defaults
  if(name == "group" && std::get<int>(value) == 0) return true;
  return false;
}

void Joint::set_xml_attrib() const {
  this->set_attribute("name", name);
  this->set_attribute("type", to_string(type));

  if(!class_.empty()) this->set_attribute("class", class_);
  if(group != 0) this->set_attribute("group", group);
  if(pos != Arr3{0.0, 0.0, 0.0}) this->set_attribute("pos", std::vector<double>(pos.begin(), pos.end()));
  if(axis != Arr3{0.0, 0.0, 1.0}) this->set_attribute("axis", std::vector<double>(axis.begin(), axis.end()));
  if(springdamper != std::array<double, 2>{0.0, 0.0}) {
    this->set_attribute("springdamper", std::vector<double>(springdamper.begin(), springdamper.end()));
  }
  if(limited) this->set_attribute("limited", limited);
  if(actuatorforcelimited) this->set_attribute("actuatorforcelimited", actuatorforcelimited);
  if(actuatorfrcrange != std::array<double, 2>{0.0, 0.0}) {
    this->set_attribute("actuatorfrcrange", std::vector<double>(actuatorfrcrange.begin(), actuatorfrcrange.end()));
  }
  if(range != std::array<double, 2>{0.0, 0.0}) {
    this->set_attribute("range", std::vector<double>(range.begin(), range.end()));
  }
  if(margin != 0.0) this->set_attribute("margin", margin);
  if(ref != 0.0) this->set_attribute("ref", ref);
  if(springref != 0.0) this->set_attribute("springref", springref);
  if(armature != 0.0) this->set_attribute("armature", armature);
  if(damping != 0.0) this->set_attribute("damping", damping);
  if(frictionloss != 0.0) this->set_attribute("frictionloss", frictionloss);
  if(solreflimit != std::array<double, 2>{0.02, 1.0}) {
    this->set_attribute("solreflimit", std::vector<double>(solreflimit.begin(), solreflimit.end()));
  }
  if(solimplimit != std::array<double, 3>{0.9, 0.95, 0.001}) {
    this->set_attribute("solimplimit", std::vector<double>(solimplimit.begin(), solimplimit.end()));
  }
  if(solreffriction != std::array<double, 2>{0.02, 1.0}) {
    this->set_attribute("solreffriction", std::vector<double>(solreffriction.begin(), solreffriction.end()));
  }
  if(solimpfriction != std::array<double, 5>{0.9, 0.95, 0.001, 0.5, 2.0}) {
    this->set_attribute("solimpfriction", std::vector<double>(solimpfriction.begin(), solimpfriction.end()));
  }
  if(stiffness != 0.0) this->set_attribute("stiffness", stiffness);
  if(user != Arr3{0.0, 0.0, 0.0}) {
    this->set_attribute("user", std::vector<double>(user.begin(), user.end()));
  }
}

bool Joint::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool Joint::is_default_value(const std::string& name, const AttributeValue& value) const {
  if(name == "group" && std::get<int>(value) == 0) return true;
  if(name == "margin" && std::get<double>(value) == 0.0) return true;
  if(name == "ref" && std::get<double>(value) == 0.0) return true;
  if(name == "springref" && std::get<double>(value) == 0.0) return true;
  if(name == "armature" && std::get<double>(value) == 0.0) return true;
  if(name == "damping" && std::get<double>(value) == 0.0) return true;
  if(name == "frictionloss" && std::get<double>(value) == 0.0) return true;
  if(name == "stiffness" && std::get<double>(value) == 0.0) return true;
  return false;
}

void Inertial::set_xml_attrib() const {

  this->set_attribute("pos", std::vector<double>(pos.begin(), pos.end()));
  this->set_attribute("mass", mass);

  if(quat != std::array<double, 4>{1.0, 0.0, 0.0, 0.0}) {
    this->set_attribute("quat", std::vector<double>(quat.begin(), quat.end()));
  }
  if(fullinertia != std::array<double, 6>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}) {
    this->set_attribute("fullinertia", std::vector<double>(fullinertia.begin(), fullinertia.end()));
  } else {
    this->set_attribute("diaginertia", std::vector<double>(diaginertia.begin(), diaginertia.end()));
  }
}

bool Inertial::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool Inertial::is_default_value(const std::string& name, const AttributeValue& value) const {
  if(name == "mass" && std::get<double>(value) == 1.0) return true;
  return false;
}

} // namespace mjcf
