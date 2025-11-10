#include "enums.hpp"
#include <stdexcept>
#include <unordered_map>

namespace mjcf {

// GeomType conversions
std::string to_string(GeomType type) {
  switch(type) {
    case GeomType::Plane: return "plane";
    case GeomType::Hfield: return "hfield";
    case GeomType::Sphere: return "sphere";
    case GeomType::Capsule: return "capsule";
    case GeomType::Ellipsoid: return "ellipsoid";
    case GeomType::Cylinder: return "cylinder";
    case GeomType::Box: return "box";
    case GeomType::Mesh: return "mesh";
  }
  throw std::invalid_argument("Invalid GeomType");
}

GeomType geom_type_from_string(const std::string& str) {
  static const std::unordered_map<std::string, GeomType> map = {{"plane", GeomType::Plane},         {"hfield", GeomType::Hfield},     {"sphere", GeomType::Sphere}, {"capsule", GeomType::Capsule},
                                                                {"ellipsoid", GeomType::Ellipsoid}, {"cylinder", GeomType::Cylinder}, {"box", GeomType::Box},       {"mesh", GeomType::Mesh}};
  auto it                                                    = map.find(str);
  if(it != map.end()) return it->second;
  throw std::invalid_argument("Invalid geom type string: " + str);
}

// SiteType conversions
std::string to_string(SiteType type) {
  switch(type) {
    case SiteType::Sphere: return "sphere";
    case SiteType::Capsule: return "capsule";
    case SiteType::Ellipsoid: return "ellipsoid";
    case SiteType::Cylinder: return "cylinder";
    case SiteType::Box: return "box";
  }
  throw std::invalid_argument("Invalid SiteType");
}

SiteType site_type_from_string(const std::string& str) {
  static const std::unordered_map<std::string, SiteType> map = {{"sphere", SiteType::Sphere}, {"capsule", SiteType::Capsule}, {"ellipsoid", SiteType::Ellipsoid}, {"cylinder", SiteType::Cylinder}, {"box", SiteType::Box}};
  auto it                                                    = map.find(str);
  if(it != map.end()) return it->second;
  throw std::invalid_argument("Invalid site type string: " + str);
}

// JointType conversions
std::string to_string(JointType type) {
  switch(type) {
    case JointType::Free: return "free";
    case JointType::Ball: return "ball";
    case JointType::Slide: return "slide";
    case JointType::Hinge: return "hinge";
  }
  throw std::invalid_argument("Invalid JointType");
}

JointType joint_type_from_string(const std::string& str) {
  static const std::unordered_map<std::string, JointType> map = {{"free", JointType::Free}, {"ball", JointType::Ball}, {"slide", JointType::Slide}, {"hinge", JointType::Hinge}};
  auto it                                                     = map.find(str);
  if(it != map.end()) return it->second;
  throw std::invalid_argument("Invalid joint type string: " + str);
}

// LightMode conversions
std::string to_string(LightMode mode) {
  switch(mode) {
    case LightMode::Fixed: return "fixed";
    case LightMode::Track: return "track";
    case LightMode::Trackcom: return "trackcom";
    case LightMode::Targetbody: return "targetbody";
    case LightMode::Targetbodycom: return "targetbodycom";
  }
  throw std::invalid_argument("Invalid LightMode");
}

LightMode light_mode_from_string(const std::string& str) {
  static const std::unordered_map<std::string, LightMode> map = {{"fixed", LightMode::Fixed}, {"track", LightMode::Track}, {"trackcom", LightMode::Trackcom}, {"targetbody", LightMode::Targetbody}, {"targetbodycom", LightMode::Targetbodycom}};
  auto it                                                     = map.find(str);
  if(it != map.end()) return it->second;
  throw std::invalid_argument("Invalid light mode string: " + str);
}

// CameraMode conversions
std::string to_string(CameraMode mode) {
  switch(mode) {
    case CameraMode::Fixed: return "fixed";
    case CameraMode::Track: return "track";
    case CameraMode::Trackcom: return "trackcom";
    case CameraMode::Targetbody: return "targetbody";
    case CameraMode::Targetbodycom: return "targetbodycom";
  }
  throw std::invalid_argument("Invalid CameraMode");
}

CameraMode camera_mode_from_string(const std::string& str) {
  static const std::unordered_map<std::string, CameraMode> map = {{"fixed", CameraMode::Fixed}, {"track", CameraMode::Track}, {"trackcom", CameraMode::Trackcom}, {"targetbody", CameraMode::Targetbody}, {"targetbodycom", CameraMode::Targetbodycom}};
  auto it                                                      = map.find(str);
  if(it != map.end()) return it->second;
  throw std::invalid_argument("Invalid camera mode string: " + str);
}

std::string to_string(SolverType type) {
  switch(type) {
    case SolverType::Newton: return "Newton";
    case SolverType::PGS: return "PGS";
    case SolverType::CG: return "CG";
  }
  throw std::invalid_argument("Invalid CameraType");
}

std::string to_string(IntegratorType type) {
  switch(type) {
    case IntegratorType::Euler: return "Euler";
    case IntegratorType::RK4: return "RK4";
    case IntegratorType::Implicit: return "implicit";
  }
  throw std::invalid_argument("Invalid IntegratorType");
}

IntegratorType integrator_type_from_string(const std::string& str) {
  static const std::unordered_map<std::string, IntegratorType> map = {{"Euler", IntegratorType::Euler}, {"RK4", IntegratorType::RK4}, {"implicit", IntegratorType::Implicit}};
  auto it                                                          = map.find(str);
  if(it != map.end()) return it->second;
  throw std::invalid_argument("Invalid integrator type string: " + str);
}

// CoordinateType conversions
std::string to_string(CoordinateType type) {
  switch(type) {
    case CoordinateType::Local: return "local";
    case CoordinateType::Global: return "global";
  }
  throw std::invalid_argument("Invalid CoordinateType");
}

CoordinateType coordinate_type_from_string(const std::string& str) {
  static const std::unordered_map<std::string, CoordinateType> map = {{"local", CoordinateType::Local}, {"global", CoordinateType::Global}};
  auto it                                                          = map.find(str);
  if(it != map.end()) return it->second;
  throw std::invalid_argument("Invalid coordinate type string: " + str);
}

// AngleUnit conversions
std::string to_string(AngleUnit unit) {
  switch(unit) {
    case AngleUnit::Degree: return "degree";
    case AngleUnit::Radian: return "radian";
  }
  throw std::invalid_argument("Invalid AngleUnit");
}

AngleUnit angle_unit_from_string(const std::string& str) {
  static const std::unordered_map<std::string, AngleUnit> map = {{"degree", AngleUnit::Degree}, {"radian", AngleUnit::Radian}};
  auto it                                                     = map.find(str);
  if(it != map.end()) return it->second;
  throw std::invalid_argument("Invalid angle unit string: " + str);
}

// ActuatorType conversions
std::string to_string(ActuatorType type) {
  switch(type) {
    case ActuatorType::Motor: return "motor";
    case ActuatorType::Position: return "position";
    case ActuatorType::Velocity: return "velocity";
    case ActuatorType::Cylinder: return "cylinder";
    case ActuatorType::Muscle: return "muscle";
    case ActuatorType::Tendon: return "tendon";
    case ActuatorType::Damper: return "damper";
    case ActuatorType::General: return "general";
    case ActuatorType::Plugin: return "plugin";
    case ActuatorType::Intvelocity: return "intvelocity";
  }
  throw std::invalid_argument("Invalid ActuatorType");
}

ActuatorType actuator_type_from_string(const std::string& str) {
  static const std::unordered_map<std::string, ActuatorType> map = {{"motor", ActuatorType::Motor},   {"position", ActuatorType::Position}, {"velocity", ActuatorType::Velocity}, {"cylinder", ActuatorType::Cylinder}, {"muscle", ActuatorType::Muscle},
                                                                    {"tendon", ActuatorType::Tendon}, {"damper", ActuatorType::Damper},     {"general", ActuatorType::General},   {"plugin", ActuatorType::Plugin},     {"intvelocity", ActuatorType::Intvelocity}};
  auto it                                                        = map.find(str);
  if(it != map.end()) return it->second;
  throw std::invalid_argument("Invalid actuator type string: " + str);
}

// TransmissionType conversions
std::string to_string(TransmissionType type) {
  switch(type) {
    case TransmissionType::Joint: return "joint";
    case TransmissionType::JointInParent: return "jointinparent";
    case TransmissionType::Slide: return "slide";
    case TransmissionType::Tendon: return "tendon";
    case TransmissionType::Site: return "site";
    case TransmissionType::Body: return "body";
  }
  throw std::invalid_argument("Invalid TransmissionType");
}

TransmissionType transmission_type_from_string(const std::string& str) {
  static const std::unordered_map<std::string, TransmissionType> map = {{"joint", TransmissionType::Joint}, {"jointinparent", TransmissionType::JointInParent}, {"slide", TransmissionType::Slide}, {"tendon", TransmissionType::Tendon}, {"site", TransmissionType::Site},
                                                                        {"body", TransmissionType::Body}};
  auto it                                                            = map.find(str);
  if(it != map.end()) return it->second;
  throw std::invalid_argument("Invalid transmission type string: " + str);
}

// TextureType conversions
std::string to_string(TextureType type) {
  switch(type) {
    case TextureType::Cube: return "cube";
    case TextureType::Skybox: return "skybox";
    case TextureType::TwoD: return "2d";
  }
  throw std::invalid_argument("Invalid TextureType");
}

TextureType texture_type_from_string(const std::string& str) {
  static const std::unordered_map<std::string, TextureType> map = {{"cube", TextureType::Cube}, {"skybox", TextureType::Skybox}, {"2d", TextureType::TwoD}};
  auto it                                                       = map.find(str);
  if(it != map.end()) return it->second;
  throw std::invalid_argument("Invalid texture type string: " + str);
}

// TextureBuiltin conversions
std::string to_string(TextureBuiltin builtin) {
  switch(builtin) {
    case TextureBuiltin::None: return "none";
    case TextureBuiltin::Gradient: return "gradient";
    case TextureBuiltin::Checker: return "checker";
    case TextureBuiltin::Flat: return "flat";
  }
  throw std::invalid_argument("Invalid TextureBuiltin");
}

TextureBuiltin texture_builtin_from_string(const std::string& str) {
  static const std::unordered_map<std::string, TextureBuiltin> map = {{"none", TextureBuiltin::None}, {"gradient", TextureBuiltin::Gradient}, {"checker", TextureBuiltin::Checker}, {"flat", TextureBuiltin::Flat}};
  auto it                                                          = map.find(str);
  if(it != map.end()) return it->second;
  throw std::invalid_argument("Invalid texture builtin string: " + str);
}

} // namespace mjcf
