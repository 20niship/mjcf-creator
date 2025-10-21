#pragma once

#include "element.hpp"
#include "enums.hpp"
#include <array>

namespace mjcf {

/**
 * @brief Body element
 */
class Body : public Element {
public:
  // Public member variables with MuJoCo default values
  std::string name                = "";
  std::array<double, 3> pos       = {0.0, 0.0, 0.0};
  std::array<double, 4> quat      = {1.0, 0.0, 0.0, 0.0}; // MuJoCo default identity quaternion
  std::array<double, 4> axisangle = {0.0, 0.0, 0.0, 0.0};
  std::array<double, 3> euler     = {0.0, 0.0, 0.0};
  std::array<double, 6> xyaxes    = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 3> zaxis     = {0.0, 0.0, 0.0};
  bool mocap                      = false;
  std::string childclass          = "";

  Body();

  std::string element_name() const override { return "body"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Geom element
 */
class Geom : public Element {
public:
  // Public member variables with MuJoCo default values
  std::string name                = "";
  std::string class_              = "";
  GeomType type                   = GeomType::Sphere; // MuJoCo default
  std::array<double, 3> size      = {0.0, 0.0, 0.0};
  std::array<double, 3> pos       = {0.0, 0.0, 0.0};
  std::array<double, 4> quat      = {1.0, 0.0, 0.0, 0.0};
  std::array<double, 4> rgba      = {0.5, 0.5, 0.5, 1.0}; // MuJoCo default gray
  std::string material            = "";
  int contype                     = 1; // MuJoCo default
  int conaffinity                 = 1; // MuJoCo default
  int condim                      = 3; // MuJoCo default
  int group                       = 0;
  int priority                    = 0;
  std::array<double, 3> friction  = {1.0, 0.005, 0.0001};         // MuJoCo default
  double solmix                   = 1.0;                          // MuJoCo default
  std::array<double, 2> solref    = {0.02, 1.0};                  // MuJoCo default
  std::array<double, 5> solimp    = {0.9, 0.95, 0.001, 0.5, 2.0}; // MuJoCo default
  double margin                   = 0.0;
  double gap                      = 0.0;
  std::array<double, 6> fromto    = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 4> axisangle = {0.0, 0.0, 0.0, 0.0};
  std::array<double, 3> euler     = {0.0, 0.0, 0.0};
  std::array<double, 6> xyaxes    = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 3> zaxis     = {0.0, 0.0, 0.0};
  std::string hfield              = "";
  std::string mesh                = "";
  double fitscale                 = 1.0; // MuJoCo default

  Geom();

  std::string element_name() const override { return "geom"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Light element
 */
class Light : public Element {
public:
  // Public member variables with MuJoCo default values
  std::string name                  = "";
  std::string class_                = "";
  bool directional                  = false;
  bool castshadow                   = true;
  bool active                       = true;
  std::array<double, 3> pos         = {0.0, 0.0, 0.0};
  std::array<double, 3> dir         = {0.0, 0.0, -1.0}; // MuJoCo default pointing down
  std::array<double, 3> attenuation = {1.0, 0.0, 0.0};  // MuJoCo default
  double cutoff                     = 45.0;             // MuJoCo default
  double exponent                   = 10.0;             // MuJoCo default
  std::array<double, 3> ambient     = {0.0, 0.0, 0.0};
  std::array<double, 3> diffuse     = {0.7, 0.7, 0.7}; // MuJoCo default
  std::array<double, 3> specular    = {0.3, 0.3, 0.3}; // MuJoCo default
  LightMode mode                    = LightMode::Fixed; // MuJoCo default

  Light() = default;
  
  // Convenience constructor for directional lights
  Light(bool directional, 
        const std::array<double, 3>& pos, 
        const std::array<double, 3>& dir, 
        const std::array<double, 3>& diffuse = {0.7, 0.7, 0.7});

  std::string element_name() const override { return "light"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Camera element
 */
class Camera : public Element {
public:
  // Public member variables with MuJoCo default values
  std::string name                = "";
  std::string class_              = "";
  CameraMode mode                 = CameraMode::Fixed; // MuJoCo default
  std::string target              = "";
  std::array<double, 3> pos       = {0.0, 0.0, 0.0};
  std::array<double, 4> quat      = {1.0, 0.0, 0.0, 0.0};
  std::array<double, 4> axisangle = {0.0, 0.0, 0.0, 0.0};
  std::array<double, 3> euler     = {0.0, 0.0, 0.0};
  std::array<double, 6> xyaxes    = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 3> zaxis     = {0.0, 0.0, 0.0};
  double fovy                     = 45.0;  // MuJoCo default
  double ipd                      = 0.068; // MuJoCo default

  Camera();

  std::string element_name() const override { return "camera"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Site element
 */
class Site : public Element {
public:
  // Public member variables with MuJoCo default values
  std::string name                = "";
  std::string class_              = "";
  SiteType type                   = SiteType::Sphere;      // MuJoCo default
  std::array<double, 3> size      = {0.005, 0.005, 0.005}; // MuJoCo default
  std::array<double, 3> pos       = {0.0, 0.0, 0.0};
  std::array<double, 4> quat      = {1.0, 0.0, 0.0, 0.0};
  std::array<double, 4> axisangle = {0.0, 0.0, 0.0, 0.0};
  std::array<double, 3> euler     = {0.0, 0.0, 0.0};
  std::array<double, 6> xyaxes    = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 3> zaxis     = {0.0, 0.0, 0.0};
  std::array<double, 4> rgba      = {0.5, 0.5, 0.5, 1.0};
  std::string material            = "";
  int group                       = 0;
  std::array<double, 6> fromto    = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  Site();

  std::string element_name() const override { return "site"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Joint element
 */
class Joint : public Element {
public:
  // Public member variables with MuJoCo default values
  std::string name                       = "";
  std::string class_                     = "";
  JointType type                         = JointType::Hinge; // MuJoCo default
  int group                              = 0;
  std::array<double, 3> pos              = {0.0, 0.0, 0.0};
  std::array<double, 3> axis             = {0.0, 0.0, 1.0}; // MuJoCo default
  std::array<double, 2> springdamper     = {0.0, 0.0};
  bool limited                           = false;
  bool actuatorforcelimited              = false;
  std::array<double, 2> actuatorfrcrange = {0.0, 0.0};
  std::array<double, 2> range            = {0.0, 0.0};
  double margin                          = 0.0;
  double ref                             = 0.0;
  double springref                       = 0.0;
  double armature                        = 0.0;
  double damping                         = 0.0;
  double frictionloss                    = 0.0;
  std::array<double, 2> solreflimit      = {0.02, 1.0};                  // MuJoCo default
  std::array<double, 5> solimplimit      = {0.9, 0.95, 0.001, 0.5, 2.0}; // MuJoCo default
  std::array<double, 2> solreffriction   = {0.02, 1.0};                  // MuJoCo default
  std::array<double, 5> solimpfriction   = {0.9, 0.95, 0.001, 0.5, 2.0}; // MuJoCo default
  double stiffness                       = 0.0;
  std::array<double, 3> user             = {0.0, 0.0, 0.0};

  Joint();

  std::string element_name() const override { return "joint"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Inertial element
 */
class Inertial : public Element {
public:
  // Public member variables with MuJoCo default values
  std::array<double, 3> pos         = {0.0, 0.0, 0.0};
  std::array<double, 4> quat        = {1.0, 0.0, 0.0, 0.0};
  double mass                       = 1.0; // MuJoCo default
  std::array<double, 3> diaginertia = {0.0, 0.0, 0.0};
  std::array<double, 6> fullinertia = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  Inertial();

  std::string element_name() const override { return "inertial"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

} // namespace mjcf