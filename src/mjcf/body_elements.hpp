#pragma once

#include "element.hpp"
#include "enums.hpp"
#include <array>

namespace mjcf {

// Helper function to sanitize body names
std::string sanitize_body_name(const std::string& name);

class Body : public Element {
public:
  std::string name;
  Arr3 pos                        = {0.0, 0.0, 0.0};
  std::array<double, 4> quat      = {1.0, 0.0, 0.0, 0.0}; // MuJoCo default identity quaternion
  std::array<double, 4> axisangle = {0.0, 0.0, 0.0, 0.0};
  Arr3 euler                      = {0.0, 0.0, 0.0};
  std::array<double, 6> xyaxes    = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Arr3 zaxis                      = {0.0, 0.0, 0.0};
  bool mocap                      = false;
  std::string childclass;

  Body();

  static std::shared_ptr<Body> Create(const std::string& name, Arr3 pos = {0.0, 0.0, 0.0}) {
    auto body  = std::make_shared<Body>();
    body->name = sanitize_body_name(name);
    body->pos  = pos;
    return body;
  }

  std::string element_name() const override { return "body"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

class Geom : public Element {
public:
  std::string name;
  std::string class_;
  GeomType type              = GeomType::Sphere; // MuJoCo default
  Arr3 size                  = {0.0, 0.0, 0.0};
  Arr3 pos                   = {0.0, 0.0, 0.0};
  std::array<double, 4> quat = {1.0, 0.0, 0.0, 0.0};
  std::array<double, 4> rgba = {0.5, 0.5, 0.5, 1.0}; // MuJoCo default gray
  std::string material;
  int contype                     = 1; // MuJoCo default
  int conaffinity                 = 1; // MuJoCo default
  int condim                      = 3; // MuJoCo default
  int group                       = 0;
  int priority                    = 0;
  Arr3 friction                   = {1.0, 0.005, 0.0001};         // MuJoCo default
  double solmix                   = 1.0;                          // MuJoCo default
  std::array<double, 2> solref    = {0.02, 1.0};                  // MuJoCo default
  std::array<double, 5> solimp    = {0.9, 0.95, 0.001, 0.5, 2.0}; // MuJoCo default
  double margin                   = 0.0;
  double gap                      = 0.0;
  std::array<double, 6> fromto    = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 4> axisangle = {0.0, 0.0, 0.0, 0.0};
  Arr3 euler                      = {0.0, 0.0, 0.0};
  std::array<double, 6> xyaxes    = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Arr3 zaxis                      = {0.0, 0.0, 0.0};
  std::string hfield;
  std::string mesh;
  double fitscale = 1.0; // MuJoCo default

  Geom();

  std::string element_name() const override { return "geom"; }

  static std::shared_ptr<Geom> Box(const std::string& name, const Arr3& size, const Arr3& pos = {0.0, 0.0, 0.0}, const std::array<double, 4>& rgba = {0.5, 0.5, 0.5, 1.0}) {
    auto geom  = std::make_shared<Geom>();
    geom->name = name;
    geom->type = GeomType::Box;
    geom->size = size;
    geom->pos  = pos;
    geom->rgba = rgba;
    return geom;
  }

  static std::shared_ptr<Geom> Sphere(const std::string& name, double radius, const Arr3& pos = {0.0, 0.0, 0.0}, const std::array<double, 4>& rgba = {0.5, 0.5, 0.5, 1.0}) {
    auto geom  = std::make_shared<Geom>();
    geom->name = name;
    geom->type = GeomType::Sphere;
    geom->size = {radius, 0.0, 0.0}; // Explicitly initialize all three elements
    geom->pos  = pos;
    geom->rgba = rgba;
    return geom;
  }

  static std::shared_ptr<Geom> Capsule(const std::string& name, const Arr3& size, const Arr3& pos = {0.0, 0.0, 0.0}, const std::array<double, 4>& rgba = {0.5, 0.5, 0.5, 1.0}) {
    auto geom  = std::make_shared<Geom>();
    geom->name = name;
    geom->type = GeomType::Capsule;
    geom->size = size;
    geom->pos  = pos;
    geom->rgba = rgba;
    return geom;
  }

  static std::shared_ptr<Geom> Cylinder(const std::string& name, const Arr3& size, const Arr3& pos = {0.0, 0.0, 0.0}, const std::array<double, 4>& rgba = {0.5, 0.5, 0.5, 1.0}) {
    auto geom  = std::make_shared<Geom>();
    geom->name = name;
    geom->type = GeomType::Cylinder;
    geom->size = size;
    geom->pos  = pos;
    geom->rgba = rgba;
    return geom;
  }

  static std::shared_ptr<Geom> Plane(const std::string& name, const Arr3& size, const Arr3& pos = {0.0, 0.0, 0.0}, const std::array<double, 4>& rgba = {0.5, 0.5, 0.5, 1.0}) {
    auto geom  = std::make_shared<Geom>();
    geom->name = name;
    geom->type = GeomType::Plane;
    geom->size = size;
    geom->pos  = pos;
    geom->rgba = rgba;
    return geom;
  }

  static std::shared_ptr<Geom> MeshGeom(const std::string& name, const std::string& mesh, const Arr3& pos = {0.0, 0.0, 0.0}, const std::array<double, 4>& rgba = {0.5, 0.5, 0.5, 1.0}) {
    auto geom  = std::make_shared<Geom>();
    geom->name = name;
    geom->type = GeomType::Mesh;
    geom->mesh = mesh;
    geom->pos  = pos;
    geom->rgba = rgba;
    return geom;
  }

  static std::shared_ptr<Geom> Ellipsoid(const std::string& name, const Arr3& size, const Arr3& pos = {0.0, 0.0, 0.0}, const std::array<double, 4>& rgba = {0.5, 0.5, 0.5, 1.0}) {
    auto geom  = std::make_shared<Geom>();
    geom->name = name;
    geom->type = GeomType::Ellipsoid;
    geom->size = size;
    geom->pos  = pos;
    geom->rgba = rgba;
    return geom;
  }

  static std::shared_ptr<Geom> Create(const std::string& name, const Arr3& pos = {0.0, 0.0, 0.0}, const std::array<double, 4>& quat = {1.0, 0.0, 0.0, 0.0}) {
    auto geom  = std::make_shared<Geom>();
    geom->name = name;
    geom->pos  = pos;
    geom->quat = quat;
    return geom;
  }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

class Light : public Element {
public:
  std::string name;
  std::string class_;
  bool directional = false;
  bool castshadow  = true;
  bool active      = true;
  Arr3 pos         = {0.0, 0.0, 0.0};
  Arr3 dir         = {0.0, 0.0, -1.0}; // MuJoCo default pointing down
  Arr3 attenuation = {1.0, 0.0, 0.0};  // MuJoCo default
  double cutoff    = 45.0;             // MuJoCo default
  double exponent  = 10.0;             // MuJoCo default
  Arr3 ambient     = {0.0, 0.0, 0.0};
  Arr3 diffuse     = {0.7, 0.7, 0.7};  // MuJoCo default
  Arr3 specular    = {0.3, 0.3, 0.3};  // MuJoCo default
  LightMode mode   = LightMode::Fixed; // MuJoCo default

  Light() = default;

  // Convenience constructor for directional lights
  Light(bool directional, const Arr3& pos, const Arr3& dir, const Arr3& diffuse = {0.7, 0.7, 0.7});

  static std::shared_ptr<Light> Create(bool directional, const Arr3& pos, const Arr3& dir, const Arr3& diffuse = {0.7, 0.7, 0.7}) { //
    return std::make_shared<Light>(directional, pos, dir, diffuse);
  }

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
  std::string name                = "";
  std::string class_              = "";
  CameraMode mode                 = CameraMode::Fixed; // MuJoCo default
  std::string target              = "";
  Arr3 pos                        = {0.0, 0.0, 0.0};
  std::array<double, 4> quat      = {1.0, 0.0, 0.0, 0.0};
  std::array<double, 4> axisangle = {0.0, 0.0, 0.0, 0.0};
  Arr3 euler                      = {0.0, 0.0, 0.0};
  std::array<double, 6> xyaxes    = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Arr3 zaxis                      = {0.0, 0.0, 0.0};
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
  std::string name                = "";
  std::string class_              = "";
  SiteType type                   = SiteType::Sphere;      // MuJoCo default
  Arr3 size                       = {0.005, 0.005, 0.005}; // MuJoCo default
  Arr3 pos                        = {0.0, 0.0, 0.0};
  std::array<double, 4> quat      = {1.0, 0.0, 0.0, 0.0};
  std::array<double, 4> axisangle = {0.0, 0.0, 0.0, 0.0};
  Arr3 euler                      = {0.0, 0.0, 0.0};
  std::array<double, 6> xyaxes    = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Arr3 zaxis                      = {0.0, 0.0, 0.0};
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

class Joint : public Element {
public:
  std::string name;
  std::string class_;
  JointType type                         = JointType::Hinge; // MuJoCo default
  int group                              = 0;
  Arr3 pos                               = {0.0, 0.0, 0.0};
  Arr3 axis                              = {0.0, 0.0, 1.0}; // MuJoCo default
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
  Arr3 user                              = {0.0, 0.0, 0.0};

  Joint() = default;

  static std::shared_ptr<Joint> Free(const std::string& name) {
    auto joint  = std::make_shared<Joint>();
    joint->name = name;
    joint->type = JointType::Free;
    return joint;
  }

  static std::shared_ptr<Joint> Hinge(const std::string& name, const Arr3& axis = {0.0, 0.0, 1.0}) {
    auto joint  = std::make_shared<Joint>();
    joint->name = name;
    joint->type = JointType::Hinge;
    joint->axis = axis;
    return joint;
  }

  static std::shared_ptr<Joint> Slide(const std::string& name, const Arr3& axis = {0.0, 0.0, 1.0}) {
    auto joint  = std::make_shared<Joint>();
    joint->name = name;
    joint->type = JointType::Slide;
    joint->axis = axis;
    return joint;
  }

  static std::shared_ptr<Joint> Ball(const std::string& name) {
    auto joint  = std::make_shared<Joint>();
    joint->name = name;
    joint->type = JointType::Ball;
    return joint;
  }

  std::string element_name() const override { return "joint"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

class Inertial : public Element {
public:
  Arr3 pos                          = {0.0, 0.0, 0.0};
  std::array<double, 4> quat        = {1.0, 0.0, 0.0, 0.0};
  double mass                       = 1.0; // MuJoCo default
  Arr3 diaginertia                  = {0.01, 0.01, 0.01};
  std::array<double, 6> fullinertia = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  Inertial() = default;

  std::string element_name() const override { return "inertial"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

} // namespace mjcf
