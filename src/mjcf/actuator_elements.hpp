#pragma once

#include "element.hpp"
#include "enums.hpp"
#include <array>

namespace mjcf {

/**
 * @brief Base class for all actuator elements
 */
class BaseActuator : public Element {
public:
  std::string name;
  std::string class_;
  int group                         = 0;
  TriState ctrllimited               = TriState::Auto; // MuJoCo default
  TriState forcelimited              = TriState::Auto; // MuJoCo default
  std::array<double, 2> ctrlrange   = {0.0, 0.0};
  std::array<double, 2> forcerange  = {0.0, 0.0};
  std::array<double, 2> lengthrange = {0.0, 0.0};
  std::array<double, 6> gear        = {1, 0, 0, 0, 0, 0};
  double cranklength                = 0.0;
  std::string joint;
  std::string jointinparent;
  std::string tendon;
  std::string cranksite;
  std::string slidersite;
  std::string site;
  std::string refsite;
  std::string body;
  std::array<double, 3> user = {0.0, 0.0, 0.0};

  BaseActuator(const std::string& element_name);

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;

private:
  std::string element_name_;
};


class Motor : public BaseActuator {
public:
  Motor();
  std::string element_name() const override { return "motor"; }
  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};


class Position : public BaseActuator {
public:
  double kp          = 1.0; // MuJoCo default
  double kv          = 0.0; // MuJoCo default
  double dampratio   = 0.0; // MuJoCo default。kvと排他
  double timeconst    = 0.0; // MuJoCo default。0より大きいとfilterexact dyntypeになる
  double inheritrange = 0.0; // MuJoCo default (0=無効)

  Position() : BaseActuator("position") {}

  static std::shared_ptr<Position> Create(const std::string& joint, const std::string& name = "", float kv = 100) {
    auto p   = std::make_shared<Position>();
    p->joint = joint;
    p->name  = name;
    p->kv    = kv;
    return p;
  }

  std::string element_name() const override { return "position"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Velocity actuator element
 */
class Velocity : public BaseActuator {
public:
  double kv = 0.0; // Velocity feedback gain (0 means not set)

  Velocity();

  std::string element_name() const override { return "velocity"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Cylinder actuator element
 */
class Cylinder : public BaseActuator {
public:
  double timeconst           = 0.0;
  double area                = 0.0;
  double diameter            = 0.0;
  std::array<double, 3> bias = {0.0, 0.0, 0.0};

  Cylinder();

  std::string element_name() const override { return "cylinder"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Muscle actuator element
 */
class Muscle : public BaseActuator {
public:
  std::array<double, 2> timeconst = {0.0, 0.0};
  std::array<double, 2> range     = {0.0, 0.0};
  double force                    = 0.0;
  double scale                    = 0.0;
  double lmin                     = 0.0;
  double lmax                     = 0.0;
  double vmax                     = 0.0;
  double fpmax                    = 0.0;
  double fvmax                    = 0.0;

  Muscle();

  std::string element_name() const override { return "muscle"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Tendon actuator element
 */
class TendonActuator : public BaseActuator {
public:
  // No additional public members - inherits from BaseActuator

  TendonActuator();

  std::string element_name() const override { return "tendon"; }

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief General actuator element
 */
class General : public BaseActuator {
public:
  std::string dyntype            = "none";                                             // Activation dynamics type
  std::string gaintype           = "fixed";                                            // Gain type
  std::string biastype           = "none";                                             // Bias type
  std::array<double, 10> dynprm  = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Dynamics parameters
  std::array<double, 10> gainprm = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Gain parameters
  std::array<double, 10> biasprm = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Bias parameters
  TriState actlimited              = TriState::Auto;    // MuJoCo default
  std::array<double, 2> actrange  = {0.0, 0.0};
  int actdim                      = -1;    // MuJoCo default
  bool actearly                   = false; // MuJoCo default

  General();

  std::string element_name() const override { return "general"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Damper actuator element: F = -kv * velocity * control (kv, ctrlrangeは非負が必須)
 */
class Damper : public BaseActuator {
public:
  double kv = 1.0; // MuJoCo default

  Damper();

  std::string element_name() const override { return "damper"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Integrated-velocity actuator element(activation stateが位置、actrangeでクランプ可能)
 */
class IntVelocity : public BaseActuator {
public:
  double kp                  = 1.0; // MuJoCo default
  double kv                  = 0.0; // MuJoCo default
  double dampratio           = 0.0; // MuJoCo default。kvと排他
  double inheritrange        = 0.0; // MuJoCo default (0=無効)
  std::array<double, 2> actrange = {0.0, 0.0};

  IntVelocity();

  std::string element_name() const override { return "intvelocity"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Adhesion(吸着)アクチュエータ。joint/tendon/site等ではなくbodyに属する全接触へ法線方向の力を加える。
 * 共通のtransmission系属性(joint/tendon/site/...)は持たないためBaseActuatorを継承しない。
 */
class Adhesion : public Element {
public:
  std::string name;
  std::string class_;
  int group                        = 0;
  TriState forcelimited             = TriState::Auto; // MuJoCo default
  std::array<double, 2> ctrlrange  = {0.0, 0.0};
  std::array<double, 2> forcerange = {0.0, 0.0};
  std::array<double, 3> user       = {0.0, 0.0, 0.0};
  std::string body; // required
  double gain                      = 1.0; // MuJoCo default

  Adhesion() = default;

  std::string element_name() const override { return "adhesion"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

} // namespace mjcf
