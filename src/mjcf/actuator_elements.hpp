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
  bool ctrllimited                  = false;
  bool forcelimited                 = false;
  std::array<double, 2> ctrlrange   = {0.0, 0.0};
  std::array<double, 2> forcerange  = {0.0, 0.0};
  std::array<double, 2> lengthrange = {0.0, 0.0};
  std::array<double, 6> gear        = {};
  double cranklength                = 0.0;
  std::string joint;
  std::string jointinparent;
  std::string tendon;
  std::string cranksite;
  std::string site;
  std::string refsite;
  std::array<double, 3> user = {0.0, 0.0, 0.0};

  BaseActuator(const std::string& element_name);

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;

private:
  std::string element_name_;
};

/**
 * @brief Motor actuator element
 */
class Motor : public BaseActuator {
public:
  Motor();
  std::string element_name() const override { return "motor"; }
  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Position actuator element
 */
class Position : public BaseActuator {
public:
  double kp = 1.0;  // Position feedback gain (1 means not set)
  double kv = 0.01; // Velocity feedback gain (0 means not set)

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
  bool from_xml(const std::string& xml_str) override;

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
  bool from_xml(const std::string& xml_str) override;

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
  bool from_xml(const std::string& xml_str) override;

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
  bool from_xml(const std::string& xml_str) override;

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

  General();

  std::string element_name() const override { return "general"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

} // namespace mjcf
