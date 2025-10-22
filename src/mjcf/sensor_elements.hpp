#pragma once

#include "element.hpp"
#include "enums.hpp"
#include <array>

namespace mjcf {

class BaseSensor : public Element {
public:
  std::string name;
  double noise               = 0.0;
  double cutoff              = 0.0;
  std::array<double, 3> user = {0.0, 0.0, 0.0};

  BaseSensor(const std::string& element_name);

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;

private:
  std::string element_name_;
};

/**
 * @brief Joint position sensor
 */
class JointPos : public BaseSensor {
public:
  std::string joint;

  JointPos();
  std::string element_name() const override { return "jointpos"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Joint velocity sensor
 */
class JointVel : public BaseSensor {
public:
  std::string joint;

  JointVel();

  std::string element_name() const override { return "jointvel"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Tendon position sensor
 */
class TendonPos : public BaseSensor {
public:
  std::string tendon = "";

  TendonPos();

  std::string element_name() const override { return "tendonpos"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Tendon velocity sensor
 */
class TendonVel : public BaseSensor {
public:
  std::string tendon = "";

  TendonVel();

  std::string element_name() const override { return "tendonvel"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Actuator position sensor
 */
class ActuatorPos : public BaseSensor {
public:
  std::string actuator = "";

  ActuatorPos();

  std::string element_name() const override { return "actuatorpos"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Actuator velocity sensor
 */
class ActuatorVel : public BaseSensor {
public:
  std::string actuator = "";

  ActuatorVel();

  std::string element_name() const override { return "actuatorvel"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Actuator force sensor
 */
class ActuatorFrc : public BaseSensor {
public:
  std::string actuator = "";

  ActuatorFrc();

  std::string element_name() const override { return "actuatorfrc"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Ball joint position sensor
 */
class BallQuat : public BaseSensor {
public:
  std::string joint = "";

  BallQuat();

  std::string element_name() const override { return "ballquat"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Ball joint angular velocity sensor
 */
class BallAngVel : public BaseSensor {
public:
  std::string joint = "";

  BallAngVel();

  std::string element_name() const override { return "ballangvel"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Site position sensor
 */
class SitePos : public BaseSensor {
public:
  std::string reftype = "body"; // MuJoCo default
  std::string objtype = "body"; // MuJoCo default
  std::string objname = "";

  SitePos();

  std::string element_name() const override { return "framepos"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Site orientation sensor
 */
class SiteQuat : public BaseSensor {
public:
  std::string reftype = "body"; // MuJoCo default
  std::string objtype = "body"; // MuJoCo default
  std::string objname = "";

  SiteQuat();

  std::string element_name() const override { return "framequat"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Linear velocity sensor
 */
class SiteLinVel : public BaseSensor {
public:
  std::string reftype = "body"; // MuJoCo default
  std::string objtype = "body"; // MuJoCo default
  std::string objname = "";

  SiteLinVel();

  std::string element_name() const override { return "framelinvel"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Angular velocity sensor
 */
class SiteAngVel : public BaseSensor {
public:
  std::string reftype = "body"; // MuJoCo default
  std::string objtype = "body"; // MuJoCo default
  std::string objname = "";

  SiteAngVel();

  std::string element_name() const override { return "frameangvel"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Subtree center of mass position sensor
 */
class SubtreeCom : public BaseSensor {
public:
  std::string body = "";

  SubtreeCom();

  std::string element_name() const override { return "subtreecom"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Subtree linear momentum sensor
 */
class SubtreeLinVel : public BaseSensor {
public:
  std::string body = "";

  SubtreeLinVel();

  std::string element_name() const override { return "subtreelinvel"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Subtree angular momentum sensor
 */
class SubtreeAngMom : public BaseSensor {
public:
  std::string body = "";

  SubtreeAngMom();

  std::string element_name() const override { return "subtreeangmom"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Gyroscope sensor
 */
class Gyro : public BaseSensor {
public:
  std::string site = "";

  Gyro();

  std::string element_name() const override { return "gyro"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Accelerometer sensor
 */
class Accelerometer : public BaseSensor {
public:
  std::string site = "";

  Accelerometer();

  std::string element_name() const override { return "accelerometer"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Magnetometer sensor
 */
class Magnetometer : public BaseSensor {
public:
  std::string site = "";

  Magnetometer();

  std::string element_name() const override { return "magnetometer"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Range finder sensor
 */
class Rangefinder : public BaseSensor {
public:
  std::string site = "";

  Rangefinder();

  std::string element_name() const override { return "rangefinder"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Force sensor
 */
class Force : public BaseSensor {
public:
  std::string site = "";

  Force();

  std::string element_name() const override { return "force"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Torque sensor
 */
class Torque : public BaseSensor {
public:
  std::string site = "";

  Torque();

  std::string element_name() const override { return "torque"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Touch sensor
 */
class Touch : public BaseSensor {
public:
  std::string site = "";

  Touch();

  std::string element_name() const override { return "touch"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

} // namespace mjcf
