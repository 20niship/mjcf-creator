#pragma once

#include "element.hpp"
#include "enums.hpp"
#include <array>
#include <cassert>
#include <map>
#include <string>

namespace mjcf {

// Forward declarations for URDF converter metadata
struct JointMetadata;
struct ActuatorMetadata;

namespace detail {

class Custom : public Element {
public:
  Custom() = default;

  [[nodiscard]] std::string element_name() const override { return "custom"; }
};

class Asset : public Element {
public:
  Asset() = default;
  [[nodiscard]] std::string element_name() const override { return "asset"; }
};

class Worldbody : public Element {
public:
  Worldbody() = default;
  [[nodiscard]] std::string element_name() const override { return "worldbody"; }
};

class Actuator : public Element {
public:
  Actuator() = default;
  [[nodiscard]] std::string element_name() const override { return "actuator"; }
};

class Sensor : public Element {
public:
  Sensor() = default;
  [[nodiscard]] std::string element_name() const override { return "sensor"; }
};

class Contact : public Element {
public:
  Contact() = default;
  [[nodiscard]] std::string element_name() const override { return "contact"; }
};

class Equality : public Element {
public:
  Equality() = default;
  [[nodiscard]] std::string element_name() const override { return "equality"; }
};

class Tendon : public Element {
public:
  Tendon() = default;
  [[nodiscard]] std::string element_name() const override { return "tendon"; }
};


class Compiler : public Element {
public:
  AngleUnit angle           = AngleUnit::Degree;     // MuJoCo default
  CoordinateType coordinate = CoordinateType::Local; // MuJoCo default (note: default value may vary based on usage)
  bool inertiafromgeom      = false;                 // MuJoCo default
  bool autolimits           = false;                 // MuJoCo default

  Compiler() = default;

  [[nodiscard]] std::string element_name() const override { return "compiler"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  [[nodiscard]] bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

class Option : public Element {
public:
  IntegratorType integrator     = IntegratorType::Euler; // MuJoCo default
  double timestep               = 0.002;                 // MuJoCo default
  std::array<double, 3> gravity = {0.0, 0.0, -9.81};     // MuJoCo default
  double viscosity              = 0.0;                   // MuJoCo default

  Option() = default;

  [[nodiscard]] std::string element_name() const override { return "option"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  [[nodiscard]] bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};
} // namespace detail

class Mujoco : public Element {
public:
  // Public member variables
  std::string model = "";

  Mujoco(const std::string& model = "") {
    this->model      = model;
    this->compiler_  = std::make_shared<detail::Compiler>();
    this->option_    = std::make_shared<detail::Option>();
    this->asset_     = std::make_shared<detail::Asset>();
    this->worldbody_ = std::make_shared<detail::Worldbody>();
    this->actuator_  = std::make_shared<detail::Actuator>();
    this->sensor_    = std::make_shared<detail::Sensor>();
    this->contact_   = std::make_shared<detail::Contact>();
    this->equality_  = std::make_shared<detail::Equality>();
    this->tendon_    = std::make_shared<detail::Tendon>();
  }

  /**
   * @brief Add URDF content to this MJCF model
   *
   * @param urdf_path Path to input URDF file
   * @param name_prefix Prefix to avoid naming conflicts (optional)
   * @param copy_meshes Whether to copy mesh files
   * @param joint_metadata Map of joint name to joint metadata
   * @param actuator_metadata Map of actuator type to actuator metadata
   * @return true if addition was successful, false otherwise
   */
  bool add_urdf(const std::string& urdf_path, const std::string& name_prefix = "", bool copy_meshes = false, const std::map<std::string, JointMetadata>& joint_metadata = {}, const std::map<std::string, ActuatorMetadata>& actuator_metadata = {});

  [[nodiscard]] std::string element_name() const override { return "mujoco"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

  void add_child([[maybe_unused]] std::shared_ptr<Element> child) override {
    assert(false && "Unsupported child element type for Mujoco");
  }

  std::shared_ptr<detail::Compiler> compiler_   = nullptr;
  std::shared_ptr<detail::Option> option_       = nullptr;
  std::shared_ptr<detail::Worldbody> worldbody_ = nullptr;
  std::shared_ptr<detail::Asset> asset_         = nullptr;
  std::shared_ptr<detail::Actuator> actuator_   = nullptr;
  std::shared_ptr<detail::Sensor> sensor_       = nullptr;
  std::shared_ptr<detail::Contact> contact_     = nullptr;
  std::shared_ptr<detail::Equality> equality_   = nullptr;
  std::shared_ptr<detail::Tendon> tendon_       = nullptr;

  [[nodiscard]] bool has_asset(const std::string& asset_name) const {
    if(!asset_) return false;
    for(const auto& child : asset_->get_children()) {
      auto name_ = child->get_attribute_public("name");
      if(name_.has_value() && std::holds_alternative<std::string>(*name_) && std::get<std::string>(*name_) == asset_name) return true;
    }
    return false;
  }

  void add_asset(std::shared_ptr<Element> asset_element) {
    assert(asset_element && "Asset element is null");
    assert(this->asset_ && "Asset container not initialized");
    asset_->add_child(asset_element);
  }

  void add_body(std::shared_ptr<Element> body_element) {
    assert(body_element && "Body element is null");
    assert(this->worldbody_ && "Worldbody container not initialized");
    worldbody_->add_child(body_element);
  }

protected:
  [[nodiscard]] bool is_default_value([[maybe_unused]] const std::string& name, [[maybe_unused]] const AttributeValue& value) const override { return false; }

  void* write_xml_element(void* doc_ptr, void* parent_ptr) const override;
};

/**
 * @brief サイズパラメータ要素
 */
class Size : public Element {
public:
  // Public member variables (defaults are automatic in MuJoCo, use 0 as sentinel)
  int njmax     = 0;
  int nconmax   = 0;
  int nstack    = 0;
  int nuserdata = 0;

  Size();

  [[nodiscard]] std::string element_name() const override { return "size"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  [[nodiscard]] bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief 視覚パラメータ要素
 */
class Visual : public Element {
public:
  Visual();

  [[nodiscard]] std::string element_name() const override { return "visual"; }
};

/**
 * @brief 統計パラメータ要素
 */
class Statistic : public Element {
public:
  Statistic();

  [[nodiscard]] std::string element_name() const override { return "statistic"; }
};

/**
 * @brief デフォルト設定要素
 */
class Default : public Element {
public:
  // Public member variables
  std::string class_ = "";

  Default(const std::string& class_name = "");

  [[nodiscard]] std::string element_name() const override { return "default"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  [[nodiscard]] bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};
} // namespace mjcf
