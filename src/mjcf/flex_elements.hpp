#pragma once

#include "element.hpp"
#include "enums.hpp"
#include <array>
#include <memory>
#include <string>
#include <vector>

namespace mjcf {

// Forward declarations
class FlexComp;

/**
 * @brief Flex contact element (child of flexcomp)
 */
class FlexContact : public Element {
public:
  int contype                  = 1;
  int conaffinity              = 1;
  int condim                   = 3;
  int priority                 = 0;
  Arr3 friction                = {1.0, 0.005, 0.0001};
  double solmix                = 1.0;
  std::array<double, 2> solref = {0.02, 1.0};
  std::array<double, 5> solimp = {0.9, 0.95, 0.001, 0.5, 2.0};
  double margin                = 0.0;
  double gap                   = 0.0;
  bool internal                = false;
  FlexSelfCollide selfcollide  = FlexSelfCollide::Auto;
  int activelayers             = 1;
  bool vertcollide             = false;
  bool passive                 = false;

  FlexContact();

  std::string element_name() const override { return "contact"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Flex edge element (child of flexcomp)
 */
class FlexEdge : public Element {
public:
  bool equality                = false;
  std::array<double, 2> solref = {0.02, 1.0};
  std::array<double, 5> solimp = {0.9, 0.95, 0.001, 0.5, 2.0};
  double stiffness             = 0.0;
  double damping               = 0.0;

  FlexEdge();

  std::string element_name() const override { return "edge"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Flex pin element (child of flexcomp)
 */
class FlexPin : public Element {
public:
  std::vector<int> id;
  std::array<double, 2> range     = {0.0, 0.0};
  std::array<int, 2> grid         = {0, 0};
  std::array<double, 2> gridrange = {0.0, 0.0};

  FlexPin();

  std::string element_name() const override { return "pin"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Flex component element (cloth simulation)
 */
class FlexComp : public Element {
public:
  // Required
  std::string name;
  FlexCompType type = FlexCompType::Grid;

  // Shape (meaning varies by type)
  std::vector<int> count;
  std::vector<double> spacing;
  Arr3 scale    = {1.0, 1.0, 1.0};
  double radius = 0.005;
  int dim       = 2;
  FlexDof dof   = FlexDof::Full;
  bool rigid    = false;

  // Mass & inertia
  double mass       = 0.0;
  double inertiabox = 0.0;

  // Position & orientation
  Arr3 pos                        = {0.0, 0.0, 0.0};
  std::array<double, 4> quat      = {1.0, 0.0, 0.0, 0.0};
  Arr3 euler                      = {0.0, 0.0, 0.0};
  std::array<double, 4> axisangle = {0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> xyaxes    = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Arr3 zaxis                      = {0.0, 0.0, 0.0};
  Arr3 origin                     = {0.0, 0.0, 0.0};

  // Appearance & files
  std::string file;
  std::string material;
  std::array<double, 4> rgba = {0.5, 0.5, 0.5, 1.0};
  bool flatskin              = false;
  int group                  = 0;

  FlexComp();

  std::string element_name() const override { return "flexcomp"; }

  void set_xml_attrib() const override;

  // Static factories
  static std::shared_ptr<FlexComp> Grid(const std::string& name, std::vector<int> count, std::vector<double> spacing, Arr3 pos = {0.0, 0.0, 0.0});

  static std::shared_ptr<FlexComp> Mesh(const std::string& name, const std::string& file, Arr3 pos = {0.0, 0.0, 0.0});

  // Child element helpers
  std::shared_ptr<FlexContact> add_contact(std::shared_ptr<FlexContact> contact = nullptr);
  std::shared_ptr<FlexEdge> add_edge(std::shared_ptr<FlexEdge> edge = nullptr);
  std::shared_ptr<FlexPin> add_pin(std::shared_ptr<FlexPin> pin = nullptr);

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

} // namespace mjcf
