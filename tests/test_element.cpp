#include "doctest.h"
#include "mjcf/element.hpp"
#include <charconv>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>

// Test helper class that exposes protected methods for testing
class TestElement : public mjcf::Element {
public:
  TestElement(const std::string& name) : name_(name) {}
  std::string element_name() const override { return name_; }

  // Expose protected methods for testing
  std::string test_stringify_value(const mjcf::AttributeValue& value) const { return stringify_value(value); }

  void test_set_attribute(const std::string& name, const mjcf::AttributeValue& value) { set_attribute(name, value); }

private:
  std::string name_;
};

TEST_SUITE("element-tests") {
  TEST_CASE("element-basic-functionality") {
    auto element = std::make_shared<TestElement>("test");
    CHECK(element->element_name() == "test");
    auto child = std::make_shared<TestElement>("child");
    element->add_child(child);
    std::string xml = element->get_xml_text();
    CHECK(xml.find("<test") != std::string::npos);
    CHECK(xml.find("<child") != std::string::npos);
  }

  TEST_CASE("attribute-value-stringification") {
    auto element                      = std::make_shared<TestElement>("test");
    mjcf::AttributeValue value_str    = std::string("hello");
    mjcf::AttributeValue value_true   = true;
    mjcf::AttributeValue value_false  = false;
    mjcf::AttributeValue value_int    = 42;
    mjcf::AttributeValue value_double = 3.14159;
    mjcf::AttributeValue value_vecd   = std::vector<double>{1.0, 2.0, 3.0};
    mjcf::AttributeValue value_veci   = std::vector<int>{1, 2, 3};
    CHECK(element->test_stringify_value(value_str) == "hello");
    CHECK(element->test_stringify_value(value_true) == "true");
    CHECK(element->test_stringify_value(value_false) == "false");
    CHECK(element->test_stringify_value(value_int) == "42");
    CHECK(element->test_stringify_value(value_double).find("3.14159") != std::string::npos);
    CHECK(element->test_stringify_value(value_vecd) == "1 2 3");
    CHECK(element->test_stringify_value(value_veci) == "1 2 3");
  }

  TEST_CASE("attribute-setting-and-xml-generation") {
    auto element = std::make_shared<TestElement>("test");
    element->test_set_attribute("name", std::string("test_name"));
    element->test_set_attribute("value", 42);
    element->test_set_attribute("enabled", true);
    std::string xml = element->get_xml_text();
    CHECK(xml.find("name=\"test_name\"") != std::string::npos);
    CHECK(xml.find("value=\"42\"") != std::string::npos);
    CHECK(xml.find("enabled=\"true\"") != std::string::npos);
  }

  TEST_CASE("xml-generation") {
    auto element    = std::make_shared<TestElement>("mujoco");
    std::string xml = element->get_xml_text();
    CHECK(xml.find("<mujoco") != std::string::npos);
    CHECK((xml.find("</mujoco>") != std::string::npos || xml.find("/>") != std::string::npos));
    auto child1 = std::make_shared<TestElement>("option");
    auto child2 = std::make_shared<TestElement>("asset");
    element->add_children({child1, child2});
    std::string xml2 = element->get_xml_text();
    CHECK(xml2.find("<option") != std::string::npos);
    CHECK(xml2.find("<asset") != std::string::npos);
    CHECK(xml2.find("</mujoco>") != std::string::npos);
  }

  TEST_CASE("null-child-handling") {
    auto element = std::make_shared<TestElement>("test");
    element->add_child(nullptr);
    std::string xml = element->get_xml_text();
    CHECK(xml.find("<test") != std::string::npos);
  }
}

TEST_CASE("double-attributes-preserve-small-inertias-and-full-range") {
  TestElement element("inertial");
  const std::vector<double> values{1e-10, 1e-8, -1e-10, 0.0, -0.0, std::nextafter(1.0, 2.0), 1e20, std::numeric_limits<double>::min(), std::numeric_limits<double>::denorm_min(), std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest()};
  for(const double value : values) {
    const auto text   = element.test_stringify_value(value);
    double parsed     = 0.0;
    const auto result = std::from_chars(text.data(), text.data() + text.size(), parsed);
    CHECK(result.ec == std::errc{});
    CHECK(result.ptr == text.data() + text.size());
    CHECK(parsed == value);
    CHECK(std::signbit(parsed) == std::signbit(value));
  }
}

TEST_CASE("double-vector-preserves-inertia-tensor-and-scientific-exponents") {
  TestElement element("inertial");
  const std::vector<double> values{1e-10, 2e-10, 3e-10, -4e-12, 0.0, 6e-12, 1e20, 10.0};
  std::istringstream tokens(element.test_stringify_value(values));
  for(const double value : values) {
    std::string token;
    REQUIRE(static_cast<bool>(tokens >> token));
    CHECK(std::stod(token) == value);
  }
  std::string extra;
  CHECK_FALSE(static_cast<bool>(tokens >> extra));
}
