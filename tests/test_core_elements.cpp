#include "doctest.h"
#include "mjcf/core_elements.hpp"

using namespace mjcf;
using namespace mjcf::detail;

TEST_SUITE("core-elements-tests") {
  TEST_CASE("mujoco-element") {
    mjcf::Mujoco mujoco;
    CHECK(mujoco.element_name() == "mujoco");
    mujoco.model    = "dynamic_model";
    std::string xml = mujoco.get_xml_text();
    CHECK(xml.find("<mujoco") != std::string::npos);
    CHECK(xml.find("model=\"dynamic_model\"") != std::string::npos);
  }
  TEST_CASE("compiler-element") {
    Compiler compiler;
    CHECK(compiler.element_name() == "compiler");
    compiler.angle           = mjcf::AngleUnit::Radian;  // Changed to non-default value
    compiler.coordinate      = mjcf::CoordinateType::Global;  // Changed to non-default value
    compiler.inertiafromgeom = true;
    compiler.autolimits      = true;
    std::string xml          = compiler.get_xml_text();
    printf("%s\n", xml.c_str());
    CHECK(xml.find("angle=\"radian\"") != std::string::npos);  // Updated expectation
    CHECK(xml.find("coordinate=\"global\"") != std::string::npos);  // Added check for coordinate
    CHECK(xml.find("inertiafromgeom=\"true\"") != std::string::npos);
    CHECK(xml.find("autolimits=\"true\"") != std::string::npos);
  }
  TEST_CASE("option-element") {
    Option option;
    CHECK(option.element_name() == "option");
    option.integrator = mjcf::IntegratorType::RK4;
    option.timestep   = 0.01;
    option.gravity    = {0.0, 0.0, -9.81};
    option.viscosity  = 0.001;
    std::string xml   = option.get_xml_text();
    CHECK(xml.find("integrator=\"RK4\"") != std::string::npos);
    CHECK(xml.find("timestep=\"0.01\"") != std::string::npos);
    CHECK(xml.find("gravity=\"0 0 -9.81\"") != std::string::npos);
    CHECK(xml.find("viscosity=\"0.001\"") != std::string::npos);
  }
  TEST_CASE("size-element") {
    mjcf::Size size;
    CHECK(size.element_name() == "size");
    size.njmax      = 1000;
    size.nconmax    = 500;
    size.nstack     = 10000;
    size.nuserdata  = 100;
    std::string xml = size.get_xml_text();
    CHECK(xml.find("njmax=\"1000\"") != std::string::npos);
    CHECK(xml.find("nconmax=\"500\"") != std::string::npos);
    CHECK(xml.find("nstack=\"10000\"") != std::string::npos);
    CHECK(xml.find("nuserdata=\"100\"") != std::string::npos);
  }
  TEST_CASE("default-element") {
    mjcf::Default default_elem;
    CHECK(default_elem.element_name() == "default");
    default_elem.class_ = "dynamic_class";
    std::string xml     = default_elem.get_xml_text();
    CHECK(xml.find("class=\"dynamic_class\"") != std::string::npos);
  }
  TEST_CASE("container-elements") {
    mjcf::Visual visual;
    mjcf::Statistic statistic;
    Custom custom;
    Asset asset;
    Worldbody worldbody;
    Actuator actuator;
    Sensor sensor;
    Contact contact;
    Equality equality;
    Tendon tendon;
    CHECK(visual.element_name() == "visual");
    CHECK(statistic.element_name() == "statistic");
    CHECK(custom.element_name() == "custom");
    CHECK(asset.element_name() == "asset");
    CHECK(worldbody.element_name() == "worldbody");
    CHECK(actuator.element_name() == "actuator");
    CHECK(sensor.element_name() == "sensor");
    CHECK(contact.element_name() == "contact");
    CHECK(equality.element_name() == "equality");
    CHECK(tendon.element_name() == "tendon");
  }
}
