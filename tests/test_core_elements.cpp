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
    std::string xml          = compiler.get_xml_text();
    printf("%s\n", xml.c_str());
    CHECK(xml.find("angle=\"radian\"") != std::string::npos);  // Updated expectation
    CHECK(xml.find("coordinate=\"global\"") != std::string::npos);  // Added check for coordinate
    CHECK(xml.find("inertiafromgeom=\"true\"") != std::string::npos);
    // autolimitsの実際のMuJoCo既定値はtrueなので、既定値のままならXMLに出力されない
    CHECK(xml.find("autolimits") == std::string::npos);
  }
  TEST_CASE("compiler-element-autolimits-false") {
    Compiler compiler;
    compiler.autolimits = false;
    std::string xml     = compiler.get_xml_text();
    CHECK(xml.find("autolimits=\"false\"") != std::string::npos);
  }
  TEST_CASE("compiler-element-new-attributes") {
    Compiler compiler;
    compiler.meshdir           = "assets/meshes";
    compiler.texturedir        = "assets/textures";
    compiler.boundmass         = 0.001;
    compiler.boundinertia      = 0.001;
    compiler.settotalmass      = 10.0;
    compiler.balanceinertia    = true;
    compiler.fitaabb           = true;
    compiler.eulerseq          = "XYZ";
    compiler.discardvisual     = true;
    compiler.fusestatic        = true;
    compiler.alignfree         = true;
    compiler.inertiagrouprange = {1, 3};
    compiler.saveinertial      = true;
    std::string xml            = compiler.get_xml_text();
    CHECK(xml.find("meshdir=\"assets/meshes\"") != std::string::npos);
    CHECK(xml.find("texturedir=\"assets/textures\"") != std::string::npos);
    CHECK(xml.find("boundmass=\"0.001\"") != std::string::npos);
    CHECK(xml.find("boundinertia=\"0.001\"") != std::string::npos);
    CHECK(xml.find("settotalmass=\"10\"") != std::string::npos);
    CHECK(xml.find("balanceinertia=\"true\"") != std::string::npos);
    CHECK(xml.find("fitaabb=\"true\"") != std::string::npos);
    CHECK(xml.find("eulerseq=\"XYZ\"") != std::string::npos);
    CHECK(xml.find("discardvisual=\"true\"") != std::string::npos);
    CHECK(xml.find("fusestatic=\"true\"") != std::string::npos);
    CHECK(xml.find("alignfree=\"true\"") != std::string::npos);
    CHECK(xml.find("inertiagrouprange=\"1 3\"") != std::string::npos);
    CHECK(xml.find("saveinertial=\"true\"") != std::string::npos);
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
  TEST_CASE("option-element-new-attributes") {
    Option option;
    option.wind      = {1.0, 0.0, 0.0};
    option.magnetic  = {0.0, 0.0, 1.0};
    option.density   = 1.2;
    option.jacobian  = "sparse";
    option.ls_iterations = 20;
    option.ls_tolerance  = 0.05;
    option.ccd_iterations = 10;
    option.ccd_tolerance  = 1e-5;
    option.sleep_tolerance = 1e-3;
    option.sdf_iterations  = 5;
    option.sdf_initpoints  = 20;
    option.o_margin  = 0.01;
    std::string xml  = option.get_xml_text();
    CHECK(xml.find("wind=\"1 0 0\"") != std::string::npos);
    CHECK(xml.find("magnetic=\"0 0 1\"") != std::string::npos);
    CHECK(xml.find("density=\"1.2\"") != std::string::npos);
    CHECK(xml.find("jacobian=\"sparse\"") != std::string::npos);
    CHECK(xml.find("ls_iterations=\"20\"") != std::string::npos);
    CHECK(xml.find("ls_tolerance=\"0.05\"") != std::string::npos);
    CHECK(xml.find("ccd_iterations=\"10\"") != std::string::npos);
    CHECK(xml.find("ccd_tolerance=\"0.00001\"") != std::string::npos);
    CHECK(xml.find("sleep_tolerance=\"0.001\"") != std::string::npos);
    CHECK(xml.find("sdf_iterations=\"5\"") != std::string::npos);
    CHECK(xml.find("sdf_initpoints=\"20\"") != std::string::npos);
    CHECK(xml.find("o_margin=\"0.01\"") != std::string::npos);
    CHECK(xml.find("o_solref=\"0.02 1\"") != std::string::npos);
    CHECK(xml.find("o_solimp=\"0.9 0.95 0.001 0.5 2\"") != std::string::npos);
    CHECK(xml.find("o_friction=\"1 1 0.005 0.0001 0.0001\"") != std::string::npos);
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
    Default default_elem;
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
