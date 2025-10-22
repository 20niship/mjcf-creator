#include "doctest.h"
#include "mjcf/body_elements.hpp"

TEST_SUITE("body-elements-tests") {
  TEST_CASE("body-element") {
    mjcf::Body body;
    CHECK(body.element_name() == "body");
    body.name       = "test_body";
    body.pos        = {1.0, 2.0, 3.0};
    body.quat       = {1.0, 1.0, 0.0, 0.0};
    body.euler      = {0.0, 0.0, 1.57};
    body.mocap      = true;
    body.childclass = "ant";
    std::string xml = body.get_xml_text();
    CHECK(xml.find("name=\"test_body\"") != std::string::npos);
    CHECK(xml.find("pos=\"1 2 3\"") != std::string::npos);
    CHECK(xml.find("quat=\"1 1 0 0\"") != std::string::npos);
    CHECK(xml.find("euler=\"0 0 1.57\"") != std::string::npos);
    CHECK(xml.find("mocap=\"true\"") != std::string::npos);
    CHECK(xml.find("childclass=\"ant\"") != std::string::npos);
  }

  TEST_CASE("geom-element") {
    mjcf::Geom geom;
    CHECK(geom.element_name() == "geom");
    geom.name       = "floor";
    geom.type       = mjcf::GeomType::Plane;
    geom.size       = std::array<double, 3>{40.0, 40.0, 40.0};
    geom.pos        = std::array<double, 3>{1.0, 0.0, 0.0};
    geom.material   = "MatPlane";
    geom.rgba       = std::array<double, 4>{0.8, 0.9, 0.8, 1.0};
    geom.contype    = 2;
    geom.condim     = 4;
    geom.group      = 2;
    geom.priority   = 3;
    geom.friction   = {2.0, 0.005, 0.0001};
    geom.solref     = {0.02, 1.0};
    geom.solimp     = {0.9, 0.95, 0.001, 0.5, 2.0};
    geom.solmix     = 1.0;
    geom.mesh       = "mymesh";
    geom.fitscale   = 1.5;
    std::string xml = geom.get_xml_text();
    CHECK(xml.find("name=\"floor\"") != std::string::npos);
    CHECK(xml.find("type=\"plane\"") != std::string::npos);
    CHECK(xml.find("size=\"40 40 40\"") != std::string::npos);
    CHECK(xml.find("pos=\"1 0 0\"") != std::string::npos);
    CHECK(xml.find("material=\"MatPlane\"") != std::string::npos);
    CHECK(xml.find("rgba=\"0.8 0.9 0.8 1\"") != std::string::npos);
    CHECK(xml.find("contype=\"2\"") != std::string::npos);
    CHECK(xml.find("condim=\"4\"") != std::string::npos);
    CHECK(xml.find("group=\"2\"") != std::string::npos);
    CHECK(xml.find("priority=\"3\"") != std::string::npos);
    CHECK(xml.find("friction=\"2 0.005 0.0001\"") != std::string::npos);
    CHECK(xml.find("fitscale=\"1.5\"") != std::string::npos);
  }

  TEST_CASE("light-element") {
    mjcf::Light light;
    CHECK(light.element_name() == "light");
    light.directional = true;
    light.castshadow  = false;
    light.active      = true;
    light.pos         = {0.0, 0.0, 1.3};
    light.dir         = {0.0, 0.0, -1.3};
    light.cutoff      = 100.0;
    light.exponent    = 1.0;
    light.diffuse     = {1.0, 1.0, 1.0};
    light.specular    = {0.1, 0.1, 0.1};
    light.mode        = mjcf::LightMode::Trackcom;
    std::string xml   = light.get_xml_text();
    CHECK(xml.find("directional=\"true\"") != std::string::npos);
    CHECK(xml.find("castshadow=\"false\"") != std::string::npos);
    CHECK(xml.find("pos=\"0 0 1.3\"") != std::string::npos);
    CHECK(xml.find("dir=\"0 0 -1.3\"") != std::string::npos);
    CHECK(xml.find("cutoff=\"100\"") != std::string::npos);
    CHECK(xml.find("exponent=\"1\"") != std::string::npos);
    CHECK(xml.find("diffuse=\"1 1 1\"") != std::string::npos);
    CHECK(xml.find("specular=\"0.1 0.1 0.1\"") != std::string::npos);
    CHECK(xml.find("mode=\"trackcom\"") != std::string::npos);
  }

  TEST_CASE("camera-element") {
    mjcf::Camera camera;
    CHECK(camera.element_name() == "camera");
    camera.name     = "main_cam";
    camera.mode     = mjcf::CameraMode::Trackcom;
    camera.target   = "torso";
    std::string xml = camera.get_xml_text();
    CHECK(xml.find("name=\"main_cam\"") != std::string::npos);
    CHECK(xml.find("mode=\"trackcom\"") != std::string::npos);
    CHECK(xml.find("target=\"torso\"") != std::string::npos);
  }

  TEST_CASE("light-convenience-constructor") {
    // Test convenience constructor for directional light
    mjcf::Light light(true, {0.0, 0.0, 1.5}, {0.0, 0.0, -1.5}, {1.0, 1.0, 1.0});
    CHECK(light.directional == true);
    CHECK(light.pos == std::array<double, 3>{0.0, 0.0, 1.5});
    CHECK(light.dir == std::array<double, 3>{0.0, 0.0, -1.5});
    CHECK(light.diffuse == std::array<double, 3>{1.0, 1.0, 1.0});
    CHECK(light.element_name() == "light");
    
    std::string xml = light.get_xml_text();
    CHECK(xml.find("directional=\"true\"") != std::string::npos);
    CHECK(xml.find("pos=\"0 0 1.5\"") != std::string::npos);
    CHECK(xml.find("dir=\"0 0 -1.5\"") != std::string::npos);
    CHECK(xml.find("diffuse=\"1 1 1\"") != std::string::npos);

    // Test with default diffuse
    mjcf::Light light2(false, {1.0, 2.0, 3.0}, {0.0, 0.0, -1.0});
    CHECK(light2.directional == false);
    CHECK(light2.pos == std::array<double, 3>{1.0, 2.0, 3.0});
    CHECK(light2.dir == std::array<double, 3>{0.0, 0.0, -1.0});
    CHECK(light2.diffuse == std::array<double, 3>{0.7, 0.7, 0.7});
  }

  TEST_CASE("body-name-sanitization") {
    // Test that "world" name is automatically renamed to "world.001"
    
    // Test with Body::Create
    auto body1 = mjcf::Body::Create("world", {1.0, 2.0, 3.0});
    CHECK(body1->name == "world.001");
    std::string xml1 = body1->get_xml_text();
    CHECK(xml1.find("name=\"world.001\"") != std::string::npos);
    CHECK(xml1.find("name=\"world\"") == std::string::npos);
    
    // Test with direct construction and assignment
    mjcf::Body body2;
    body2.name = "world";
    body2.pos = {1.0, 2.0, 3.0};
    // Note: Direct assignment doesn't sanitize, but XML generation does
    CHECK(body2.name == "world"); // Internal name is still "world"
    std::string xml2 = body2.get_xml_text();
    CHECK(xml2.find("name=\"world.001\"") != std::string::npos);
    CHECK(xml2.find("name=\"world\"") == std::string::npos);
    
    // Test that other names are not affected
    auto body3 = mjcf::Body::Create("my_body", {0.0, 0.0, 0.0});
    CHECK(body3->name == "my_body");
    std::string xml3 = body3->get_xml_text();
    CHECK(xml3.find("name=\"my_body\"") != std::string::npos);
    
    // Test empty name
    auto body4 = mjcf::Body::Create("", {0.0, 0.0, 0.0});
    CHECK(body4->name == "");
    
    // Test case sensitivity - "World" should not be renamed
    auto body5 = mjcf::Body::Create("World", {0.0, 0.0, 0.0});
    CHECK(body5->name == "World");
    std::string xml5 = body5->get_xml_text();
    CHECK(xml5.find("name=\"World\"") != std::string::npos);
  }
}
