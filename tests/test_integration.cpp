#include "doctest.h"
#include "mjcf/mjcf.hpp"
#include <fstream>

TEST_SUITE("Integration Tests") {
  TEST_CASE("Create empty world model") {
    // Recreate the gen_empty.py example in C++

    auto mujoco = std::make_shared<mjcf::Mujoco>("empty");

    mujoco->option_->integrator = mjcf::IntegratorType::RK4;
    mujoco->option_->timestep   = 0.01;

    auto asset     = mujoco->asset_;
    auto worldbody = mujoco->worldbody_;

    // Level 3 - Assets
    auto tex1     = std::make_shared<mjcf::Texture>();
    tex1->builtin = mjcf::TextureBuiltin::Gradient;
    tex1->height  = 100;
    tex1->rgb1    = {1.0, 1.0, 1.0};
    tex1->rgb2    = {0.0, 0.0, 0.0};
    tex1->type    = mjcf::TextureType::Skybox;
    tex1->width   = 100;

    auto tex2     = std::make_shared<mjcf::Texture>();
    tex2->builtin = mjcf::TextureBuiltin::Flat;
    tex2->height  = 1278;
    tex2->mark    = "cross";
    tex2->markrgb = {1.0, 1.0, 1.0};
    tex2->name    = "texgeom";
    tex2->random  = 0.01;
    tex2->rgb1    = {0.8, 0.6, 0.4};
    tex2->rgb2    = {0.8, 0.6, 0.4};
    tex2->type    = mjcf::TextureType::Cube;
    tex2->width   = 127;

    auto tex3     = std::make_shared<mjcf::Texture>();
    tex3->builtin = mjcf::TextureBuiltin::Checker;
    tex3->height  = 100;
    tex3->name    = "texplane";
    tex3->rgb1    = {0.0, 0.0, 0.0};
    tex3->rgb2    = {0.8, 0.8, 0.8};
    tex3->type    = mjcf::TextureType::TwoD;
    tex3->width   = 100;

    auto mat1         = std::make_shared<mjcf::Material>();
    mat1->name        = "MatPlane";
    mat1->reflectance = 0.5;
    mat1->shininess   = 1.0;
    mat1->specular    = 1.0;
    mat1->texrepeat   = {60.0, 60.0};
    mat1->texture     = "texplane";

    auto mat2        = std::make_shared<mjcf::Material>();
    mat2->name       = "geom";
    mat2->texture    = "texgeom";
    mat2->texuniform = true;

    asset->add_children({tex1, tex2, tex3, mat1, mat2});

    // Worldbody elements
    auto light         = std::make_shared<mjcf::Light>();
    light->cutoff      = 100.0;
    light->diffuse     = {1.0, 1.0, 1.0};
    light->dir         = {0.0, 0.0, -1.3};
    light->directional = true;
    light->exponent    = 1.0;
    light->pos         = {0.0, 0.0, 1.3};
    light->specular    = {0.1, 0.1, 0.1};

    auto floor_geom         = std::make_shared<mjcf::Geom>();
    floor_geom->conaffinity = 1;
    floor_geom->condim      = 3;
    floor_geom->material    = "MatPlane";
    floor_geom->name        = "floor";
    floor_geom->pos         = std::array<double, 3>{0.0, 0.0, 0.0};
    floor_geom->rgba        = std::array<double, 4>{0.8, 0.9, 0.8, 1.0};
    floor_geom->size        = std::array<double, 3>{40.0, 40.0, 40.0};
    floor_geom->type        = mjcf::GeomType::Plane;

    worldbody->add_children({light, floor_geom});

    // Generate XML
    std::string model_xml = mujoco->get_xml_text();

    // Verify structure
    CHECK(model_xml.find("<mujoco") != std::string::npos);
    CHECK(model_xml.find("model=\"empty\"") != std::string::npos);
    CHECK(model_xml.find("<option") != std::string::npos);
    CHECK(model_xml.find("integrator=\"RK4\"") != std::string::npos);
    CHECK(model_xml.find("timestep=\"0.01\"") != std::string::npos);
    CHECK(model_xml.find("<asset") != std::string::npos);
    CHECK(model_xml.find("<texture") != std::string::npos);
    CHECK(model_xml.find("builtin=\"gradient\"") != std::string::npos);
    CHECK(model_xml.find("<material") != std::string::npos);
    CHECK(model_xml.find("name=\"MatPlane\"") != std::string::npos);
    CHECK(model_xml.find("<worldbody") != std::string::npos);
    CHECK(model_xml.find("<light") != std::string::npos);
    CHECK(model_xml.find("directional=\"true\"") != std::string::npos);
    CHECK(model_xml.find("<geom") != std::string::npos);
    CHECK(model_xml.find("type=\"plane\"") != std::string::npos);
    CHECK(model_xml.find("</mujoco>") != std::string::npos);

    // Check that XML is well-formed and not empty
    CHECK(model_xml.size() > 500); // Should be a substantial XML document

    // Optional: Save to file for manual inspection
    // This helps debug XML generation issues
    std::ofstream outfile("empty-gen.xml.out");
    if(outfile.is_open()) {
      outfile << model_xml;
      outfile.close();
    }
  }

  TEST_CASE("Factory methods") {
    // Test the Elements factory class
    auto mujoco = std::make_shared<mjcf::Mujoco>("test_model");
    auto option = mujoco->option_;
    auto asset  = mujoco->asset_;

    CHECK(mujoco->element_name() == "mujoco");
    CHECK(option->element_name() == "option");
    CHECK(asset->element_name() == "asset");

    // Test that factory creates valid objects
    option->timestep = 0.005;
    std::string xml  = option->get_xml_text();
    CHECK(xml.find("timestep=\"0.005\"") != std::string::npos);

    // Test adding children through factory-created objects
    auto texture  = std::make_shared<mjcf::Texture>();
    texture->name = "test_tex";
    asset->add_child(texture);

    std::string asset_xml = asset->get_xml_text();
    CHECK(asset_xml.find("<texture") != std::string::npos);
    CHECK(asset_xml.find("name=\"test_tex\"") != std::string::npos);
  }

  TEST_CASE("Complex hierarchy") {
    // Test deep nesting and complex structures
    auto mujoco    = std::make_shared<mjcf::Mujoco>("complex_model");
    auto worldbody = mujoco->worldbody_;

    // Create a simple robot arm structure
    auto base_body  = std::make_shared<mjcf::Body>();
    base_body->name = "base";
    base_body->pos  = {0.0, 0.0, 0.1};

    auto base_geom  = std::make_shared<mjcf::Geom>();
    base_geom->name = "base_geom";
    base_geom->type = mjcf::GeomType::Box;
    base_geom->size = std::array<double, 3>{0.1, 0.1, 0.1};

    auto base_joint  = std::make_shared<mjcf::Joint>();
    base_joint->name = "base_joint";
    base_joint->type = mjcf::JointType::Hinge;
    base_joint->axis = {0.0, 0.0, 1.0};

    base_body->add_children({base_geom, base_joint});

    // Add second link
    auto link1_body  = std::make_shared<mjcf::Body>();
    link1_body->name = "link1";
    link1_body->pos  = {0.0, 0.0, 0.2};

    auto link1_geom  = std::make_shared<mjcf::Geom>();
    link1_geom->name = "link1_geom";
    link1_geom->type = mjcf::GeomType::Capsule;
    link1_geom->size = {0.05, 0.2, 0.0}; // Capsule uses radius and half-length

    auto link1_joint     = std::make_shared<mjcf::Joint>();
    link1_joint->name    = "link1_joint";
    link1_joint->type    = mjcf::JointType::Hinge;
    link1_joint->axis    = {1.0, 0.0, 0.0};
    link1_joint->range   = {-90.0, 90.0};
    link1_joint->limited = true;

    link1_body->add_children({link1_geom, link1_joint});
    base_body->add_child(link1_body);

    worldbody->add_child(base_body);

    std::string xml = mujoco->get_xml_text();

    // Verify nested structure
    CHECK(xml.find("<mujoco") != std::string::npos);
    CHECK(xml.find("<worldbody") != std::string::npos);
    CHECK(xml.find("name=\"base\"") != std::string::npos);
    CHECK(xml.find("name=\"base_geom\"") != std::string::npos);
    CHECK(xml.find("name=\"base_joint\"") != std::string::npos);
    CHECK(xml.find("name=\"link1\"") != std::string::npos);
    CHECK(xml.find("name=\"link1_geom\"") != std::string::npos);
    CHECK(xml.find("name=\"link1_joint\"") != std::string::npos);
    CHECK(xml.find("limited=\"true\"") != std::string::npos);
    CHECK(xml.find("range=\"-90 90\"") != std::string::npos);

    // Check for proper nesting (link1 body should be inside base body)
    size_t base_pos  = xml.find("name=\"base\"");
    size_t link1_pos = xml.find("name=\"link1\"");
    CHECK(base_pos < link1_pos); // base should appear before link1

    // Optional: Save complex model for inspection
    std::ofstream outfile("complex-gen.xml.out");
    if(outfile.is_open()) {
      outfile << xml;
      outfile.close();
    }
  }

  TEST_CASE("Body with 'world' name is sanitized in full model") {
    // Create a full model with a body named "world"
    auto mujoco = std::make_shared<mjcf::Mujoco>("test_world_name");
    
    auto worldbody = mujoco->worldbody_;
    
    // Create a body with the reserved name "world"
    auto world_body = mjcf::Body::Create("world", {1.0, 0.0, 1.0});
    
    // Add a geom to the body
    auto world_geom = std::make_shared<mjcf::Geom>();
    world_geom->name = "world_geom";
    world_geom->type = mjcf::GeomType::Sphere;
    world_geom->size = {0.1, 0.0, 0.0};
    
    world_body->add_child(world_geom);
    worldbody->add_child(world_body);
    
    // Also add a normal body for comparison
    auto normal_body = mjcf::Body::Create("normal_body", {2.0, 0.0, 1.0});
    auto normal_geom = std::make_shared<mjcf::Geom>();
    normal_geom->name = "normal_geom";
    normal_geom->type = mjcf::GeomType::Box;
    normal_geom->size = {0.1, 0.1, 0.1};
    
    normal_body->add_child(normal_geom);
    worldbody->add_child(normal_body);
    
    // Generate XML
    std::string xml = mujoco->get_xml_text();
    
    // Verify that "world" was renamed to "world.001"
    CHECK(xml.find("name=\"world.001\"") != std::string::npos);
    CHECK(xml.find("<body name=\"world\"") == std::string::npos);
    
    // Verify normal body is unchanged
    CHECK(xml.find("name=\"normal_body\"") != std::string::npos);
    
    // Verify geoms are present
    CHECK(xml.find("name=\"world_geom\"") != std::string::npos);
    CHECK(xml.find("name=\"normal_geom\"") != std::string::npos);
  }
}
