#include "doctest.h"
#include "mjcf/asset_elements.hpp"

#define STR_CONTAINS(haystack, needle) (haystack.find(needle) != std::string::npos)

TEST_CASE("texture-element") {
  mjcf::Texture texture;
  CHECK(texture.element_name() == "texture");
  texture.builtin    = mjcf::TextureBuiltin::Gradient;
  texture.type       = mjcf::TextureType::Skybox;
  texture.file       = "texture.png";
  texture.name       = "mytexture";
  texture.width      = 100;
  texture.height     = 200;
  texture.rgb1       = {1.0, 1.0, 1.0};
  texture.rgb2       = {0.5, 0.0, 0.0};
  texture.mark       = "cross";
  texture.markrgb    = {1.0, 1.0, 1.0};
  texture.random     = 0.01;
  texture.gridsize   = {127, 127};
  texture.gridlayout = "uniform";

  std::string xml = texture.get_xml_text();
  CHECK(STR_CONTAINS(xml, "builtin=\"gradient\""));
  CHECK(STR_CONTAINS(xml, "type=\"skybox\""));
  CHECK(STR_CONTAINS(xml, "file=\"texture.png\""));
  CHECK(STR_CONTAINS(xml, "name=\"mytexture\""));
  CHECK(STR_CONTAINS(xml, "width=\"100\""));
  CHECK(STR_CONTAINS(xml, "height=\"200\""));
  CHECK(STR_CONTAINS(xml, "rgb1=\"1 1 1\""));
  CHECK(STR_CONTAINS(xml, "rgb2=\"0.5 0 0\""));
  CHECK(STR_CONTAINS(xml, "mark=\"cross\""));
  CHECK(STR_CONTAINS(xml, "markrgb=\"1 1 1\""));
  CHECK(STR_CONTAINS(xml, "random=\"0.01\""));
  CHECK(STR_CONTAINS(xml, "gridsize=\"127 127\""));
  CHECK(STR_CONTAINS(xml, "gridlayout=\"uniform\""));
}

TEST_CASE("material-element") {
  mjcf::Material material;
  CHECK(material.element_name() == "material");
  material.name        = "MatPlane";
  material.texture     = "texplane";
  material.texrepeat   = {60.0, 60.0};
  material.texuniform  = true;
  material.emission    = 0.1;
  material.specular    = 1.0;
  material.shininess   = 1.0;
  material.reflectance = 0.5;
  material.rgba        = {0.8, 0.6, 0.4, 1.0};

  std::string xml = material.get_xml_text();
  CHECK(STR_CONTAINS(xml, "name=\"MatPlane\""));
  CHECK(STR_CONTAINS(xml, "texture=\"texplane\""));
  CHECK(STR_CONTAINS(xml, "texrepeat=\"60 60\""));
  CHECK(STR_CONTAINS(xml, "texuniform=\"true\""));
  CHECK(STR_CONTAINS(xml, "emission=\"0.1\""));
  CHECK(STR_CONTAINS(xml, "specular=\"1\""));
  CHECK(STR_CONTAINS(xml, "shininess=\"1\""));
  CHECK(STR_CONTAINS(xml, "reflectance=\"0.5\""));
  CHECK(STR_CONTAINS(xml, "rgba=\"0.8 0.6 0.4 1\""));
}

TEST_CASE("mesh-element") {
  mjcf::Mesh mesh;
  CHECK(mesh.element_name() == "mesh");
  mesh.name         = "mymesh";
  mesh.file         = "model.obj";
  mesh.scale        = {1.0, 1.0, 1.0};
  mesh.smoothnormal = false;
  mesh.vertex       = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0};
  mesh.face         = {0, 1, 2};

  std::string xml = mesh.get_xml_text();
  CHECK(STR_CONTAINS(xml, "name=\"mymesh\""));
  CHECK(STR_CONTAINS(xml, "file=\"model.obj\""));
  // CHECK(STR_CONTAINS(xml, "scale=\"1 1 1\""));
  CHECK(STR_CONTAINS(xml, "smoothnormal=\"false\""));
  CHECK(STR_CONTAINS(xml, "vertex=\"0 0 0 1 0 0\""));
  CHECK(STR_CONTAINS(xml, "face=\"0 1 2\""));
}

TEST_CASE("hfield-element") {
  mjcf::Hfield hfield;
  CHECK(hfield.element_name() == "hfield");
  hfield.name   = "terrain";
  hfield.file   = "terrain.png";
  hfield.size   = {10.0, 10.0, 1.0, 0.1};
  hfield.nrow   = 256;
  hfield.ncol   = 256;

  std::string xml = hfield.get_xml_text();
  CHECK(STR_CONTAINS(xml, "name=\"terrain\""));
  CHECK(STR_CONTAINS(xml, "file=\"terrain.png\""));
  CHECK(STR_CONTAINS(xml, "size=\"10 10 1 0.1\""));
  CHECK(STR_CONTAINS(xml, "nrow=\"256\""));
  CHECK(STR_CONTAINS(xml, "ncol=\"256\""));
}

TEST_CASE("numeric-element") {
  mjcf::Numeric numeric;
  CHECK(numeric.element_name() == "numeric");
  numeric.name = "init_qpos";
  numeric.data = "0.0 0.0 0.75";
  numeric.size = 3;

  std::string xml = numeric.get_xml_text();
  CHECK(STR_CONTAINS(xml, "name=\"init_qpos\""));
  CHECK(STR_CONTAINS(xml, "data=\"0.0 0.0 0.75\""));
  CHECK(STR_CONTAINS(xml, "size=\"3\""));
}

TEST_CASE("text-element") {
  mjcf::Text text;
  CHECK(text.element_name() == "text");
  text.name = "description";
  text.data = "Test model description";

  std::string xml = text.get_xml_text();
  CHECK(STR_CONTAINS(xml, "name=\"description\""));
  CHECK(STR_CONTAINS(xml, "data=\"Test model description\""));
}

TEST_CASE("tuple-element") {
  mjcf::Tuple tuple;
  CHECK(tuple.element_name() == "tuple");
  tuple.name = "mytuple";

  std::string xml = tuple.get_xml_text();
  CHECK(STR_CONTAINS(xml, "name=\"mytuple\""));
}

TEST_CASE("texture-convenience-constructors") {
  SUBCASE("file-based texture constructor") {
    mjcf::Texture texture("mytexture", "texture.png");
    CHECK(texture.name == "mytexture");
    CHECK(texture.file == "texture.png");
    CHECK(texture.element_name() == "texture");
    
    std::string xml = texture.get_xml_text();
    CHECK(STR_CONTAINS(xml, "name=\"mytexture\""));
    CHECK(STR_CONTAINS(xml, "file=\"texture.png\""));
  }

  SUBCASE("procedural texture constructor") {
    mjcf::Texture texture("mytexture", mjcf::TextureBuiltin::Gradient, mjcf::TextureType::Skybox,
                          {1.0, 1.0, 1.0}, {0.0, 0.0, 0.0}, 256, 256);
    CHECK(texture.name == "mytexture");
    CHECK(texture.builtin == mjcf::TextureBuiltin::Gradient);
    CHECK(texture.type == mjcf::TextureType::Skybox);
    CHECK(texture.rgb1 == std::array<double, 3>{1.0, 1.0, 1.0});
    CHECK(texture.rgb2 == std::array<double, 3>{0.0, 0.0, 0.0});
    CHECK(texture.width == 256);
    CHECK(texture.height == 256);
    
    std::string xml = texture.get_xml_text();
    CHECK(STR_CONTAINS(xml, "name=\"mytexture\""));
    CHECK(STR_CONTAINS(xml, "builtin=\"gradient\""));
    CHECK(STR_CONTAINS(xml, "type=\"skybox\""));
  }

  SUBCASE("checker texture static factory") {
    auto texture = mjcf::Texture::CheckerTexture("checker", {0.1, 0.2, 0.3}, {0.2, 0.3, 0.4}, 512, 512);
    CHECK(texture.name == "checker");
    CHECK(texture.builtin == mjcf::TextureBuiltin::Checker);
    CHECK(texture.type == mjcf::TextureType::TwoD);
    CHECK(texture.rgb1 == std::array<double, 3>{0.1, 0.2, 0.3});
    CHECK(texture.rgb2 == std::array<double, 3>{0.2, 0.3, 0.4});
    CHECK(texture.width == 512);
    CHECK(texture.height == 512);
    
    std::string xml = texture.get_xml_text();
    CHECK(STR_CONTAINS(xml, "name=\"checker\""));
    CHECK(STR_CONTAINS(xml, "builtin=\"checker\""));
    // type="2d" is the default and won't be output
    CHECK(STR_CONTAINS(xml, "rgb1=\"0.1 0.2 0.3\""));
    CHECK(STR_CONTAINS(xml, "rgb2=\"0.2 0.3 0.4\""));
  }
}

TEST_CASE("material-convenience-constructors") {
  SUBCASE("simple rgba material constructor") {
    std::array<double, 4> rgba = {0.8, 0.6, 0.4, 1.0};
    mjcf::Material material("mymat", rgba);
    CHECK(material.name == "mymat");
    CHECK(material.rgba == std::array<double, 4>{0.8, 0.6, 0.4, 1.0});
    CHECK(material.element_name() == "material");
    
    std::string xml = material.get_xml_text();
    CHECK(STR_CONTAINS(xml, "name=\"mymat\""));
    CHECK(STR_CONTAINS(xml, "rgba=\"0.8 0.6 0.4 1\""));
  }

  SUBCASE("textured material constructor") {
    mjcf::Material material("mymat", "mytexture", {10.0, 10.0});
    CHECK(material.name == "mymat");
    CHECK(material.texture == "mytexture");
    CHECK(material.texrepeat == std::array<double, 2>{10.0, 10.0});
    CHECK(material.element_name() == "material");
    
    std::string xml = material.get_xml_text();
    CHECK(STR_CONTAINS(xml, "name=\"mymat\""));
    CHECK(STR_CONTAINS(xml, "texture=\"mytexture\""));
    CHECK(STR_CONTAINS(xml, "texrepeat=\"10 10\""));
  }
}
