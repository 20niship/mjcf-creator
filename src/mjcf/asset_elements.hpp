#pragma once

#include "element.hpp"
#include "enums.hpp"
#include <array>

namespace mjcf {

class Texture : public Element {
public:
  // Public member variables - use empty strings/zeros as sentinels
  TextureBuiltin builtin        = TextureBuiltin::None;
  TextureType type              = TextureType::TwoD;
  std::string file              = "";
  std::string name              = "";
  int width                     = 0;
  int height                    = 0;
  std::array<double, 3> rgb1    = {0.0, 0.0, 0.0};
  std::array<double, 3> rgb2    = {0.0, 0.0, 0.0};
  std::string mark              = "";
  std::array<double, 3> markrgb = {0.0, 0.0, 0.0};
  double random                 = 0.0;
  std::array<int, 2> gridsize   = {0, 0};
  std::string gridlayout        = "";

  Texture() = default;

  // Convenience constructor for file-based texture
  Texture(const std::string& name, const std::string& file);

  // Convenience constructor for procedural texture
  Texture(const std::string& name, TextureBuiltin builtin, TextureType type, const std::array<double, 3>& rgb1, const std::array<double, 3>& rgb2, int width = 512, int height = 512);

  // Static factory method for checkerboard texture
  static Texture CheckerTexture(const std::string& name, const std::array<double, 3>& rgb1 = {0.1, 0.2, 0.3}, const std::array<double, 3>& rgb2 = {0.2, 0.3, 0.4}, int width = 512, int height = 512);

  std::string element_name() const override { return "texture"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

class Material : public Element {
public:
  std::string name;
  std::string texture;
  std::array<double, 2> texrepeat = {0.0, 0.0};
  bool texuniform                 = false;
  double emission                 = 0.0;
  double specular                 = 0.0;
  double shininess                = 0.0;
  double reflectance              = 0.0;
  std::array<double, 4> rgba      = {0.0, 0.0, 0.0, 0.0};

  Material() = default;

  Material(const std::string& name, const std::array<double, 4>& rgba);
  Material(const std::string& name, const std::string& texture, const std::array<double, 2>& texrepeat = {1.0, 1.0});

  std::string element_name() const override { return "material"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

class Mesh : public Element {
public:
  std::string name;
  std::string file;
  Arr3 scale        = {1.0, 1.0, 1.0}; // MuJoCo default
  bool smoothnormal = true;            // MuJoCo default
  std::vector<double> vertex;          // Keep as vector for variable length
  std::vector<double> normal;          // Keep as vector for variable length
  std::vector<int> face;               // Keep as vector for variable length

  Mesh() = default;
  static std::shared_ptr<Mesh> Create(const std::string& name, const std::string& file, const Arr3& scale = {1.0, 1.0, 1.0}){
    auto mesh         = std::make_shared<Mesh>();
    mesh->name        = name;
    mesh->file        = file;
    mesh->scale       = scale;
    mesh->smoothnormal = true;
    return mesh;
  }

  std::string element_name() const override { return "mesh"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Height field asset element
 */
class Hfield : public Element {
public:
  std::string name = "";
  std::string file = "";
  std::vector<double> size; // Keep as vector for variable length
  int nrow = 0;
  int ncol = 0;

  Hfield();

  std::string element_name() const override { return "hfield"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Numeric custom element
 */
class Numeric : public Element {
public:
  std::string name = "";
  std::string data = "";
  int size         = 0;

  Numeric();

  std::string element_name() const override { return "numeric"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Text custom element
 */
class Text : public Element {
public:
  std::string name = "";
  std::string data = "";

  Text();

  std::string element_name() const override { return "text"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Tuple custom element
 *
 * Tupleは、MuJoCoオブジェクトのリストを作成するためのカスタム要素です。
 * オブジェクトを名前で参照してリストを作成します。
 * std::tupleとは異なり、XMLでMuJoCoモデルの複数の要素をグループ化する目的で使用されます。
 */
class Tuple : public Element {
public:
  std::string name = "";

  Tuple();

  std::string element_name() const override { return "tuple"; }

  void set_xml_attrib() const override;
  bool from_xml(const std::string& xml_str) override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

} // namespace mjcf
