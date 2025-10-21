#include "asset_elements.hpp"

namespace mjcf {

Texture::Texture(const std::string& name_, const std::string& file_) {
  this->name = name_;
  this->file = file_;
}

Texture::Texture(const std::string& name_, TextureBuiltin builtin_, TextureType type_, const std::array<double, 3>& rgb1_, const std::array<double, 3>& rgb2_, int width_, int height_) {
  this->name    = name_;
  this->builtin = builtin_;
  this->type    = type_;
  this->rgb1    = rgb1_;
  this->rgb2    = rgb2_;    
  this->width   = width_;
  this->height  = height_;  
}

Texture Texture::CheckerTexture(const std::string& name, const std::array<double, 3>& rgb1, const std::array<double, 3>& rgb2, int width, int height) { return Texture(name, TextureBuiltin::Checker, TextureType::TwoD, rgb1, rgb2, width, height); }

void Texture::set_xml_attrib() const {
  auto* mutable_this = const_cast<Texture*>(this);

  // Only set non-default values
  if(builtin != TextureBuiltin::None) mutable_this->set_attribute("builtin", to_string(builtin));
  if(type != TextureType::TwoD) mutable_this->set_attribute("type", to_string(type));
  if(!file.empty()) mutable_this->set_attribute("file", file);
  if(!name.empty()) mutable_this->set_attribute("name", name);
  if(width != 0) mutable_this->set_attribute("width", width);
  if(height != 0) mutable_this->set_attribute("height", height);

  if(rgb1 != std::array<double, 3>{0.0, 0.0, 0.0}) mutable_this->set_attribute("rgb1", std::vector<double>(rgb1.begin(), rgb1.end()));
  if(rgb2 != std::array<double, 3>{0.0, 0.0, 0.0}) mutable_this->set_attribute("rgb2", std::vector<double>(rgb2.begin(), rgb2.end()));
  if(!mark.empty()) mutable_this->set_attribute("mark", mark);
  if(markrgb != std::array<double, 3>{0.0, 0.0, 0.0}) mutable_this->set_attribute("markrgb", std::vector<double>(markrgb.begin(), markrgb.end()));
  if(random != 0.0) mutable_this->set_attribute("random", random);
  if(gridsize != std::array<int, 2>{0, 0}) mutable_this->set_attribute("gridsize", std::vector<int>(gridsize.begin(), gridsize.end()));
  if(!gridlayout.empty()) mutable_this->set_attribute("gridlayout", gridlayout);
}

bool Texture::from_xml([[maybe_unused]] const std::string& xml_str) {
  // Default implementation - could be extended to parse XML
  return false;
}

bool Texture::is_default_value(const std::string& name, const AttributeValue& value) const {
  if(name == "type" && std::get<std::string>(value) == to_string(TextureType::TwoD)) return true;
  if(name == "builtin" && std::get<std::string>(value) == to_string(TextureBuiltin::None)) return true;
  if(name == "width" && std::get<int>(value) == 0) return true;
  if(name == "height" && std::get<int>(value) == 0) return true;
  if(name == "random" && std::get<double>(value) == 0.0) return true;
  return false;
}

// Material implementation
Material::Material(const std::string& name_, const std::array<double, 4>& rgba_) : name(name_), rgba(rgba_) {}

Material::Material(const std::string& name_, const std::string& texture_, const std::array<double, 2>& texrepeat_) : name(name_), texture(texture_), texrepeat(texrepeat_) {}

void Material::set_xml_attrib() const {
  auto* mutable_this = const_cast<Material*>(this);

  // Only set non-default values
  if(!name.empty()) mutable_this->set_attribute("name", name);
  if(!texture.empty()) mutable_this->set_attribute("texture", texture);

  if(texrepeat != std::array<double, 2>{0.0, 0.0}) {
    mutable_this->set_attribute("texrepeat", std::vector<double>(texrepeat.begin(), texrepeat.end()));
  }

  if(texuniform) mutable_this->set_attribute("texuniform", texuniform);
  if(emission != 0.0) mutable_this->set_attribute("emission", emission);
  if(specular != 0.0) mutable_this->set_attribute("specular", specular);
  if(shininess != 0.0) mutable_this->set_attribute("shininess", shininess);
  if(reflectance != 0.0) mutable_this->set_attribute("reflectance", reflectance);

  if(rgba != std::array<double, 4>{0.0, 0.0, 0.0, 0.0}) {
    mutable_this->set_attribute("rgba", std::vector<double>(rgba.begin(), rgba.end()));
  }
}

bool Material::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool Material::is_default_value(const std::string& name, const AttributeValue& value) const {
  if(name == "emission" && std::get<double>(value) == 0.0) return true;
  if(name == "specular" && std::get<double>(value) == 0.0) return true;
  if(name == "shininess" && std::get<double>(value) == 0.0) return true;
  if(name == "reflectance" && std::get<double>(value) == 0.0) return true;
  if(name == "texuniform" && !std::get<bool>(value)) return true;
  return false;
}

// Mesh implementation
Mesh::Mesh() = default;

void Mesh::set_xml_attrib() const {
  auto* mutable_this = const_cast<Mesh*>(this);

  // Only set non-default values
  if(!name.empty()) mutable_this->set_attribute("name", name);
  if(!file.empty()) mutable_this->set_attribute("file", file);
  if(scale != std::array<double, 3>{1.0, 1.0, 1.0}) {
    mutable_this->set_attribute("scale", std::vector<double>(scale.begin(), scale.end()));
  }
  if(!smoothnormal) mutable_this->set_attribute("smoothnormal", smoothnormal);
  if(!vertex.empty()) mutable_this->set_attribute("vertex", vertex);
  if(!normal.empty()) mutable_this->set_attribute("normal", normal);
  if(!face.empty()) mutable_this->set_attribute("face", face);
}

bool Mesh::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool Mesh::is_default_value(const std::string& name, const AttributeValue& value) const {
  if(name == "smoothnormal" && std::get<bool>(value)) return true;
  return false;
}

// Hfield implementation
Hfield::Hfield() = default;

void Hfield::set_xml_attrib() const {
  // Set attributes from public members before generating XML
  auto* mutable_this = const_cast<Hfield*>(this);

  // Only set non-default values
  if(!name.empty()) mutable_this->set_attribute("name", name);
  if(!file.empty()) mutable_this->set_attribute("file", file);
  if(!size.empty()) mutable_this->set_attribute("size", size);
  if(nrow != 0) mutable_this->set_attribute("nrow", nrow);
  if(ncol != 0) mutable_this->set_attribute("ncol", ncol);
}

bool Hfield::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool Hfield::is_default_value([[maybe_unused]] const std::string& name, [[maybe_unused]] const AttributeValue& value) const { return false; }

// Numeric implementation
Numeric::Numeric() = default;

void Numeric::set_xml_attrib() const {
  // Set attributes from public members before generating XML
  auto* mutable_this = const_cast<Numeric*>(this);

  // Only set non-default values
  if(!name.empty()) mutable_this->set_attribute("name", name);
  if(!data.empty()) mutable_this->set_attribute("data", data);
  if(size != 0) mutable_this->set_attribute("size", size);
}

bool Numeric::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool Numeric::is_default_value(const std::string& name, const AttributeValue& value) const {
  if(name == "data" && std::get<std::string>(value) == "0 0 ...") return true;
  return false;
}

// Text implementation
Text::Text() = default;

void Text::set_xml_attrib() const {
  auto* mutable_this = const_cast<Text*>(this);

  if(!name.empty()) mutable_this->set_attribute("name", name);
  if(!data.empty()) mutable_this->set_attribute("data", data);
}

bool Text::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool Text::is_default_value([[maybe_unused]] const std::string& name, [[maybe_unused]] const AttributeValue& value) const { return false; }

// Tuple implementation
Tuple::Tuple() = default;

void Tuple::set_xml_attrib() const {
  auto* mutable_this = const_cast<Tuple*>(this);
  if(!name.empty()) mutable_this->set_attribute("name", name);
}

bool Tuple::from_xml([[maybe_unused]] const std::string& xml_str) { return false; }

bool Tuple::is_default_value([[maybe_unused]] const std::string& name, [[maybe_unused]] const AttributeValue& value) const { return false; }

} // namespace mjcf
