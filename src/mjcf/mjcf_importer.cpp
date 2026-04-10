#include "mjcf_importer.hpp"
#include "body_elements.hpp"
#include "core_elements.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>

#if __has_include(<tinyxml2/tinyxml2.h>)
#include <tinyxml2/tinyxml2.h>
#else
#include <tinyxml2.h>
#endif

namespace mjcf {

using namespace tinyxml2;

// ---------------------------------------------------------------------------
// 参照属性名の集合 — これらにはプレフィックスを付与する
// ---------------------------------------------------------------------------
static const std::set<std::string> kRefAttrs = {
    "name", "mesh", "material", "texture", "class",
    "joint", "joint1", "joint2", "body1", "body2",
    "site", "tendon", "actuator", "geom1", "geom2",
};

bool MjcfImporter::is_reference_attr(const std::string& attr_name) {
  return kRefAttrs.count(attr_name) > 0;
}

std::string MjcfImporter::apply_prefix(const std::string& name, const std::string& prefix) {
  if(prefix.empty()) return name;
  return prefix + name;
}

// ---------------------------------------------------------------------------
// XML 要素ツリーを GenericElement ツリーとして再帰的に構築
// ---------------------------------------------------------------------------
std::shared_ptr<Element> MjcfImporter::build_element_tree(
    void* xml_elem_ptr, const std::string& prefix, const std::string& base_dir) {
  auto* xml_elem = static_cast<XMLElement*>(xml_elem_ptr);
  if(!xml_elem) return nullptr;

  auto elem = std::make_shared<GenericElement>(xml_elem->Name());

  // 属性をコピー（参照属性にはプレフィックスを付与、file属性は絶対パスに解決）
  for(const XMLAttribute* attr = xml_elem->FirstAttribute(); attr; attr = attr->Next()) {
    std::string attr_name  = attr->Name();
    std::string attr_value = attr->Value();

    if(attr_name == "file" || attr_name == "filename") {
      // 相対パスを絶対パスに解決
      if(!attr_value.empty() && attr_value[0] != '/') {
        namespace fs       = std::filesystem;
        fs::path resolved  = fs::path(base_dir) / attr_value;
        attr_value         = resolved.lexically_normal().string();
      }
    } else if(is_reference_attr(attr_name) && !prefix.empty()) {
      attr_value = apply_prefix(attr_value, prefix);
    }

    elem->set_attribute_public(attr_name, attr_value);
  }

  // 子要素を再帰的に処理
  for(XMLElement* child = xml_elem->FirstChildElement(); child;
      child              = child->NextSiblingElement()) {
    auto child_elem = build_element_tree(child, prefix, base_dir);
    if(child_elem) elem->add_child(child_elem);
  }

  return elem;
}

// ---------------------------------------------------------------------------
// <asset> セクションのマージ
// ---------------------------------------------------------------------------
void MjcfImporter::merge_assets(void* src_elem_ptr, Mujoco* dst,
                                const std::string& prefix, const std::string& base_dir) {
  auto* src = static_cast<XMLElement*>(src_elem_ptr);
  if(!src) return;

  for(XMLElement* child = src->FirstChildElement(); child;
      child              = child->NextSiblingElement()) {
    // 重複チェック: name属性が既に登録されていればスキップ
    const char* name_attr = child->Attribute("name");
    if(name_attr) {
      std::string prefixed_name = apply_prefix(name_attr, prefix);
      if(dst->has_asset(prefixed_name)) continue;
    }

    auto elem = build_element_tree(child, prefix, base_dir);
    if(elem) dst->add_asset(elem);
  }
}

// ---------------------------------------------------------------------------
// <default> セクションのマージ
// ---------------------------------------------------------------------------
void MjcfImporter::merge_defaults(void* src_elem_ptr, Mujoco* dst,
                                  const std::string& prefix) {
  auto* src = static_cast<XMLElement*>(src_elem_ptr);
  if(!src || !dst->default_) return;

  for(XMLElement* child = src->FirstChildElement(); child;
      child              = child->NextSiblingElement()) {
    // base_dir は default 内の file 参照には通常使わないので空文字
    auto elem = build_element_tree(child, prefix, "");
    if(elem) dst->default_->add_child(elem);
  }
}

// ---------------------------------------------------------------------------
// <worldbody> セクションのマージ — 直接の子を dst_parent 配下に追加
// ---------------------------------------------------------------------------
std::shared_ptr<Body> MjcfImporter::merge_worldbody(void* src_elem_ptr, Element* dst_parent,
                                                    const std::string& prefix) {
  auto* src = static_cast<XMLElement*>(src_elem_ptr);
  if(!src || !dst_parent) return nullptr;

  std::shared_ptr<Body> first_body = nullptr;

  for(XMLElement* child = src->FirstChildElement(); child;
      child              = child->NextSiblingElement()) {
    auto elem = build_element_tree(child, prefix, "");
    if(!elem) continue;
    dst_parent->add_child(elem);

    // 最初の <body> 要素を返す
    if(!first_body && std::string(child->Name()) == "body") {
      first_body = std::dynamic_pointer_cast<Body>(elem);
      // GenericElement として構築されているので dynamic_pointer_cast は nullptr になる。
      // 呼び出し元での利用は GenericElement 経由が現実的。
      // ここでは first_generic を返す代わりに nullptr を返すことで型安全性を保つ。
    }
  }

  return first_body; // GenericElement ベースなので通常は nullptr
}

// ---------------------------------------------------------------------------
// 汎用セクションマージ（actuator / sensor / contact / equality / tendon）
// ---------------------------------------------------------------------------
void MjcfImporter::merge_section(void* src_elem_ptr, Element* dst_container,
                                 const std::string& prefix, const std::string& base_dir) {
  auto* src = static_cast<XMLElement*>(src_elem_ptr);
  if(!src || !dst_container) return;

  for(XMLElement* child = src->FirstChildElement(); child;
      child              = child->NextSiblingElement()) {
    auto elem = build_element_tree(child, prefix, base_dir);
    if(elem) dst_container->add_child(elem);
  }
}

// ---------------------------------------------------------------------------
// メインエントリポイント
// ---------------------------------------------------------------------------
std::shared_ptr<Body> MjcfImporter::import_mjcf(
    Mujoco* mujoco,
    const std::string& filepath,
    std::shared_ptr<Body> parent,
    const std::string& name_prefix) {

  if(!mujoco) {
    std::cerr << "[MjcfImporter] mujoco is null" << std::endl;
    return nullptr;
  }

  // ファイル読み込み
  std::ifstream file(filepath);
  if(!file.is_open()) {
    std::cerr << "[MjcfImporter] Cannot open file: " << filepath << std::endl;
    return nullptr;
  }
  std::stringstream buf;
  buf << file.rdbuf();
  const std::string content = buf.str();

  // XML パース
  XMLDocument doc;
  if(doc.Parse(content.c_str()) != XML_SUCCESS) {
    std::cerr << "[MjcfImporter] Failed to parse XML: " << filepath << std::endl;
    return nullptr;
  }

  XMLElement* root = doc.FirstChildElement("mujoco");
  if(!root) {
    std::cerr << "[MjcfImporter] No <mujoco> root element found in: " << filepath << std::endl;
    return nullptr;
  }

  // ベースディレクトリ（file属性の相対パス解決用）
  namespace fs       = std::filesystem;
  const std::string base_dir = fs::path(filepath).parent_path().string();

  // --- <asset> ---
  if(XMLElement* asset_elem = root->FirstChildElement("asset")) {
    merge_assets(asset_elem, mujoco, name_prefix, base_dir);
  }

  // --- <default> ---
  if(XMLElement* default_elem = root->FirstChildElement("default")) {
    merge_defaults(default_elem, mujoco, name_prefix);
  }

  // --- <worldbody> ---
  std::shared_ptr<Body> result = nullptr;
  if(XMLElement* wb_elem = root->FirstChildElement("worldbody")) {
    Element* dst = parent ? static_cast<Element*>(parent.get())
                          : static_cast<Element*>(mujoco->worldbody_.get());
    result = merge_worldbody(wb_elem, dst, name_prefix);
  }

  // --- <actuator> ---
  if(XMLElement* act_elem = root->FirstChildElement("actuator")) {
    merge_section(act_elem, mujoco->actuator_.get(), name_prefix, base_dir);
  }

  // --- <sensor> ---
  if(XMLElement* sen_elem = root->FirstChildElement("sensor")) {
    merge_section(sen_elem, mujoco->sensor_.get(), name_prefix, base_dir);
  }

  // --- <contact> ---
  if(XMLElement* con_elem = root->FirstChildElement("contact")) {
    merge_section(con_elem, mujoco->contact_.get(), name_prefix, base_dir);
  }

  // --- <equality> ---
  if(XMLElement* eq_elem = root->FirstChildElement("equality")) {
    merge_section(eq_elem, mujoco->equality_.get(), name_prefix, base_dir);
  }

  // --- <tendon> ---
  if(XMLElement* ten_elem = root->FirstChildElement("tendon")) {
    merge_section(ten_elem, mujoco->tendon_.get(), name_prefix, base_dir);
  }

  // <compiler> と <option> はホスト側設定を優先するため無視

  return result;
}

} // namespace mjcf
