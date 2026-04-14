#pragma once

#include "element.hpp"
#include <memory>
#include <string>

namespace mjcf {

class Body;
class Mujoco;

/**
 * @brief 外部MJCFファイルを既存のMujocoシーンに取り込むユーティリティクラス
 *
 * add_xml_file() の実装バックエンド。UrdfConverter と同じパターンに従う。
 */
class MjcfImporter {
public:
  /**
   * @brief 外部MJCFファイルを mujoco シーンにマージする
   *
   * @param mujoco       マージ先のルート Mujoco オブジェクト
   * @param filepath     取り込む MJCF XML ファイルのパス
   * @param parent       worldbody 内の挿入先 Body (nullptr = worldbody 直下)
   * @param name_prefix  名前衝突を避けるためのプレフィックス
   * @return             取り込まれた最初の Body (存在しなければ nullptr)
   */
  static std::shared_ptr<Body> import_mjcf(
      Mujoco* mujoco,
      const std::string& filepath,
      std::shared_ptr<Body> parent,
      const std::string& name_prefix);

private:
  // Forward declaration of TinyXML2 types to avoid header dependency
  struct XMLElementPtr;

  static void merge_assets(void* src_elem, Mujoco* dst, const std::string& prefix,
                           const std::string& base_dir);
  static void merge_defaults(void* src_elem, Mujoco* dst, const std::string& prefix);
  static std::shared_ptr<Body> merge_worldbody(void* src_elem, Element* dst_parent,
                                               const std::string& prefix);
  static void merge_section(void* src_elem, Element* dst_container,
                            const std::string& prefix, const std::string& base_dir);

  // XML要素ツリーを GenericElement ツリーとして再帰的に構築
  static std::shared_ptr<Element> build_element_tree(void* xml_elem,
                                                     const std::string& prefix,
                                                     const std::string& base_dir);

  // name/mesh/material/texture/class 等の参照属性にプレフィックスを付与
  static std::string apply_prefix(const std::string& name, const std::string& prefix);
  static bool is_reference_attr(const std::string& attr_name);
};

} // namespace mjcf
