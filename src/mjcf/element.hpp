#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace mjcf {

using Arr3 = std::array<double, 3>;

class Element;

// 属性値の型エイリアス
using AttributeValue = std::variant<std::string, int, double, bool, std::vector<double>, std::vector<int>>;

/**
 * @brief 全MJCF要素の基底クラス
 *
 * このクラスはMJCF XML要素を作成するためのコア機能を提供します。
 * 属性管理、子要素、XML直列化などの機能が含まれます。
 */
class Element {
public:
  Element();
  virtual ~Element() = default;

  /**
   * @brief 子要素を追加
   * @param child 子要素への共有ポインタ
   */
  virtual void add_child(std::shared_ptr<Element> child);

  /**
   * @brief 複数の子要素を追加
   * @param children 子要素への共有ポインタのベクタ
   */
  void add_children(const std::vector<std::shared_ptr<Element>>& children);

  virtual void set_xml_attrib() const {}

  /**
   * @brief 要素名を取得（クラス名から派生）
   * @return 小文字の要素名
   */
  [[nodiscard]] virtual std::string element_name() const = 0;

  /**
   * @brief 属性値をXML出力用の文字列に変換
   * @param value 属性値
   * @return 文字列表現
   */
  [[nodiscard]] std::string stringify_value(const AttributeValue& value) const;

  [[nodiscard]] const std::vector<std::shared_ptr<Element>>& get_children() const;
  std::vector<std::shared_ptr<Element>>& get_children();

  [[nodiscard]] std::optional<AttributeValue> get_attribute_public(const std::string& name) const;

  /**
   * @brief Set attribute value (public access for merging)
   * @param name Attribute name
   * @param value Attribute value
   */
  void set_attribute_public(const std::string& name, const AttributeValue& value);

  [[nodiscard]] std::string get_xml_text() const { return xml(); }

protected:
  /**
   * @brief 属性値を設定
   * @param name 属性名
   * @param value 属性値
   */
  void set_attribute(const std::string& name, const AttributeValue& value) const;

  [[nodiscard]] std::optional<AttributeValue> get_attribute(const std::string& name) const;

  /**
   * @brief 属性がデフォルト値かどうかをチェック
   * @param name 属性名
   * @param value 現在の値
   * @return 値がデフォルトの場合true
   */
  [[nodiscard]] virtual bool is_default_value(const std::string& name, const AttributeValue& value) const;
  [[nodiscard]] std::string xml() const;

  void* write_xml_element_base(void* doc, void* parent) const;

public:
  virtual void* write_xml_element(void* doc, void* parent) const { return write_xml_element_base(doc, parent); }

private:
  mutable std::map<std::string, AttributeValue> attributes_;
  std::vector<std::shared_ptr<Element>> children_;
};

} // namespace mjcf
