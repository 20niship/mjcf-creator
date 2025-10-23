#include "element.hpp"
#include "../ext/tinyxml2.h"
#include <iomanip>
#include <sstream>

namespace mjcf {

Element::Element() = default;

void Element::add_child(std::shared_ptr<Element> child) {
  if(child) children_.push_back(child);
}

void Element::add_children(const std::vector<std::shared_ptr<Element>>& children) {
  for(const auto& child : children) {
    if(child == nullptr) continue;
    add_child(child);
  }
}

const std::vector<std::shared_ptr<Element>>& Element::get_children() const { return children_; }

std::vector<std::shared_ptr<Element>>& Element::get_children() { return children_; }

std::optional<AttributeValue> Element::get_attribute_public(const std::string& name) const { return get_attribute(name); }

void Element::set_attribute_public(const std::string& name, const AttributeValue& value) { set_attribute(name, value); }

void Element::set_attribute(const std::string& name, const AttributeValue& value) const { attributes_[name] = value; }

std::optional<AttributeValue> Element::get_attribute(const std::string& name) const {
  auto it = attributes_.find(name);
  if(it != attributes_.end()) {
    return it->second;
  }
  return std::nullopt;
}

std::string Element::stringify_value(const AttributeValue& value) const {
  return std::visit(
    [](const auto& v) -> std::string {
      using T = std::decay_t<decltype(v)>;
      if constexpr(std::is_same_v<T, std::string>) {
        return v;
      } else if constexpr(std::is_same_v<T, bool>) {
        return v ? "true" : "false";
      } else if constexpr(std::is_same_v<T, int>) {
        return std::to_string(v);
      } else if constexpr(std::is_same_v<T, double>) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6) << v;
        std::string result = oss.str();
        // Remove trailing zeros
        result.erase(result.find_last_not_of('0') + 1, std::string::npos);
        result.erase(result.find_last_not_of('.') + 1, std::string::npos);
        return result;
      } else if constexpr(std::is_same_v<T, std::vector<double>>) {
        std::ostringstream oss;
        for(size_t i = 0; i < v.size(); ++i) {
          if(i > 0) oss << " ";
          oss << std::fixed << std::setprecision(6) << v[i];
        }
        std::string result = oss.str();
        // Remove trailing zeros from each number
        std::istringstream iss(result);
        std::ostringstream clean_oss;
        std::string number;
        bool first = true;
        while(iss >> number) {
          if(!first) clean_oss << " ";
          first = false;
          if(number.find('.') != std::string::npos) {
            number.erase(number.find_last_not_of('0') + 1, std::string::npos);
            number.erase(number.find_last_not_of('.') + 1, std::string::npos);
          }
          clean_oss << number;
        }
        return clean_oss.str();
      } else if constexpr(std::is_same_v<T, std::vector<int>>) {
        std::ostringstream oss;
        for(size_t i = 0; i < v.size(); ++i) {
          if(i > 0) oss << " ";
          oss << v[i];
        }
        return oss.str();
      } else {
        return "";
      }
    },
    value);
}

bool Element::is_default_value(const std::string& name, const AttributeValue& value) const {
  // Default implementation - subclasses can override
  return false;
}

std::string Element::xml() const {
  tinyxml2::XMLDocument doc;
  auto root = write_xml_element(&doc, nullptr);

  tinyxml2::XMLPrinter printer;
  doc.Print(&printer);
  return printer.CStr();
}

void* Element::write_xml_element_base(void* doc_ptr, void* parent_ptr) const {
  auto doc    = static_cast<tinyxml2::XMLDocument*>(doc_ptr);
  auto parent = static_cast<tinyxml2::XMLElement*>(parent_ptr);

  tinyxml2::XMLElement* element;
  this->set_xml_attrib();

  if(parent) {
    element = doc->NewElement(element_name().c_str());
    parent->InsertEndChild(element);
  } else {
    element = doc->NewElement(element_name().c_str());
    doc->InsertFirstChild(element);
  }

  // Set attributes
  for(const auto& [name, value] : attributes_) {
    // Note: Default value filtering is handled in set_xml_attrib() methods
    // is_default_value() check is not used here to avoid inconsistencies
    element->SetAttribute(name.c_str(), stringify_value(value).c_str());
  }

  for(const auto& child : children_)
    if(child) child->write_xml_element(doc, element);
  return element;
}

} // namespace mjcf
