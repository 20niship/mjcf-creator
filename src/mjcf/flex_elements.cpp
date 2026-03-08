#include "flex_elements.hpp"

namespace mjcf {

FlexContact::FlexContact() = default;

void FlexContact::set_xml_attrib() const {
  if(contype != 1) this->set_attribute("contype", contype);
  if(conaffinity != 1) this->set_attribute("conaffinity", conaffinity);
  if(condim != 3) this->set_attribute("condim", condim);
  if(priority != 0) this->set_attribute("priority", priority);
  if(friction != Arr3{1.0, 0.005, 0.0001}) {
    this->set_attribute("friction", std::vector<double>(friction.begin(), friction.end()));
  }
  if(solmix != 1.0) this->set_attribute("solmix", solmix);
  if(solref != std::array<double, 2>{0.02, 1.0}) {
    this->set_attribute("solref", std::vector<double>(solref.begin(), solref.end()));
  }
  if(solimp != std::array<double, 5>{0.9, 0.95, 0.001, 0.5, 2.0}) {
    this->set_attribute("solimp", std::vector<double>(solimp.begin(), solimp.end()));
  }
  if(margin != 0.0) this->set_attribute("margin", margin);
  if(gap != 0.0) this->set_attribute("gap", gap);
  if(internal) this->set_attribute("internal", internal);
  if(selfcollide != FlexSelfCollide::Auto) {
    this->set_attribute("selfcollide", to_string(selfcollide));
  }
  if(activelayers != 1) this->set_attribute("activelayers", activelayers);
  if(vertcollide) this->set_attribute("vertcollide", vertcollide);
  if(passive) this->set_attribute("passive", passive);
}

bool FlexContact::is_default_value(const std::string& name, const AttributeValue& value) const { return false; }

FlexEdge::FlexEdge() = default;

void FlexEdge::set_xml_attrib() const {
  if(equality) this->set_attribute("equality", equality);
  if(equality) {
    if(solref != std::array<double, 2>{0.02, 1.0}) {
      this->set_attribute("solref", std::vector<double>(solref.begin(), solref.end()));
    }
    if(solimp != std::array<double, 5>{0.9, 0.95, 0.001, 0.5, 2.0}) {
      this->set_attribute("solimp", std::vector<double>(solimp.begin(), solimp.end()));
    }
  }
  if(stiffness != 0.0) this->set_attribute("stiffness", stiffness);
  if(damping != 0.0) this->set_attribute("damping", damping);
}

bool FlexEdge::is_default_value(const std::string& name, const AttributeValue& value) const { return false; }

FlexPin::FlexPin() = default;

void FlexPin::set_xml_attrib() const {
  if(!id.empty()) {
    std::vector<int> id_vec = id;
    this->set_attribute("id", id_vec);
  }
  if(range != std::array<double, 2>{0.0, 0.0}) {
    this->set_attribute("range", std::vector<double>(range.begin(), range.end()));
  }
  if(grid != std::array<int, 2>{0, 0}) {
    this->set_attribute("grid", std::vector<int>(grid.begin(), grid.end()));
  }
  if(gridrange != std::array<double, 2>{0.0, 0.0}) {
    this->set_attribute("gridrange", std::vector<double>(gridrange.begin(), gridrange.end()));
  }
}

bool FlexPin::is_default_value(const std::string& name, const AttributeValue& value) const { return false; }

FlexComp::FlexComp() = default;

void FlexComp::set_xml_attrib() const {
  if(!name.empty()) this->set_attribute("name", name);
  this->set_attribute("type", to_string(type));
  if(!count.empty()) this->set_attribute("count", count);
  if(!spacing.empty()) this->set_attribute("spacing", spacing);
  if(scale != Arr3{1.0, 1.0, 1.0}) this->set_attribute("scale", std::vector<double>(scale.begin(), scale.end()));
  if(radius != 0.005) this->set_attribute("radius", radius);
  if(dim != 2) this->set_attribute("dim", dim);
  if(dof != FlexDof::Full) this->set_attribute("dof", to_string(dof));
  if(rigid) this->set_attribute("rigid", rigid);
  if(mass != 0.0) this->set_attribute("mass", mass);
  if(inertiabox != 0.0) this->set_attribute("inertiabox", inertiabox);
  if(pos != Arr3{0.0, 0.0, 0.0}) this->set_attribute("pos", std::vector<double>(pos.begin(), pos.end()));
  if(quat != std::array<double, 4>{1.0, 0.0, 0.0, 0.0}) this->set_attribute("quat", std::vector<double>(quat.begin(), quat.end()));
  if(euler != Arr3{0.0, 0.0, 0.0}) this->set_attribute("euler", std::vector<double>(euler.begin(), euler.end()));
  if(axisangle != std::array<double, 4>{0.0, 0.0, 0.0, 0.0}) this->set_attribute("axisangle", std::vector<double>(axisangle.begin(), axisangle.end()));
  if(xyaxes != std::array<double, 6>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}) this->set_attribute("xyaxes", std::vector<double>(xyaxes.begin(), xyaxes.end()));
  if(zaxis != Arr3{0.0, 0.0, 0.0}) this->set_attribute("zaxis", std::vector<double>(zaxis.begin(), zaxis.end()));
  if(origin != Arr3{0.0, 0.0, 0.0}) this->set_attribute("origin", std::vector<double>(origin.begin(), origin.end()));
  if(!file.empty()) this->set_attribute("file", file);
  if(!material.empty()) this->set_attribute("material", material);
  if(rgba != std::array<double, 4>{0.5, 0.5, 0.5, 1.0}) this->set_attribute("rgba", std::vector<double>(rgba.begin(), rgba.end()));
  if(flatskin) this->set_attribute("flatskin", flatskin);
  if(group != 0) this->set_attribute("group", group);
}

bool FlexComp::is_default_value(const std::string& name, const AttributeValue& value) const { return false; }

std::shared_ptr<FlexComp> FlexComp::Grid(const std::string& name, std::vector<int> count, std::vector<double> spacing, Arr3 pos) {
  auto fc     = std::make_shared<FlexComp>();
  fc->name    = name;
  fc->type    = FlexCompType::Grid;
  fc->count   = count;
  fc->spacing = spacing;
  fc->pos     = pos;
  return fc;
}

std::shared_ptr<FlexComp> FlexComp::Mesh(const std::string& name, const std::string& file, Arr3 pos) {
  auto fc  = std::make_shared<FlexComp>();
  fc->name = name;
  fc->type = FlexCompType::Mesh;
  fc->file = file;
  fc->pos  = pos;
  return fc;
}

std::shared_ptr<FlexContact> FlexComp::add_contact(std::shared_ptr<FlexContact> contact) {
  if(!contact) {
    contact = std::make_shared<FlexContact>();
  }
  add_child(contact);
  return contact;
}

std::shared_ptr<FlexEdge> FlexComp::add_edge(std::shared_ptr<FlexEdge> edge) {
  if(!edge) {
    edge = std::make_shared<FlexEdge>();
  }
  add_child(edge);
  return edge;
}

std::shared_ptr<FlexPin> FlexComp::add_pin(std::shared_ptr<FlexPin> pin) {
  if(!pin) {
    pin = std::make_shared<FlexPin>();
  }
  add_child(pin);
  return pin;
}

} // namespace mjcf
