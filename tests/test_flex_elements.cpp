#include "doctest.h"
#include "mjcf/flex_elements.hpp"
#include "mjcf/core_elements.hpp"
#include <sstream>

using namespace mjcf;

TEST_SUITE("FlexContact") {
  TEST_CASE("FlexContact default values - selfcollide=auto should not emit") {
    auto contact = std::make_shared<FlexContact>();
    contact->set_xml_attrib();

    // Default selfcollide is Auto, should not be emitted
    auto selfcollide_attr = contact->get_attribute_public("selfcollide");
    CHECK(!selfcollide_attr.has_value());
  }

  TEST_CASE("FlexContact with selfcollide=none should emit") {
    auto contact = std::make_shared<FlexContact>();
    contact->selfcollide = FlexSelfCollide::None;
    contact->set_xml_attrib();

    auto selfcollide_attr = contact->get_attribute_public("selfcollide");
    CHECK(selfcollide_attr.has_value());
    CHECK(std::get<std::string>(selfcollide_attr.value()) == "none");
  }

  TEST_CASE("FlexContact with non-default solref") {
    auto contact = std::make_shared<FlexContact>();
    contact->solref = {0.003, 1.0};
    contact->set_xml_attrib();

    auto solref_attr = contact->get_attribute_public("solref");
    CHECK(solref_attr.has_value());
    auto vec = std::get<std::vector<double>>(solref_attr.value());
    CHECK(vec.size() == 2);
    CHECK(vec[0] == 0.003);
    CHECK(vec[1] == 1.0);
  }

  TEST_CASE("FlexContact with non-default condim") {
    auto contact = std::make_shared<FlexContact>();
    contact->condim = 4;
    contact->set_xml_attrib();

    auto condim_attr = contact->get_attribute_public("condim");
    CHECK(condim_attr.has_value());
    CHECK(std::get<int>(condim_attr.value()) == 4);
  }

  TEST_CASE("FlexContact with internal=true") {
    auto contact = std::make_shared<FlexContact>();
    contact->internal = true;
    contact->set_xml_attrib();

    auto internal_attr = contact->get_attribute_public("internal");
    CHECK(internal_attr.has_value());
    CHECK(std::get<bool>(internal_attr.value()) == true);
  }
}

TEST_SUITE("FlexEdge") {
  TEST_CASE("FlexEdge default values - equality should not emit") {
    auto edge = std::make_shared<FlexEdge>();
    edge->set_xml_attrib();

    auto equality_attr = edge->get_attribute_public("equality");
    CHECK(!equality_attr.has_value());
  }

  TEST_CASE("FlexEdge with equality=true should emit") {
    auto edge = std::make_shared<FlexEdge>();
    edge->equality = true;
    edge->set_xml_attrib();

    auto equality_attr = edge->get_attribute_public("equality");
    CHECK(equality_attr.has_value());
    CHECK(std::get<bool>(equality_attr.value()) == true);
  }

  TEST_CASE("FlexEdge with equality=true and damping") {
    auto edge = std::make_shared<FlexEdge>();
    edge->equality = true;
    edge->damping = 0.1;
    edge->set_xml_attrib();

    auto damping_attr = edge->get_attribute_public("damping");
    CHECK(damping_attr.has_value());
    CHECK(std::get<double>(damping_attr.value()) == 0.1);
  }

  TEST_CASE("FlexEdge with equality=true and custom solref") {
    auto edge = std::make_shared<FlexEdge>();
    edge->equality = true;
    edge->solref = {0.01, 0.9};
    edge->set_xml_attrib();

    auto solref_attr = edge->get_attribute_public("solref");
    CHECK(solref_attr.has_value());
    auto vec = std::get<std::vector<double>>(solref_attr.value());
    CHECK(vec.size() == 2);
    CHECK(vec[0] == 0.01);
    CHECK(vec[1] == 0.9);
  }

  TEST_CASE("FlexEdge with equality=false solref should not emit") {
    auto edge = std::make_shared<FlexEdge>();
    edge->equality = false;
    edge->solref = {0.01, 0.9};  // Custom, but equality is false
    edge->set_xml_attrib();

    auto solref_attr = edge->get_attribute_public("solref");
    CHECK(!solref_attr.has_value());
  }
}

TEST_SUITE("FlexPin") {
  TEST_CASE("FlexPin with id list") {
    auto pin = std::make_shared<FlexPin>();
    pin->id = {0, 5, 10, 15};
    pin->set_xml_attrib();

    auto id_attr = pin->get_attribute_public("id");
    CHECK(id_attr.has_value());
    auto id_vec = std::get<std::vector<int>>(id_attr.value());
    CHECK(id_vec.size() == 4);
    CHECK(id_vec[0] == 0);
    CHECK(id_vec[3] == 15);
  }

  TEST_CASE("FlexPin with range") {
    auto pin = std::make_shared<FlexPin>();
    pin->range = {0.1, 0.5};
    pin->set_xml_attrib();

    auto range_attr = pin->get_attribute_public("range");
    CHECK(range_attr.has_value());
    auto range_vec = std::get<std::vector<double>>(range_attr.value());
    CHECK(range_vec.size() == 2);
    CHECK(range_vec[0] == 0.1);
    CHECK(range_vec[1] == 0.5);
  }

  TEST_CASE("FlexPin empty should not emit") {
    auto pin = std::make_shared<FlexPin>();
    pin->set_xml_attrib();

    auto id_attr = pin->get_attribute_public("id");
    auto range_attr = pin->get_attribute_public("range");
    auto grid_attr = pin->get_attribute_public("grid");
    CHECK(!id_attr.has_value());
    CHECK(!range_attr.has_value());
    CHECK(!grid_attr.has_value());
  }

  TEST_CASE("FlexPin with grid") {
    auto pin = std::make_shared<FlexPin>();
    pin->grid = {2, 3};
    pin->set_xml_attrib();

    auto grid_attr = pin->get_attribute_public("grid");
    CHECK(grid_attr.has_value());
    auto grid_vec = std::get<std::vector<int>>(grid_attr.value());
    CHECK(grid_vec.size() == 2);
    CHECK(grid_vec[0] == 2);
    CHECK(grid_vec[1] == 3);
  }
}

TEST_SUITE("FlexComp") {
  TEST_CASE("FlexComp::Grid factory") {
    auto cloth = FlexComp::Grid("test_grid", {10, 10}, {0.05, 0.05}, {0.0, 0.0, 1.0});

    CHECK(cloth->name == "test_grid");
    CHECK(cloth->type == FlexCompType::Grid);
    CHECK(cloth->count.size() == 2);
    CHECK(cloth->count[0] == 10);
    CHECK(cloth->spacing.size() == 2);
    CHECK(cloth->spacing[0] == 0.05);
    CHECK(cloth->pos[2] == 1.0);
  }

  TEST_CASE("FlexComp::Mesh factory") {
    auto cloth = FlexComp::Mesh("test_mesh", "cloth.obj", {0.0, 0.0, 1.5});

    CHECK(cloth->name == "test_mesh");
    CHECK(cloth->type == FlexCompType::Mesh);
    CHECK(cloth->file == "cloth.obj");
    CHECK(cloth->pos[2] == 1.5);
  }

  TEST_CASE("FlexComp Grid - count and spacing emit") {
    auto cloth = FlexComp::Grid("grid_cloth", {8, 8}, {0.1, 0.1});
    cloth->set_xml_attrib();

    auto count_attr = cloth->get_attribute_public("count");
    auto spacing_attr = cloth->get_attribute_public("spacing");
    CHECK(count_attr.has_value());
    CHECK(spacing_attr.has_value());
  }

  TEST_CASE("FlexComp Mesh - radius emits if not default") {
    auto cloth = FlexComp::Mesh("mesh_cloth", "cloth.obj");
    cloth->radius = 0.01;
    cloth->set_xml_attrib();

    auto radius_attr = cloth->get_attribute_public("radius");
    CHECK(radius_attr.has_value());
    CHECK(std::get<double>(radius_attr.value()) == 0.01);
  }

  TEST_CASE("FlexComp Mesh - dim=2 does not emit") {
    auto cloth = FlexComp::Mesh("mesh_cloth", "cloth.obj");
    cloth->set_xml_attrib();

    auto dim_attr = cloth->get_attribute_public("dim");
    CHECK(!dim_attr.has_value());  // Default 2 should not emit
  }

  TEST_CASE("FlexComp - dim=3 emits") {
    auto cloth = std::make_shared<FlexComp>();
    cloth->name = "test_cloth";
    cloth->type = FlexCompType::Mesh;
    cloth->file = "cloth.obj";
    cloth->dim = 3;
    cloth->set_xml_attrib();

    auto dim_attr = cloth->get_attribute_public("dim");
    CHECK(dim_attr.has_value());
    CHECK(std::get<int>(dim_attr.value()) == 3);
  }

  TEST_CASE("FlexComp - dof=Trilinear emits") {
    auto cloth = std::make_shared<FlexComp>();
    cloth->name = "test_cloth";
    cloth->type = FlexCompType::Grid;
    cloth->count = {10, 10};
    cloth->spacing = {0.05, 0.05};
    cloth->dof = FlexDof::Trilinear;
    cloth->set_xml_attrib();

    auto dof_attr = cloth->get_attribute_public("dof");
    CHECK(dof_attr.has_value());
    CHECK(std::get<std::string>(dof_attr.value()) == "trilinear");
  }

  TEST_CASE("FlexComp - scale emits if not {1,1,1}") {
    auto cloth = std::make_shared<FlexComp>();
    cloth->name = "test_cloth";
    cloth->type = FlexCompType::Grid;
    cloth->count = {10, 10};
    cloth->spacing = {0.05, 0.05};
    cloth->scale = {2.0, 2.0, 1.0};
    cloth->set_xml_attrib();

    auto scale_attr = cloth->get_attribute_public("scale");
    CHECK(scale_attr.has_value());
  }

  TEST_CASE("FlexComp - rgba custom emits") {
    auto cloth = std::make_shared<FlexComp>();
    cloth->name = "test_cloth";
    cloth->type = FlexCompType::Mesh;
    cloth->file = "cloth.obj";
    cloth->rgba = {1.0, 0.0, 0.0, 1.0};  // Red
    cloth->set_xml_attrib();

    auto rgba_attr = cloth->get_attribute_public("rgba");
    CHECK(rgba_attr.has_value());
    auto rgba_vec = std::get<std::vector<double>>(rgba_attr.value());
    CHECK(rgba_vec[0] == 1.0);
    CHECK(rgba_vec[1] == 0.0);
  }

  TEST_CASE("FlexComp - rigid=true emits") {
    auto cloth = std::make_shared<FlexComp>();
    cloth->name = "test_cloth";
    cloth->type = FlexCompType::Grid;
    cloth->count = {5, 5};
    cloth->spacing = {0.1, 0.1};
    cloth->rigid = true;
    cloth->set_xml_attrib();

    auto rigid_attr = cloth->get_attribute_public("rigid");
    CHECK(rigid_attr.has_value());
    CHECK(std::get<bool>(rigid_attr.value()) == true);
  }

  TEST_CASE("FlexComp - position and orientation") {
    auto cloth = std::make_shared<FlexComp>();
    cloth->name = "positioned_cloth";
    cloth->type = FlexCompType::Mesh;
    cloth->file = "cloth.obj";
    cloth->pos = {1.0, 2.0, 3.0};
    cloth->quat = {0.7071, 0.7071, 0.0, 0.0};
    cloth->set_xml_attrib();

    auto pos_attr = cloth->get_attribute_public("pos");
    auto quat_attr = cloth->get_attribute_public("quat");
    CHECK(pos_attr.has_value());
    CHECK(quat_attr.has_value());
  }

  TEST_CASE("FlexComp - material and file") {
    auto cloth = std::make_shared<FlexComp>();
    cloth->name = "mat_cloth";
    cloth->type = FlexCompType::Mesh;
    cloth->file = "cloth_rectangle.obj";
    cloth->material = "cloth_material";
    cloth->set_xml_attrib();

    auto file_attr = cloth->get_attribute_public("file");
    auto mat_attr = cloth->get_attribute_public("material");
    CHECK(file_attr.has_value());
    CHECK(mat_attr.has_value());
    CHECK(std::get<std::string>(file_attr.value()) == "cloth_rectangle.obj");
    CHECK(std::get<std::string>(mat_attr.value()) == "cloth_material");
  }
}

TEST_SUITE("FlexComp Child Elements") {
  TEST_CASE("FlexComp add_contact creates child") {
    auto cloth = FlexComp::Mesh("cloth", "cloth.obj");
    auto contact = cloth->add_contact();

    CHECK(contact != nullptr);
    CHECK(cloth->get_children().size() == 1);
  }

  TEST_CASE("FlexComp add_edge creates child") {
    auto cloth = FlexComp::Grid("cloth", {10, 10}, {0.05, 0.05});
    auto edge = cloth->add_edge();

    CHECK(edge != nullptr);
    CHECK(cloth->get_children().size() == 1);
  }

  TEST_CASE("FlexComp add_pin creates child") {
    auto cloth = FlexComp::Mesh("cloth", "cloth.obj");
    auto pin = cloth->add_pin();

    CHECK(pin != nullptr);
    CHECK(cloth->get_children().size() == 1);
  }

  TEST_CASE("FlexComp add_contact with provided contact") {
    auto cloth = FlexComp::Mesh("cloth", "cloth.obj");
    auto contact = std::make_shared<FlexContact>();
    contact->selfcollide = FlexSelfCollide::Narrow;

    auto returned = cloth->add_contact(contact);

    CHECK(returned == contact);
    CHECK(cloth->get_children().size() == 1);
  }

  TEST_CASE("FlexComp multiple children") {
    auto cloth = FlexComp::Mesh("cloth", "cloth.obj");
    auto contact = cloth->add_contact();
    auto edge = cloth->add_edge();
    auto pin = cloth->add_pin();

    CHECK(cloth->get_children().size() == 3);
    CHECK(cloth->get_children()[0] == contact);
    CHECK(cloth->get_children()[1] == edge);
    CHECK(cloth->get_children()[2] == pin);
  }
}

TEST_SUITE("FlexComp Integration") {
  TEST_CASE("FlexComp in Mujoco worldbody") {
    auto mujoco = std::make_shared<Mujoco>("cloth_test");

    auto cloth = FlexComp::Mesh("cloth_rect", "cloth_rectangle.obj", {0.0, 0.0, 1.5});
    cloth->radius = 0.01;
    cloth->quat = {0.7071, 0.7071, 0.0, 0.0};

    auto contact = cloth->add_contact();
    contact->solref = {0.003, 1.0};
    contact->selfcollide = FlexSelfCollide::None;

    auto edge = cloth->add_edge();
    edge->equality = true;
    edge->damping = 0.1;

    mujoco->worldbody_->add_child(cloth);

    // Verify structure
    CHECK(mujoco->worldbody_->get_children().size() > 0);
    CHECK(cloth->get_children().size() == 2);
  }
}

TEST_SUITE("Enum Conversions") {
  TEST_CASE("FlexCompType to_string and from_string") {
    CHECK(to_string(FlexCompType::Grid) == "grid");
    CHECK(to_string(FlexCompType::Mesh) == "mesh");
    CHECK(to_string(FlexCompType::Gmsh) == "gmsh");

    CHECK(flex_comp_type_from_string("grid") == FlexCompType::Grid);
    CHECK(flex_comp_type_from_string("mesh") == FlexCompType::Mesh);
    CHECK(flex_comp_type_from_string("circle") == FlexCompType::Circle);
  }

  TEST_CASE("FlexSelfCollide to_string and from_string") {
    CHECK(to_string(FlexSelfCollide::None) == "none");
    CHECK(to_string(FlexSelfCollide::Narrow) == "narrow");
    CHECK(to_string(FlexSelfCollide::Auto) == "auto");

    CHECK(flex_self_collide_from_string("none") == FlexSelfCollide::None);
    CHECK(flex_self_collide_from_string("bvh") == FlexSelfCollide::Bvh);
    CHECK(flex_self_collide_from_string("auto") == FlexSelfCollide::Auto);
  }

  TEST_CASE("FlexDof to_string and from_string") {
    CHECK(to_string(FlexDof::Full) == "full");
    CHECK(to_string(FlexDof::Trilinear) == "trilinear");
    CHECK(to_string(FlexDof::Quadratic) == "quadratic");

    CHECK(flex_dof_from_string("full") == FlexDof::Full);
    CHECK(flex_dof_from_string("radial") == FlexDof::Radial);
    CHECK(flex_dof_from_string("trilinear") == FlexDof::Trilinear);
  }
}
