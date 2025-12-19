#include "doctest.h"
#include "mjcf/mjcf.hpp"

using namespace mjcf;

TEST_SUITE("name-utilities-tests") {
  TEST_CASE("get_body_names") {
    auto mujoco = std::make_shared<Mujoco>("test_model");
    auto worldbody = mujoco->worldbody_;

    // Initially empty
    auto names = mujoco->get_body_names();
    CHECK(names.empty());

    // Add bodies
    auto body1 = std::make_shared<Body>();
    body1->name = "body1";
    worldbody->add_child(body1);

    auto body2 = std::make_shared<Body>();
    body2->name = "body2";
    worldbody->add_child(body2);

    // Check names
    names = mujoco->get_body_names();
    CHECK(names.size() == 2);
    CHECK(names.count("body1") == 1);
    CHECK(names.count("body2") == 1);

    // Add nested body
    auto body3 = std::make_shared<Body>();
    body3->name = "body3";
    body1->add_child(body3);

    names = mujoco->get_body_names();
    CHECK(names.size() == 3);
    CHECK(names.count("body3") == 1);
  }

  TEST_CASE("get_geom_names") {
    auto mujoco = std::make_shared<Mujoco>("test_model");
    auto worldbody = mujoco->worldbody_;

    // Initially empty
    auto names = mujoco->get_geom_names();
    CHECK(names.empty());

    // Add geoms
    auto geom1 = Geom::Box("geom1", {1.0, 1.0, 1.0});
    worldbody->add_child(geom1);

    auto geom2 = Geom::Sphere("geom2", 0.5);
    worldbody->add_child(geom2);

    // Check names
    names = mujoco->get_geom_names();
    CHECK(names.size() == 2);
    CHECK(names.count("geom1") == 1);
    CHECK(names.count("geom2") == 1);

    // Add geom nested in a body
    auto body = std::make_shared<Body>();
    body->name = "test_body";
    auto geom3 = Geom::Capsule("geom3", {0.1, 0.5, 0.0});
    body->add_child(geom3);
    worldbody->add_child(body);

    names = mujoco->get_geom_names();
    CHECK(names.size() == 3);
    CHECK(names.count("geom3") == 1);
  }

  TEST_CASE("get_joint_names") {
    auto mujoco = std::make_shared<Mujoco>("test_model");
    auto worldbody = mujoco->worldbody_;

    // Initially empty
    auto names = mujoco->get_joint_names();
    CHECK(names.empty());

    // Add body with joints
    auto body = std::make_shared<Body>();
    body->name = "test_body";

    auto joint1 = Joint::Hinge("joint1", {0.0, 0.0, 1.0});
    body->add_child(joint1);

    auto joint2 = Joint::Slide("joint2", {1.0, 0.0, 0.0});
    body->add_child(joint2);

    worldbody->add_child(body);

    // Check names
    names = mujoco->get_joint_names();
    CHECK(names.size() == 2);
    CHECK(names.count("joint1") == 1);
    CHECK(names.count("joint2") == 1);
  }

  TEST_CASE("get_site_names") {
    auto mujoco = std::make_shared<Mujoco>("test_model");
    auto worldbody = mujoco->worldbody_;

    // Initially empty
    auto names = mujoco->get_site_names();
    CHECK(names.empty());

    // Add body with sites
    auto body = std::make_shared<Body>();
    body->name = "test_body";

    auto site1 = std::make_shared<Site>();
    site1->name = "site1";
    body->add_child(site1);

    auto site2 = std::make_shared<Site>();
    site2->name = "site2";
    body->add_child(site2);

    worldbody->add_child(body);

    // Check names
    names = mujoco->get_site_names();
    CHECK(names.size() == 2);
    CHECK(names.count("site1") == 1);
    CHECK(names.count("site2") == 1);
  }

  TEST_CASE("get_camera_names") {
    auto mujoco = std::make_shared<Mujoco>("test_model");
    auto worldbody = mujoco->worldbody_;

    // Initially empty
    auto names = mujoco->get_camera_names();
    CHECK(names.empty());

    // Add cameras
    auto camera1 = std::make_shared<Camera>();
    camera1->name = "camera1";
    worldbody->add_child(camera1);

    auto camera2 = std::make_shared<Camera>();
    camera2->name = "camera2";
    worldbody->add_child(camera2);

    // Check names
    names = mujoco->get_camera_names();
    CHECK(names.size() == 2);
    CHECK(names.count("camera1") == 1);
    CHECK(names.count("camera2") == 1);
  }

  TEST_CASE("get_light_names") {
    auto mujoco = std::make_shared<Mujoco>("test_model");
    auto worldbody = mujoco->worldbody_;

    // Initially empty
    auto names = mujoco->get_light_names();
    CHECK(names.empty());

    // Add lights
    auto light1 = std::make_shared<Light>();
    light1->name = "light1";
    worldbody->add_child(light1);

    auto light2 = std::make_shared<Light>();
    light2->name = "light2";
    worldbody->add_child(light2);

    // Check names
    names = mujoco->get_light_names();
    CHECK(names.size() == 2);
    CHECK(names.count("light1") == 1);
    CHECK(names.count("light2") == 1);
  }

  TEST_CASE("get_asset_names") {
    auto mujoco = std::make_shared<Mujoco>("test_model");
    auto asset = mujoco->asset_;

    // Initially empty
    auto names = mujoco->get_asset_names();
    CHECK(names.empty());

    // Add textures
    auto texture1 = std::make_shared<Texture>();
    texture1->name = "texture1";
    asset->add_child(texture1);

    auto texture2 = std::make_shared<Texture>();
    texture2->name = "texture2";
    asset->add_child(texture2);

    // Add material
    auto material = std::make_shared<Material>();
    material->name = "material1";
    asset->add_child(material);

    // Add mesh
    auto mesh = Mesh::Create("mesh1", "test.obj");
    asset->add_child(mesh);

    // Check names
    names = mujoco->get_asset_names();
    CHECK(names.size() == 4);
    CHECK(names.count("texture1") == 1);
    CHECK(names.count("texture2") == 1);
    CHECK(names.count("material1") == 1);
    CHECK(names.count("mesh1") == 1);
  }

  TEST_CASE("get_sensor_names") {
    auto mujoco = std::make_shared<Mujoco>("test_model");
    auto sensor = mujoco->sensor_;

    // Initially empty
    auto names = mujoco->get_sensor_names();
    CHECK(names.empty());

    // Add sensors
    auto sensor1 = std::make_shared<JointPos>();
    sensor1->name = "sensor1";
    sensor->add_child(sensor1);

    auto sensor2 = std::make_shared<JointVel>();
    sensor2->name = "sensor2";
    sensor->add_child(sensor2);

    // Check names
    names = mujoco->get_sensor_names();
    CHECK(names.size() == 2);
    CHECK(names.count("sensor1") == 1);
    CHECK(names.count("sensor2") == 1);
  }

  TEST_CASE("get_actuator_names") {
    auto mujoco = std::make_shared<Mujoco>("test_model");
    auto actuator = mujoco->actuator_;

    // Initially empty
    auto names = mujoco->get_actuator_names();
    CHECK(names.empty());

    // Add actuators
    auto motor1 = std::make_shared<Motor>();
    motor1->name = "motor1";
    actuator->add_child(motor1);

    auto motor2 = std::make_shared<Motor>();
    motor2->name = "motor2";
    actuator->add_child(motor2);

    // Check names
    names = mujoco->get_actuator_names();
    CHECK(names.size() == 2);
    CHECK(names.count("motor1") == 1);
    CHECK(names.count("motor2") == 1);
  }

  TEST_CASE("get_all_names") {
    auto mujoco = std::make_shared<Mujoco>("test_model");

    // Initially empty
    auto names = mujoco->get_all_names();
    CHECK(names.empty());

    // Add various elements
    auto worldbody = mujoco->worldbody_;
    auto asset = mujoco->asset_;
    auto sensor = mujoco->sensor_;
    auto actuator = mujoco->actuator_;

    // Body
    auto body = std::make_shared<Body>();
    body->name = "body1";
    worldbody->add_child(body);

    // Geom
    auto geom = Geom::Box("geom1", {1.0, 1.0, 1.0});
    body->add_child(geom);

    // Joint
    auto joint = Joint::Hinge("joint1", {0.0, 0.0, 1.0});
    body->add_child(joint);

    // Site
    auto site = std::make_shared<Site>();
    site->name = "site1";
    body->add_child(site);

    // Camera
    auto camera = std::make_shared<Camera>();
    camera->name = "camera1";
    worldbody->add_child(camera);

    // Light
    auto light = std::make_shared<Light>();
    light->name = "light1";
    worldbody->add_child(light);

    // Texture
    auto texture = std::make_shared<Texture>();
    texture->name = "texture1";
    asset->add_child(texture);

    // Material
    auto material = std::make_shared<Material>();
    material->name = "material1";
    asset->add_child(material);

    // Sensor
    auto sensor1 = std::make_shared<JointPos>();
    sensor1->name = "sensor1";
    sensor->add_child(sensor1);

    // Actuator
    auto motor = std::make_shared<Motor>();
    motor->name = "motor1";
    actuator->add_child(motor);

    // Check all names
    names = mujoco->get_all_names();
    CHECK(names.size() == 10);
    CHECK(names.count("body1") == 1);
    CHECK(names.count("geom1") == 1);
    CHECK(names.count("joint1") == 1);
    CHECK(names.count("site1") == 1);
    CHECK(names.count("camera1") == 1);
    CHECK(names.count("light1") == 1);
    CHECK(names.count("texture1") == 1);
    CHECK(names.count("material1") == 1);
    CHECK(names.count("sensor1") == 1);
    CHECK(names.count("motor1") == 1);
  }

  TEST_CASE("names_without_name_attribute_ignored") {
    auto mujoco = std::make_shared<Mujoco>("test_model");
    auto worldbody = mujoco->worldbody_;

    // Add body without name
    auto body1 = std::make_shared<Body>();
    // name is not set
    worldbody->add_child(body1);

    // Add body with empty name
    auto body2 = std::make_shared<Body>();
    body2->name = "";
    worldbody->add_child(body2);

    // Add body with actual name
    auto body3 = std::make_shared<Body>();
    body3->name = "body3";
    worldbody->add_child(body3);

    // Check that only named body is returned
    auto names = mujoco->get_body_names();
    CHECK(names.size() == 1);
    CHECK(names.count("body3") == 1);
  }

  TEST_CASE("duplicate_names_handled") {
    auto mujoco = std::make_shared<Mujoco>("test_model");
    auto worldbody = mujoco->worldbody_;

    // Add two bodies with same name (which is not allowed in MuJoCo but possible in the library)
    auto body1 = std::make_shared<Body>();
    body1->name = "duplicate";
    worldbody->add_child(body1);

    auto body2 = std::make_shared<Body>();
    body2->name = "duplicate";
    worldbody->add_child(body2);

    // Check that set contains only one entry
    auto names = mujoco->get_body_names();
    CHECK(names.size() == 1);
    CHECK(names.count("duplicate") == 1);
  }

  TEST_CASE("complex_hierarchy") {
    auto mujoco = std::make_shared<Mujoco>("test_model");
    auto worldbody = mujoco->worldbody_;

    // Create complex hierarchy
    auto root_body = std::make_shared<Body>();
    root_body->name = "root";
    worldbody->add_child(root_body);

    auto child_body1 = std::make_shared<Body>();
    child_body1->name = "child1";
    root_body->add_child(child_body1);

    auto child_body2 = std::make_shared<Body>();
    child_body2->name = "child2";
    root_body->add_child(child_body2);

    auto grandchild = std::make_shared<Body>();
    grandchild->name = "grandchild";
    child_body1->add_child(grandchild);

    // Add geoms at various levels
    auto geom1 = Geom::Box("geom_root", {1.0, 1.0, 1.0});
    root_body->add_child(geom1);

    auto geom2 = Geom::Sphere("geom_child", 0.5);
    child_body1->add_child(geom2);

    auto geom3 = Geom::Capsule("geom_grandchild", {0.1, 0.5, 0.0});
    grandchild->add_child(geom3);

    // Check body names
    auto body_names = mujoco->get_body_names();
    CHECK(body_names.size() == 4);
    CHECK(body_names.count("root") == 1);
    CHECK(body_names.count("child1") == 1);
    CHECK(body_names.count("child2") == 1);
    CHECK(body_names.count("grandchild") == 1);

    // Check geom names
    auto geom_names = mujoco->get_geom_names();
    CHECK(geom_names.size() == 3);
    CHECK(geom_names.count("geom_root") == 1);
    CHECK(geom_names.count("geom_child") == 1);
    CHECK(geom_names.count("geom_grandchild") == 1);
  }
}
