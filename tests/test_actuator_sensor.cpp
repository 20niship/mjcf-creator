#include "doctest.h"
#include "mjcf/mjcf.hpp"

TEST_CASE("motor") {
  mjcf::Motor motor;
  motor.name      = "motor1";
  motor.joint     = "joint1";
  motor.ctrlrange = {-10.0, 10.0};
  motor.gear      = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  std::string xml = motor.get_xml_text();
  CHECK(xml.find("name=\"motor1\"") != std::string::npos);
  CHECK(xml.find("joint=\"joint1\"") != std::string::npos);
  CHECK(xml.find("ctrlrange=\"-10 10\"") != std::string::npos);
  CHECK(xml.find("gear=\"1 0 0 0 0 0\"") != std::string::npos);
}

TEST_CASE("position-actuator") {
  mjcf::Position position;
  position.name  = "pos_actuator";
  position.joint = "hinge_joint";
  position.kp    = 100.0;
  position.kv    = 10.0;

  std::string xml = position.get_xml_text();
  CHECK(xml.find("name=\"pos_actuator\"") != std::string::npos);
  CHECK(xml.find("joint=\"hinge_joint\"") != std::string::npos);
  CHECK(xml.find("kp=\"100\"") != std::string::npos);
  CHECK(xml.find("kv=\"10\"") != std::string::npos);
}

TEST_CASE("velocity-actuator") {
  mjcf::Velocity velocity;
  velocity.name  = "vel_actuator";
  velocity.joint = "ball_joint";
  velocity.kv    = 5.0;

  std::string xml = velocity.get_xml_text();
  CHECK(xml.find("name=\"vel_actuator\"") != std::string::npos);
  CHECK(xml.find("joint=\"ball_joint\"") != std::string::npos);
  CHECK(xml.find("kv=\"5\"") != std::string::npos);
}

TEST_CASE("muscle-actuator") {
  mjcf::Muscle muscle;
  muscle.name      = "bicep";
  muscle.tendon    = "bicep_tendon";
  muscle.force     = 100.0;
  muscle.range     = {0.8, 1.2};
  muscle.timeconst = {0.01, 0.04};

  std::string xml = muscle.get_xml_text();
  CHECK(xml.find("name=\"bicep\"") != std::string::npos);
  CHECK(xml.find("tendon=\"bicep_tendon\"") != std::string::npos);
  CHECK(xml.find("force=\"100\"") != std::string::npos);
  CHECK(xml.find("range=\"0.8 1.2\"") != std::string::npos);
  CHECK(xml.find("timeconst=\"0.01 0.04\"") != std::string::npos);
}

TEST_CASE("joint-position-sensor") {
  mjcf::JointPos joint_pos;
  joint_pos.name  = "joint_position_sensor";
  joint_pos.joint = "knee_joint";
  joint_pos.noise = 0.01;

  std::string xml = joint_pos.get_xml_text();
  CHECK(xml.find("name=\"joint_position_sensor\"") != std::string::npos);
  CHECK(xml.find("joint=\"knee_joint\"") != std::string::npos);
  CHECK(xml.find("noise=\"0.01\"") != std::string::npos);
}

TEST_CASE("joint-velocity-sensor") {
  mjcf::JointVel joint_vel;
  joint_vel.name   = "joint_velocity_sensor";
  joint_vel.joint  = "ankle_joint";
  joint_vel.cutoff = 10.0;

  std::string xml = joint_vel.get_xml_text();
  CHECK(xml.find("name=\"joint_velocity_sensor\"") != std::string::npos);
  CHECK(xml.find("joint=\"ankle_joint\"") != std::string::npos);
  CHECK(xml.find("cutoff=\"10\"") != std::string::npos);
}

TEST_CASE("actuator-position-sensor") {
  mjcf::ActuatorPos actuator_pos;
  actuator_pos.name     = "actuator_position_sensor";
  actuator_pos.actuator = "motor1";

  std::string xml = actuator_pos.get_xml_text();
  CHECK(xml.find("name=\"actuator_position_sensor\"") != std::string::npos);
  CHECK(xml.find("actuator=\"motor1\"") != std::string::npos);
}

TEST_CASE("force-sensor") {
  mjcf::Force force;
  force.name = "force_sensor";
  force.site = "contact_point";

  std::string xml = force.get_xml_text();
  CHECK(xml.find("name=\"force_sensor\"") != std::string::npos);
  CHECK(xml.find("site=\"contact_point\"") != std::string::npos);
}

TEST_CASE("gyroscope-sensor") {
  mjcf::Gyro gyro;
  gyro.name  = "imu_gyro";
  gyro.site  = "imu_site";
  gyro.noise = 0.001;

  std::string xml = gyro.get_xml_text();
  CHECK(xml.find("name=\"imu_gyro\"") != std::string::npos);
  CHECK(xml.find("site=\"imu_site\"") != std::string::npos);
  CHECK(xml.find("noise=\"0.001\"") != std::string::npos);
}

TEST_CASE("accelerometer-sensor") {
  mjcf::Accelerometer accel;
  accel.name = "imu_accel";
  accel.site = "imu_site";

  std::string xml = accel.get_xml_text();
  CHECK(xml.find("name=\"imu_accel\"") != std::string::npos);
  CHECK(xml.find("site=\"imu_site\"") != std::string::npos);
}

TEST_CASE("complete-model-with-actuators-and-sensors") {
  auto mujoco             = std::make_shared<mjcf::Mujoco>("robot_model");
  auto worldbody          = mujoco->worldbody_;
  auto actuator_container = mujoco->actuator_;
  auto sensor_container   = mujoco->sensor_;

  // Create a simple robot body with joint
  auto robot_body  = std::make_shared<mjcf::Body>();
  robot_body->name = "robot";

  auto geom  = std::make_shared<mjcf::Geom>();
  geom->name = "robot_geom";
  geom->type = mjcf::GeomType::Box;
  geom->size = std::array<double, 3>{0.1, 0.1, 0.1};

  auto joint  = std::make_shared<mjcf::Joint>();
  joint->name = "robot_joint";
  joint->type = mjcf::JointType::Hinge;

  robot_body->add_children({geom, joint});
  worldbody->add_child(robot_body);

  // Add motor actuator
  auto motor       = std::make_shared<mjcf::Motor>();
  motor->name      = "robot_motor";
  motor->joint     = "robot_joint";
  motor->ctrlrange = {-1.0, 1.0};
  actuator_container->add_child(motor);

  // Add joint position sensor
  auto joint_sensor   = std::make_shared<mjcf::JointPos>();
  joint_sensor->name  = "joint_position";
  joint_sensor->joint = "robot_joint";
  sensor_container->add_child(joint_sensor);

  std::string xml = mujoco->get_xml_text();

  CHECK(xml.find("<mujoco model=\"robot_model\">") != std::string::npos);
  CHECK(xml.find("<worldbody>") != std::string::npos);
  CHECK(xml.find("<body name=\"robot\">") != std::string::npos);
  CHECK(xml.find("<geom name=\"robot_geom\" size=\"0.1 0.1 0.1\" type=\"box\"/>") != std::string::npos);
  CHECK(xml.find("<joint name=\"robot_joint\" type=\"hinge\"/>") != std::string::npos);
  CHECK(xml.find("<actuator>") != std::string::npos);
  CHECK(xml.find("<sensor>") != std::string::npos);
  CHECK(xml.find("<jointpos joint=\"robot_joint\" name=\"joint_position\"/>") != std::string::npos);
}

TEST_CASE("enum-type-usage") {
  auto geom  = std::make_shared<mjcf::Geom>();
  geom->type = mjcf::GeomType::Sphere;

  auto joint  = std::make_shared<mjcf::Joint>();
  joint->type = mjcf::JointType::Ball;

  auto site  = std::make_shared<mjcf::Site>();
  site->type = mjcf::SiteType::Capsule;

  std::string geom_xml  = geom->get_xml_text();
  std::string joint_xml = joint->get_xml_text();
  std::string site_xml  = site->get_xml_text();
  CHECK(geom_xml.find("type=\"sphere\"") != std::string::npos);
  CHECK(joint_xml.find("type=\"ball\"") != std::string::npos);
  CHECK(site_xml.find("type=\"capsule\"") != std::string::npos);
}
