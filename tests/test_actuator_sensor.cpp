#include "doctest.h"
#include "mjcf/mjcf.hpp"

TEST_CASE("motor") {
  mjcf::Motor motor;
  motor.name        = "motor1";
  motor.joint       = "joint1";
  motor.ctrllimited = true;
  motor.ctrlrange   = {-10.0, 10.0};
  motor.gear        = {10.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  std::string xml = motor.get_xml_text();
  CHECK(xml.find("name=\"motor1\"") != std::string::npos);
  CHECK(xml.find("joint=\"joint1\"") != std::string::npos);
  CHECK(xml.find("ctrlrange=\"-10 10\"") != std::string::npos);
  CHECK(xml.find("gear=\"10 0 0 0 0 0\"") != std::string::npos);
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

// ===== フェーズ0: 既存改善テスト =====

TEST_CASE("frame-sensor-refname") {
  mjcf::SitePos site_pos;
  site_pos.name    = "test_framepos";
  site_pos.objname = "test_body";
  site_pos.refname = "ref_body";

  std::string xml = site_pos.get_xml_text();
  CHECK(xml.find("name=\"test_framepos\"") != std::string::npos);
  CHECK(xml.find("objname=\"test_body\"") != std::string::npos);
  CHECK(xml.find("refname=\"ref_body\"") != std::string::npos);
}

TEST_CASE("base-sensor-extended-defaults") {
  mjcf::JointPos joint_pos;
  joint_pos.name     = "test_sensor";
  joint_pos.joint    = "test_joint";
  joint_pos.nsample  = 5;
  joint_pos.interp   = "linear";
  joint_pos.delay    = 0.01;
  joint_pos.interval = 0.02;

  std::string xml = joint_pos.get_xml_text();
  CHECK(xml.find("nsample=\"5\"") != std::string::npos);
  CHECK(xml.find("interp=\"linear\"") != std::string::npos);
  CHECK(xml.find("delay=\"0.01\"") != std::string::npos);
  CHECK(xml.find("interval=\"0.02\"") != std::string::npos);
}

// ===== フェーズ1: 優先度高テスト =====

TEST_CASE("velocimeter-sensor") {
  mjcf::Velocimeter velocimeter;
  velocimeter.name  = "vel_sensor";
  velocimeter.site  = "sensor_site";
  velocimeter.noise = 0.005;

  std::string xml = velocimeter.get_xml_text();
  CHECK(xml.find("name=\"vel_sensor\"") != std::string::npos);
  CHECK(xml.find("site=\"sensor_site\"") != std::string::npos);
  CHECK(xml.find("noise=\"0.005\"") != std::string::npos);
}

TEST_CASE("clock-sensor") {
  mjcf::Clock clock;
  clock.name = "sim_clock";

  std::string xml = clock.get_xml_text();
  CHECK(xml.find("name=\"sim_clock\"") != std::string::npos);
  CHECK(xml.find("<clock") != std::string::npos);
}

TEST_CASE("framelinacc-sensor") {
  mjcf::FrameLinAcc frame_linacc;
  frame_linacc.name    = "linear_accel";
  frame_linacc.objtype = "site";
  frame_linacc.objname = "imu_site";
  frame_linacc.cutoff  = 20.0;

  std::string xml = frame_linacc.get_xml_text();
  CHECK(xml.find("name=\"linear_accel\"") != std::string::npos);
  CHECK(xml.find("objtype=\"site\"") != std::string::npos);
  CHECK(xml.find("objname=\"imu_site\"") != std::string::npos);
  CHECK(xml.find("cutoff=\"20\"") != std::string::npos);
}

TEST_CASE("frameangacc-sensor") {
  mjcf::FrameAngAcc frame_angacc;
  frame_angacc.name    = "angular_accel";
  frame_angacc.objtype = "body";
  frame_angacc.objname = "robot_body";

  std::string xml = frame_angacc.get_xml_text();
  CHECK(xml.find("name=\"angular_accel\"") != std::string::npos);
  CHECK(xml.find("objname=\"robot_body\"") != std::string::npos);
  // objtype="body" はデフォルトなので出力されない
  CHECK(xml.find("objtype=") == std::string::npos);
}

TEST_CASE("potential-energy-sensor") {
  mjcf::PotentialEnergy pe;
  pe.name = "potential_e";

  std::string xml = pe.get_xml_text();
  CHECK(xml.find("name=\"potential_e\"") != std::string::npos);
  CHECK(xml.find("<e_potential") != std::string::npos);
}

TEST_CASE("kinetic-energy-sensor") {
  mjcf::KineticEnergy ke;
  ke.name = "kinetic_e";

  std::string xml = ke.get_xml_text();
  CHECK(xml.find("name=\"kinetic_e\"") != std::string::npos);
  CHECK(xml.find("<e_kinetic") != std::string::npos);
}

TEST_CASE("jointactuatorfrc-sensor") {
  mjcf::JointActuatorFrc joint_frc;
  joint_frc.name  = "joint_actuator_force";
  joint_frc.joint = "knee_joint";
  joint_frc.noise = 0.1;

  std::string xml = joint_frc.get_xml_text();
  CHECK(xml.find("name=\"joint_actuator_force\"") != std::string::npos);
  CHECK(xml.find("joint=\"knee_joint\"") != std::string::npos);
  CHECK(xml.find("noise=\"0.1\"") != std::string::npos);
}

TEST_CASE("tendonactuatorfrc-sensor") {
  mjcf::TendonActuatorFrc tendon_frc;
  tendon_frc.name   = "tendon_actuator_force";
  tendon_frc.tendon = "bicep_tendon";

  std::string xml = tendon_frc.get_xml_text();
  CHECK(xml.find("name=\"tendon_actuator_force\"") != std::string::npos);
  CHECK(xml.find("tendon=\"bicep_tendon\"") != std::string::npos);
}

// ===== フェーズ2: 優先度中テスト =====

TEST_CASE("jointlimitpos-sensor") {
  mjcf::JointLimitPos joint_limit;
  joint_limit.name  = "joint_limit_position";
  joint_limit.joint = "hip_joint";

  std::string xml = joint_limit.get_xml_text();
  CHECK(xml.find("name=\"joint_limit_position\"") != std::string::npos);
  CHECK(xml.find("joint=\"hip_joint\"") != std::string::npos);
}

TEST_CASE("jointlimitvel-sensor") {
  mjcf::JointLimitVel joint_limit_vel;
  joint_limit_vel.name  = "joint_limit_velocity";
  joint_limit_vel.joint = "ankle_joint";

  std::string xml = joint_limit_vel.get_xml_text();
  CHECK(xml.find("name=\"joint_limit_velocity\"") != std::string::npos);
  CHECK(xml.find("joint=\"ankle_joint\"") != std::string::npos);
}

TEST_CASE("jointlimitfrc-sensor") {
  mjcf::JointLimitFrc joint_limit_frc;
  joint_limit_frc.name  = "joint_limit_force";
  joint_limit_frc.joint = "elbow_joint";

  std::string xml = joint_limit_frc.get_xml_text();
  CHECK(xml.find("name=\"joint_limit_force\"") != std::string::npos);
  CHECK(xml.find("joint=\"elbow_joint\"") != std::string::npos);
}

TEST_CASE("tendonlimitpos-sensor") {
  mjcf::TendonLimitPos tendon_limit;
  tendon_limit.name   = "tendon_limit_position";
  tendon_limit.tendon = "finger_tendon";

  std::string xml = tendon_limit.get_xml_text();
  CHECK(xml.find("name=\"tendon_limit_position\"") != std::string::npos);
  CHECK(xml.find("tendon=\"finger_tendon\"") != std::string::npos);
}

TEST_CASE("tendonlimitvel-sensor") {
  mjcf::TendonLimitVel tendon_limit_vel;
  tendon_limit_vel.name   = "tendon_limit_velocity";
  tendon_limit_vel.tendon = "leg_tendon";

  std::string xml = tendon_limit_vel.get_xml_text();
  CHECK(xml.find("name=\"tendon_limit_velocity\"") != std::string::npos);
  CHECK(xml.find("tendon=\"leg_tendon\"") != std::string::npos);
}

TEST_CASE("tendonlimitfrc-sensor") {
  mjcf::TendonLimitFrc tendon_limit_frc;
  tendon_limit_frc.name   = "tendon_limit_force";
  tendon_limit_frc.tendon = "arm_tendon";

  std::string xml = tendon_limit_frc.get_xml_text();
  CHECK(xml.find("name=\"tendon_limit_force\"") != std::string::npos);
  CHECK(xml.find("tendon=\"arm_tendon\"") != std::string::npos);
}

TEST_CASE("distance-sensor-geom-pair") {
  mjcf::Distance distance;
  distance.name  = "geom_distance";
  distance.geom1 = "geom_a";
  distance.geom2 = "geom_b";

  std::string xml = distance.get_xml_text();
  CHECK(xml.find("name=\"geom_distance\"") != std::string::npos);
  CHECK(xml.find("geom1=\"geom_a\"") != std::string::npos);
  CHECK(xml.find("geom2=\"geom_b\"") != std::string::npos);
  // body属性は設定されていないので出力されない
  CHECK(xml.find("body1=") == std::string::npos);
  CHECK(xml.find("body2=") == std::string::npos);
}

TEST_CASE("distance-sensor-body-pair") {
  mjcf::Distance distance;
  distance.name  = "body_distance";
  distance.body1 = "body_a";
  distance.body2 = "body_b";

  std::string xml = distance.get_xml_text();
  CHECK(xml.find("name=\"body_distance\"") != std::string::npos);
  CHECK(xml.find("body1=\"body_a\"") != std::string::npos);
  CHECK(xml.find("body2=\"body_b\"") != std::string::npos);
  // geom属性は設定されていないので出力されない
  CHECK(xml.find("geom1=") == std::string::npos);
  CHECK(xml.find("geom2=") == std::string::npos);
}

TEST_CASE("normal-sensor") {
  mjcf::Normal normal;
  normal.name  = "surface_normal";
  normal.geom1 = "floor";
  normal.geom2 = "foot";

  std::string xml = normal.get_xml_text();
  CHECK(xml.find("name=\"surface_normal\"") != std::string::npos);
  CHECK(xml.find("geom1=\"floor\"") != std::string::npos);
  CHECK(xml.find("geom2=\"foot\"") != std::string::npos);
}

TEST_CASE("fromto-sensor") {
  mjcf::FromTo fromto;
  fromto.name  = "vector_sensor";
  fromto.body1 = "start_body";
  fromto.body2 = "end_body";

  std::string xml = fromto.get_xml_text();
  CHECK(xml.find("name=\"vector_sensor\"") != std::string::npos);
  CHECK(xml.find("body1=\"start_body\"") != std::string::npos);
  CHECK(xml.find("body2=\"end_body\"") != std::string::npos);
}

TEST_CASE("contact-sensor") {
  mjcf::Contact contact;
  contact.name = "contact_flag";

  std::string xml = contact.get_xml_text();
  CHECK(xml.find("name=\"contact_flag\"") != std::string::npos);
  CHECK(xml.find("<contact") != std::string::npos);
}

TEST_CASE("tactile-sensor") {
  mjcf::Tactile tactile;
  tactile.name = "touch_sensor";
  tactile.geom = "finger_tip";
  tactile.noise = 0.02;

  std::string xml = tactile.get_xml_text();
  CHECK(xml.find("name=\"touch_sensor\"") != std::string::npos);
  CHECK(xml.find("geom=\"finger_tip\"") != std::string::npos);
  CHECK(xml.find("noise=\"0.02\"") != std::string::npos);
}
