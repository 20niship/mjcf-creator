#include "mjcf/mjcf.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>

void create_motor_actuator_model() {
  std::cout << "Creating motor actuator model..." << std::endl;
  auto mujoco = std::make_shared<mjcf::Mujoco>("motor_test");

  mujoco->option_->integrator = mjcf::IntegratorType::RK4;
  mujoco->option_->timestep   = 0.002;

  auto worldbody = mujoco->worldbody_;
  auto actuator  = mujoco->actuator_;
  auto sensor    = mujoco->sensor_;

  // Create a simple pendulum
  auto pendulum  = std::make_shared<mjcf::Body>();
  pendulum->name = "pendulum";
  pendulum->pos  = std::array<double, 3>{0, 0, 1};

  auto geom  = std::make_shared<mjcf::Geom>();
  geom->name = "pendulum_geom";
  geom->type = mjcf::GeomType::Capsule;
  geom->size = std::array<double, 3>{0.05, 0.5, 0};
  geom->rgba = std::array<double, 4>{0.8, 0.2, 0.2, 1.0};

  auto joint   = std::make_shared<mjcf::Joint>();
  joint->name  = "pendulum_joint";
  joint->type  = mjcf::JointType::Hinge;
  joint->axis  = std::array<double, 3>{0, 1, 0};
  joint->range = std::array<double, 2>{-3.14, 3.14};

  pendulum->add_children({geom, joint});
  worldbody->add_child(pendulum);

  // Add motor actuator
  auto motor       = std::make_shared<mjcf::Motor>();
  motor->name      = "pendulum_motor";
  motor->joint     = "pendulum_joint";
  motor->ctrlrange = std::array<double, 2>{-5.0, 5.0};
  motor->gear      = std::array<double, 6>{1.0, 0, 0, 0, 0, 0};
  actuator->add_child(motor);

  // Add sensors
  auto joint_pos_sensor   = std::make_shared<mjcf::JointPos>();
  joint_pos_sensor->name  = "pendulum_position";
  joint_pos_sensor->joint = "pendulum_joint";

  auto joint_vel_sensor   = std::make_shared<mjcf::JointVel>();
  joint_vel_sensor->name  = "pendulum_velocity";
  joint_vel_sensor->joint = "pendulum_joint";

  auto actuator_force_sensor      = std::make_shared<mjcf::ActuatorFrc>();
  actuator_force_sensor->name     = "motor_force";
  actuator_force_sensor->actuator = "pendulum_motor";

  sensor->add_children({joint_pos_sensor, joint_vel_sensor, actuator_force_sensor});

  std::ofstream file("motor_actuator_model.xml");
  file << mujoco->get_xml_text();
  file.close();
}

void create_position_control_model() {
  std::cout << "Creating position control model..." << std::endl;
  auto mujoco = std::make_shared<mjcf::Mujoco>("position_control");

  auto worldbody = mujoco->worldbody_;
  auto actuator  = mujoco->actuator_;
  auto sensor    = mujoco->sensor_;

  auto base  = std::make_shared<mjcf::Body>();
  base->name = "base";

  auto base_geom  = std::make_shared<mjcf::Geom>();
  base_geom->name = "base_geom";
  base_geom->type = mjcf::GeomType::Cylinder;
  base_geom->size = std::array<double, 3>{0.1, 0.05, 0};
  base_geom->rgba = std::array<double, 4>{0.2, 0.2, 0.8, 1.0};

  auto shoulder_joint   = std::make_shared<mjcf::Joint>();
  shoulder_joint->name  = "shoulder";
  shoulder_joint->type  = mjcf::JointType::Hinge;
  shoulder_joint->axis  = std::array<double, 3>{0, 0, 1};
  shoulder_joint->range = std::array<double, 2>{-1.57, 1.57};

  base->add_children({base_geom, shoulder_joint});

  // Upper arm
  auto upper_arm  = std::make_shared<mjcf::Body>();
  upper_arm->name = "upper_arm";
  upper_arm->pos  = std::array<double, 3>{0.2, 0, 0};

  auto upper_arm_geom  = std::make_shared<mjcf::Geom>();
  upper_arm_geom->name = "upper_arm_geom";
  upper_arm_geom->type = mjcf::GeomType::Capsule;
  upper_arm_geom->size = std::array<double, 3>{0.03, 0.15, 0};
  upper_arm_geom->rgba = std::array<double, 4>{0.8, 0.8, 0.2, 1.0};

  auto elbow_joint   = std::make_shared<mjcf::Joint>();
  elbow_joint->name  = "elbow";
  elbow_joint->type  = mjcf::JointType::Hinge;
  elbow_joint->axis  = std::array<double, 3>{0, 1, 0};
  elbow_joint->range = std::array<double, 2>{-1.57, 1.57};

  upper_arm->add_children({upper_arm_geom, elbow_joint});
  base->add_child(upper_arm);
  worldbody->add_child(base);

  // Add position actuators
  auto shoulder_pos       = std::make_shared<mjcf::Position>();
  shoulder_pos->name      = "shoulder_position";
  shoulder_pos->joint     = "shoulder";
  shoulder_pos->kp        = 1000.0;
  shoulder_pos->kv        = 100.0;
  shoulder_pos->ctrlrange = std::array<double, 2>{-1.57, 1.57};

  auto elbow_pos       = std::make_shared<mjcf::Position>();
  elbow_pos->name      = "elbow_position";
  elbow_pos->joint     = "elbow";
  elbow_pos->kp        = 500.0;
  elbow_pos->kv        = 50.0;
  elbow_pos->ctrlrange = std::array<double, 2>{-1.57, 1.57};

  actuator->add_children({shoulder_pos, elbow_pos});

  // Add comprehensive sensors
  auto shoulder_pos_sensor   = std::make_shared<mjcf::JointPos>();
  shoulder_pos_sensor->name  = "shoulder_position_sensor";
  shoulder_pos_sensor->joint = "shoulder";

  auto elbow_vel_sensor   = std::make_shared<mjcf::JointVel>();
  elbow_vel_sensor->name  = "elbow_velocity_sensor";
  elbow_vel_sensor->joint = "elbow";

  sensor->add_children({shoulder_pos_sensor, elbow_vel_sensor});
  mujoco->add_children({actuator, sensor});

  std::ofstream file("position_control_model.xml");
  file << mujoco->get_xml_text();
  file.close();
}

void create_muscle_actuator_model() {
  std::cout << "Creating muscle actuator model..." << std::endl;
  auto mujoco = std::make_shared<mjcf::Mujoco>("biomechanical");

  auto worldbody = mujoco->worldbody_;
  auto actuator  = mujoco->actuator_;
  auto sensor    = mujoco->sensor_;

  // Create a simple finger with muscle actuation
  auto finger  = std::make_shared<mjcf::Body>();
  finger->name = "finger";

  auto finger_geom  = std::make_shared<mjcf::Geom>();
  finger_geom->name = "finger_geom";
  finger_geom->type = mjcf::GeomType::Capsule;
  finger_geom->size = std::array<double, 3>{0.02, 0.08, 0};
  finger_geom->rgba = std::array<double, 4>{0.9, 0.7, 0.6, 1.0};

  auto finger_joint   = std::make_shared<mjcf::Joint>();
  finger_joint->name  = "finger_joint";
  finger_joint->type  = mjcf::JointType::Hinge;
  finger_joint->axis  = std::array<double, 3>{0, 1, 0};
  finger_joint->range = std::array<double, 2>{0, 1.57};

  finger->add_children({finger_geom, finger_joint});
  worldbody->add_child(finger);

  auto flexor       = std::make_shared<mjcf::Muscle>();
  flexor->name      = "flexor_muscle";
  flexor->joint     = "finger_joint";
  flexor->force     = 50.0;
  flexor->range     = std::array<double, 2>{0.8, 1.6};
  flexor->timeconst = std::array<double, 2>{0.01, 0.04};
  flexor->lmin      = 0.5;
  flexor->lmax      = 1.6;
  flexor->vmax      = 1.5;

  actuator->add_child(flexor);

  // Add force and position sensors
  auto position_sensor   = std::make_shared<mjcf::JointPos>();
  position_sensor->name  = "finger_position";
  position_sensor->joint = "finger_joint";

  auto velocity_sensor   = std::make_shared<mjcf::JointVel>();
  velocity_sensor->name  = "finger_velocity";
  velocity_sensor->joint = "finger_joint";

  sensor->add_children({position_sensor, velocity_sensor});
  mujoco->add_children({actuator, sensor});

  std::ofstream file("muscle_actuator_model.xml");
  file << mujoco->get_xml_text();
  file.close();
}

void create_imu_sensor_model() {
  std::cout << "Creating IMU sensor model..." << std::endl;
  auto mujoco = std::make_shared<mjcf::Mujoco>("imu_test");

  auto worldbody = mujoco->worldbody_;
  auto sensor    = mujoco->sensor_;

  auto cube  = std::make_shared<mjcf::Body>();
  cube->name = "floating_cube";
  cube->pos  = std::array<double, 3>{0, 0, 2};

  auto cube_geom  = std::make_shared<mjcf::Geom>();
  cube_geom->name = "cube_geom";
  cube_geom->type = mjcf::GeomType::Box;
  cube_geom->size = std::array<double, 3>{0.1, 0.1, 0.1};
  cube_geom->rgba = std::array<double, 4>{0.5, 0.8, 0.3, 1.0};

  auto free_joint  = std::make_shared<mjcf::Joint>();
  free_joint->name = "free_joint";
  free_joint->type = mjcf::JointType::Free;

  // IMU site
  auto imu_site  = std::make_shared<mjcf::Site>();
  imu_site->name = "imu_site";
  imu_site->type = mjcf::SiteType::Sphere;
  imu_site->size = {0.01};
  imu_site->rgba = std::array<double, 4>{1.0, 0, 0, 1.0};

  cube->add_children({cube_geom, free_joint, imu_site});
  worldbody->add_child(cube);

  // Add IMU sensors
  auto gyro   = std::make_shared<mjcf::Gyro>();
  gyro->name  = "imu_gyroscope";
  gyro->site  = "imu_site";
  gyro->noise = 0.001;

  auto accel   = std::make_shared<mjcf::Accelerometer>();
  accel->name  = "imu_accelerometer";
  accel->site  = "imu_site";
  accel->noise = 0.01;

  sensor->add_children({gyro, accel});

  mujoco->add_children({worldbody, sensor});

  std::ofstream file("imu_sensor_model.xml");
  file << mujoco->get_xml_text();
  file.close();
}

void create_force_sensor_model() {
  std::cout << "Creating force sensor model..." << std::endl;
  auto mujoco = std::make_shared<mjcf::Mujoco>("force_sensing");

  auto worldbody = mujoco->worldbody_;
  auto sensor    = mujoco->sensor_;

  // Create a manipulator with force sensing
  auto manipulator  = std::make_shared<mjcf::Body>();
  manipulator->name = "manipulator";

  auto manip_geom  = std::make_shared<mjcf::Geom>();
  manip_geom->name = "manipulator_geom";
  manip_geom->type = mjcf::GeomType::Capsule;
  manip_geom->size = std::array<double, 3>{0.02, 0.1, 0};
  manip_geom->rgba = std::array<double, 4>{0.7, 0.7, 0.7, 1.0};

  auto manip_joint   = std::make_shared<mjcf::Joint>();
  manip_joint->name  = "manipulator_joint";
  manip_joint->type  = mjcf::JointType::Slide;
  manip_joint->axis  = std::array<double, 3>{0, 0, 1};
  manip_joint->range = std::array<double, 2>{0, 0.5};

  // Force sensing site at the tip
  auto force_site  = std::make_shared<mjcf::Site>();
  force_site->name = "force_site";
  force_site->type = mjcf::SiteType::Sphere;
  force_site->pos  = std::array<double, 3>{0, 0, 0.1};
  force_site->size = {0.005};
  force_site->rgba = std::array<double, 4>{1.0, 1.0, 0, 1.0};

  manipulator->add_children({manip_geom, manip_joint, force_site});
  worldbody->add_child(manipulator);

  // Add force and torque sensors
  auto force_sensor   = std::make_shared<mjcf::Force>();
  force_sensor->name  = "tip_force";
  force_sensor->site  = "force_site";
  force_sensor->noise = 0.1;

  auto torque_sensor   = std::make_shared<mjcf::Torque>();
  torque_sensor->name  = "tip_torque";
  torque_sensor->site  = "force_site";
  torque_sensor->noise = 0.01;

  auto touch_sensor  = std::make_shared<mjcf::Touch>();
  touch_sensor->name = "tip_touch";
  touch_sensor->site = "force_site";

  sensor->add_children({force_sensor, torque_sensor, touch_sensor});

  mujoco->add_children({sensor});

  std::ofstream file("force_sensor_model.xml");
  file << mujoco->get_xml_text();
  file.close();
}

int main() {
  std::cout << "MJCF Comprehensive Robot Demo" << std::endl;
  std::cout << "=============================" << std::endl;

  // Create output directory
  std::filesystem::create_directories("mjcf_test_models");
  std::filesystem::current_path("mjcf_test_models");

  // Create various models demonstrating different actuator and sensor types
  create_motor_actuator_model();
  create_position_control_model();
  create_muscle_actuator_model();
  create_imu_sensor_model();
  create_force_sensor_model();

  std::cout << std::endl;
  std::cout << "Generated 5 comprehensive MJCF models in mjcf_test_models/ directory:" << std::endl;
  std::cout << "1. motor_actuator_model.xml - Motor actuators with joint sensors" << std::endl;
  std::cout << "2. position_control_model.xml - Position actuators for precise control" << std::endl;
  std::cout << "3. muscle_actuator_model.xml - Biomechanical muscle actuation" << std::endl;
  std::cout << "4. imu_sensor_model.xml - IMU sensors (gyro + accelerometer)" << std::endl;
  std::cout << "5. force_sensor_model.xml - Force, torque, and touch sensors" << std::endl;
  std::cout << std::endl;
  std::cout << "All models use enum classes for type safety and std::array for fixed-size parameters." << std::endl;
  std::cout << "These files can be loaded and validated with MuJoCo." << std::endl;

  return 0;
}
