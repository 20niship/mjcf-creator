#include "mjcf/mjcf.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

using namespace mjcf;

/**
 * @brief Creates a MJCF scene with falling objects
 *
 * Scene includes:
 * - Checkerboard ground plane
 * - Multiple geometric objects (sphere, box, cylinder, capsule) in the air
 * - Objects fall due to gravity when simulation starts
 */
void create_falling_objects_scene() {
  std::cout << "Creating falling objects scene..." << std::endl;

  auto mujoco    = std::make_shared<mjcf::Mujoco>("falling_objects");
  auto asset     = mujoco->asset_;
  auto worldbody = mujoco->worldbody_;

  auto checker_texture     = std::make_shared<mjcf::Texture>();
  checker_texture->name    = "checker";
  checker_texture->type    = mjcf::TextureType::TwoD;
  checker_texture->builtin = mjcf::TextureBuiltin::Checker;
  checker_texture->rgb1    = {0.2, 0.3, 0.4};
  checker_texture->rgb2    = {0.3, 0.4, 0.5};
  checker_texture->width   = 512;
  checker_texture->height  = 512;

  auto floor_mat         = std::make_shared<mjcf::Material>();
  floor_mat->name        = "floor_mat";
  floor_mat->texture     = "checker";
  floor_mat->texrepeat   = {10, 10};
  floor_mat->reflectance = 0.1;

  asset->add_children({checker_texture, floor_mat});


  auto light = Light::Create(true, {0, 0, 3}, {0, 0, -1}, {0.8, 0.8, 0.8});
  worldbody->add_child(light);

  auto floor      = Geom::Plane("floor", {5, 5, 0.1});
  floor->material = "floor_mat";
  floor->condim   = 3;
  worldbody->add_child(floor);

  auto sphere_body  = Body::Create("sphere", {-1.5, 0, 2.0});
  auto sphere_geom  = Geom::Sphere("sphere_geom", 0.2, {0, 0, 0}, {1, 0, 0, 1});
  auto sphere_joint = Joint::Free("sphere_joint");

  sphere_body->add_children({sphere_geom, sphere_joint});
  worldbody->add_child(sphere_body);

  auto box_body  = Body::Create("box", {-0.5, 0, 2.5});
  auto box_geom  = Geom::Box("box_geom", {0.15, 0.15, 0.15}, {0, 0, 0}, {0, 1, 0, 1});
  auto box_joint = Joint::Free("box_joint");

  box_body->add_children({box_geom, box_joint});
  worldbody->add_child(box_body);

  // Object 3: Cylinder (blue)
  auto cylinder_body  = std::make_shared<mjcf::Body>();
  cylinder_body->name = "cylinder";
  cylinder_body->pos  = std::array<double, 3>{0.5, 0, 3.0};

  auto cylinder_geom  = std::make_shared<mjcf::Geom>();
  cylinder_geom->name = "cylinder_geom";
  cylinder_geom->type = mjcf::GeomType::Cylinder;
  cylinder_geom->size = std::array<double, 3>{0.1, 0.3, 0};
  cylinder_geom->rgba = std::array<double, 4>{0, 0, 1, 1};

  auto cylinder_joint  = std::make_shared<mjcf::Joint>();
  cylinder_joint->name = "cylinder_joint";
  cylinder_joint->type = mjcf::JointType::Free;

  cylinder_body->add_children({cylinder_geom, cylinder_joint});
  worldbody->add_child(cylinder_body);

  // Object 4: Capsule (yellow)
  auto capsule_body  = std::make_shared<mjcf::Body>();
  capsule_body->name = "capsule";
  capsule_body->pos  = std::array<double, 3>{1.5, 0, 2.2};

  auto capsule_geom  = std::make_shared<mjcf::Geom>();
  capsule_geom->name = "capsule_geom";
  capsule_geom->type = mjcf::GeomType::Capsule;
  capsule_geom->size = std::array<double, 3>{0.1, 0.25, 0};
  capsule_geom->rgba = std::array<double, 4>{1, 1, 0, 1};

  auto capsule_joint  = std::make_shared<mjcf::Joint>();
  capsule_joint->name = "capsule_joint";
  capsule_joint->type = mjcf::JointType::Free;

  capsule_body->add_children({capsule_geom, capsule_joint});
  worldbody->add_child(capsule_body);

  // Object 5: Another sphere (cyan)
  auto sphere2_body  = std::make_shared<mjcf::Body>();
  sphere2_body->name = "sphere2";
  sphere2_body->pos  = std::array<double, 3>{0, 1.5, 2.8};

  auto sphere2_geom  = std::make_shared<mjcf::Geom>();
  sphere2_geom->name = "sphere2_geom";
  sphere2_geom->type = mjcf::GeomType::Sphere;
  sphere2_geom->size = std::array<double, 3>{0.18, 0, 0};
  sphere2_geom->rgba = std::array<double, 4>{0, 1, 1, 1};

  auto sphere2_joint  = std::make_shared<mjcf::Joint>();
  sphere2_joint->name = "sphere2_joint";
  sphere2_joint->type = mjcf::JointType::Free;

  sphere2_body->add_children({sphere2_geom, sphere2_joint});
  worldbody->add_child(sphere2_body);

  // Object 6: Another box (magenta)
  auto box2_body  = std::make_shared<mjcf::Body>();
  box2_body->name = "box2";
  box2_body->pos  = std::array<double, 3>{0, -1.5, 3.2};

  auto box2_geom  = std::make_shared<mjcf::Geom>();
  box2_geom->name = "box2_geom";
  box2_geom->type = mjcf::GeomType::Box;
  box2_geom->size = std::array<double, 3>{0.2, 0.1, 0.15};
  box2_geom->rgba = std::array<double, 4>{1, 0, 1, 1};

  auto box2_joint  = std::make_shared<mjcf::Joint>();
  box2_joint->name = "box2_joint";
  box2_joint->type = mjcf::JointType::Free;

  box2_body->add_children({box2_geom, box2_joint});
  worldbody->add_child(box2_body);

  std::ofstream file("../output/falling_objects.xml");
  file << mujoco->get_xml_text();
  file.close();

  std::cout << "  Generated: ../output/falling_objects.xml" << std::endl;
}

/**
 * @brief Creates a MJCF scene with a double pendulum
 *
 * Scene includes:
 * - Fixed base joint
 * - Two connected elongated rods (capsules)
 * - Initial position: horizontal
 * - Swings due to gravity
 */
void create_double_pendulum_scene() {
  std::cout << "Creating double pendulum scene..." << std::endl;

  auto mujoco = std::make_shared<mjcf::Mujoco>("double_pendulum");

  // Compiler settings
  mujoco->compiler_->angle           = mjcf::AngleUnit::Degree;
  mujoco->compiler_->inertiafromgeom = true;

  mujoco->option_->gravity    = std::array<double, 3>{0, 0, -9.81};
  mujoco->option_->timestep   = 0.005;
  mujoco->option_->integrator = mjcf::IntegratorType::RK4;

  auto asset     = mujoco->asset_;
  auto worldbody = mujoco->worldbody_;

  // Grid texture
  auto texture     = std::make_shared<mjcf::Texture>();
  texture->name    = "grid";
  texture->type    = mjcf::TextureType::TwoD;
  texture->builtin = mjcf::TextureBuiltin::Checker;
  texture->rgb1    = {0.1, 0.2, 0.3};
  texture->rgb2    = {0.2, 0.3, 0.4};
  texture->width   = 512;
  texture->height  = 512;

  // Floor material
  auto floor_mat       = std::make_shared<mjcf::Material>();
  floor_mat->name      = "floor_mat";
  floor_mat->texture   = "grid";
  floor_mat->texrepeat = {8, 8};

  asset->add_children({texture, floor_mat});

  // Default settings
  auto default_elem       = std::make_shared<mjcf::detail::Default>();
  auto default_joint      = std::make_shared<mjcf::Joint>();
  default_joint->damping  = 0.1;
  default_joint->armature = 0.01;
  default_elem->add_child(default_joint);


  auto light         = std::make_shared<mjcf::Light>();
  light->directional = true;
  light->pos         = std::array<double, 3>{0, 0, 3};
  light->dir         = std::array<double, 3>{0, 0, -1};
  light->diffuse     = std::array<double, 3>{0.8, 0.8, 0.8};
  worldbody->add_child(light);

  auto floor      = std::make_shared<mjcf::Geom>();
  floor->name     = "floor";
  floor->type     = mjcf::GeomType::Plane;
  floor->size     = std::array<double, 3>{5, 5, 0.1};
  floor->material = "floor_mat";
  floor->condim   = 3;
  worldbody->add_child(floor);

  // Fixed base
  auto base_body  = std::make_shared<mjcf::Body>();
  base_body->name = "base";
  base_body->pos  = std::array<double, 3>{0, 0, 1.5};

  auto base_geom  = std::make_shared<mjcf::Geom>();
  base_geom->name = "base_geom";
  base_geom->type = mjcf::GeomType::Sphere;
  base_geom->size = std::array<double, 3>{0.08, 0, 0};
  base_geom->rgba = std::array<double, 4>{0.5, 0.5, 0.5, 1};
  base_body->add_child(base_geom);

  auto link1_body  = std::make_shared<mjcf::Body>();
  link1_body->name = "link1";
  link1_body->pos  = std::array<double, 3>{0.5, 0, 0}; // Horizontal position

  auto joint1     = std::make_shared<mjcf::Joint>();
  joint1->name    = "joint1";
  joint1->type    = mjcf::JointType::Hinge;
  joint1->axis    = std::array<double, 3>{0, 1, 0}; // Y axis
  joint1->pos     = std::array<double, 3>{-0.5, 0, 0};
  joint1->limited = false;

  // Link 1 geometry
  auto link1_geom    = std::make_shared<mjcf::Geom>();
  link1_geom->name   = "link1_geom";
  link1_geom->type   = mjcf::GeomType::Capsule;
  link1_geom->fromto = std::array<double, 6>{-0.5, 0, 0, 0.5, 0, 0};
  link1_geom->size   = std::array<double, 3>{0.05, 0, 0};
  link1_geom->rgba   = std::array<double, 4>{0.8, 0.2, 0.2, 1};

  link1_body->add_children({joint1, link1_geom});
  base_body->add_child(link1_body);

  // Second pendulum link (lower link)
  auto link2_body  = std::make_shared<mjcf::Body>();
  link2_body->name = "link2";
  link2_body->pos  = std::array<double, 3>{1.0, 0, 0}; // At end of link1

  // Joint at connection point
  auto joint2     = std::make_shared<mjcf::Joint>();
  joint2->name    = "joint2";
  joint2->type    = mjcf::JointType::Hinge;
  joint2->axis    = std::array<double, 3>{0, 1, 0};
  joint2->pos     = std::array<double, 3>{-0.5, 0, 0};
  joint2->limited = false;

  // Link 2 geometry
  auto link2_geom    = std::make_shared<mjcf::Geom>();
  link2_geom->name   = "link2_geom";
  link2_geom->type   = mjcf::GeomType::Capsule;
  link2_geom->fromto = std::array<double, 6>{-0.5, 0, 0, 0.5, 0, 0};
  link2_geom->size   = std::array<double, 3>{0.05, 0, 0};
  link2_geom->rgba   = std::array<double, 4>{0.2, 0.2, 0.8, 1};

  // Tip marker
  auto tip_geom  = std::make_shared<mjcf::Geom>();
  tip_geom->name = "tip";
  tip_geom->type = mjcf::GeomType::Sphere;
  tip_geom->pos  = std::array<double, 3>{0.5, 0, 0};
  tip_geom->size = std::array<double, 3>{0.08, 0, 0};
  tip_geom->rgba = std::array<double, 4>{1, 1, 0, 1};

  link2_body->add_children({joint2, link2_geom, tip_geom});
  link1_body->add_child(link2_body);

  worldbody->add_child(base_body);

  std::ofstream file("../output/double_pendulum.xml");
  file << mujoco->get_xml_text();
  file.close();

  std::cout << "  Generated: ../output/double_pendulum.xml" << std::endl;
}

/**
 * @brief Creates a MJCF scene with a 4-wheeled vehicle
 *
 * Scene includes:
 * - Box-shaped body
 * - 4 cylindrical wheels
 * - Motors connecting wheels to body
 * - Can move forward/backward
 */
void create_vehicle_scene() {
  std::cout << "Creating 4-wheel vehicle scene..." << std::endl;

  auto mujoco = std::make_shared<mjcf::Mujoco>("vehicle");

  mujoco->compiler_->angle           = mjcf::AngleUnit::Degree;
  mujoco->compiler_->inertiafromgeom = true;

  mujoco->option_->gravity    = std::array<double, 3>{0, 0, -9.81};
  mujoco->option_->timestep   = 0.01;
  mujoco->option_->integrator = mjcf::IntegratorType::RK4;

  auto asset     = mujoco->asset_;
  auto worldbody = mujoco->worldbody_;

  // Ground texture
  auto texture     = std::make_shared<mjcf::Texture>();
  texture->name    = "ground";
  texture->type    = mjcf::TextureType::TwoD;
  texture->builtin = mjcf::TextureBuiltin::Checker;
  texture->rgb1    = {0.3, 0.3, 0.3};
  texture->rgb2    = {0.4, 0.4, 0.4};
  texture->width   = 512;
  texture->height  = 512;

  auto ground_mat         = std::make_shared<mjcf::Material>();
  ground_mat->name        = "ground_mat";
  ground_mat->texture     = "ground";
  ground_mat->texrepeat   = {20, 20};
  ground_mat->reflectance = 0.1;

  asset->add_children({texture, ground_mat});

  auto default_elem       = std::make_shared<mjcf::detail::Default>();
  auto default_joint      = std::make_shared<mjcf::Joint>();
  default_joint->damping  = 0.5;
  default_joint->armature = 0.1;
  default_elem->add_child(default_joint);

  auto light         = std::make_shared<mjcf::Light>();
  light->directional = true;
  light->pos         = std::array<double, 3>{0, 0, 5};
  light->dir         = std::array<double, 3>{0, 0, -1};
  light->diffuse     = std::array<double, 3>{0.8, 0.8, 0.8};
  worldbody->add_child(light);

  auto floor      = std::make_shared<mjcf::Geom>();
  floor->name     = "floor";
  floor->type     = mjcf::GeomType::Plane;
  floor->size     = std::array<double, 3>{20, 20, 0.1};
  floor->material = "ground_mat";
  floor->condim   = 3;
  floor->friction = std::array<double, 3>{1, 0.5, 0.5};
  worldbody->add_child(floor);

  // Vehicle body
  auto vehicle_body  = std::make_shared<mjcf::Body>();
  vehicle_body->name = "chassis";
  vehicle_body->pos  = std::array<double, 3>{0, 0, 0.3};

  // Free joint for chassis
  auto chassis_joint  = std::make_shared<mjcf::Joint>();
  chassis_joint->name = "chassis_free";
  chassis_joint->type = mjcf::JointType::Free;
  vehicle_body->add_child(chassis_joint);

  // Chassis geometry
  auto chassis_geom  = std::make_shared<mjcf::Geom>();
  chassis_geom->name = "chassis_geom";
  chassis_geom->type = mjcf::GeomType::Box;
  chassis_geom->size = std::array<double, 3>{0.6, 0.3, 0.15};
  chassis_geom->rgba = std::array<double, 4>{0.7, 0.3, 0.3, 1};
  vehicle_body->add_child(chassis_geom);

  // Wheel positions
  struct WheelInfo {
    std::string name;
    double x, y, z;
  };

  std::vector<WheelInfo> wheel_positions = {{"front_left", 0.4, 0.35, 0.0}, {"front_right", 0.4, -0.35, 0.0}, {"rear_left", -0.4, 0.35, 0.0}, {"rear_right", -0.4, -0.35, 0.0}};

  // Create wheels
  for(const auto& wheel_info : wheel_positions) {
    auto wheel_body  = std::make_shared<mjcf::Body>();
    wheel_body->name = wheel_info.name;
    wheel_body->pos  = std::array<double, 3>{wheel_info.x, wheel_info.y, wheel_info.z};

    // Hinge joint
    auto wheel_joint     = std::make_shared<mjcf::Joint>();
    wheel_joint->name    = wheel_info.name + "_joint";
    wheel_joint->type    = mjcf::JointType::Hinge;
    wheel_joint->axis    = std::array<double, 3>{0, 1, 0};
    wheel_joint->limited = false;
    wheel_joint->damping = 0.1;

    // Wheel geometry
    auto wheel_geom      = std::make_shared<mjcf::Geom>();
    wheel_geom->name     = wheel_info.name + "_geom";
    wheel_geom->type     = mjcf::GeomType::Cylinder;
    wheel_geom->size     = std::array<double, 3>{0.15, 0.08, 0};
    wheel_geom->rgba     = std::array<double, 4>{0.2, 0.2, 0.2, 1};
    wheel_geom->friction = std::array<double, 3>{1.5, 0.5, 0.5};
    // Rotate cylinder to be horizontal (around Y axis)
    wheel_geom->quat = std::array<double, 4>{0.707107, 0, 0.707107, 0};

    wheel_body->add_children({wheel_joint, wheel_geom});
    vehicle_body->add_child(wheel_body);
  }

  worldbody->add_child(vehicle_body);

  auto actuator = mujoco->actuator_;
  for(const auto& wheel_info : wheel_positions) {
    auto motor       = std::make_shared<mjcf::Position>();
    motor->name      = wheel_info.name + "_motor";
    motor->joint     = wheel_info.name + "_joint";
    motor->gear      = {100, 0, 0, 0, 0, 0};
    motor->kp        = 100;
    motor->kv        = 10;
    motor->ctrlrange = {-1, 1};
    actuator->add_child(motor);
  }

  std::ofstream file("../output/vehicle.xml");
  file << mujoco->get_xml_text();
  file.close();

  std::cout << "  Generated: output/vehicle.xml" << std::endl;
}

void convert_urdf_to_mjcf() {
  std::cout << "URDF to MJCF conversion example..." << std::endl;

  std::string urdf_path   = "../examples/urdf_files/simple_robot.urdf";
  std::string output_path = "../output/urdf_converted.xml";

  if(!std::filesystem::exists(urdf_path)) {
    std::cerr << "  Error: URDF file not found: " << urdf_path << std::endl;
    std::cerr << "  Available URDF files:" << std::endl;
    for(const auto& entry : std::filesystem::directory_iterator("urdf_files")) {
      if(entry.path().extension() == ".urdf") {
        std::cerr << "    - " << entry.path().filename() << std::endl;
      }
    }
    return;
  }

  auto mujoco = std::make_shared<mjcf::Mujoco>();
  mujoco->add_urdf(urdf_path);

  std::ofstream file(output_path);
  file << mujoco->get_xml_text();
  file.close();
  std::cout << "  Generated: " << output_path << std::endl;
}

void print_usage() {
  std::cout << "MJCF Sample Generator" << std::endl;
  std::cout << "=====================" << std::endl;
  std::cout << std::endl;
  std::cout << "Usage: generate_samples [option]" << std::endl;
  std::cout << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "  all              Generate all samples (default)" << std::endl;
  std::cout << "  falling          Generate falling objects scene" << std::endl;
  std::cout << "  pendulum         Generate double pendulum" << std::endl;
  std::cout << "  vehicle          Generate 4-wheel vehicle" << std::endl;
  std::cout << "  urdf             Generate URDF conversion example" << std::endl;
  std::cout << "  help             Show this help message" << std::endl;
  std::cout << std::endl;
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char* argv[]) {
  if(!std::filesystem::exists("../output")) {
    std::filesystem::create_directory("../output");
  }

  create_falling_objects_scene();
  create_double_pendulum_scene();
  create_vehicle_scene();
  convert_urdf_to_mjcf();

  std::cout << std::endl;
  std::cout << "Done! Generated files are in examples/output/" << std::endl;

  return 0;
}
