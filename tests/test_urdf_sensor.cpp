#include "doctest.h"
#include <filesystem>
#include <fstream>
#include <mjcf/mjcf.hpp>

TEST_CASE("FT Sensor URDF parsing") {
  const std::string urdf_content = R"(
    <?xml version="1.0"?>
    <robot name="test_robot">
      <link name="base_link"><inertial><mass value="1.0"/></inertial></link>
      <link name="sensor_link"><inertial><mass value="0.1"/></inertial></link>
      <joint name="sensor_joint" type="fixed">
        <parent link="base_link"/>
        <child link="sensor_link"/>
        <origin xyz="0 0 0.1"/>
      </joint>
      <gazebo>
        <plugin name="ft_sensor" filename="libgazebo_ros_ft_sensor.so">
          <updateRate>100.0</updateRate>
          <topicName>ft_sensor_topic</topicName>
          <jointName>sensor_joint</jointName>
        </plugin>
      </gazebo>
    </robot>
  )";

  std::string tmp = "test_ft_sensor.urdf";
  std::ofstream f(tmp);
  f << urdf_content;
  f.close();

  auto mujoco         = std::make_shared<mjcf::Mujoco>();
  auto [body, joint]  = mujoco->add_urdf(tmp);
  CHECK(body != nullptr);

  std::string xml = mujoco->get_xml_text();
  CHECK(xml.find("<force") != std::string::npos);
  CHECK(xml.find("<torque") != std::string::npos);
  CHECK(xml.find("<site") != std::string::npos);

  auto sensors = mujoco->get_sensors();
  CHECK(sensors.size() == 2);

  std::filesystem::remove(tmp);
}

TEST_CASE("IMU Sensor URDF parsing with plugin tag") {
  const std::string urdf_content = R"(
    <?xml version="1.0"?>
    <robot name="test_robot">
      <link name="base_link"><inertial><mass value="1.0"/></inertial></link>
      <link name="imu_link"><inertial><mass value="0.01"/></inertial></link>
      <joint name="imu_joint" type="fixed">
        <parent link="base_link"/>
        <child link="imu_link"/>
      </joint>
      <gazebo>
        <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
          <frameName>imu_link</frameName>
        </plugin>
      </gazebo>
    </robot>
  )";

  std::string tmp = "test_imu_sensor.urdf";
  std::ofstream f(tmp);
  f << urdf_content;
  f.close();

  auto mujoco = std::make_shared<mjcf::Mujoco>();
  mujoco->add_urdf(tmp);

  std::string xml = mujoco->get_xml_text();
  CHECK(xml.find("<gyro") != std::string::npos);
  CHECK(xml.find("<accelerometer") != std::string::npos);

  auto sensors = mujoco->get_sensors();
  CHECK(sensors.size() == 2);

  std::filesystem::remove(tmp);
}

TEST_CASE("IMU Sensor URDF parsing with gazebo reference tag") {
  const std::string urdf_content = R"(
    <?xml version="1.0"?>
    <robot name="test_robot">
      <link name="base_link"><inertial><mass value="1.0"/></inertial></link>
      <link name="imu_link"><inertial><mass value="0.01"/></inertial></link>
      <joint name="imu_joint" type="fixed">
        <parent link="base_link"/>
        <child link="imu_link"/>
      </joint>
      <gazebo reference="imu_link">
        <sensor type="imu" name="imu_sensor">
          <update_rate>100</update_rate>
        </sensor>
      </gazebo>
    </robot>
  )";

  std::string tmp = "test_imu_ref.urdf";
  std::ofstream f(tmp);
  f << urdf_content;
  f.close();

  auto mujoco = std::make_shared<mjcf::Mujoco>();
  mujoco->add_urdf(tmp);

  std::string xml = mujoco->get_xml_text();
  CHECK(xml.find("<gyro") != std::string::npos);
  CHECK(xml.find("<accelerometer") != std::string::npos);

  std::filesystem::remove(tmp);
}

TEST_CASE("FT and IMU sensors in same URDF") {
  const std::string urdf_content = R"(
    <?xml version="1.0"?>
    <robot name="test_robot">
      <link name="base_link"><inertial><mass value="1.0"/></inertial></link>
      <link name="sensor_link"><inertial><mass value="0.1"/></inertial></link>
      <link name="imu_link"><inertial><mass value="0.01"/></inertial></link>
      <joint name="sensor_joint" type="fixed">
        <parent link="base_link"/>
        <child link="sensor_link"/>
      </joint>
      <joint name="imu_joint" type="fixed">
        <parent link="base_link"/>
        <child link="imu_link"/>
      </joint>
      <gazebo>
        <plugin name="ft_sensor" filename="libgazebo_ros_ft_sensor.so">
          <jointName>sensor_joint</jointName>
        </plugin>
      </gazebo>
      <gazebo reference="imu_link">
        <sensor type="imu" name="imu_sensor"/>
      </gazebo>
    </robot>
  )";

  std::string tmp = "test_ft_imu.urdf";
  std::ofstream f(tmp);
  f << urdf_content;
  f.close();

  auto mujoco = std::make_shared<mjcf::Mujoco>();
  mujoco->add_urdf(tmp);

  // force + torque + gyro + accelerometer = 4 sensors
  auto sensors = mujoco->get_sensors();
  CHECK(sensors.size() == 4);

  std::filesystem::remove(tmp);
}

TEST_SUITE("Sensor Tests") {
  TEST_CASE("Simple sensor test") {
    auto mujoco  = std::make_shared<mjcf::Mujoco>();
    auto sensors = mujoco->get_sensors();
    CHECK(sensors.size() == 0);
  }
}

TEST_CASE("FT sensor metadata via get_sensors with root") {
  const std::string urdf_content = R"(
    <?xml version="1.0"?>
    <robot name="test_robot">
      <link name="base_link"><inertial><mass value="1.0"/></inertial></link>
      <link name="sensor_link"><inertial><mass value="0.1"/></inertial></link>
      <joint name="sensor_joint" type="fixed">
        <parent link="base_link"/>
        <child link="sensor_link"/>
        <origin xyz="0 0 0.1"/>
      </joint>
      <gazebo>
        <plugin name="ft_sensor" filename="libgazebo_ros_ft_sensor.so">
          <jointName>sensor_joint</jointName>
        </plugin>
      </gazebo>
    </robot>
  )";

  std::string tmp = "test_ft_body_sensors.urdf";
  std::ofstream f(tmp);
  f << urdf_content;
  f.close();

  auto mujoco        = std::make_shared<mjcf::Mujoco>();
  auto [body, joint] = mujoco->add_urdf(tmp);
  REQUIRE(body != nullptr);

  auto body_sensors = mujoco->get_sensors(body);
  REQUIRE(body_sensors.size() == 2);

  std::vector<std::string> names;
  for(const auto& s : body_sensors) names.push_back(s->name);
  CHECK(std::find(names.begin(), names.end(), "ft_sensor_force") != names.end());
  CHECK(std::find(names.begin(), names.end(), "ft_sensor_torque") != names.end());

  std::string xml = mujoco->get_xml_text();
  CHECK(xml.find("ft_sensor_force") != std::string::npos);
  CHECK(xml.find("ft_sensor_torque") != std::string::npos);

  std::filesystem::remove(tmp);
}

TEST_CASE("IMU sensor metadata via get_sensors with root") {
  const std::string urdf_content = R"(
    <?xml version="1.0"?>
    <robot name="test_robot">
      <link name="base_link"><inertial><mass value="1.0"/></inertial></link>
      <link name="imu_link"><inertial><mass value="0.01"/></inertial></link>
      <joint name="imu_joint" type="fixed">
        <parent link="base_link"/>
        <child link="imu_link"/>
      </joint>
      <gazebo reference="imu_link">
        <sensor type="imu" name="imu_sensor">
          <update_rate>100</update_rate>
        </sensor>
      </gazebo>
    </robot>
  )";

  std::string tmp = "test_imu_body_sensors.urdf";
  std::ofstream f(tmp);
  f << urdf_content;
  f.close();

  auto mujoco        = std::make_shared<mjcf::Mujoco>();
  auto [body, joint] = mujoco->add_urdf(tmp);
  REQUIRE(body != nullptr);

  auto body_sensors = mujoco->get_sensors(body);
  REQUIRE(body_sensors.size() == 2);

  std::vector<std::string> names;
  for(const auto& s : body_sensors) names.push_back(s->name);
  CHECK(std::find(names.begin(), names.end(), "imu_sensor_gyro") != names.end());
  CHECK(std::find(names.begin(), names.end(), "imu_sensor_accelerometer") != names.end());

  std::filesystem::remove(tmp);
}

TEST_CASE("get_sensors with root returns empty when no Gazebo sensors present") {
  const std::string urdf_content = R"(
    <?xml version="1.0"?>
    <robot name="test_robot">
      <link name="base_link"><inertial><mass value="1.0"/></inertial></link>
    </robot>
  )";

  std::string tmp = "test_no_sensors.urdf";
  std::ofstream f(tmp);
  f << urdf_content;
  f.close();

  auto mujoco        = std::make_shared<mjcf::Mujoco>();
  auto [body, joint] = mujoco->add_urdf(tmp);
  REQUIRE(body != nullptr);

  CHECK(mujoco->get_sensors(body).empty());
  CHECK(mujoco->get_sensors().empty());

  std::filesystem::remove(tmp);
}
