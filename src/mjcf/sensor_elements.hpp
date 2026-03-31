#pragma once

#include "element.hpp"
#include "enums.hpp"
#include <array>

namespace mjcf {

class BaseSensor : public Element {
public:
  std::string name;
  double noise               = 0.0;
  double cutoff              = 0.0;
  int nsample                = 0;
  std::string interp         = "";
  double delay               = 0.0;
  double interval            = 0.0;
  std::array<double, 3> user = {0.0, 0.0, 0.0};

  BaseSensor(const std::string& element_name);

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;

private:
  std::string element_name_;
};

/**
 * @brief Joint position sensor
 */
class JointPos : public BaseSensor {
public:
  std::string joint;

  JointPos();
  std::string element_name() const override { return "jointpos"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Joint velocity sensor
 */
class JointVel : public BaseSensor {
public:
  std::string joint;

  JointVel();

  std::string element_name() const override { return "jointvel"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Tendon position sensor
 */
class TendonPos : public BaseSensor {
public:
  std::string tendon = "";

  TendonPos();

  std::string element_name() const override { return "tendonpos"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Tendon velocity sensor
 */
class TendonVel : public BaseSensor {
public:
  std::string tendon = "";

  TendonVel();

  std::string element_name() const override { return "tendonvel"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Actuator position sensor
 */
class ActuatorPos : public BaseSensor {
public:
  std::string actuator = "";

  ActuatorPos();

  std::string element_name() const override { return "actuatorpos"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Actuator velocity sensor
 */
class ActuatorVel : public BaseSensor {
public:
  std::string actuator = "";

  ActuatorVel();

  std::string element_name() const override { return "actuatorvel"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Actuator force sensor
 */
class ActuatorFrc : public BaseSensor {
public:
  std::string actuator = "";

  ActuatorFrc();

  std::string element_name() const override { return "actuatorfrc"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Ball joint position sensor
 */
class BallQuat : public BaseSensor {
public:
  std::string joint = "";

  BallQuat();

  std::string element_name() const override { return "ballquat"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Ball joint angular velocity sensor
 */
class BallAngVel : public BaseSensor {
public:
  std::string joint = "";

  BallAngVel();

  std::string element_name() const override { return "ballangvel"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Site position sensor
 */
class SitePos : public BaseSensor {
public:
  std::string reftype = "body"; // MuJoCo default
  std::string objtype = "body"; // MuJoCo default
  std::string objname = "";
  std::string refname = "";

  SitePos();

  std::string element_name() const override { return "framepos"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Site orientation sensor
 */
class SiteQuat : public BaseSensor {
public:
  std::string reftype = "body"; // MuJoCo default
  std::string objtype = "body"; // MuJoCo default
  std::string objname = "";
  std::string refname = "";

  SiteQuat();

  std::string element_name() const override { return "framequat"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Linear velocity sensor
 */
class SiteLinVel : public BaseSensor {
public:
  std::string reftype = "body"; // MuJoCo default
  std::string objtype = "body"; // MuJoCo default
  std::string objname = "";
  std::string refname = "";

  SiteLinVel();

  std::string element_name() const override { return "framelinvel"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Angular velocity sensor
 */
class SiteAngVel : public BaseSensor {
public:
  std::string reftype = "body"; // MuJoCo default
  std::string objtype = "body"; // MuJoCo default
  std::string objname = "";
  std::string refname = "";

  SiteAngVel();

  std::string element_name() const override { return "frameangvel"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Subtree center of mass position sensor
 */
class SubtreeCom : public BaseSensor {
public:
  std::string body = "";

  SubtreeCom();

  std::string element_name() const override { return "subtreecom"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Subtree linear momentum sensor
 */
class SubtreeLinVel : public BaseSensor {
public:
  std::string body = "";

  SubtreeLinVel();

  std::string element_name() const override { return "subtreelinvel"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Subtree angular momentum sensor
 */
class SubtreeAngMom : public BaseSensor {
public:
  std::string body = "";

  SubtreeAngMom();

  std::string element_name() const override { return "subtreeangmom"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Gyroscope sensor
 */
class Gyro : public BaseSensor {
public:
  std::string site = "";

  Gyro();

  std::string element_name() const override { return "gyro"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Accelerometer sensor
 */
class Accelerometer : public BaseSensor {
public:
  std::string site = "";

  Accelerometer();

  std::string element_name() const override { return "accelerometer"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Magnetometer sensor
 */
class Magnetometer : public BaseSensor {
public:
  std::string site = "";

  Magnetometer();

  std::string element_name() const override { return "magnetometer"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Range finder sensor
 */
class Rangefinder : public BaseSensor {
public:
  std::string site = "";

  Rangefinder();

  std::string element_name() const override { return "rangefinder"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Force sensor
 */
class Force : public BaseSensor {
public:
  std::string site = "";

  Force();

  std::string element_name() const override { return "force"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Torque sensor
 */
class Torque : public BaseSensor {
public:
  std::string site = "";

  Torque();

  std::string element_name() const override { return "torque"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief Touch sensor
 */
class Touch : public BaseSensor {
public:
  std::string site = "";

  Touch();

  std::string element_name() const override { return "touch"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

// ===== フェーズ1: 優先度高 =====

/**
 * @brief 速度センサー（サイト速度測定）
 *
 * 指定されたサイトの速度を測定するセンサー。
 */
class Velocimeter : public BaseSensor {
public:
  std::string site = "";

  Velocimeter();

  std::string element_name() const override { return "velocimeter"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief クロックセンサー（シミュレーション時刻測定）
 *
 * シミュレーションの経過時間を測定するセンサー。
 */
class Clock : public BaseSensor {
public:
  Clock();

  std::string element_name() const override { return "clock"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief フレーム線形加速度センサー
 *
 * 指定されたフレームの線形加速度を測定するセンサー。
 */
class FrameLinAcc : public BaseSensor {
public:
  std::string objtype = "body"; // MuJoCo default
  std::string objname = "";

  FrameLinAcc();

  std::string element_name() const override { return "framelinacc"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief フレーム角加速度センサー
 *
 * 指定されたフレームの角加速度を測定するセンサー。
 */
class FrameAngAcc : public BaseSensor {
public:
  std::string objtype = "body"; // MuJoCo default
  std::string objname = "";

  FrameAngAcc();

  std::string element_name() const override { return "frameangacc"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief ポテンシャルエネルギーセンサー
 *
 * システム全体のポテンシャルエネルギーを測定するセンサー。
 */
class PotentialEnergy : public BaseSensor {
public:
  PotentialEnergy();

  std::string element_name() const override { return "e_potential"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief 運動エネルギーセンサー
 *
 * システム全体の運動エネルギーを測定するセンサー。
 */
class KineticEnergy : public BaseSensor {
public:
  KineticEnergy();

  std::string element_name() const override { return "e_kinetic"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief ジョイントアクチュエータ力センサー
 *
 * 指定されたジョイントにかかるアクチュエータ力を測定するセンサー。
 */
class JointActuatorFrc : public BaseSensor {
public:
  std::string joint = "";

  JointActuatorFrc();

  std::string element_name() const override { return "jointactuatorfrc"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief 腱アクチュエータ力センサー
 *
 * 指定された腱にかかるアクチュエータ力を測定するセンサー。
 */
class TendonActuatorFrc : public BaseSensor {
public:
  std::string tendon = "";

  TendonActuatorFrc();

  std::string element_name() const override { return "tendonactuatorfrc"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

// ===== フェーズ2: 優先度中 =====

/**
 * @brief ジョイント位置リミットセンサー
 *
 * 指定されたジョイントの位置限界値を監視するセンサー。
 */
class JointLimitPos : public BaseSensor {
public:
  std::string joint = "";

  JointLimitPos();

  std::string element_name() const override { return "jointlimitpos"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief ジョイント速度リミットセンサー
 *
 * 指定されたジョイントの速度限界値を監視するセンサー。
 */
class JointLimitVel : public BaseSensor {
public:
  std::string joint = "";

  JointLimitVel();

  std::string element_name() const override { return "jointlimitvel"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief ジョイント力リミットセンサー
 *
 * 指定されたジョイントの力限界値を監視するセンサー。
 */
class JointLimitFrc : public BaseSensor {
public:
  std::string joint = "";

  JointLimitFrc();

  std::string element_name() const override { return "jointlimitfrc"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief 腱位置リミットセンサー
 *
 * 指定された腱の位置限界値を監視するセンサー。
 */
class TendonLimitPos : public BaseSensor {
public:
  std::string tendon = "";

  TendonLimitPos();

  std::string element_name() const override { return "tendonlimitpos"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief 腱速度リミットセンサー
 *
 * 指定された腱の速度限界値を監視するセンサー。
 */
class TendonLimitVel : public BaseSensor {
public:
  std::string tendon = "";

  TendonLimitVel();

  std::string element_name() const override { return "tendonlimitvel"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief 腱力リミットセンサー
 *
 * 指定された腱の力限界値を監視するセンサー。
 */
class TendonLimitFrc : public BaseSensor {
public:
  std::string tendon = "";

  TendonLimitFrc();

  std::string element_name() const override { return "tendonlimitfrc"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief 距離センサー
 *
 * 2つのジオメトリまたはボディ間の距離を測定するセンサー。
 * geom1/geom2 または body1/body2 のいずれかを指定可能。
 */
class Distance : public BaseSensor {
public:
  std::string geom1 = "";
  std::string geom2 = "";
  std::string body1 = "";
  std::string body2 = "";

  Distance();

  std::string element_name() const override { return "distance"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief 法線センサー
 *
 * 2つのジオメトリまたはボディ間の法線ベクトルを測定するセンサー。
 * geom1/geom2 または body1/body2 のいずれかを指定可能。
 */
class Normal : public BaseSensor {
public:
  std::string geom1 = "";
  std::string geom2 = "";
  std::string body1 = "";
  std::string body2 = "";

  Normal();

  std::string element_name() const override { return "normal"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief FromToセンサー
 *
 * 2つのジオメトリまたはボディ間のベクトルを測定するセンサー。
 * geom1/geom2 または body1/body2 のいずれかを指定可能。
 */
class FromTo : public BaseSensor {
public:
  std::string geom1 = "";
  std::string geom2 = "";
  std::string body1 = "";
  std::string body2 = "";

  FromTo();

  std::string element_name() const override { return "fromto"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief 接触センサー
 *
 * システム内の接触フラグを検出するセンサー。
 */
class Contact : public BaseSensor {
public:
  Contact();

  std::string element_name() const override { return "contact"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

/**
 * @brief 触覚センサー
 *
 * 指定されたジオメトリの接触力を測定するセンサー。
 */
class Tactile : public BaseSensor {
public:
  std::string geom = "";

  Tactile();

  std::string element_name() const override { return "tactile"; }

  void set_xml_attrib() const override;

protected:
  bool is_default_value(const std::string& name, const AttributeValue& value) const override;
};

} // namespace mjcf
