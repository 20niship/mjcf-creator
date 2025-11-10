#pragma once

#include <string>

namespace mjcf {

enum class GeomType { Plane, Hfield, Sphere, Capsule, Ellipsoid, Cylinder, Box, Mesh };
enum class SiteType { Sphere, Capsule, Ellipsoid, Cylinder, Box };
enum class JointType { Free, Ball, Slide, Hinge };
enum class LightMode { Fixed, Track, Trackcom, Targetbody, Targetbodycom };
enum class CameraMode { Fixed, Track, Trackcom, Targetbody, Targetbodycom };
enum class IntegratorType { Euler, RK4, Implicit };
enum class SolverType { Newton, PGS, CG };
enum class CoordinateType { Local, Global };
enum class AngleUnit { Degree, Radian };
enum class ActuatorType { Motor, Position, Velocity, Cylinder, Muscle, Tendon, Damper, General, Plugin, Intvelocity };
enum class TransmissionType { Joint, JointInParent, Slide, Tendon, Site, Body };
enum class TextureType { Cube, Skybox, TwoD };
enum class TextureBuiltin { None, Gradient, Checker, Flat };

// Utility functions to convert enums to strings
std::string to_string(GeomType type);
std::string to_string(SiteType type);
std::string to_string(JointType type);
std::string to_string(LightMode mode);
std::string to_string(CameraMode mode);
std::string to_string(IntegratorType type);
std::string to_string(CoordinateType type);
std::string to_string(SolverType type);
std::string to_string(AngleUnit unit);
std::string to_string(ActuatorType type);
std::string to_string(TransmissionType type);
std::string to_string(TextureType type);
std::string to_string(TextureBuiltin builtin);

// Utility functions to convert strings to enums
GeomType geom_type_from_string(const std::string& str);
SiteType site_type_from_string(const std::string& str);
JointType joint_type_from_string(const std::string& str);
LightMode light_mode_from_string(const std::string& str);
CameraMode camera_mode_from_string(const std::string& str);
IntegratorType integrator_type_from_string(const std::string& str);
CoordinateType coordinate_type_from_string(const std::string& str);
AngleUnit angle_unit_from_string(const std::string& str);
ActuatorType actuator_type_from_string(const std::string& str);
TransmissionType transmission_type_from_string(const std::string& str);
TextureType texture_type_from_string(const std::string& str);
TextureBuiltin texture_builtin_from_string(const std::string& str);

} // namespace mjcf
