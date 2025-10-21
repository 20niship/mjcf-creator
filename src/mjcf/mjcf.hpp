#pragma once

#include "actuator_elements.hpp"
#include "asset_elements.hpp"
#include "body_elements.hpp"
#include "core_elements.hpp"
#include "element.hpp"
#include "enums.hpp"
#include "sensor_elements.hpp"
#include "urdf_converter.hpp"

/**
 * @file mjcf.hpp
 * @brief Main header for the MJCF C++ library
 *
 * This library provides C++ classes for creating MuJoCo MJCF XML files.
 * It follows the same design patterns as the original Python mjcf library
 * but adapted for C++17/20.
 *
 * @example
 * ```cpp
 * #include "mjcf/mjcf.hpp"
 *
 * int main() {
 *     auto mujoco = std::make_shared<mjcf::Mujoco>("empty");
 *     auto option = std::make_shared<mjcf::Option>();
 *     option->set_integrator("RK4");
 *     option->set_timestep(0.01);
 *
 *     auto asset = std::make_shared<mjcf::Asset>();
 *     auto worldbody = std::make_shared<mjcf::Worldbody>();
 *
 *     mujoco->add_children({option, asset, worldbody});
 *
 *     std::cout << mujoco->xml() << std::endl;
 *     return 0;
 * }
 * ```
 */

namespace mjcf {} // namespace mjcf