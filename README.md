# MJCF Creator C++ Library

A C++17/20 library for creating MuJoCo MJCF (MuJoCo Modeling Language) XML files. This library is a C++ port of the Python MJCF library originally in `misc/tmp/`, providing a clean, object-oriented API for programmatically generating MJCF models.

## Features

- **Complete MJCF Element Support**: All major MJCF elements (Mujoco, Option, Asset, Worldbody, Geom, Light, etc.)
- **Type-Safe API**: Strong typing with proper C++17/20 features and enum types
- **XML Generation**: Automatic XML serialization using TinyXML2
- **Smart Pointer Management**: Clean memory management using `std::shared_ptr`
- **Comprehensive Testing**: Full test coverage using Doctest framework
- **CMake Integration**: Modern CMake build system with configurable options

## Building

### Requirements
- C++17 compatible compiler (GCC 7+, Clang 6+, MSVC 2017+)
- CMake 3.16 or later

### Build Steps

```bash
mkdir build

cd build

cmake -DBUILD_TESTS=ON -DBUILD_DEMO=ON .

# Build
make -j

# Run tests (optional)
./tests/tests

# Run demo program
./demo
```

### CMake Options

- `BUILD_TESTS`: Build test suite (default: OFF)
- `BUILD_DEMO`: Build demo program (default: ON)

## Usage

### Basic Example

```cpp
#include "mjcf/mjcf.hpp"
#include <iostream>
#include <fstream>

int main() {
    // Create main model
    auto mujoco = std::make_shared<mjcf::Mujoco>("my_model");
    
    // Configure simulation options
    mujoco->option_->integrator = mjcf::IntegratorType::RK4;
    mujoco->option_->timestep = 0.01;
    
    // Access built-in asset and worldbody containers
    auto asset = mujoco->asset_;
    auto worldbody = mujoco->worldbody_;
    
    // Create a texture
    auto texture = std::make_shared<mjcf::Texture>();
    texture->builtin = mjcf::TextureBuiltin::Checker;
    texture->name = "grid";
    texture->rgb1 = {1.0, 1.0, 1.0};
    texture->rgb2 = {0.0, 0.0, 0.0};
    texture->width = 100;
    texture->height = 100;
    texture->type = mjcf::TextureType::TwoD;
    
    // Create a material
    auto material = std::make_shared<mjcf::Material>();
    material->name = "grid_mat";
    material->texture = "grid";
    
    asset->add_children({texture, material});
    
    // Create a floor
    auto floor_geom = std::make_shared<mjcf::Geom>();
    floor_geom->name = "floor";
    floor_geom->type = mjcf::GeomType::Plane;
    floor_geom->material = "grid_mat";
    floor_geom->size = {10.0, 10.0, 0.1};
    
    worldbody->add_child(floor_geom);
    
    // Generate XML
    std::string xml = mujoco->get_xml_text();
    
    // Save to file
    std::ofstream file("model.xml");
    file << xml;
    file.close();
    
    return 0;
}
```

### Element Types

The library provides classes for all major MJCF elements:

#### Core Elements
- `Mujoco`: Root element
- `Compiler`: Compiler settings
- `Option`: Simulation options
- `Size`: Size parameters
- `Visual`, `Statistic`: Display settings
- `Default`: Default element settings
- `Custom`: Custom data container
- `Asset`: Asset definitions
- `Worldbody`: World body container
- `Actuator`, `Sensor`: Actuators and sensors
- `Contact`, `Equality`, `Tendon`: Constraints

#### Asset Elements
- `Texture`: Texture definitions
- `Material`: Material properties
- `Mesh`: 3D mesh assets
- `Hfield`: Height field terrain
- `Numeric`, `Text`, `Tuple`: Custom data elements

#### Body Elements
- `Body`: Rigid body
- `Geom`: Geometric shapes
- `Light`: Light sources  
- `Camera`: Camera definitions
- `Site`: Reference points
- `Joint`: Joint definitions
- `Inertial`: Inertial properties

### Creating Elements

Use `std::make_shared<>` to create MJCF elements:

```cpp
// Create the root model
auto model = std::make_shared<mjcf::Mujoco>("robot");

// Access built-in containers
auto option = model->option_;        // Option settings
auto asset = model->asset_;          // Asset definitions
auto worldbody = model->worldbody_;  // World body container

// Create individual elements
auto body = std::make_shared<mjcf::Body>();
auto geom = std::make_shared<mjcf::Geom>();
auto texture = std::make_shared<mjcf::Texture>();
auto material = std::make_shared<mjcf::Material>();
// ... etc
```

### Setting Attributes

All elements provide public member variables for their attributes:

```cpp
// String attributes
geom->name = "my_geom";
geom->type = mjcf::GeomType::Box;

// Numeric attributes  
geom->size = {1.0, 2.0, 3.0};  // std::array or std::vector
option->timestep = 0.01;        // Single double

// Boolean attributes
light->directional = true;
```

### Building Hierarchies

Use `add_child()` and `add_children()` to build element hierarchies:

```cpp
// Create a body with geometry and a joint
auto body = std::make_shared<mjcf::Body>();
auto geom = std::make_shared<mjcf::Geom>();
auto joint = std::make_shared<mjcf::Joint>();

body->add_children({geom, joint});

// Add the body to the world
auto mujoco = std::make_shared<mjcf::Mujoco>("model");
mujoco->worldbody_->add_child(body);
```

### Name Management Utilities

MuJoCo requires unique names for all elements. The library provides utility functions to retrieve all registered names and help avoid naming conflicts:

```cpp
auto mujoco = std::make_shared<mjcf::Mujoco>("model");

// Get names by category
auto body_names = mujoco->get_body_names();       // All body names
auto geom_names = mujoco->get_geom_names();       // All geom names
auto joint_names = mujoco->get_joint_names();     // All joint names
auto site_names = mujoco->get_site_names();       // All site names
auto camera_names = mujoco->get_camera_names();   // All camera names
auto light_names = mujoco->get_light_names();     // All light names
auto asset_names = mujoco->get_asset_names();     // All asset names (textures, materials, meshes, hfields)
auto sensor_names = mujoco->get_sensor_names();   // All sensor names
auto actuator_names = mujoco->get_actuator_names(); // All actuator names

// Get all names at once
auto all_names = mujoco->get_all_names();         // All names from all categories

// Check for name conflicts
std::string proposed_name = "my_body";
if(all_names.find(proposed_name) != all_names.end()) {
    // Name is already in use, generate alternative
    proposed_name = proposed_name + "_2";
}
```

These functions are useful when:
- Adding URDF models with potential name conflicts
- Generating unique names programmatically
- Validating model consistency
- Debugging naming issues

See `examples/name_utilities_example.cpp` for a complete usage example.

## Architecture

The library is organized into several header/source pairs:

- `element.hpp/cpp`: Base `Element` class with XML serialization
- `core_elements.hpp/cpp`: Core MJCF elements (Mujoco, Option, etc.)
- `asset_elements.hpp/cpp`: Asset-related elements (Texture, Material, etc.)
- `body_elements.hpp/cpp`: Body-related elements (Body, Geom, Light, etc.)
- `mjcf.hpp`: Main header with factory class

### External Dependencies

- **TinyXML2**: XML parsing and generation (included in `src/ext/`)
- **Doctest**: Testing framework (downloaded automatically during build)

## Testing

The library includes comprehensive tests covering:

- Base Element functionality
- All element types and their attributes
- XML generation and serialization
- Integration tests with complete models

Run tests with:
```bash
cmake -DBUILD_TESTS=ON .
make
./tests/tests
```

## Migration from Python Version

The C++ library closely mirrors the Python API from `misc/tmp/mjcf/`:

| Python | C++ |
|--------|-----|
| `e.Mujoco(model="test")` | `std::make_shared<mjcf::Mujoco>("test")` |
| `element.add_children([...])` | `element->add_children({...})` |
| `element.xml()` | `element->get_xml_text()` |
| `option.set_timestep(0.01)` | `option->timestep = 0.01` |
| `geom.set_type("box")` | `geom->type = mjcf::GeomType::Box` |

Key differences:
- Use smart pointers (`std::shared_ptr`) for element management
- Method calls use `->` instead of `.`
- Vector initialization uses `{...}` syntax for `std::array`
- Attributes are accessed as public member variables, not setter methods
- Enum types are used for type-safe attribute values (e.g., `GeomType::Box`, `IntegratorType::RK4`)

## Demo Program

A complete demo program is provided in `demo.cpp` that recreates the `gen_empty.py` example from the Python version. Build and run with:

```bash
make demo
./demo
```

This will generate `empty-demo.xml` demonstrating a complete MJCF model.

## License

This project follows the same license as the original repository.
