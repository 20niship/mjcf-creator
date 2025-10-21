# MJCF Creator C++ Library

A C++17/20 library for creating MuJoCo MJCF (MuJoCo Modeling Language) XML files. This library is a C++ port of the Python MJCF library originally in `misc/tmp/`, providing a clean, object-oriented API for programmatically generating MJCF models.

## Features

- **Complete MJCF Element Support**: All major MJCF elements (Mujoco, Option, Asset, Worldbody, Geom, Light, etc.)
- **Type-Safe API**: Strong typing with proper C++17/20 features
- **XML Generation**: Automatic XML serialization using TinyXML2
- **Factory Pattern**: Convenient factory methods for creating elements
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
    auto mujoco = mjcf::Elements::mujoco("my_model");
    
    // Add simulation options
    auto option = mjcf::Elements::option();
    option->set_integrator("RK4");
    option->set_timestep(0.01);
    
    // Create asset container
    auto asset = mjcf::Elements::asset();
    
    // Create world body
    auto worldbody = mjcf::Elements::worldbody();
    
    // Add to model
    mujoco->add_children({option, asset, worldbody});
    
    // Create a texture
    auto texture = mjcf::Elements::texture();
    texture->set_builtin("checker");
    texture->set_name("grid");
    texture->set_rgb1({1.0, 1.0, 1.0});
    texture->set_rgb2({0.0, 0.0, 0.0});
    
    // Create a material
    auto material = mjcf::Elements::material();
    material->set_name("grid_mat");
    material->set_texture("grid");
    
    asset->add_children({texture, material});
    
    // Create a floor
    auto floor_geom = mjcf::Elements::geom();
    floor_geom->set_name("floor");
    floor_geom->set_type("plane");
    floor_geom->set_material("grid_mat");
    floor_geom->set_size({10.0, 10.0, 0.1});
    
    worldbody->add_child(floor_geom);
    
    // Generate XML
    std::string xml = mujoco->xml();
    
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

### Factory Methods

Use the `mjcf::Elements` factory class for convenient object creation:

```cpp
auto model = mjcf::Elements::mujoco("robot");
auto option = mjcf::Elements::option();
auto body = mjcf::Elements::body();
auto geom = mjcf::Elements::geom();
// ... etc
```

### Setting Attributes

All elements provide setter methods for their attributes:

```cpp
// String attributes
geom->set_name("my_geom");
geom->set_type("box");

// Numeric attributes  
geom->set_size({1.0, 2.0, 3.0});  // Vector of doubles
option->set_timestep(0.01);        // Single double

// Boolean attributes
light->set_directional(true);
```

### Building Hierarchies

Use `add_child()` and `add_children()` to build element hierarchies:

```cpp
auto body = mjcf::Elements::body();
auto geom = mjcf::Elements::geom();
auto joint = mjcf::Elements::joint();

body->add_children({geom, joint});

auto worldbody = mjcf::Elements::worldbody();
worldbody->add_child(body);
```

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
| `e.Mujoco(model="test")` | `mjcf::Elements::mujoco("test")` |
| `element.add_children([...])` | `element->add_children({...})` |
| `element.xml()` | `element->xml()` |

Key differences:
- Use smart pointers (`std::shared_ptr`) for element management
- Method calls use `->` instead of `.`
- Vector initialization uses `{...}` syntax
- Factory methods provide convenient object creation

## Demo Program

A complete demo program is provided in `demo.cpp` that recreates the `gen_empty.py` example from the Python version. Build and run with:

```bash
make demo
./demo
```

This will generate `empty-demo.xml` demonstrating a complete MJCF model.

## License

This project follows the same license as the original repository.
