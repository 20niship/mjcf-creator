# Temporary File Tracking

## Overview

When converting URDF files to MJCF format with `copy_meshes=true`, the library creates temporary copies of mesh files to ensure they're available for MuJoCo to load. The `Mujoco` class now provides functionality to track and manage these temporary files.

## Why is this needed?

When using `add_urdf()` with `copy_meshes=true`:
- Mesh files referenced in the URDF are copied to a new location with hash-based filenames
- These copied files are needed while MuJoCo loads and uses the MJCF
- After loading the MJCF into MuJoCo, these temporary files can be safely deleted to free up disk space

## API Reference

### `get_temporary_files()`

```cpp
const std::vector<std::string>& get_temporary_files() const
```

Returns a list of file paths for all temporary files created during MJCF generation.

**Returns:** Vector of file paths (as strings)

**Example:**
```cpp
auto mujoco = std::make_shared<mjcf::Mujoco>("my_model");
mujoco->add_urdf("robot.urdf", "", true);  // copy_meshes=true

const auto& temp_files = mujoco->get_temporary_files();
std::cout << "Created " << temp_files.size() << " temporary files:\n";
for (const auto& file : temp_files) {
    std::cout << "  - " << file << "\n";
}
```

### `clear_temporary_files()`

```cpp
size_t clear_temporary_files()
```

Deletes all tracked temporary files and clears the internal tracking list.

**Returns:** Number of files successfully deleted

**Example:**
```cpp
auto mujoco = std::make_shared<mjcf::Mujoco>("my_model");
mujoco->add_urdf("robot.urdf", "", true);

// Generate MJCF and load into MuJoCo
std::string xml = mujoco->get_xml_text();
// ... load xml into MuJoCo ...

// Clean up temporary files
size_t deleted = mujoco->clear_temporary_files();
std::cout << "Deleted " << deleted << " temporary file(s)\n";
```

### `add_temporary_file()`

```cpp
void add_temporary_file(const std::string& filepath)
```

Manually adds a file path to the temporary file tracking list. Useful if you create additional temporary files that should be cleaned up later.

**Parameters:**
- `filepath`: Path to the temporary file to track

**Example:**
```cpp
auto mujoco = std::make_shared<mjcf::Mujoco>("my_model");

// Create a temporary config file
std::string temp_config = "/tmp/my_temp_config.txt";
std::ofstream(temp_config) << "temp data";

// Track it for cleanup
mujoco->add_temporary_file(temp_config);

// Later, clean up all tracked files
mujoco->clear_temporary_files();
```

## Usage Pattern

The typical workflow for using temporary file tracking:

```cpp
#include "mjcf/mjcf.hpp"
#include <fstream>

int main() {
    // 1. Create MJCF model
    auto mujoco = std::make_shared<mjcf::Mujoco>("robot_scene");
    
    // 2. Add URDF(s) with mesh copying enabled
    mujoco->add_urdf("robot1.urdf", "robot1", true);  // copy_meshes=true
    mujoco->add_urdf("robot2.urdf", "robot2", true);
    
    // 3. Generate MJCF XML
    std::string mjcf_xml = mujoco->get_xml_text();
    
    // 4. Save MJCF to file
    std::ofstream("scene.xml") << mjcf_xml;
    
    // 5. Load MJCF into MuJoCo (using MuJoCo's API)
    // mjModel* model = mj_loadXML("scene.xml", nullptr, nullptr, 0);
    
    // 6. After MuJoCo has loaded the model, clean up temporary files
    size_t deleted = mujoco->clear_temporary_files();
    std::cout << "Cleaned up " << deleted << " temporary mesh files\n";
    
    return 0;
}
```

## Important Notes

1. **When are files tracked?**
   - Files are automatically tracked when `add_urdf()` is called with `copy_meshes=true`
   - You can manually track files using `add_temporary_file()`

2. **When to clean up?**
   - Call `clear_temporary_files()` **after** MuJoCo has loaded the MJCF
   - If you call it before MuJoCo loads the MJCF, MuJoCo won't be able to find the mesh files

3. **Thread safety**
   - The tracking methods are not thread-safe
   - If using multiple threads, ensure proper synchronization

4. **Error handling**
   - `clear_temporary_files()` handles non-existent files gracefully
   - It will attempt to delete each file and return the count of successfully deleted files
   - Errors during deletion are logged to stderr but don't stop the cleanup process

## See Also

- Example: [examples/temporary_files_demo.cpp](../examples/temporary_files_demo.cpp)
- Tests: [tests/test_temporary_files.cpp](../tests/test_temporary_files.cpp)
- URDF Conversion: [add_urdf() documentation](../README.md#urdf-conversion)
