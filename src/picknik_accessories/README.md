# PickNik Accessories

PickNik assets for common objects and attachments.

# Using MuJoCo assets:

The `mujoco_assets` folder in this package can be used in any `picknik_mujoco_ros/MujocoSystem` based simulation using the `include` tag in MuJoCo XML.
Since MuJoCo XML files all assume that included files are relative to the main XML file (the `mujoco_model` parameter set as part of the `ros2_control` entry in your robot description), shared assets must be moved to the same directory as the top level XML file.
To achieve this without duplicating assets, we can place symlinks in the install directory of your package that point back to the `mujoco_assets` folder of this package.
Add the following to your CMakeLists.txt to add symlinks to this package's assets:

```
# Install all XML files in directory
set(PICKNIK_ACCESSORIES_SHARE_DIR
"${CMAKE_INSTALL_PREFIX}/../picknik_accessories/share/picknik_accessories/mujoco_assets/"
)
# Destination directory
set(DEST_DIR "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/description/")

install(DIRECTORY "${PICKNIK_ACCESSORIES_SHARE_DIR}"
        DESTINATION "${DEST_DIR}"
        FILES_MATCHING PATTERN "*")
```

Alternatively, if some files in this package conflict with files in your workspace, you can choose specific files to symlink:

```
# Install individual XML files
set(PICKNIK_ACCESSORIES_SHARE_DIR
"${CMAKE_INSTALL_PREFIX}/../picknik_accessories/share/picknik_accessories/mujoco_assets/"
)
# Destination directory
set(DEST_DIR "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/description/")

set(EXTERNAL_XML_FILES
  "${PICKNIK_ACCESSORIES_SHARE_DIR}/ur5e/ur5e_globals.xml"
  "${PICKNIK_ACCESSORIES_SHARE_DIR}/ur5e/ur5e.xml"
  # Add additional files here if desired
)
foreach(xml_file IN LISTS EXTERNAL_XML_FILES)
  install(FILES "${xml_file}"
          DESTINATION "${DEST_DIR}"
  )
endforeach()
```

Whether you chose to install all assets or just a selection, the above CMake code will merge the contents of `picknik_accessories/mujoco_assets/` with `your_package_name/description/` in the install folder, so that XML files in the `picknik_accessories` package can be included as if they were within your package.

# UR5e Example

Below is a minimal example using the UR5e asset:

```
<mujoco model="ur5e scene">
  <include file="ur5e/ur5e_globals.xml" />

  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true" />

    <geom
      name="floor"
      pos="0 0 0"
      size="0 0 0.05"
      type="plane"
      material="groundplane"
    />

    <include file="ur5e/ur5e.xml" />
  </worldbody>

</mujoco>
```

Note: Since you cannot define global properties within a `worldbody` tag, the assets must be divided in to two XML files.
The `ur5e_globals.xml` file contains properties that must be defined as global properties outside `worldbody` (like `default` and `actuator`), while the `ur5e.xml` contains only `body` tags (and XML tags that are allowed within a `body`).