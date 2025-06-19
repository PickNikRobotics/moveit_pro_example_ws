# How to draw images and text

In this folder, you will find several images and text that can be used in a dual arm configuration for Franka.

When you open one of these SVG, you will find an image embedded inside a square 20cm x 20cm centered at the origin of the document.

The square represents the area that can be reached by both Franka arms. You can draw anything, as long as it remain inside the green area.

If not already done, you must convert all images into path, before exporting them to YAML format.

We use the command `svg_to_yaml.py` which requires an additional parameter `-d` to determine the number of generated waypoints.

Following the commands used to convert the current SVG images.

```bash
python3 svg_to_yaml.py loves.svg loves.yaml -d 50
python3 svg_to_yaml.py franka.svg franka.yaml -d 200
python3 svg_to_yaml.py moveit_pro.svg moveit_pro.yaml -d 200
```
