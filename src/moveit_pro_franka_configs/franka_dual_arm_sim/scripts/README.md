# How to draw images and text

In this folder, you will find several SVG images that can be used in a dual arm configuration for Franka.

When you open one of these images, you will find a vector image embedded inside a square 20cm x 20cm centered at the origin of the document.

The square represents the area that can be reached by both Franka arms. You can draw anything, as long as it remain inside the square.

You must convert all images into paths, before exporting them to YAML format. Additionally, if you want to draw letters in the correct order, you must brake the text into subpaths and order them accordingly.

We use the command `svg_to_yaml.py` which requires an additional parameter `-d` to determine the number of generated waypoints.

Following the commands used to convert the current SVG images.

```bash
python3 svg_to_yaml.py loves.svg loves.yaml -d 50
python3 svg_to_yaml.py franka.svg franka.yaml -d 400
python3 svg_to_yaml.py moveit_pro.svg moveit_pro.yaml -d 400
python3 svg_to_yaml.py wipe.svg wipe.yaml -d 200
python3 svg_to_yaml.py wipe_zig_zag.svg wipe_zig_zag.yaml -d 200
```
