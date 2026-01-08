#!/usr/bin/env python3
"""
Enhance behaviors.xml with usage information from Objectives.

This script reads behaviors.xml (from REST API) and adds Metadata fields
with used_in information showing which Objectives use each behavior.

Run this script in moveit_pro_example_ws workspace.
"""

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Dict, List, Optional
from xml.etree import ElementTree as ET

try:
    import yaml
except ImportError:
    print(
        "Error: PyYAML is required. Install with: pip install pyyaml", file=sys.stderr
    )
    sys.exit(1)


def normalize_behavior_id(behavior_id: str) -> str:
    """Normalize behavior ID to match format used in XML."""
    return behavior_id.lower().replace(" ", "").replace("-", "").replace("_", "")


def discover_configs(workspace_path: Path) -> List[Dict[str, str]]:
    """
    Walk workspace, find dirs with package.xml + config/config.yaml.
    Skip COLCON_IGNORE dirs.
    """
    configs = []
    workspace_path = Path(workspace_path).resolve()

    for root, dirs, files in os.walk(workspace_path):
        # Skip build, install, log directories
        if any(skip in root for skip in ["/build/", "/install/", "/log/", "/.git/"]):
            dirs[:] = []
            continue

        # Check for COLCON_IGNORE
        if (Path(root) / "COLCON_IGNORE").exists():
            dirs[:] = []
            continue

        # Check if this directory has package.xml and config/config.yaml
        package_xml = Path(root) / "package.xml"
        config_yaml = Path(root) / "config" / "config.yaml"

        if package_xml.exists() and config_yaml.exists():
            # Skip cookiecutter templates (they contain Jinja2 syntax, not valid YAML)
            try:
                with open(config_yaml, "r") as f:
                    content = f.read()
                    if "{{" in content or "cookiecutter" in root:
                        continue
            except Exception:
                continue

            # Extract package name from package.xml
            try:
                tree = ET.parse(package_xml)
                root_elem = tree.getroot()
                name_elem = root_elem.find("name")
                if name_elem is not None:
                    package_name = name_elem.text.strip()
                    configs.append(
                        {
                            "name": package_name,
                            "path": root,
                            "config_path": str(config_yaml),
                        }
                    )
            except Exception:
                continue

    return configs


def parse_config_yaml(config_path: Path) -> Dict:
    """Load config.yaml, handle based_on_package inheritance."""
    with open(config_path, "r") as f:
        config_data = yaml.safe_load(f) or {}

    # Handle inheritance
    if "based_on_package" in config_data:
        base_package = config_data["based_on_package"]
        # Try to find base config
        workspace_path = config_path.parent.parent.parent
        base_config_path = workspace_path / base_package / "config" / "config.yaml"
        if base_config_path.exists():
            base_config = parse_config_yaml(base_config_path)
            # Merge base config with current config (current overrides base)
            merged = {**base_config, **config_data}
            return merged

    return config_data


def find_package_share_path(package_name: str, workspace_path: Path) -> Optional[Path]:
    """Find package share directory in workspace."""
    for root, dirs, files in os.walk(workspace_path):
        if "/build/" in root or "/install/" in root or "/.git/" in root:
            continue
        package_xml = Path(root) / "package.xml"
        if package_xml.exists():
            try:
                tree = ET.parse(package_xml)
                root_elem = tree.getroot()
                name_elem = root_elem.find("name")
                if name_elem is not None and name_elem.text == package_name:
                    return Path(root).resolve()
            except Exception:
                continue
    return None


def extract_behaviors_used_in_objectives(
    objective_library_paths: Dict, config_name: str, workspace_path: Path
) -> Dict[str, List[Dict]]:
    """
    For each Objective, find which behaviors are used.
    Returns mapping: behavior_id -> [list of objectives where it's used]
    """
    behavior_to_objectives: Dict[str, List[Dict]] = {}
    objective_files = []

    # Collect all objective XML files
    for library_name, library_info in objective_library_paths.items():
        package_name = library_info.get("package_name")
        relative_path = library_info.get("relative_path", "objectives")

        if not package_name:
            continue

        package_path = find_package_share_path(package_name, workspace_path)
        if not package_path:
            install_path = (
                workspace_path
                / "install"
                / package_name
                / "share"
                / package_name
                / relative_path
            )
            if install_path.exists():
                package_path = install_path.parent.parent.parent.parent / "src"
                package_path = find_package_share_path(package_name, package_path)
            if not package_path:
                continue

        objectives_dir = package_path / relative_path
        if not objectives_dir.exists():
            continue

        for xml_file in objectives_dir.rglob("*.xml"):
            objective_files.append(xml_file)

    # Parse each objective XML file
    for objective_file in objective_files:
        try:
            tree = ET.parse(objective_file)
            root = tree.getroot()

            # Find all BehaviorTree elements (Objectives)
            for behavior_tree in root.findall(".//BehaviorTree"):
                objective_id = behavior_tree.get("ID")
                if not objective_id:
                    continue

                # Find all behaviors used in this Objective
                # 1. SubTree references
                for subtree in behavior_tree.findall(".//SubTree[@ID]"):
                    behavior_id = subtree.get("ID")
                    if behavior_id:
                        behavior_id_lower = normalize_behavior_id(behavior_id)
                        if behavior_id_lower not in behavior_to_objectives:
                            behavior_to_objectives[behavior_id_lower] = []
                        behavior_to_objectives[behavior_id_lower].append(
                            {
                                "config": config_name,
                                "objective_id": objective_id,
                                "objective_file": str(
                                    objective_file.relative_to(workspace_path)
                                ),
                                "usage_type": "SubTree",
                                "source_type": "objective_usage",
                            }
                        )

                # 2. Action nodes
                for action in behavior_tree.findall(".//Action[@ID]"):
                    behavior_id = action.get("ID")
                    if behavior_id:
                        behavior_id_lower = normalize_behavior_id(behavior_id)
                        if behavior_id_lower not in behavior_to_objectives:
                            behavior_to_objectives[behavior_id_lower] = []
                        behavior_to_objectives[behavior_id_lower].append(
                            {
                                "config": config_name,
                                "objective_id": objective_id,
                                "objective_file": str(
                                    objective_file.relative_to(workspace_path)
                                ),
                                "usage_type": "Action",
                                "source_type": "objective_usage",
                            }
                        )

                # 3. Control nodes
                for control in behavior_tree.findall(".//Control[@ID]"):
                    behavior_id = control.get("ID")
                    if behavior_id:
                        behavior_id_lower = normalize_behavior_id(behavior_id)
                        if behavior_id_lower not in behavior_to_objectives:
                            behavior_to_objectives[behavior_id_lower] = []
                        behavior_to_objectives[behavior_id_lower].append(
                            {
                                "config": config_name,
                                "objective_id": objective_id,
                                "objective_file": str(
                                    objective_file.relative_to(workspace_path)
                                ),
                                "usage_type": "Control",
                                "source_type": "objective_usage",
                            }
                        )

                # 4. Decorator nodes
                for decorator in behavior_tree.findall(".//Decorator[@ID]"):
                    behavior_id = decorator.get("ID")
                    if behavior_id:
                        behavior_id_lower = normalize_behavior_id(behavior_id)
                        if behavior_id_lower not in behavior_to_objectives:
                            behavior_to_objectives[behavior_id_lower] = []
                        behavior_to_objectives[behavior_id_lower].append(
                            {
                                "config": config_name,
                                "objective_id": objective_id,
                                "objective_file": str(
                                    objective_file.relative_to(workspace_path)
                                ),
                                "usage_type": "Decorator",
                                "source_type": "objective_usage",
                            }
                        )

        except Exception as e:
            print(f"Warning: Failed to parse {objective_file}: {e}", file=sys.stderr)
            continue

    return behavior_to_objectives


def enhance_behaviors_xml(
    xml_path: Path, workspace_path: Path, output_path: Path = None
) -> None:
    """Enhance behaviors.xml with usage information."""

    # Read existing XML
    print(f"Reading {xml_path}...", file=sys.stderr)
    tree = ET.parse(xml_path)
    root = tree.getroot()

    tree_nodes_model = root.find("./TreeNodesModel")
    if tree_nodes_model is None:
        print("Error: No TreeNodesModel found in XML", file=sys.stderr)
        sys.exit(1)

    # Create a map of behavior IDs to elements
    behavior_elements = {}
    for elem in tree_nodes_model:
        behavior_id_attr = elem.get("ID")
        if behavior_id_attr:
            behavior_id = normalize_behavior_id(behavior_id_attr)
            behavior_elements[behavior_id] = elem

    print(f"Found {len(behavior_elements)} behaviors in XML", file=sys.stderr)

    # Discover configs and extract usage information
    print(f"Discovering configs in {workspace_path}...", file=sys.stderr)
    configs = discover_configs(workspace_path)
    print(f"Found {len(configs)} configs", file=sys.stderr)

    # Collect all objective usages
    all_objective_usages: Dict[str, List[Dict]] = {}

    # Process each config
    for config in configs:
        config_name = config["name"]
        config_path = Path(config["config_path"])

        print(f"Processing config: {config_name}", file=sys.stderr)

        try:
            config_data = parse_config_yaml(config_path)
            objective_library_paths = config_data.get("objectives", {}).get(
                "objective_library_paths", {}
            )

            # Extract which behaviors are used in which Objectives
            objective_usages = extract_behaviors_used_in_objectives(
                objective_library_paths,
                config_name,
                workspace_path,
            )

            # Merge into global map
            for behavior_id, usages in objective_usages.items():
                if behavior_id not in all_objective_usages:
                    all_objective_usages[behavior_id] = []
                all_objective_usages[behavior_id].extend(usages)

            print(
                f"  Found {len(objective_usages)} behaviors used in Objectives",
                file=sys.stderr,
            )

        except Exception as e:
            print(f"  Error processing {config_name}: {e}", file=sys.stderr)
            import traceback

            traceback.print_exc()
            continue

    # Add used_in Metadata to each behavior element
    print("Adding usage information to XML...", file=sys.stderr)
    behaviors_with_usage = 0

    for behavior_id, elem in behavior_elements.items():
        # Remove existing used_in Metadata if any
        metadata_fields = elem.find(".//MetadataFields")
        if metadata_fields is not None:
            for metadata in metadata_fields.findall(".//Metadata[@used_in]"):
                metadata_fields.remove(metadata)
        else:
            # Create MetadataFields if it doesn't exist
            metadata_fields = ET.SubElement(elem, "MetadataFields")

        # Add new used_in Metadata
        if behavior_id in all_objective_usages:
            usages = all_objective_usages[behavior_id]
            # Serialize usages to JSON string
            used_in_json = json.dumps(usages, ensure_ascii=False)

            # Add Metadata element with used_in attribute
            used_in_metadata = ET.SubElement(metadata_fields, "Metadata")
            used_in_metadata.set("used_in", used_in_json)

            behaviors_with_usage += 1

    print(
        f"Added usage information to {behaviors_with_usage} behaviors", file=sys.stderr
    )

    # Write enhanced XML
    output_file = output_path or xml_path
    print(f"Writing enhanced XML to {output_file}...", file=sys.stderr)

    # Pretty print XML (indent)
    ET.indent(tree, space="  ")
    tree.write(output_file, encoding="utf-8", xml_declaration=True)

    print(f"Done! Enhanced {len(behavior_elements)} behaviors.", file=sys.stderr)
    print(f"Total behaviors with usage: {behaviors_with_usage}", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(
        description="Enhance behaviors.xml with usage information from Objectives"
    )
    parser.add_argument(
        "--xml",
        type=str,
        required=True,
        help="Path to behaviors.xml file",
    )
    parser.add_argument(
        "--workspace",
        type=str,
        default=os.environ.get("USER_WS", "."),
        help="Path to ROS workspace (default: $USER_WS or current directory)",
    )
    parser.add_argument(
        "--output",
        type=str,
        help="Output XML file path (default: overwrite input file)",
    )

    args = parser.parse_args()

    xml_path = Path(args.xml).resolve()
    if not xml_path.exists():
        print(f"Error: XML file does not exist: {xml_path}", file=sys.stderr)
        sys.exit(1)

    workspace_path = Path(args.workspace).resolve()
    if not workspace_path.exists():
        print(
            f"Error: Workspace path does not exist: {workspace_path}", file=sys.stderr
        )
        sys.exit(1)

    output_path = Path(args.output).resolve() if args.output else None

    enhance_behaviors_xml(xml_path, workspace_path, output_path)


if __name__ == "__main__":
    main()
