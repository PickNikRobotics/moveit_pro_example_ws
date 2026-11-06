# Copyright 2026 PickNik Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Check every market Objective against the installed Behavior loader manifests."""

import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from ament_index_python.packages import get_package_prefix
from moveit_studio_utils_py.system_config import load_system_config

PACKAGE = Path(__file__).resolve().parents[1]
NODE_KINDS = {"Action", "Condition", "Control", "Decorator", "SubTree"}


@pytest.fixture(scope="module")
def installed_nodes(tmp_path_factory: pytest.TempPathFactory) -> dict[str, ET.Element]:
    """Generate the catalog from this config's actual installed plugins, not a saved snapshot."""
    directory = tmp_path_factory.mktemp("market_behavior_manifest")
    workspace = directory / "workspace"
    workspace.mkdir()
    config = load_system_config("market_sim", workspace)[0]
    prefix = Path(get_package_prefix("market_sim"))
    exporter = prefix / "lib/market_sim/market_sim_test_behavior_manifest"
    manifest = directory / "nodes.xml"
    subprocess.run(
        [
            str(exporter),
            str(manifest),
            *sorted(
                {
                    loader
                    for loaders in config.objectives.behavior_loader_plugins.values()
                    for loader in loaders
                }
            ),
        ],
        check=True,
        capture_output=True,
        text=True,
        timeout=30,
    )
    model = ET.parse(manifest).find("TreeNodesModel")
    assert model is not None and len(model), "Installed Behavior manifest is empty"
    nodes = {node.attrib["ID"]: node for node in model}
    # Runtime Objective libraries may override installed trees by ID; local definitions win.
    for location in config.objectives.objective_library_paths.values():
        for path in sorted(location.share_path.rglob("*.xml")):
            for tree in ET.parse(path).findall("BehaviorTree"):
                nodes[tree.attrib["ID"]] = ET.Element("SubTree", ID=tree.attrib["ID"])
    return nodes


def test_undock_ports_match_installed_manifest(
    installed_nodes: dict[str, ET.Element],
) -> None:
    """The retreat and cleanup must use port names exported by their installed Behaviors."""
    root = ET.parse(PACKAGE / "objectives/undock_from_d4.xml")
    for tree in root.findall("BehaviorTree"):
        for node in tree.iter():
            if node is tree:
                continue
            model = installed_nodes.get(node.get("ID"))
            assert model is not None, f"Unknown undock Behavior: {node.get('ID')}"
            ports = {port.get("name") for port in model if port.tag.endswith("_port")}
            authored = {
                name
                for name in node.attrib
                if name not in ("ID", "name") and not name.startswith("_")
            }
            assert (
                authored <= ports
            ), f"{node.get('ID')}: unknown ports {sorted(authored - ports)}"


@pytest.mark.parametrize(
    "objective",
    sorted((PACKAGE / "objectives").glob("*.xml")),
    ids=lambda path: path.name,
)
def test_regression_objective_node_ids_are_installed(
    objective: Path, installed_nodes: dict[str, ET.Element]
) -> None:
    """Every executable node must have an installed Behavior or a declared Objective ID."""
    missing = []
    for tree in ET.parse(objective).findall("BehaviorTree"):
        for node in tree.iter():
            if node is tree:
                continue
            node_id = node.get("ID") if node.tag in NODE_KINDS else node.tag
            expected_kind = node.tag if node.tag in NODE_KINDS else None
            model = installed_nodes.get(node_id)
            if model is None or (
                expected_kind is not None and model.tag != expected_kind
            ):
                missing.append(f"{node.tag} {node_id}")
    assert (
        not missing
    ), f"{objective.name}: unknown or wrong-kind nodes: {', '.join(missing)}"
