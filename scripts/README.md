# Scripts

Utility scripts for maintaining the PickNik website, in particular the Behaviors Hub.

### How to update the Behavior Hub data source
These steps allow you to get the updated behaviors.xml, which should happen every release.

```bash
# In moveitpro repo when your REST API is running, get the generated XML data
curl -X GET http://localhost:3200/behaviors/data > behaviors_raw.xml
```

### enrich_behaviors_with_usage.py

Enhances `behaviors_raw.xml` with usage information (Metadata fields with `used_in` attribute) showing which Objectives use each behavior.

**Run this script in `moveit_pro_example_ws` workspace.**

#### Prerequisites

1. Save `behaviors_raw.xml` (downloaded from REST API) to the `scripts/` directory in `moveit_pro_example_ws` workspace.

#### Usage

```bash
# From moveit_pro_example_ws root directory
python3 scripts/enrich_behaviors_with_usage.py --xml scripts/behaviors_raw.xml --workspace . --output scripts/behaviors_with_usage.xml

# Example with custom workspace path
python3 scripts/enrich_behaviors_with_usage.py --xml scripts/behaviors_raw.xml --workspace /path/to/workspace --output scripts/behaviors_with_usage.xml
```

#### What it does

- Discovers all robot configs in the workspace
- Extracts which Objectives use which behaviors by parsing Objective XML files
- Preserves all existing XML data (ports, metadata, etc.)
- Adds `Metadata` elements with `used_in` attribute to each behavior's `MetadataFields` showing:
  - Which configs use the behavior
  - Which Objectives use the behavior
  - The file path of each Objective
  - The usage type (Action, Control, Decorator, SubTree)
- The `used_in` attribute contains a JSON array of usage objects
- Outputs enhanced XML file ready for conversion to JSON

#### Requirements

- Python 3
- PyYAML (`pip install pyyaml`)
- `behaviors_raw.xml` file (from REST API endpoint)
- `moveit_pro_example_ws` workspace with robot configs

### update_behaviors_from_xml.py

Updates `_data/behaviors.json` with port information and metadata extracted from `behaviors_with_usage.xml`.

**Run this script in the PickNik website repository.**

#### Usage

```bash
# From project root
python scripts/update_behaviors_from_xml.py [path_to_behaviors_with_usage.xml]

# Example with default path
python scripts/update_behaviors_from_xml.py

# Example with custom XML path
python scripts/update_behaviors_from_xml.py /path/to/behaviors_with_usage.xml
```

#### What it does

- Extracts port information (input, output, input/output) from XML Action and SubTree elements
- Extracts metadata (subcategory, description, deprecated status)
- Extracts `used_in` information from `Metadata` elements with `used_in` attribute (if present)
- Updates existing behaviors in JSON or adds new ones
- Preserves existing data like links and other fields
- Updates the export date

### Complete Workflow

```bash
# 1. In moveitpro repo - Get behaviors.xml from REST API and save to scripts/ directory
curl -X GET http://localhost:3200/behaviors/data > scripts/behaviors_raw.xml

# 2. In moveit_pro_example_ws root - Enhance XML with usage information
python3 scripts/enrich_behaviors_with_usage.py --xml scripts/behaviors_raw.xml --workspace . --output scripts/behaviors_with_usage.xml

# 3. In PickNik website repo - Convert to JSON
python scripts/update_behaviors_from_xml.py _data/behaviors_with_usage.xml
```
