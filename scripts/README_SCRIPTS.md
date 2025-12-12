# Scripts

Utility scripts for maintaining the PickNik website.

## Get updated behaviors.xml

```bash
# In moveitpro repo when your rest api is running, run a curl command to get the generated XML data
# Save it to scripts/ directory in moveit_pro_example_ws workspace
curl -X GET http://localhost:3200/behaviors/data > scripts/behaviors.xml
```

## enhance_behaviors_xml.py

Enhances `behaviors.xml` with usage information (`<UsedIn>` elements) showing which Objectives use each behavior.

**Run this script in `moveit_pro_example_ws` workspace.**

### Prerequisites

1. Save `behaviors.xml` (downloaded from REST API) to the `scripts/` directory in `moveit_pro_example_ws` workspace.

### Usage

```bash
# From moveit_pro_example_ws root directory
python3 scripts/enhance_behaviors_xml.py --xml scripts/behaviors.xml --workspace . --output scripts/behaviors_enhanced.xml

# Example with custom workspace path
python3 scripts/enhance_behaviors_xml.py --xml scripts/behaviors.xml --workspace /path/to/workspace --output scripts/behaviors_enhanced.xml
```

### What it does

- Discovers all robot configs in the workspace
- Extracts which Objectives use which behaviors by parsing Objective XML files
- Adds `<UsedIn>` elements to each behavior in the XML showing:
  - Which configs use the behavior
  - Which Objectives use the behavior
  - The file path of each Objective
  - The usage type (Action, Control, Decorator, SubTree)
- Outputs enhanced XML file ready for conversion to JSON

### Requirements

- Python 3
- PyYAML (`pip install pyyaml`)
- `behaviors.xml` file (from REST API endpoint)
- `moveit_pro_example_ws` workspace with robot configs

## update_behaviors_from_xml.py

Updates `_data/behaviors.json` with port information and metadata extracted from `behaviors.xml`.

**Run this script in the PickNik website repository.**

### Usage

```bash
# From project root
python scripts/update_behaviors_from_xml.py [path_to_behaviors_enhanced.xml]

# Example with default path
python scripts/update_behaviors_from_xml.py

# Example with custom XML path
python scripts/update_behaviors_from_xml.py /path/to/behaviors_enhanced.xml
```

### What it does

- Extracts port information (input, output, input/output) from XML Action and SubTree elements
- Extracts metadata (subcategory, description, deprecated status)
- Extracts `used_in` information from `<UsedIn>` elements (if present)
- Updates existing behaviors in JSON or adds new ones
- Preserves existing data like links and other fields
- Updates the export date

## Complete Workflow

```bash
# 1. In moveitpro repo - Get behaviors.xml from REST API and save to scripts/ directory
curl -X GET http://localhost:3200/behaviors/data > scripts/behaviors.xml

# 2. In moveit_pro_example_ws root - Enhance XML with usage information
python3 scripts/enhance_behaviors_xml.py --xml scripts/behaviors.xml --workspace . --output scripts/behaviors_enhanced.xml

# 3. In PickNik website repo - Convert to JSON
python scripts/update_behaviors_from_xml.py _data/behaviors_enhanced.xml
```

