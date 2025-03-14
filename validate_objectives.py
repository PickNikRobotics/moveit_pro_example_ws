#!/usr/bin/env python3

from glob import glob
from multiprocessing import Pool
from typing import Optional
import xml.etree.ElementTree as ET
import time


def validate_objective(xml_file: str) -> Optional[str]:
    """
    Open an XML file, parse it, and see if it's an objective. If it is, validate it.
    Returns a string error message if validation fails, None otherwise.
    """
    try:
        root = ET.parse(xml_file).getroot()
        if (behavior_tree := root.find(".//BehaviorTree")) is not None:
            # It's a behavior tree, start validating
            assert (
                tree_nodes_model := root.find(".//TreeNodesModel")
            ) is not None, "TreeNodesModel not found"
            assert (
                subtree_definition := tree_nodes_model.find("SubTree")
            ) is not None, "SubTree definition not found"
            assert (
                metadata_fields := subtree_definition.find("MetadataFields")
            ) is not None, "MetadataFields not found"
            assert (
                metadata_fields.find(".//Metadata[@subcategory]") is not None
            ), "Objective subcategory not found"
            # We still support both of these kinds of description, but we need to deprecate the older one.
            assert (
                behavior_tree.attrib.get("_description") is not None
                or metadata_fields.find(".//Metadata[@description]") is not None
            ), "Objective description not found"
    except (ET.ParseError, AssertionError) as e:
        return f"Error validating {xml_file}: {str(e)}"


if __name__ == "__main__":
    start_time = time.time()
    xml_files = glob("**/*.xml", recursive=True)
    with Pool() as pool:
        results = pool.map(validate_objective, xml_files)
        errors = [result for result in results if result is not None]
        for error in errors:
            print(error)
        print(
            f"Validated {len(xml_files)} files in {time.time() - start_time:.2f} seconds."
        )
        if len(errors) > 0:
            raise SystemExit(f"Validation failed for {len(errors)} files.")
