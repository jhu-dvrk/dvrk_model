"""Run with ROS sourced: python3 -m unittest discover -s scripts/tests."""

import shutil
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch

import xacro
import yaml


class InstrumentAssemblyTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        source = Path(__file__).resolve().parents[2]
        shutil.copytree(source / "urdf", self.root / "urdf")
        self.parts = self.root / "urdf/common/instruments"
        self.table_path = self.parts / "instruments.yaml"
        self.table = yaml.safe_load(self.table_path.read_text())
        self.lookup = patch(
            "ament_index_python.packages.get_package_share_directory",
            return_value=str(self.root),
        )
        self.lookup.start()
        self.addCleanup(self.lookup.stop)

    def expand(self, instrument="420006"):
        self.table_path.write_text(yaml.safe_dump(self.table))
        return ET.fromstring(xacro.process_file(
            str(self.root / "urdf/Si/PSM1.urdf.xacro"),
            mappings={"instrument": instrument},
        ).toxml())

    def test_new_parts_are_selected_without_dispatch_changes(self):
        # A new roll and tip require only their files and a YAML mapping.
        roll = (self.parts / "roll/roll_4670.urdf.xacro").read_text()
        (self.parts / "roll/roll_4321.urdf.xacro").write_text(
            roll.replace("4670", "4321").replace("0.4670", "0.4321")
        )
        tip = (self.parts / "tip/tip_006.urdf.xacro").read_text()
        (self.parts / "tip/tip_999.urdf.xacro").write_text(
            tip.replace("tip_006", "tip_999").replace(
                'name="${prefix}jaw_link"', 'name="${prefix}custom_jaw_link"'
            )
        )
        self.table["instruments"]["429999"] = dict(
            self.table["instruments"]["420006"], roll="4321", tip="999"
        )
        urdf = self.expand("429999")
        roll_origin = urdf.find("joint[@name='roll']/origin")
        self.assertEqual(float(roll_origin.get("xyz").split()[2]), 0.4321)
        self.assertIsNotNone(urdf.find("link[@name='PSM1_custom_jaw_link']"))

    def test_placeholder_is_explicit(self):
        self.table["instruments"]["420006"]["tip"] = "placeholder"
        urdf = self.expand()
        self.assertIsNotNone(urdf.find("link[@name='PSM1_jaw_link']/visual/geometry/box"))

    def test_missing_part_fails_instead_of_using_placeholder(self):
        self.table["instruments"]["420006"]["tip"] = "999"
        with self.assertRaises(xacro.XacroException) as error:
            self.expand()
        self.assertIn("tip_999.urdf.xacro", str(error.exception))


if __name__ == "__main__":
    unittest.main()
