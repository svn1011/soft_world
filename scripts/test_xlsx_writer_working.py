#!/usr/bin/env python3

"""Offline regression checks for the dependency-free working XLSX writer."""

import tempfile
import unittest
from pathlib import Path
from zipfile import ZipFile

from xlsx_writer_working import write_xlsx


class XlsxWriterTests(unittest.TestCase):
    def test_writes_required_workbook_parts_and_values(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "soft_3_working_test.xlsx"
            write_xlsx(output, {
                "Metadata": [["key", "value"], ["robot", "soft_3_working"]],
                "Samples": [["voltage", "valid"], [0.125, True]],
            })
            self.assertTrue(output.is_file())
            with ZipFile(output) as archive:
                names = set(archive.namelist())
                self.assertIn("xl/workbook.xml", names)
                self.assertIn("xl/worksheets/sheet2.xml", names)
                sample_xml = archive.read("xl/worksheets/sheet2.xml").decode()
                self.assertIn("0.125", sample_xml)
                self.assertIn('t="b"><v>1</v>', sample_xml)


if __name__ == "__main__":
    unittest.main()
