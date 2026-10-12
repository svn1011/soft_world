#!/usr/bin/env python3

"""Remove selected optional columns from a SoftWorld Sensor_Data worksheet."""

import argparse
from collections import OrderedDict
from pathlib import Path
import re
from xml.etree import ElementTree
from zipfile import ZipFile

from xlsx_writer_working import write_xlsx


REMOVED_FIELDS = {
    "bridge_valid",
    "measured_theta_deg",
    "repetition",
    "case_label",
    "sample_number_in_case",
    "raw_sample_number",
    "recorded_at_utc",
    "elapsed_s",
}
MAIN_NS = "http://schemas.openxmlformats.org/spreadsheetml/2006/main"


def column_index(reference):
    letters = re.match(r"[A-Z]+", reference).group(0)
    value = 0
    for letter in letters:
        value = value * 26 + ord(letter) - ord("A") + 1
    return value - 1


def cell_value(cell):
    cell_type = cell.get("t")
    if cell_type == "inlineStr":
        return "".join(cell.itertext())
    value = cell.findtext(f"{{{MAIN_NS}}}v")
    if value is None:
        return None
    if cell_type == "b":
        return value == "1"
    try:
        number = float(value)
        return int(number) if number.is_integer() else number
    except ValueError:
        return value


def read_workbook(path):
    sheets = OrderedDict()
    with ZipFile(path) as archive:
        workbook = ElementTree.fromstring(archive.read("xl/workbook.xml"))
        sheet_nodes = workbook.find(f"{{{MAIN_NS}}}sheets")
        for sheet_number, sheet_node in enumerate(sheet_nodes, start=1):
            name = sheet_node.get("name")
            root = ElementTree.fromstring(
                archive.read(f"xl/worksheets/sheet{sheet_number}.xml")
            )
            rows = []
            for row_node in root.findall(f".//{{{MAIN_NS}}}row"):
                values = []
                for cell in row_node.findall(f"{{{MAIN_NS}}}c"):
                    position = column_index(cell.get("r"))
                    if position >= len(values):
                        values.extend([None] * (position + 1 - len(values)))
                    values[position] = cell_value(cell)
                rows.append(values)
            sheets[name] = rows
    return sheets


def compact_sensor_data(sheets):
    rows = sheets.get("Sensor_Data")
    if not rows:
        raise ValueError("Sensor_Data worksheet is missing or empty")
    keep = [index for index, name in enumerate(rows[0]) if name not in REMOVED_FIELDS]
    removed = [name for name in rows[0] if name in REMOVED_FIELDS]
    if set(removed) != REMOVED_FIELDS:
        missing = sorted(REMOVED_FIELDS.difference(removed))
        raise ValueError(f"Expected removable columns are missing: {missing}")
    sheets["Sensor_Data"] = [
        [row[index] if index < len(row) else None for index in keep]
        for row in rows
    ]
    return removed


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    sheets = read_workbook(args.input)
    removed = compact_sensor_data(sheets)
    write_xlsx(args.output, sheets)
    print(f"Removed {len(removed)} fields: {', '.join(removed)}")
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
