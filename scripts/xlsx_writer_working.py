#!/usr/bin/env python3

"""Small dependency-free XLSX writer for SoftWorld research datasets."""

from datetime import date, datetime
import math
from pathlib import Path
from xml.sax.saxutils import escape
from zipfile import ZIP_DEFLATED, ZipFile


def _column_name(index):
    result = ""
    while index:
        index, remainder = divmod(index - 1, 26)
        result = chr(65 + remainder) + result
    return result


def _xml_text(value):
    text = str(value)
    preserve = ' xml:space="preserve"' if text[:1].isspace() or text[-1:].isspace() else ""
    return f"<t{preserve}>{escape(text)}</t>"


def _cell_xml(reference, value, header=False):
    style = ' s="1"' if header else ""
    if value is None:
        return ""
    if isinstance(value, bool):
        return f'<c r="{reference}"{style} t="b"><v>{int(value)}</v></c>'
    if isinstance(value, (int, float)):
        number = float(value)
        if not math.isfinite(number):
            return ""
        return f'<c r="{reference}"{style}><v>{value}</v></c>'
    if isinstance(value, (datetime, date)):
        value = value.isoformat()
    return (
        f'<c r="{reference}"{style} t="inlineStr"><is>'
        f'{_xml_text(value)}</is></c>'
    )


def _sheet_xml(rows):
    row_xml = []
    maximum_columns = 1
    for row_index, row in enumerate(rows, start=1):
        maximum_columns = max(maximum_columns, len(row))
        cells = []
        for column_index, value in enumerate(row, start=1):
            reference = f"{_column_name(column_index)}{row_index}"
            cell = _cell_xml(reference, value, header=row_index == 1)
            if cell:
                cells.append(cell)
        row_xml.append(f'<row r="{row_index}">{"".join(cells)}</row>')
    last_cell = f"{_column_name(maximum_columns)}{max(1, len(rows))}"
    auto_filter = ""
    if rows and rows[0]:
        auto_filter = f'<autoFilter ref="A1:{_column_name(len(rows[0]))}{len(rows)}"/>'
    return (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main">'
        f'<dimension ref="A1:{last_cell}"/>'
        '<sheetViews><sheetView workbookViewId="0">'
        '<pane ySplit="1" topLeftCell="A2" activePane="bottomLeft" state="frozen"/>'
        '</sheetView></sheetViews>'
        '<sheetFormatPr defaultRowHeight="15"/>'
        f'<sheetData>{"".join(row_xml)}</sheetData>{auto_filter}'
        '</worksheet>'
    )


def write_xlsx(path, sheets):
    """Write ``{sheet_name: rows}`` to an Excel-compatible XLSX workbook."""
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    items = list(sheets.items())
    if not items:
        raise ValueError("At least one worksheet is required")

    sheet_entries = []
    relationship_entries = []
    content_overrides = []
    for index, (name, _) in enumerate(items, start=1):
        safe_name = str(name)[:31]
        sheet_entries.append(
            f'<sheet name="{escape(safe_name)}" sheetId="{index}" r:id="rId{index}"/>'
        )
        relationship_entries.append(
            '<Relationship '
            f'Id="rId{index}" '
            'Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" '
            f'Target="worksheets/sheet{index}.xml"/>'
        )
        content_overrides.append(
            f'<Override PartName="/xl/worksheets/sheet{index}.xml" '
            'ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>'
        )

    styles_relationship_id = len(items) + 1
    relationship_entries.append(
        '<Relationship '
        f'Id="rId{styles_relationship_id}" '
        'Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" '
        'Target="styles.xml"/>'
    )

    content_types = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
        '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
        '<Default Extension="xml" ContentType="application/xml"/>'
        '<Override PartName="/xl/workbook.xml" '
        'ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/>'
        + "".join(content_overrides)
        + '<Override PartName="/xl/styles.xml" '
        'ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.styles+xml"/>'
        '</Types>'
    )
    root_relationships = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
        '<Relationship Id="rId1" '
        'Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" '
        'Target="xl/workbook.xml"/>'
        '</Relationships>'
    )
    workbook = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<workbook xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" '
        'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">'
        f'<sheets>{"".join(sheet_entries)}</sheets></workbook>'
    )
    workbook_relationships = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
        f'{"".join(relationship_entries)}</Relationships>'
    )
    styles = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<styleSheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main">'
        '<fonts count="2"><font><sz val="11"/><name val="Calibri"/></font>'
        '<font><b/><color rgb="FFFFFFFF"/><sz val="11"/><name val="Calibri"/></font></fonts>'
        '<fills count="3"><fill><patternFill patternType="none"/></fill>'
        '<fill><patternFill patternType="gray125"/></fill>'
        '<fill><patternFill patternType="solid"><fgColor rgb="FF500000"/>'
        '<bgColor indexed="64"/></patternFill></fill></fills>'
        '<borders count="1"><border><left/><right/><top/><bottom/><diagonal/></border></borders>'
        '<cellStyleXfs count="1"><xf numFmtId="0" fontId="0" fillId="0" borderId="0"/></cellStyleXfs>'
        '<cellXfs count="2"><xf numFmtId="0" fontId="0" fillId="0" borderId="0" xfId="0"/>'
        '<xf numFmtId="0" fontId="1" fillId="2" borderId="0" xfId="0" applyFont="1" applyFill="1"/></cellXfs>'
        '<cellStyles count="1"><cellStyle name="Normal" xfId="0" builtinId="0"/></cellStyles>'
        '</styleSheet>'
    )

    with ZipFile(destination, "w", ZIP_DEFLATED) as archive:
        archive.writestr("[Content_Types].xml", content_types)
        archive.writestr("_rels/.rels", root_relationships)
        archive.writestr("xl/workbook.xml", workbook)
        archive.writestr("xl/_rels/workbook.xml.rels", workbook_relationships)
        archive.writestr("xl/styles.xml", styles)
        for index, (_, rows) in enumerate(items, start=1):
            archive.writestr(f"xl/worksheets/sheet{index}.xml", _sheet_xml(rows))

    return destination
