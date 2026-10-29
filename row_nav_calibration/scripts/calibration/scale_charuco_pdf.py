#!/usr/bin/env python3
"""Scale a one-page calibration-board PDF to a target ISO paper size."""

from __future__ import annotations

import argparse
from io import BytesIO
from pathlib import Path

from pypdf import PdfReader, PdfWriter
from pypdf.generic import RectangleObject
from reportlab.pdfgen import canvas


ISO_PAGE_MM = {
    "A1": (594.0, 841.0),
    "A2": (420.0, 594.0),
    "A3": (297.0, 420.0),
}
MM_TO_PT = 72.0 / 25.4


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--paper", choices=ISO_PAGE_MM, default="A2")
    args = parser.parse_args()

    reader = PdfReader(args.source)
    if len(reader.pages) != 1:
        raise ValueError("calibration-board source must contain exactly one page")

    page = reader.pages[0]
    target_mm = ISO_PAGE_MM[args.paper]
    target_width = target_mm[0] * MM_TO_PT
    target_height = target_mm[1] * MM_TO_PT

    source_width = float(page.mediabox.width)
    source_height = float(page.mediabox.height)
    scale = min(target_width / source_width, target_height / source_height)
    page.add_transformation([scale, 0, 0, scale, 0, 0])

    scaled_width = source_width * scale
    scaled_height = source_height * scale
    offset_x = (target_width - scaled_width) / 2.0
    offset_y = (target_height - scaled_height) / 2.0
    if offset_x or offset_y:
        page.add_transformation([1, 0, 0, 1, offset_x, offset_y])

    page.mediabox = RectangleObject((0, 0, target_width, target_height))
    page.cropbox = RectangleObject((0, 0, target_width, target_height))

    # The source includes a physical scale check. Scaling the page also scales
    # that line, so replace it with a true 100 mm line on the target page.
    overlay_stream = BytesIO()
    overlay = canvas.Canvas(overlay_stream, pagesize=(target_width, target_height))
    footer_height = 8.0 * MM_TO_PT
    line_y = 4.0 * MM_TO_PT
    line_x = 12.0 * MM_TO_PT
    line_length = 100.0 * MM_TO_PT
    tick_height = 2.0 * MM_TO_PT
    overlay.setFillColorRGB(1, 1, 1)
    overlay.rect(0, 0, target_width, footer_height, fill=1, stroke=0)
    overlay.setStrokeColorRGB(0, 0, 0)
    overlay.setLineWidth(0.35)
    overlay.line(line_x, line_y, line_x + line_length, line_y)
    overlay.line(line_x, line_y - tick_height / 2, line_x, line_y + tick_height / 2)
    overlay.line(
        line_x + line_length,
        line_y - tick_height / 2,
        line_x + line_length,
        line_y + tick_height / 2,
    )
    overlay.setFillColorRGB(0, 0, 0)
    overlay.setFont("Helvetica", 5)
    overlay.drawString(line_x + line_length + 3.0 * MM_TO_PT, line_y - 1.5, "100 mm print-scale check")
    overlay.save()
    overlay_stream.seek(0)
    page.merge_page(PdfReader(overlay_stream).pages[0])

    writer = PdfWriter()
    writer.add_page(page)
    writer.add_metadata(
        {
            "/Title": f"CMD108 ChArUco 7x10 {args.paper} calibration board",
            "/Author": "RK Crop Row Navigation Project",
            "/Subject": (
                f"Print on {args.paper} at 100% actual size; retain the white border; "
                "measure square and marker dimensions before calibration"
            ),
        }
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("wb") as stream:
        writer.write(stream)


if __name__ == "__main__":
    main()
