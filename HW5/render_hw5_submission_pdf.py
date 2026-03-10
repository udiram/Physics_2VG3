import argparse
import html
import re
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import landscape, letter
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import inch
from reportlab.platypus import Paragraph, Preformatted, SimpleDocTemplate, Spacer


HEADING_RE = re.compile(r"^(#{1,3})\s+(.*)$")
LIST_RE = re.compile(r"^(- |\d+\.\s+)(.*)$")


def parse_blocks(text: str) -> list[tuple[str, str | list[str]]]:
    blocks: list[tuple[str, str | list[str]]] = []
    mode: str | None = None
    buffer: list[str] = []

    def flush() -> None:
        nonlocal mode, buffer
        if not buffer:
            mode = None
            return
        if mode == "paragraph":
            blocks.append(("paragraph", " ".join(line.strip() for line in buffer)))
        elif mode in {"table", "code", "list"}:
            blocks.append((mode, buffer[:]))
        buffer = []
        mode = None

    in_code = False
    for raw_line in text.splitlines():
        line = raw_line.rstrip()
        if line.startswith("```"):
            if in_code:
                flush()
                in_code = False
            else:
                flush()
                in_code = True
                mode = "code"
            continue

        if in_code:
            buffer.append(line)
            continue

        heading_match = HEADING_RE.match(line)
        if heading_match:
            flush()
            hashes, title = heading_match.groups()
            blocks.append((f"heading{len(hashes)}", title))
            continue

        if not line.strip():
            flush()
            continue

        if line.startswith("|"):
            if mode not in {None, "table"}:
                flush()
            mode = "table"
            buffer.append(line)
            continue

        list_match = LIST_RE.match(line)
        if list_match:
            if mode not in {None, "list"}:
                flush()
            mode = "list"
            buffer.append(line)
            continue

        if mode not in {None, "paragraph"}:
            flush()
        mode = "paragraph"
        buffer.append(line)

    flush()
    return blocks


def add_page_number(canvas, doc) -> None:
    canvas.setFont("Helvetica", 9)
    canvas.setFillColor(colors.HexColor("#444444"))
    canvas.drawRightString(doc.pagesize[0] - 0.65 * inch, 0.4 * inch, f"Page {canvas.getPageNumber()}")


def build_styles():
    styles = getSampleStyleSheet()
    base = styles["BodyText"]
    base.fontName = "Helvetica"
    base.fontSize = 10
    base.leading = 13
    base.spaceAfter = 4
    base.alignment = TA_LEFT

    heading1 = ParagraphStyle(
        "Heading1Custom",
        parent=styles["Heading1"],
        fontName="Helvetica-Bold",
        fontSize=18,
        leading=22,
        textColor=colors.HexColor("#18243a"),
        spaceAfter=10,
        spaceBefore=2,
    )
    heading2 = ParagraphStyle(
        "Heading2Custom",
        parent=styles["Heading2"],
        fontName="Helvetica-Bold",
        fontSize=13,
        leading=16,
        textColor=colors.HexColor("#223b63"),
        spaceBefore=8,
        spaceAfter=6,
    )
    heading3 = ParagraphStyle(
        "Heading3Custom",
        parent=styles["Heading3"],
        fontName="Helvetica-Bold",
        fontSize=11,
        leading=14,
        textColor=colors.HexColor("#304f7d"),
        spaceBefore=6,
        spaceAfter=4,
    )
    bullet = ParagraphStyle(
        "BulletCustom",
        parent=base,
        leftIndent=16,
        firstLineIndent=-10,
        spaceAfter=2,
    )
    mono = ParagraphStyle(
        "MonoBlock",
        parent=base,
        fontName="Courier",
        fontSize=8.2,
        leading=10,
        backColor=colors.HexColor("#f3f5f8"),
        borderColor=colors.HexColor("#d7dce3"),
        borderWidth=0.5,
        borderPadding=6,
        borderRadius=2,
        spaceAfter=6,
    )
    return {
        "body": base,
        "heading1": heading1,
        "heading2": heading2,
        "heading3": heading3,
        "bullet": bullet,
        "mono": mono,
    }


def markdown_inline_to_para(text: str) -> str:
    escaped = html.escape(text)
    escaped = re.sub(r"`([^`]+)`", r"<font name='Courier'>\1</font>", escaped)
    return escaped


def render_markdown_to_pdf(input_path: Path, output_path: Path) -> None:
    styles = build_styles()
    story = []

    for block_type, payload in parse_blocks(input_path.read_text(encoding="utf-8")):
        if block_type == "heading1":
            story.append(Paragraph(markdown_inline_to_para(str(payload)), styles["heading1"]))
        elif block_type == "heading2":
            story.append(Paragraph(markdown_inline_to_para(str(payload)), styles["heading2"]))
        elif block_type == "heading3":
            story.append(Paragraph(markdown_inline_to_para(str(payload)), styles["heading3"]))
        elif block_type == "paragraph":
            story.append(Paragraph(markdown_inline_to_para(str(payload)), styles["body"]))
        elif block_type == "list":
            for line in payload:
                item = LIST_RE.match(line)
                if item is None:
                    continue
                story.append(Paragraph(markdown_inline_to_para(f"- {item.group(2)}"), styles["bullet"]))
        elif block_type in {"table", "code"}:
            story.append(Preformatted("\n".join(payload), styles["mono"]))
        story.append(Spacer(1, 0.04 * inch))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    doc = SimpleDocTemplate(
        str(output_path),
        pagesize=landscape(letter),
        leftMargin=0.65 * inch,
        rightMargin=0.65 * inch,
        topMargin=0.65 * inch,
        bottomMargin=0.55 * inch,
        title="2VG3 Homework 5 Submission Answers",
        author="Codex",
    )
    doc.build(story, onFirstPage=add_page_number, onLaterPages=add_page_number)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Render the HW5 submission markdown answers to PDF.")
    parser.add_argument(
        "--input",
        type=Path,
        default=Path("HW5_submission_answers.md"),
        help="Markdown source file.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("output/pdf/HW5_submission_answers.pdf"),
        help="Destination PDF path.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    render_markdown_to_pdf(args.input, args.output)


if __name__ == "__main__":
    main()
