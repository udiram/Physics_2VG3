from pathlib import Path
import textwrap


def escape_pdf_text(text: str) -> str:
    return text.replace("\\", "\\\\").replace("(", "\\(").replace(")", "\\)")


def markdown_to_lines(markdown: str) -> list[str]:
    lines: list[str] = []
    for raw in markdown.splitlines():
        raw = raw.rstrip()
        if raw.startswith("#"):
            raw = raw.lstrip("#").strip().upper()
        if not raw:
            lines.append("")
            continue
        lines.extend(textwrap.wrap(raw, width=92, replace_whitespace=False, drop_whitespace=False))
    return lines


def make_pdf_bytes(lines: list[str]) -> bytes:
    page_lines = 46
    pages = [lines[i : i + page_lines] for i in range(0, len(lines), page_lines)] or [[]]

    objects: list[bytes] = [b"<< /Type /Catalog /Pages 2 0 R >>", b""]
    kids: list[str] = []
    font_object_num = 3 + 2 * len(pages)

    for page_index, page_lines_content in enumerate(pages):
        page_object_num = 3 + page_index * 2
        content_object_num = 4 + page_index * 2
        kids.append(f"{page_object_num} 0 R")

        stream_lines = ["BT", "/F1 11 Tf", "50 780 Td", "14 TL"]
        first = True
        for line in page_lines_content:
            escaped = escape_pdf_text(line)
            if first:
                stream_lines.append(f"({escaped}) Tj")
                first = False
            else:
                stream_lines.append(f"T* ({escaped}) Tj")
        stream_lines.append("ET")
        content = "\n".join(stream_lines).encode("latin-1", errors="replace")

        page_object = (
            f"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 612 792] "
            f"/Resources << /Font << /F1 {font_object_num} 0 R >> >> "
            f"/Contents {content_object_num} 0 R >>"
        ).encode("latin-1")
        content_object = f"<< /Length {len(content)} >>\nstream\n".encode("latin-1") + content + b"\nendstream"

        objects.append(page_object)
        objects.append(content_object)

    objects[1] = f"<< /Type /Pages /Count {len(pages)} /Kids [{' '.join(kids)}] >>".encode("latin-1")
    objects.append(b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>")

    output = [b"%PDF-1.4\n%\xe2\xe3\xcf\xd3\n"]
    offsets = [0]
    current_offset = len(output[0])
    for object_number, obj in enumerate(objects, start=1):
        serialized = f"{object_number} 0 obj\n".encode("latin-1") + obj + b"\nendobj\n"
        offsets.append(current_offset)
        output.append(serialized)
        current_offset += len(serialized)

    xref_offset = current_offset
    output.append(f"xref\n0 {len(objects) + 1}\n0000000000 65535 f \n".encode("latin-1"))
    for offset in offsets[1:]:
        output.append(f"{offset:010d} 00000 n \n".encode("latin-1"))
    output.append(
        f"trailer\n<< /Size {len(objects) + 1} /Root 1 0 R >>\nstartxref\n{xref_offset}\n%%EOF\n".encode("latin-1")
    )
    return b"".join(output)


source = Path("HW7_writeup.md").read_text()
pdf_bytes = make_pdf_bytes(markdown_to_lines(source))
Path("HW7_writeup.pdf").write_bytes(pdf_bytes)
