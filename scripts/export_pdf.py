"""DOSSIER.md -> docs/DOSSIER.pdf (python-markdown -> HTML -> headless Edge/Chrome print).

    python scripts/export_pdf.py

The browser runs with its own temporary profile, so it does not touch the user's browser.
"""

from __future__ import annotations

import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import markdown

ROOT = Path(__file__).resolve().parents[1]
BROWSERS = [r"C:\Program Files\Google\Chrome\Application\chrome.exe",
            r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"]

CSS = """
@page { size: A4; margin: 16mm 14mm 16mm 14mm; }
body { font-family: "Segoe UI", Arial, sans-serif; font-size: 9.5pt; line-height: 1.38; color: #222; }
h1 { font-size: 17pt; margin: 0 0 4pt 0; }
h2 { font-size: 13pt; margin-top: 16pt; border-bottom: 1px solid #bbb; padding-bottom: 2pt; page-break-after: avoid; }
h3 { font-size: 11pt; margin-top: 11pt; page-break-after: avoid; }
table { border-collapse: collapse; width: 100%; margin: 6pt 0; font-size: 8pt; page-break-inside: auto; }
tr { page-break-inside: avoid; }
th, td { border: 1px solid #ccc; padding: 2pt 4pt; vertical-align: top; text-align: left; }
th { background: #f1f1f1; }
code { font-family: Consolas, monospace; font-size: 8.5pt; background: #f5f5f5; padding: 0 2px; }
pre { background: #f7f7f7; border: 1px solid #e3e3e3; padding: 6pt; font-size: 8pt; white-space: pre; overflow: hidden; }
blockquote { border-left: 3px solid #2a78d6; margin: 6pt 0; padding: 2pt 8pt; color: #333; background: #f6f9fd; }
img { max-width: 100%; }
hr { border: 0; border-top: 1px solid #ddd; }
"""


LIST_RE = re.compile(r"^(\s*)([-*]|\d+\.)\s")


def normalise_lists(md: str) -> str:
    """GitHub-style lists -> python-markdown lists.

    It inserts a blank line before a list that follows a paragraph, and maps 2-space
    nesting (and 3-space nesting under numbered items) to the 4-space nesting
    python-markdown expects.
    """
    out, in_code = [], False
    prev = ""
    for line in md.splitlines():
        if line.lstrip().startswith("```"):
            in_code = not in_code
            out.append(line)
            prev = line
            continue
        if not in_code:
            m = len(line) - len(line.lstrip(" "))
            if m and line.strip():
                level = (m - 1) // 2 if m % 2 else m // 2
                line = " " * (4 * level) + line.lstrip(" ")
            if LIST_RE.match(line) and prev.strip() and not LIST_RE.match(prev) \
                    and not prev.lstrip().startswith("|"):
                out.append("")
        out.append(line)
        prev = line
    return "\n".join(out)


def main() -> None:
    md = (ROOT / "DOSSIER.md").read_text(encoding="utf-8")
    md = re.sub(r"\A---\n.*?\n---\n", "", md, flags=re.S)  # drop the YAML front matter
    md = normalise_lists(md)
    html_body = markdown.markdown(md, extensions=["tables", "fenced_code", "sane_lists"])
    html_body = html_body.replace('src="docs/img/', f'src="{(ROOT / "docs" / "img").as_uri()}/')
    # figures referenced from the text are embedded at the end as an appendix
    figs = sorted((ROOT / "docs" / "img").glob("*.png"))
    appendix = "<h2>Appendix: figures</h2>" + "".join(
        f'<p><b>{p.name}</b><br><img src="{p.as_uri()}"></p>' for p in figs)
    html = f"<!doctype html><html><head><meta charset='utf-8'><style>{CSS}</style></head><body>{html_body}{appendix}</body></html>"
    out_html = ROOT / "docs" / "DOSSIER.html"
    out_html.write_text(html, encoding="utf-8")
    out_pdf = ROOT / "docs" / "DOSSIER.pdf"
    exe = next((b for b in BROWSERS if Path(b).exists()), None)
    if exe is None:
        sys.exit("no Edge/Chrome found; open docs/DOSSIER.html and print to PDF")
    prof = tempfile.mkdtemp(prefix="dossier_pdf_")
    try:
        subprocess.run([exe, "--headless=new", "--disable-gpu", f"--user-data-dir={prof}", "--no-first-run",
                        "--no-pdf-header-footer", f"--print-to-pdf={out_pdf}", out_html.as_uri()],
                       check=False, timeout=180)
        import time
        for _ in range(60):  # the browser may return before the file is flushed
            if out_pdf.exists() and out_pdf.stat().st_size > 0:
                break
            time.sleep(1)
    finally:
        shutil.rmtree(prof, ignore_errors=True)
    out_html.unlink()
    print(f"wrote {out_pdf} ({out_pdf.stat().st_size // 1024} kB)")


if __name__ == "__main__":
    main()
