#!/usr/bin/env python3
"""Render the project's small Markdown subset to a self-contained offline guide."""

import html
from pathlib import Path
import re


def inline(value):
    tokens = re.split(r'(`[^`]+`|\[[^\]]+\]\([^)]+\)|\*\*[^*]+\*\*)', value)
    result = []
    for token in tokens:
        if token.startswith('`') and token.endswith('`'):
            result.append('<code>' + html.escape(token[1:-1]) + '</code>')
        elif token.startswith('**') and token.endswith('**'):
            result.append('<strong>' + html.escape(token[2:-2]) + '</strong>')
        elif re.fullmatch(r'\[[^\]]+\]\([^)]+\)', token):
            label, target = re.fullmatch(r'\[([^\]]+)\]\(([^)]+)\)', token).groups()
            result.append(f'<a href="{html.escape(target, quote=True)}">{html.escape(label)}</a>')
        else:
            result.append(html.escape(token))
    return ''.join(result)


def render(source):
    lines = source.splitlines()
    body, contents = [], []
    index = 0
    while index < len(lines):
        line = lines[index]
        if not line.strip():
            index += 1
            continue
        if line.startswith('```'):
            code = []
            index += 1
            while index < len(lines) and not lines[index].startswith('```'):
                code.append(lines[index])
                index += 1
            body.append('<pre><code>' + html.escape('\n'.join(code)) + '</code></pre>')
            index += 1
            continue
        heading = re.match(r'^(#{1,6}) (.*)', line)
        if heading:
            level, title = len(heading[1]), heading[2]
            anchor = re.sub(r'[^a-z0-9]+', '-', title.lower()).strip('-')
            body.append(f'<h{level} id="{anchor}">{inline(title)}</h{level}>')
            if level == 2:
                contents.append(f'<li><a href="#{anchor}">{html.escape(title)}</a></li>')
            index += 1
            continue
        if line.startswith('|'):
            rows = []
            while index < len(lines) and lines[index].startswith('|'):
                cells = [c.strip() for c in lines[index].strip('|').split('|')]
                if not all(re.fullmatch(r':?-+:?', c) for c in cells):
                    tag = 'th' if not rows else 'td'
                    rows.append('<tr>' + ''.join(f'<{tag}>{inline(c)}</{tag}>' for c in cells) + '</tr>')
                index += 1
            body.append('<div class="table-wrap"><table>' + ''.join(rows) + '</table></div>')
            continue
        paragraph = []
        while index < len(lines) and lines[index].strip():
            if lines[index].startswith(('```', '#', '|')):
                break
            paragraph.append(lines[index])
            index += 1
        body.append('<p>' + inline(' '.join(paragraph)) + '</p>')
    return '\n'.join(body), '\n'.join(contents)


def main():
    root = Path(__file__).resolve().parents[1]
    body, contents = render((root / 'docs/offline-robot-guide.md').read_text())
    document = '''<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>JetRover Mecanum — Offline Robot Guide</title>
<style>
:root { color-scheme: light; font: 17px/1.6 system-ui, sans-serif; color: #20242b; background: #f3f4f6; }
body { max-width: 1000px; margin: 0 auto; padding: 28px 22px 70px; }
main, nav { background: white; padding: 28px 36px; border: 1px solid #ddd; border-radius: 8px; }
nav { margin-bottom: 20px; }
nav ol { columns: 2; margin: 8px 0 0; padding: 0; list-style: none; }
nav li { break-inside: avoid; padding: 3px 0; }
h1 { font-size: 2rem; line-height: 1.2; margin-top: 0; }
h2 { border-top: 2px solid #27364d; padding-top: 24px; margin-top: 42px; font-size: 1.4rem; }
a { color: #174c89; text-decoration-thickness: 1px; }
p { margin: 14px 0; }
code { font: .88em/1.55 ui-monospace, Menlo, Consolas, monospace; overflow-wrap: anywhere; }
p code, td code { background: #eef1f4; padding: 1px 4px; border-radius: 3px; }
pre { background: #f0f2f5; border-left: 3px solid #536b89; padding: 16px; overflow-x: auto; }
pre code { white-space: pre; overflow-wrap: normal; }
.table-wrap { overflow-x: auto; }
table { width: 100%; border-collapse: collapse; font-size: .88rem; }
th, td { border: 1px solid #ccd2d9; padding: 10px; text-align: left; vertical-align: top; }
th { background: #edf0f4; }
.toolbar { display: flex; justify-content: space-between; gap: 16px; margin-bottom: 16px; font-size: .9rem; }
button { font: inherit; border: 1px solid #79818c; background: white; padding: 6px 12px; border-radius: 5px; cursor: pointer; }
@media (max-width: 650px) { main, nav { padding: 20px 16px; } nav ol { columns: 1; } body { padding: 12px; } }
@media print {
  :root { font-size: 10pt; background: white; color: black; }
  body { padding: 0; max-width: none; }
  main, nav { border: 0; padding: 0; }
  .toolbar { display: none; }
  h2 { break-after: avoid; margin-top: 24px; padding-top: 12px; }
  pre { white-space: pre-wrap; overflow: visible; break-inside: avoid; }
  pre code { white-space: pre-wrap; overflow-wrap: anywhere; }
  .table-wrap { overflow: visible; } tr { break-inside: avoid; }
  a { color: black; } @page { margin: 16mm; }
}
</style></head><body>
<div class="toolbar"><span>Offline · Mecanum · October 13, 2026</span>
<button onclick="window.print()">Print / Save PDF</button></div>
<nav aria-label="Contents"><strong>Jump to a step</strong><ol>CONTENTS</ol></nav>
<main>BODY</main></body></html>'''
    output = root / 'docs/offline-robot-guide.html'
    output.write_text(document.replace('CONTENTS', contents).replace('BODY', body))
    print(output)


if __name__ == '__main__':
    main()
