#!/usr/bin/env python3
"""docs/説明資料_PAI最終課題.md を画像入りの PDF にする。

使い方: python3 scripts/build_report_pdf.py [入力.md] [出力.pdf]
必要なもの: pip の markdown、Pillow、snap の chromium-browser、Noto Sans CJK。
mermaid 図は CDN の mermaid.js で描画するのでネットワークが要る。
snap の Chromium は /tmp を読めないため、作業ディレクトリは docs/.pdfbuild に作る。
"""
import os, re, shutil, subprocess, sys, pathlib
import markdown
from PIL import Image

ROOT = pathlib.Path(__file__).resolve().parent.parent
SRC = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / 'docs' / '説明資料_PAI最終課題.md'
OUT = pathlib.Path(sys.argv[2]) if len(sys.argv) > 2 else SRC.with_suffix('.pdf')
BUILD = SRC.parent / '.pdfbuild'
shutil.rmtree(BUILD, ignore_errors=True); (BUILD / 'img').mkdir(parents=True)

src = SRC.read_text(encoding='utf-8')
# 冒頭の太字ヘッダ行（課題名・アカウント・動画・コード）は 1 行ずつ改行する
lines = src.split('\n'); lines[:10] = [(l.rstrip() + '  ') if l.startswith('**') else l for l in lines[:10]]; src = '\n'.join(lines)
# mermaid: フェンスを <pre class="mermaid"> に。A4 縦に合わせて縦方向レイアウト
src = re.sub(r'```mermaid\n(.*?)```\n', lambda m: '<pre class="mermaid">\n' + m.group(1).replace('flowchart LR', 'flowchart TB') + '</pre>\n', src, flags=re.S)
# 画像は長辺 1600 px の JPEG に縮小してサイズを抑える
def shrink(m):
    name = m.group(1); out = BUILD / 'img' / (pathlib.Path(name).stem + '.jpg')
    im = Image.open(SRC.parent / 'photo' / name).convert('RGB'); im.thumbnail((1600, 1600), Image.LANCZOS); im.save(out, quality=85, optimize=True)
    return f'src="img/{out.name}"'
src = re.sub(r'src="photo/([^"]+)"', shrink, src)

body = markdown.markdown(src, extensions=['tables', 'fenced_code', 'sane_lists'])
css = '''
@page { size: A4; margin: 18mm 16mm 18mm 16mm; }
html { font-family: "Noto Sans CJK JP", "Noto Sans JP", sans-serif; font-size: 10.5pt; line-height: 1.6; color: #222; }
body { margin: 0; }
h1 { font-size: 20pt; border-bottom: 3px solid #be2832; padding-bottom: 4px; margin: 0 0 14px; }
h2 { font-size: 15pt; color: #be2832; border-left: 6px solid #be2832; padding-left: 8px; margin: 22px 0 10px; page-break-after: avoid; }
h3 { font-size: 12pt; margin: 16px 0 6px; page-break-after: avoid; }
p { margin: 6px 0; }
ul, ol { margin: 4px 0 8px; padding-left: 22px; }
li { margin: 3px 0; }
table { border-collapse: collapse; margin: 8px 0 12px; font-size: 9.5pt; page-break-inside: avoid; }
th, td { border: 1px solid #bbb; padding: 4px 8px; vertical-align: top; }
th { background: #f0f0ee; }
img { max-width: 100%; height: auto; page-break-inside: avoid; }
pre { background: #f5f5f3; border: 1px solid #ddd; padding: 8px; font-size: 9pt; white-space: pre-wrap; word-break: break-all; page-break-inside: avoid; }
code { font-family: "Noto Sans Mono CJK JP", monospace; font-size: 9.5pt; background: #f3f3f1; padding: 0 3px; }
pre code { background: none; padding: 0; }
pre.mermaid { background: none; border: none; text-align: center; }
pre.mermaid svg { max-width: 100%; max-height: 230mm; height: auto; }
hr { border: none; border-top: 1px solid #ccc; margin: 16px 0; }
'''
html = f'''<!doctype html><html lang="ja"><head><meta charset="utf-8">
<base href="file://{BUILD.as_posix()}/"><style>{css}</style>
<script src="https://cdn.jsdelivr.net/npm/mermaid@10.9.1/dist/mermaid.min.js"></script>
<script>mermaid.initialize({{startOnLoad: true, theme: 'neutral', themeVariables: {{fontSize: '15px'}}, flowchart: {{useMaxWidth: true, nodeSpacing: 30, rankSpacing: 40}}}});</script>
</head><body>{body}</body></html>'''
(BUILD / 'doc.html').write_text(html, encoding='utf-8')

cmd = ['/usr/bin/chromium-browser', '--headless=new', '--disable-gpu', '--no-sandbox', f'--user-data-dir={BUILD}/ud',
       '--virtual-time-budget=15000', '--run-all-compositor-stages-before-draw', '--no-pdf-header-footer',
       f'--print-to-pdf={OUT}', f'file://{BUILD}/doc.html']
r = subprocess.run(cmd, capture_output=True, text=True, timeout=240)
ok = OUT.exists() and OUT.stat().st_size > 10000
shutil.rmtree(BUILD, ignore_errors=True)
if not ok:
    print(r.stderr[-2000:]); sys.exit(1)
pages = len(re.findall(rb'/Type\s*/Page[^s]', OUT.read_bytes()))
print(f'{OUT}  {OUT.stat().st_size/1e6:.1f} MB  {pages} pages')
