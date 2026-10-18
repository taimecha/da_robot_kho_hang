#!/usr/bin/env python3
"""Create an editable DOCX supplement from docs/PHAN_MO_PHONG_NCKH.md."""
import html
import os
import re
import subprocess
import tempfile
from pathlib import Path
from zipfile import ZipFile

from lxml import etree
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / 'docs'
SOURCE = DOCS / 'PHAN_MO_PHONG_NCKH.md'
HTML = DOCS / 'PHAN_MO_PHONG_NCKH.html'
DOCX = DOCS / 'PHAN_MO_PHONG_NCKH.docx'


def embed_images():
    """Writer/Web exports HTML images as links; make the DOCX self-contained."""
    with ZipFile(DOCX) as source:
        rels = etree.fromstring(source.read('word/_rels/document.xml.rels'))
        doc = etree.fromstring(source.read('word/document.xml'))
        image_rel = 'http://schemas.openxmlformats.org/officeDocument/2006/relationships/image'
        embedded = []
        for i, rel in enumerate(rels, 1):
            if rel.get('Type') != image_rel or rel.get('TargetMode') != 'External':
                continue
            uri = rel.get('Target')
            path = Path(uri.removeprefix('file://'))
            if not path.exists():
                raise FileNotFoundError(path)
            name = f'word/media/research_{i}{path.suffix}'
            embedded.append((name, path.read_bytes()))
            rel.set('Target', f'media/research_{i}{path.suffix}')
            rel.attrib.pop('TargetMode', None)
            rid = rel.get('Id')
            blip_ns = {'a': 'http://schemas.openxmlformats.org/drawingml/2006/main'}
            r_ns = 'http://schemas.openxmlformats.org/officeDocument/2006/relationships'
            for blip in doc.xpath('//a:blip', namespaces=blip_ns):
                if blip.get(f'{{{r_ns}}}link') == rid:
                    blip.attrib.pop(f'{{{r_ns}}}link')
                    blip.set(f'{{{r_ns}}}embed', rid)
        fd, tmp_name = tempfile.mkstemp(suffix='.docx', dir=DOCS)
        os.close(fd)
        try:
            with ZipFile(tmp_name, 'w') as target:
                for item in source.infolist():
                    data = source.read(item.filename)
                    if item.filename == 'word/_rels/document.xml.rels':
                        data = etree.tostring(rels, xml_declaration=True, encoding='UTF-8')
                    elif item.filename == 'word/document.xml':
                        data = etree.tostring(doc, xml_declaration=True, encoding='UTF-8')
                    target.writestr(item, data)
                for name, data in embedded:
                    target.writestr(name, data)
        except BaseException:
            Path(tmp_name).unlink(missing_ok=True)
            raise
    os.replace(tmp_name, DOCX)


def inline(src):
    src = html.escape(src)
    src = re.sub(r'\[([^]]+)\]\(([^)]+)\)', r'<a href="\2">\1</a>', src)
    src = re.sub(r'\*\*([^*]+)\*\*', r'<strong>\1</strong>', src)
    src = re.sub(r'`([^`]+)`', r'<code>\1</code>', src)
    return src


def render():
    lines = SOURCE.read_text().splitlines()
    body = []
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if not line:
            i += 1
            continue
        if line.startswith('## '):
            body.append(f'<h2>{inline(line[3:])}</h2>')
        elif line.startswith('# '):
            body.append(f'<h1>{inline(line[2:])}</h1>')
        elif line.startswith('!['):
            match = re.match(r'!\[([^]]*)\]\(([^)]+)\)', line)
            if match:
                path = (DOCS / match.group(2)).resolve()
                uri = path.as_uri()
                with Image.open(path) as img:
                    w, h = img.size
                width = 570 if path.name == 'ket_qua_bon_tinh_huong.png' else 520
                height = round(h * width / w)
                body.append(f'<p class="figure"><img src="{uri}" width="{width}" '
                            f'height="{height}" alt="{html.escape(match.group(1))}"></p>')
        elif line.startswith('|'):
            cells = [s.strip() for s in line.strip('|').split('|')]
            if not all(re.fullmatch(r':?-+:?', c) for c in cells):
                tag = 'th' if i + 1 < len(lines) and lines[i + 1].lstrip().startswith('|---') else 'td'
                body.append('<tr>' + ''.join(f'<{tag}>{inline(c)}</{tag}>' for c in cells) + '</tr>')
                if tag == 'th':
                    body.insert(len(body)-1,
                                '<table width="570" style="table-layout:fixed;">'
                                '<colgroup><col width="95"><col width="145">'
                                '<col width="260"><col width="70"></colgroup>')
            if i + 1 == len(lines) or not lines[i + 1].startswith('|'):
                body.append('</table>')
        elif line.startswith('**Hình '):
            body.append(f'<p class="caption">{inline(line)}</p>')
        else:
            body.append(f'<p>{inline(line)}</p>')
        i += 1
    style = '''
    <style>
    @page { size: A4; margin: 2.2cm 2.0cm 2.0cm 2.4cm; }
    body { font-family: "Times New Roman", serif; font-size: 12pt; line-height: 1.35; color: #111; }
    h1 { font-size: 15pt; margin: 0 0 8pt; }
    h2 { font-size: 13pt; margin: 9pt 0 6pt; }
    p { text-align: justify; margin: 0 0 8pt; }
    .figure { text-align: center; margin: 12pt 0 2pt; page-break-inside: avoid; }
    .figure img { width: 16cm; max-width: 100%; height: auto; }
    .caption { text-align: center; font-style: italic; margin: 0 0 11pt; }
    table { border-collapse: collapse; width: 15cm; table-layout: fixed; font-size: 9pt; margin: 8pt 0 12pt; }
    th, td { border: 1px solid #555; padding: 5pt; vertical-align: top; }
    th { background: #e8eef6; }
    code { font-family: "Liberation Mono", monospace; font-size: 10pt; }
    </style>'''
    HTML.write_text('<!doctype html><html lang="vi"><head><meta charset="utf-8">'
                    + style + '</head><body>' + '\n'.join(body) + '</body></html>')
    subprocess.run(['libreoffice', '-env:UserInstallation=file:///tmp/lo_fleet_research',
                    '--headless', '--convert-to', 'docx:Office Open XML Text',
                    '--outdir', str(DOCS), str(HTML)],
                   check=True)
    embed_images()


if __name__ == '__main__':
    render()
