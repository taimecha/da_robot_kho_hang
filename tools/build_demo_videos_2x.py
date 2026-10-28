#!/usr/bin/env python3
"""Xuất bộ video demo 2×; ghép cảnh thực, RViz và hai camera cho Video 6."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
BOLD = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
NAMES = {
    1: ('Video 1 - Nhập đích cho hai xe.mp4', 'video_1_nhap_dich_2x.mp4'),
    2: ('Video 2 - Hai xe đối đầu, đổi chỗ.mp4', 'video_2_doi_dau_doi_cho_2x.mp4'),
    3: ('Video 3 - Hai tuyến giao nhau tại ô (3,3).mp4', 'video_3_giao_nhau_3_3_2x.mp4'),
    4: ('Video 4 - Né vật cản tĩnh.mp4', 'video_4_ne_vat_can_tinh_2x.mp4'),
    5: ('Video 5 - Né vật cản động.mp4', 'video_5_ne_vat_can_dong_2x.mp4'),
    6: ('', 'video_6_thuc_te_rviz_camera_2x.mp4'),
}


def probe(path):
    return json.loads(subprocess.check_output([
        'ffprobe', '-v', 'error', '-show_format', '-show_streams', '-of', 'json', str(path)
    ]))


def sha256(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def title(text, x, y, size=26, color='white', bold=False, enable=None):
    f = (f"drawtext=fontfile='{BOLD if bold else FONT}':text='{text}':"
         f'fontsize={size}:fontcolor={color}:x={x}:y={y}')
    if enable:
        f += f":enable='{enable}'"
    return f


def build(k, args):
    output = args.output / NAMES[k][1]
    log = args.output / '_nguon' / f'ffmpeg_video_{k}.log'
    cmd = ['ffmpeg', '-hide_banner', '-loglevel', 'warning', '-y', '-threads', '4']
    hold = 0
    if k < 6:
        source = args.source_dir / NAMES[k][0]
        info = probe(source)
        video = next(s for s in info['streams'] if s['codec_type'] == 'video')
        source_duration = float(video.get('duration', info['format']['duration']))
        sources = [source]
        cmd += ['-i', str(source), '-filter_threads', '2', '-vf',
                'setpts=(PTS-STARTPTS)/2,fps=30,setsar=1', '-map', '0:v:0']
        if any(s['codec_type'] == 'audio' for s in info['streams']):
            cmd += ['-map', '0:a:0', '-af', 'asetpts=PTS-STARTPTS,atempo=2',
                    '-c:a', 'aac', '-b:a', '128k']
        else:
            cmd += ['-an']
    else:
        sources = [args.real6, args.screen6]
        info = probe(args.real6)
        video = next(s for s in info['streams'] if s['codec_type'] == 'video')
        source_duration = float(video['duration'])
        hold = 2.0
        # Constant offset checked against movement near the beginning, observation,
        # return journey and delivery. This is an edit alignment, not hardware sync.
        off = args.screen_offset
        screen_end = off + source_duration
        graph = [
            f'[0:v]trim=duration={source_duration},setpts=(PTS-STARTPTS)/2,'
            'scale=1220:686:flags=lanczos,setsar=1,'
            'pad=1920:1080:0:96:color=0x101722[base]',
            f'[1:v]trim=start={off}:end={screen_end},setpts=(PTS-STARTPTS)/2,'
            'split=2[mapin][camin]',
            '[mapin]crop=680:680:780:150,scale=550:550:flags=lanczos,setsar=1[map]',
            '[camin]crop=680:410:0:66,setsar=1[cam]',
            '[base][map]overlay=1305:84:shortest=1[withmap]',
            '[withmap][cam]overlay=1240:670:shortest=1[panels]',
        ]
        labels = [
            'drawbox=x=0:y=55:w=1920:h=2:color=0x425269:t=fill',
            'drawbox=x=1232:y=64:w=2:h=1016:color=0x425269:t=fill',
            title('VIDEO 6 · HAI XE ĐO VÀ GIAO HÀNG', 24, 13, 30, bold=True),
            title('TỐC ĐỘ 2×', 1710, 15, 26, '0xffd166', True),
            title('THỰC TẾ', 24, 65, 22, '0xc5d5e8', True),
            title('RViz · Xe 1 xanh dương · Xe 2 cam', 1252, 60, 21, '0xc5d5e8'),
            title('CAMERA XE 1 VÀ XE 2', 1252, 640, 23, '0xc5d5e8', True),
            title('LiDAR + camera · Đo và phân loại theo chiều cao', 24, 815, 26, bold=True),
            title('Ô giao · AGV1 (1,2) · AGV2 (1,1)', 24, 862, 25, '0xc5d5e8'),
            # Values are the one-decimal readings visible in the source GUI.
            # Display only after that GUI has actually finalized its measurement.
            title('AGV1 · Rộng 19,6 cm · Cao 31,7 cm', 24, 917, 28,
                  '0x76b4ff', True, 'gte(t,21.2)'),
            title('AGV2 · Rộng 12,3 cm · Cao 23,9 cm', 24, 963, 28,
                  '0xffbd69', True, 'gte(t,27.8)'),
            title('HAI XE ĐÃ TỚI Ô GIAO', 24, 1024, 27,
                  '0x8edb9b', True, 'gte(t,57.8)'),
        ]
        graph.append('[panels]' + ','.join(labels)
                     + f',fps=30,tpad=stop_mode=clone:stop_duration={hold}[outv]')
        if any(s['codec_type'] == 'audio' for s in info['streams']):
            graph.append(f'[0:a]atrim=duration={source_duration},asetpts=PTS-STARTPTS,'
                         f'atempo=2,apad=pad_dur={hold}[outa]')
        graph_path = args.output / '_nguon' / 'video6_filter.ffgraph'
        graph_path.write_text(';\n'.join(graph), encoding='utf-8')
        cmd += ['-i', str(args.real6), '-i', str(args.screen6),
                '-filter_complex_threads', '2', '-filter_complex_script', str(graph_path),
                '-map', '[outv]']
        if '[outa]' in graph[-1]:
            cmd += ['-map', '[outa]', '-c:a', 'aac', '-b:a', '128k']
        else:
            cmd += ['-an']
        cmd += ['-t', str(source_duration / 2 + hold)]
    cmd += ['-c:v', 'libx264', '-preset', 'medium', '-crf', '19', '-threads', '4',
            '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
            '-metadata', f'title=AGV demo {k} - 2x', str(output)]
    print(f'Bắt đầu Video {k}', flush=True)
    with log.open('w') as err:
        subprocess.run(cmd, stderr=err, check=True)
    result = probe(output)
    entry = {
        'video': k, 'output': output.name, 'speed_relative_to_inputs': 2,
        'source_motion_duration_s': source_duration, 'final_hold_s': hold,
        'output_duration_s': float(result['format']['duration']),
        'sources': [{'path': str(p), 'sha256': sha256(p)} for p in sources],
        'output_sha256': sha256(output), 'ffmpeg_command': cmd,
    }
    if k == 6:
        entry.update(screen_time_minus_real_time_s=args.screen_offset,
                     synchronization='Ước lượng từ chuyển động và các mốc cùng lượt thử; không phải đồng bộ phần cứng.',
                     camera_crop_xywh=[0, 66, 680, 410], rviz_crop_xywh=[780, 150, 680, 680])
    (args.output / '_nguon' / f'video_{k}.json').write_text(
        json.dumps(entry, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f'Xong Video {k} · {entry["output_duration_s"]:.2f} s', flush=True)
    return entry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-dir', type=Path, default=Path.home() / 'Downloads/06. Video demo')
    parser.add_argument('--real6', type=Path, default=Path.home() / 'Downloads/7244486068532648805.mp4')
    parser.add_argument('--screen6', type=Path, default=Path.home() / 'Videos/Screencasts/Screencast from 2026-10-26 16-09-05.webm')
    parser.add_argument('--screen-offset', type=float, default=9.6)
    parser.add_argument('--output', type=Path, default=ROOT / 'docs/video_demo_2x')
    parser.add_argument('--only', type=int, nargs='+', choices=list(NAMES), default=list(NAMES))
    parser.add_argument('--jobs', type=int, default=2)
    args = parser.parse_args()
    (args.output / '_nguon').mkdir(parents=True, exist_ok=True)
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        list(pool.map(lambda k: build(k, args), args.only))


if __name__ == '__main__':
    main()
