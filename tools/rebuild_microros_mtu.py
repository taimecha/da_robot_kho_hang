#!/usr/bin/env python3
"""Rebuild ESP32 micro-ROS transport objects from the library's pinned sources.

For micro_ros_arduino 2.0.7-iron and 3.0.0-iron: preserves all other archive members, generated
ROS message code and pool settings. Rebuilds ALL XRCE client and RMW objects
together because increasing MTU changes shared structure sizes. Requires the
ESP32 Arduino compiler already installed. Makes a backup before installation.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess


SOURCES = {
    'libmicroxrcedds_client': ('https://github.com/eProsima/Micro-XRCE-DDS-Client.git', 'xrce', 'src/c'),
    'librmw_microxrcedds': ('https://github.com/micro-ROS/rmw-microxrcedds.git', 'rmw', 'rmw_microxrcedds_c/src'),
}


def run(args, **kw):
    return subprocess.run([str(a) for a in args], check=True, text=True,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, **kw).stdout


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def symbols(nm, path):
    return set(re.findall(r'^[0-9a-fA-F]+\s+[A-Z]\s+(\S+)$',
                         run([nm, '-g', '--defined-only', path]), re.M))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--library', type=Path, default=Path.home()/'Arduino/libraries/micro_ros_arduino')
    ap.add_argument('--build-dir', type=Path, default=Path.home()/'microros_mtu1472_build')
    ap.add_argument('--compiler', type=Path)
    ap.add_argument('--mtu', type=int, default=1472)
    ap.add_argument('--install', action='store_true', help='install only after successful rebuild and checks')
    a = ap.parse_args()
    if not 512 <= a.mtu <= 1472:
        ap.error('MTU must be between 512 and 1472')
    lib = a.library.resolve()
    version = (lib/'library.properties').read_text()
    if not any('version='+v in version for v in ('2.0.7-iron', '3.0.0-iron')):
        ap.error('This rebuild is scoped to micro_ros_arduino 2.0.7-iron or 3.0.0-iron')
    cc = a.compiler
    if cc is None:
        candidates = sorted((Path.home()/'.arduino15/packages/esp32/tools').glob('**/bin/xtensa-esp32-elf-gcc'))
        if not candidates:
            ap.error('ESP32 compiler not found; pass --compiler')
        cc = candidates[-1]
    prefix = str(cc).removesuffix('gcc')
    ar, nm, objdump = (prefix + x for x in ('ar', 'nm', 'objdump'))
    build = a.build_dir.resolve()
    build.mkdir(parents=True, exist_ok=True)
    original = lib/'src/esp32/libmicroros.a'
    original_hash = sha(original)
    manifest = dict(line.split() for line in (lib/'built_packages').read_text().splitlines() if line.strip())
    members = run([ar, 't', original]).splitlines()
    selected = []
    commits = {}
    for group, (url, directory, source_subdir) in SOURCES.items():
        commit = manifest[url]
        repo = build/directory
        if not repo.exists():
            run(['git', 'clone', '--no-checkout', url, repo])
            run(['git', '-C', repo, 'checkout', '--detach', commit])
        if run(['git', '-C', repo, 'status', '--porcelain']).strip():
            raise RuntimeError(f'Source tree has local changes: {repo}')
        run(['git', '-C', repo, 'checkout', '--detach', commit])
        commits[url] = commit
        paths = list((repo/source_subdir).rglob('*.c'))
        for member in members:
            if member.startswith(group+'-'):
                basename = member[len(group)+1:].removesuffix('.obj')
                matches = [p for p in paths if p.name == basename]
                if len(matches) != 1:
                    raise RuntimeError(f'Ambiguous or missing source for {member}: {matches}')
                selected.append((member, matches[0], repo))
    if not selected:
        raise RuntimeError('No matching archive members')
    headers = build/'headers'
    shutil.copytree(lib/'src', headers, dirs_exist_ok=True, ignore=shutil.ignore_patterns('*.a'))
    config = headers/'uxr/client/config.h'
    text, count = re.subn(r'(#define\s+UXR_CONFIG_CUSTOM_TRANSPORT_MTU\s+)\d+',
                          lambda m: m[1]+str(a.mtu), config.read_text())
    if count != 1:
        raise RuntimeError('Custom transport MTU configuration not found')
    config.write_text(text)
    objects = build/'objects'
    objects.mkdir(exist_ok=True)
    flags = ['-std=c11', '-Os', '-mlongcalls', '-ffunction-sections', '-fdata-sections',
             '-DRCUTILS_LOG_MIN_SEVERITY=RCUTILS_LOG_MIN_SEVERITY_NONE',
             '-DCLOCK_MONOTONIC=0', '-D__attribute__(x)=', '-DHAVE_C_TYPESUPPORT',
             '-I'+str(headers)]

    def compile_one(item):
        member, src, repo = item
        run([cc, *flags, '-I'+str(repo/'include'), '-I'+str(repo/'src/c'),
             '-I'+str(repo/'rmw_microxrcedds_c/include'),
             '-I'+str(repo/'rmw_microxrcedds_c/src'),
             '-c', src, '-o', objects/member])

    print(f'Rebuilding {len(selected)} pinned XRCE/RMW objects, MTU={a.mtu}', flush=True)
    with ThreadPoolExecutor(max_workers=4) as executor:
        list(executor.map(compile_one, selected))
    result = build/'libmicroros.a'
    shutil.copy2(original, result)
    run([ar, 'rcs', result, *(objects/member for member, _, _ in selected)])
    if run([ar, 't', result]).splitlines() != members:
        raise RuntimeError('Archive member list changed')
    if symbols(nm, result) != symbols(nm, original):
        raise RuntimeError('Exported symbol set changed; refusing installation')
    assembly = run([objdump, '-d', result])
    (build/'disassembly.txt').write_text(assembly)
    section = assembly.split('<uxr_init_custom_transport>:', 1)[1].split('Disassembly of section', 1)[0]
    if not re.search(r'movi\s+\w+,\s*(?:0x'+format(a.mtu, 'x')+r'|'+str(a.mtu)+r')\b', section):
        raise RuntimeError('Could not verify compiled MTU in transport disassembly')
    report = dict(mtu=a.mtu, source_commits=commits, compiler=str(cc),
                  old_sha256=original_hash, new_sha256=sha(result),
                  rebuilt_members=[x[0] for x in selected],
                  checks=['same archive members', 'same exported symbols', 'compiled MTU verified'])
    if a.install:
        if sha(original) != original_hash:
            raise RuntimeError('Installed archive changed while building')
        backup = Path.home()/'firmware_backups'/('microros_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
        backup.mkdir(parents=True)
        shutil.copy2(original, backup/'libmicroros.a')
        shutil.copy2(lib/'src/uxr/client/config.h', backup/'uxr_config.h')
        shutil.copy2(result, original)
        shutil.copy2(config, lib/'src/uxr/client/config.h')
        report['backup'] = str(backup)
        report['installed'] = str(original)
    (build/'rebuild_report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    try:
        main()
    except subprocess.CalledProcessError as e:
        raise SystemExit(f'Build command failed: {e.cmd}\n{e.stdout}\n{e.stderr}')
