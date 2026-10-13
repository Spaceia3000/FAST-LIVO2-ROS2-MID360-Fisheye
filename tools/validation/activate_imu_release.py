#!/usr/bin/env python3
"""Verify a staged release, atomically activate its existing executable, or rollback.
Never compiles or starts FAST; never rewrites an old mapped executable/library."""
import argparse, datetime, hashlib, json, os, pathlib, re, subprocess, tempfile
P=pathlib.Path
WS=P('/home/breidy/Documents/fast_livo2_baseline_ws')
SOURCE=WS/'src/FAST-LIVO2-ROS2-MID360-Fisheye'
INSTALLED=WS/'install/fast_livo/lib/fast_livo/fastlivo_mapping'
def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()
def utc(): return datetime.datetime.now(datetime.timezone.utc).isoformat()
def command(*args): return subprocess.run(args,capture_output=True,text=True,check=True).stdout
def write_new(path,data):
    with path.open('x') as f:json.dump(data,f,indent=2);f.write('\n')
def mapped_fast():
    result={}
    for proc in P('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            paths=sorted({line.split()[-1] for line in (proc/'maps').read_text().splitlines()
                          if any(name in line for name in ['/fastlivo_mapping','/libimu_proc.so','/liblaser_mapping.so'])})
            if paths:result[proc.name]=paths
        except (FileNotFoundError,ProcessLookupError,PermissionError):pass
    return result
def atomic_link(target):
    # A new symlink in the same directory permits an atomic rename. os.replace
    # replaces only the symlink inode, not its target or any mapped old library.
    fd,name=tempfile.mkstemp(prefix='.fastlivo_activation_',dir=INSTALLED.parent)
    os.close(fd);os.unlink(name)
    try:
        os.symlink(target,name)
        os.replace(name,INSTALLED)
    finally:
        if os.path.lexists(name):os.unlink(name)
def capture_native_artifacts(executable):
    output=command('ldd',str(executable))
    required={'liblaser_mapping.so','libvio.so','liblio.so','libpre.so','libimu_proc.so','libutils.so'}
    artifacts={str(executable):sha(executable)};found=set()
    if 'not found' in output:raise RuntimeError('Old runtime dependency unresolved')
    for line in output.splitlines():
        parts=line.strip().split()
        if len(parts)>2 and parts[0] in required and parts[1]=='=>':
            path=P(parts[2]).resolve();artifacts[str(path)]=sha(path);found.add(parts[0])
    if found!=required:raise RuntimeError('Old native library identity incomplete')
    return artifacts

def stage(release,expected,runtime_evidence):
    release=release.resolve()
    if release.parent!=WS/'releases':raise RuntimeError('Release outside dedicated release directory')
    runtime=json.loads(runtime_evidence.read_text())
    if runtime.get('passed') is not True:raise RuntimeError('Runtime regression must PASS before staging')
    actual=command('git','-C',str(SOURCE),'rev-parse','HEAD').strip()
    command('git','-C',str(SOURCE),'diff','--quiet')
    command('git','-C',str(SOURCE),'diff','--cached','--quiet')
    command('git','-C',str(SOURCE),'diff','--quiet',runtime['source_commit'],actual,'--','src','include','CMakeLists.txt','config','launch')
    if expected!=actual:raise RuntimeError('Production source SHA mismatch')
    executable=release/'build/fast_livo/fastlivo_mapping'
    if not executable.is_file():raise RuntimeError('Missing release executable')
    elf=command('readelf','-d',str(executable));ldd=command('ldd',str(executable))
    if 'not found' in ldd:raise RuntimeError('Unresolved runtime dependencies')
    required={'liblaser_mapping.so','libvio.so','liblio.so','libpre.so','libimu_proc.so','libutils.so'}
    paths={}
    for line in ldd.splitlines():
        parts=line.strip().split()
        if len(parts)>2 and parts[0] in required and parts[1]=='=>':
            actual_path=P(parts[2]).resolve()
            if actual_path.parent!=executable.parent:raise RuntimeError('FAST dependency outside release: '+line)
            paths[parts[0]]=actual_path
    if set(paths)!=required:raise RuntimeError('Missing required native library bindings: '+str(required-set(paths)))
    if str(executable.parent) not in elf:raise RuntimeError('Release build path absent in ELF runtime path')
    flags=(release/'build/fast_livo/CMakeFiles/laser_mapping.dir/flags.make').read_text()
    if str(SOURCE) not in flags:raise RuntimeError('ROOT_DIR is not production source')
    artifacts={str(path):{'sha256':sha(path),'bytes':path.stat().st_size} for path in [executable,*paths.values()]}
    (release/'readelf.txt').write_text(elf);(release/'ldd.txt').write_text(ldd)
    write_new(release/'release_manifest.json',{'created_utc':utc(),'source_sha':actual,'source_path':str(SOURCE),
        'executable':str(executable),'artifacts':artifacts,'root_dir':str(SOURCE),
        'scope':'Staged release build and ELF bindings verified; this production ROOT_DIR binary was not launched.',
        'native_regression_reference':'docs/evidence/imu_time_origin_20261013',
        'passing_runtime_evidence':{'path':str(runtime_evidence.resolve()),'sha256':sha(runtime_evidence),'source_commit':runtime['source_commit'],'passed':True},
        'runtime_source_code_identical_to_production':True,
        'runtime_regression_scope':'Separate isolated build of the same source logic; ROOT_DIR differs.'})
    print(release/'release_manifest.json')
def activate(manifest):
    data=json.loads(manifest.read_text());release=manifest.parent
    if release.parent!=WS/'releases':raise RuntimeError('Unexpected manifest path')
    if not INSTALLED.is_symlink():raise RuntimeError('Expected installed executable symlink; refusing regular file')
    if command('git','-C',str(SOURCE),'rev-parse','HEAD').strip()!=data['source_sha']:raise RuntimeError('Source changed since staging')
    for name,item in data['artifacts'].items():
        if sha(P(name))!=item['sha256']:raise RuntimeError('Artifact hash changed: '+name)
    old=os.readlink(INSTALLED);old_resolved=INSTALLED.resolve();old_hash=sha(old_resolved);old_artifacts=capture_native_artifacts(old_resolved)
    activation={'created_utc':utc(),'installed_symlink':str(INSTALLED),'previous_target':old,'previous_resolved':str(old_resolved),
                'previous_executable_sha256':old_hash,'previous_native_artifact_hashes':old_artifacts,'new_target':data['executable'],'manifest_sha256':sha(manifest),
                'mapped_processes_before':mapped_fast(),'scope':'Atomic symlink switch only. Existing mapped binaries/logs untouched; no FAST launch.'}
    target=release/'activation.json'
    if target.exists():raise RuntimeError('Activation evidence already exists')
    # Persist rollback metadata before changing the link.
    write_new(target,activation)
    atomic_link(data['executable'])
    if INSTALLED.resolve()!=P(data['executable']) or sha(old_resolved)!=old_hash:raise RuntimeError('Activation verification failed')
    for path,digest in old_artifacts.items():
        if sha(P(path))!=digest:raise RuntimeError('Original mapped artifact changed: '+path)
    write_new(release/'activation_verified.json',{'verified_utc':utc(),'installed_resolved':str(INSTALLED.resolve()),
              'old_executable_preserved':True,'old_native_artifact_hashes_preserved':old_artifacts,'mapped_processes_after':mapped_fast()})
    print(target)
def rollback(record):
    data=json.loads(record.read_text())
    if not INSTALLED.is_symlink() or os.readlink(INSTALLED)!=data['new_target']:raise RuntimeError('Installed target changed; refusing rollback')
    old=P(data['previous_resolved'])
    if sha(old)!=data['previous_executable_sha256']:raise RuntimeError('Original executable changed')
    result=record.parent/'rollback_verified.json'
    if result.exists():raise RuntimeError('Rollback already recorded')
    atomic_link(data['previous_target'])
    write_new(result,{'rolled_back_utc':utc(),'installed_target':os.readlink(INSTALLED),'scope':'Atomic symlink rollback; no processes signaled'})
    print(result)
def main():
    ap=argparse.ArgumentParser();group=ap.add_mutually_exclusive_group(required=True)
    group.add_argument('--stage',type=P);group.add_argument('--activate',type=P);group.add_argument('--rollback',type=P)
    ap.add_argument('--expected-source');ap.add_argument('--runtime-evidence',type=P)
    a=ap.parse_args()
    if a.stage:
        if not a.expected_source or not a.runtime_evidence:ap.error('--stage requires --expected-source and --runtime-evidence')
        stage(a.stage,a.expected_source,a.runtime_evidence)
    elif a.activate:activate(a.activate)
    else:rollback(a.rollback)
if __name__=='__main__':main()
