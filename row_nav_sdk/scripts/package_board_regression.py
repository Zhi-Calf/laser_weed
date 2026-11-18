#!/usr/bin/env python3
"""Package a built ARM64/RKNN SDK and offline fixed images; no system modifications."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile

def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()

def require_arm(path):
    with path.open('rb') as f:h=f.read(20)
    if h[:6]!=b'\x7fELF\x02\x01' or int.from_bytes(h[18:20],'little')!=183:raise ValueError('Not ARM64 ELF: '+str(path))

def build(args):
    source=Path(__file__).resolve().parents[1]
    out=args.output.resolve();archive=Path(str(out)+'.tar.gz')
    if out.exists() or archive.exists():raise ValueError('Output exists')
    cache=(args.build/'CMakeCache.txt').read_text()
    if f'CMAKE_HOME_DIRECTORY:INTERNAL={source}\n' not in cache or 'RK_NAV_WITH_RKNN:BOOL=ON' not in cache:raise ValueError('Build must belong to this source and have RKNN enabled')
    samples=sorted(p for p in args.images.iterdir() if p.is_file() and p.suffix.lower() in ('.jpg','.jpeg','.png'))
    if args.count<2 or len(samples)<args.count:raise ValueError('Not enough input images')
    indices=[i*(len(samples)-1)//(args.count-1) for i in range(args.count)]
    selected=[samples[i] for i in indices]
    require_arm(args.runtime)
    with tempfile.TemporaryDirectory(prefix='board-kit-install-') as temp:
        install=Path(temp)
        subprocess.run(['cmake','--install',str(args.build),'--prefix',str(install)],check=True,stdout=subprocess.DEVNULL)
        require_arm(install/'bin/segmentation_replay');require_arm(install/'lib/librk_row_nav.so')
        info=(install/'sdk-build-info.txt').read_text()
        if 'processor=aarch64' not in info or 'rknn_enabled=ON' not in info:raise ValueError('Target metadata mismatch')
        version=next(line.split('=',1)[1] for line in info.splitlines() if line.startswith('SDK='))
        out.mkdir(parents=True)
        for n in ('bin','lib','include','models','images','reference-masks'): (out/n).mkdir()
        shutil.copy2(install/'bin/segmentation_replay',out/'bin/segmentation_replay')
        sdk=(install/'lib/librk_row_nav.so').resolve()
        shutil.copy2(sdk,out/'lib'/sdk.name)
        (out/'lib/librk_row_nav.so.1').symlink_to(sdk.name);(out/'lib/librk_row_nav.so').symlink_to('librk_row_nav.so.1')
        shutil.copy2(args.runtime,out/'lib/librknnrt.so')
        shutil.copy2(install/'include/rk_row_nav.h',out/'include/rk_row_nav.h')
        shutil.copy2(install/'sdk-build-info.txt',out/'sdk-build-info.txt')
        shutil.copy2(args.model,out/'models/model.rknn')
        if args.model.with_suffix(args.model.suffix+'.json').exists():shutil.copy2(args.model.with_suffix(args.model.suffix+'.json'),out/'models/conversion-info.json')
        shutil.copy2(source/'scripts/board/run_regression.py',out/'run_regression.py')
        shutil.copy2(source/'docs/board-regression-guide.md',out/'README.md')
        mapping=[]
        reference=json.loads((args.reference/'manifest.json').read_text())
        if reference.get('completed') is not True:raise ValueError('Reference incomplete')
        by_path={r['path']:(i,r) for i,r in enumerate(reference['inputs'])}
        for index,image in enumerate(selected):
            original_index,entry=by_path[str(image.resolve())]
            if sha(image)!=entry['sha256']:raise ValueError('Reference image changed')
            name=f'{index:03d}{image.suffix.lower()}'
            shutil.copy2(image,out/'images'/name)
            ref=args.reference/'masks'/f'{original_index}.png';shutil.copy2(ref,out/'reference-masks'/f'{index}.png')
            mapping.append(dict(index=index,image=name,source=str(image.resolve()),source_sha256=sha(image),reference_sha256=sha(ref),reference_index=original_index))
        file_hashes={str(p.relative_to(source)):sha(p) for p in sorted(source.rglob('*')) if p.is_file() and (p.suffix in ('.cpp','.hpp','.h','.cmake','.py','.txt','.sh') or p.name=='CMakeLists.txt') and '__pycache__' not in p.parts and 'build' not in p.parts}
        fingerprint=hashlib.sha256(json.dumps(file_hashes,sort_keys=True).encode()).hexdigest()
        metadata=dict(sdk_version=version,target='RK3588 ARM64 Linux',quantization='INT8',model_source=str(args.model.resolve()),model_sha256=sha(args.model),sample_count=len(mapping),
            sample_strategy='Evenly spaced indices of filename-sorted existing test split; not random accuracy sample',samples=mapping,
            reference_manifest_sha256=sha(args.reference/'manifest.json'),source_tree_sha256=fingerprint,source_files=file_hashes,
            qualification='Cross-compiled offline regression kit; real board execution pending; no camera/BEV/vehicle control test',
            dependencies='Board system OpenCV4.5 shared libraries and compatible NPU kernel driver; packaged librknnrt scoped to test process')
        (out/'package-info.json').write_text(json.dumps(metadata,indent=2)+'\n')
        hashes={str(p.relative_to(out)):sha(p) for p in sorted(out.rglob('*')) if p.is_file()}
        (out/'checksums.json').write_text(json.dumps(hashes,indent=2)+'\n')
        with tarfile.open(archive,'w:gz') as tar:tar.add(out,arcname=out.name)
        Path(str(archive)+'.sha256').write_text(sha(archive)+'  '+archive.name+'\n')
        return archive
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for n in ('build','model','runtime','images','reference','output'):p.add_argument('--'+n,type=Path,required=True)
    p.add_argument('--count',type=int,default=12)
    print(build(p.parse_args()))
