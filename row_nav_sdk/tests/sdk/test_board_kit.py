"""Kit integrity/failure collection tests, no hardware/NPU inference."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

script=Path(sys.argv[1]).resolve()
with tempfile.TemporaryDirectory() as t:
    root=Path(t);kit=root/'kit';kit.mkdir()
    shutil.copy2(script,kit/script.name)
    (kit/'package-info.json').write_text(json.dumps(dict(sdk_version='0.2.4',sample_count=2)))
    (kit/'sample.bin').write_bytes(b'known data')
    hashes={str(p.relative_to(kit)):hashlib.sha256(p.read_bytes()).hexdigest() for p in kit.iterdir()}
    (kit/'checksums.json').write_text(json.dumps(hashes))
    result=subprocess.run([sys.executable,str(kit/script.name),'--output',str(root/'inspect')],capture_output=True,text=True)
    assert result.returncode==0,result.stdout+result.stderr
    report=json.loads((root/'inspect/report.json').read_text())
    assert report['status']=='package_integrity_ok_not_board_test' and report['board_acceptance'] is False
    assert (root/'inspect.tar.gz').is_file()
    # Refuse overwrite without changing old result.
    assert subprocess.run([sys.executable,str(kit/script.name),'--output',str(root/'inspect')],capture_output=True).returncode
    (kit/'sample.bin').write_bytes(b'changed')
    result=subprocess.run([sys.executable,str(kit/script.name),'--run','--output',str(root/'tampered')],capture_output=True,text=True)
    assert result.returncode!=0
    report=json.loads((root/'tampered/report.json').read_text())
    assert 'Checksum mismatch' in report['error'] and 'inference' not in report
    assert (root/'tampered.tar.gz').is_file()
print('Kit integrity, tamper rejection, result archive and no false board acceptance passed')
