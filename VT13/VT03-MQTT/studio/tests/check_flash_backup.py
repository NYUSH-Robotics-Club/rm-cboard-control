"""Offline ELF32 PT_LOAD vs raw Flash dump; does not attach to the target."""
import hashlib
import json
from pathlib import Path
import struct
import sys
def check(elf_path, dump_path):
    elf=Path(elf_path).read_bytes();raw=Path(dump_path).read_bytes()
    if elf[:6]!=b'\x7fELF\x01\x01':raise ValueError('ELF32 LE required')
    offset=struct.unpack_from('<I',elf,28)[0]
    size,count=struct.unpack_from('<HH',elf,42)
    checked=[]
    for i in range(count):
        kind,file_offset,_,physical,file_size,_,_,_=struct.unpack_from('<8I',elf,offset+i*size)
        if kind!=1 or not file_size or not 0x08000000<=physical<0x08100000:continue
        position=physical-0x08000000
        actual=raw[position:position+file_size];expected=elf[file_offset:file_offset+file_size]
        if len(actual)!=file_size or actual!=expected:raise ValueError(f'Flash mismatch at {physical:#x}')
        checked.append({'address':hex(physical),'bytes':file_size})
    if not checked:raise ValueError('no Flash segments checked')
    return {'passed':True,'dump_bytes':len(raw),'elf_sha256':hashlib.sha256(elf).hexdigest(),'dump_sha256':hashlib.sha256(raw).hexdigest(),'segments':checked}
if __name__=='__main__':print(json.dumps(check(*sys.argv[1:3])))
