#!/usr/bin/env python

if __name__ == "__main__":
    with open('ackerman_6_wheels.py', 'rb+') as f:
        content = f.read()
        f.seek(0)
        f.write(content.replace(b'\r', b''))
        f.truncate()
        
    with open('arm_and_mast.py', 'rb+') as f:
        content = f.read()
        f.seek(0)
        f.write(content.replace(b'\r', b''))
        f.truncate()
