#!/usr/bin/env python3
from os import path
import sys
import os

# print('\033[0;33mThe "setup.py" installation method is planned to be abandoned.\n'
#     'Please execute "install.py" to install.\n\033[0m')

def run_command(cmd=""):
    import subprocess
    p = subprocess.Popen(cmd,
                         shell=True,
                         stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT)
    result = p.stdout.read().decode('utf-8')
    status = p.poll()
    return status, result


def install():
    _is_bsps = ''
    exit_code, _ = run_command("pip3 help install|grep break-system-packages")
    if exit_code == 0: # if true
        _is_bsps = "--break-system-packages"

    os.system(f"pip3 install ./ {_is_bsps} -v") # -v for verbose

if 'install' in sys.argv:
    here = path.abspath(path.dirname(__file__))
    os.chdir(here)
    args = ' '.join(sys.argv[1:])
    # os.system(f'python3 install.py {args}')
    install()
    exit()

# necessary for pip3 install ./ , 
# if you need both `setup.py`` and `pyproject.toml`` to exist
from setuptools import setup
setup()

