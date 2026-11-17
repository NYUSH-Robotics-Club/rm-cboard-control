default:
    @just --list

build mode="Debug":
    python3 tools/diagnosis/manage.py build --mode {{mode}}

test:
    sh tests/host/run_tests.sh

# Explicitly programs the selected board, verifies, then starts passive firmware.
flash *args:
    python3 tools/diagnosis/manage.py flash {{args}}

# Keep this process running; no reset/halt/flash on live attachment.
connect *args:
    python3 tools/diagnosis/manage.py connect {{args}}

interface mode="Debug":
    python3 tools/diagnosis/interface.py --elf build/{{mode}}/CBoard_Diagnosis.elf

demo:
    python3 tools/diagnosis/interface.py --demo
