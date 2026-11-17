#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
test_dir=$(mktemp -d)
trap 'rm -rf "$test_dir"' EXIT
${CC:-cc} -std=c11 -Wall -Wextra -Werror -Icore/diagnosis -Iservices/diagnosis -Ibsp/diagnosis \
    tests/host/test_diagnosis.c services/diagnosis/diagnosis.c -o "$test_dir/diagnosis"
"$test_dir/diagnosis"
${CC:-cc} -std=c11 -Wall -Wextra -Werror -Itests/host/stubs/diagnosis -Icore/diagnosis -Ibsp/diagnosis \
    tests/host/test_diagnosis_can.c bsp/diagnosis/diagnosis_can.c -o "$test_dir/can"
"$test_dir/can"
${CC:-cc} -std=c11 -Wall -Wextra -Werror -Icore/diagnosis -Iservices/diagnosis -Ibsp/diagnosis \
    tests/host/test_diagnosis_terminal.c services/diagnosis/diagnosis_terminal.c -o "$test_dir/terminal"
"$test_dir/terminal"
python3 -m unittest discover -s tools/diagnosis -p 'test_*.py'
python3 -m unittest discover -s tools -p 'test_stlink_usb.py'
