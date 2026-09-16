# Python acts as the recipe interpreter; Windows does not need sh or Git Bash.
set shell := ["python3", "tools/firmware.py", "--just"]
set windows-shell := ["py", "-3", "tools/firmware.py", "--just"]

default:
    help

configure robot mode="Debug" run="yes":
    configure {{robot}} --mode {{mode}} --run-after {{run}} --allow-single yes

doctor:
    doctor

build robot="":
    build {{robot}}

flash robot="":
    flash {{robot}}

flash-plan robot="":
    flash-plan {{robot}}

# Read the running board; first build/flash the firmware containing the snapshot.
monitor hz="20":
    monitor --hz {{hz}}

# Start the RTT telemetry dashboard in a browser.
logger *args:
    logger {{args}}

# Start the terminal RTT dashboard.
logger-cli *args:
    logger-cli {{args}}

lg *args:
    just logger {{args}}

lc *args:
    just logger-cli {{args}}

