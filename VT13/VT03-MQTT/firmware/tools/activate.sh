# Source from Bash to activate this checkout's Python and saved firmware tools.
# Changes only this shell; missing configuration leaves PATH unchanged.
_rm_repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [ ! -f "$_rm_repo/.firmware.local.json" ] || [ ! -f "$_rm_repo/.venv/bin/activate" ]; then
    echo "Run bootstrap and configure, and create .venv before activation." >&2
    unset _rm_repo
    return 1
fi
_rm_tool_path=$("$_rm_repo/.venv/bin/python" -c 'import json, os, pathlib, sys; c=json.load(open(sys.argv[1])); print(os.pathsep.join(dict.fromkeys(str(pathlib.Path(p).parent) for p in c["tools"].values())))' "$_rm_repo/.firmware.local.json") || return 1
source "$_rm_repo/.venv/bin/activate"
export PATH="$_rm_repo/.venv/bin:$_rm_tool_path:$PATH"
unset _rm_repo _rm_tool_path
