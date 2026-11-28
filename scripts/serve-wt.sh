#!/usr/bin/env bash
# Start the LAN server from a chosen git worktree.
# Usage: serve-wt.sh [main|<branch>|<name-fragment>|<path>] [-- extra serve.mjs args]
#   no arg -> interactive menu. Env: PORT (8765), HOST (0.0.0.0)
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mapfile -t lines < <(git -C "$here" worktree list --porcelain | awk '
  /^worktree /{p=substr($0,10)} /^branch /{b=$2; sub("refs/heads/","",b)} /^detached/{b="(detached)"}
  /^$/{if(p)print p"\t"b; p="";b=""} END{if(p)print p"\t"b}')
sel="${1:-}"; [[ $# -gt 0 ]] && shift
[[ "${1:-}" == "--" ]] && shift
pick=""
if [[ -n "$sel" ]]; then
  for l in "${lines[@]}"; do
    p="${l%%$'\t'*}"; b="${l#*$'\t'}"
    if [[ "$sel" == "$p" || "$sel" == "$b" || ( "$sel" == main && "$b" == main ) ]]; then pick="$p"; break; fi
  done
  if [[ -z "$pick" ]]; then
    for l in "${lines[@]}"; do [[ "$l" == *"$sel"* ]] && { pick="${l%%$'\t'*}"; break; }; done
  fi
  [[ -n "$pick" ]] || { echo "No worktree matches '$sel'" >&2; exit 1; }
else
  echo "Worktrees:"
  i=1; for l in "${lines[@]}"; do printf "  %d) %-14s %s\n" "$i" "${l#*$'\t'}" "${l%%$'\t'*}"; i=$((i+1)); done
  read -rp "Select [1-${#lines[@]}]: " n
  [[ "$n" =~ ^[0-9]+$ && n -ge 1 && n -le ${#lines[@]} ]] || { echo "Invalid" >&2; exit 1; }
  pick="${lines[n-1]%%$'\t'*}"
fi
cd "$pick"
echo "▶ Serving worktree: $pick"
exec node scripts/serve.mjs --host "${HOST:-0.0.0.0}" --port "${PORT:-8765}" "$@"
