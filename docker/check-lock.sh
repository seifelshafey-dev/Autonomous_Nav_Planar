#!/bin/sh
# Usage: docker/check-lock.sh <image>
# Exit 1 if the image has any package/version not in docker/apt-lock.txt (lock is generated from the
# sim image, a superset of ci, so lock-only entries are expected for the ci image).
set -eu
img=${1:?usage: check-lock.sh <image>}
here=$(dirname "$0")
now=$(mktemp); lock=$(mktemp)
trap 'rm -f "$now" "$lock"' EXIT
docker run --rm --entrypoint dpkg-query "$img" -W -f='${Package}=${Version}\n' | LC_ALL=C sort > "$now"
tail -n +2 "$here/apt-lock.txt" | LC_ALL=C sort > "$lock"
extra=$(LC_ALL=C comm -13 "$lock" "$now")
if [ -n "$extra" ]; then echo "drift: in image, not in lock:"; echo "$extra"; exit 1; fi
echo "lock OK"
