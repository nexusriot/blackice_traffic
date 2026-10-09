#!/usr/bin/env bash
# Build the hermetic image and run the end-to-end suite inside it.
#
#   tests/e2e/run.sh            both passes
#   tests/e2e/run.sh root       privileged pass only
#   tests/e2e/run.sh user       unprivileged pass only
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$here/../.." && pwd)"
image="${BLACKICE_E2E_IMAGE:-blackice-e2e}"
want="${1:-both}"

if ! command -v docker >/dev/null 2>&1; then
  echo "docker is not installed — the end-to-end suite needs it for the" >&2
  echo "private network namespace. The unit suite runs without it:" >&2
  echo "    make test" >&2
  exit 1
fi

echo "==> building $image"
docker build -q -t "$image" "$here" >/dev/null

# --network none is the whole point: a private namespace whose socket table
# holds nothing but what the tests put there. NET_ADMIN is for `ip addr add`,
# which gives us a non-loopback address to talk to. The project is mounted
# read-only, so anything the app writes has to go to HOME — which proves it
# never writes into its own install directory.
common=(
  --rm -i
  --network none
  -v "$root:/app:ro"
  -w /app
  --tmpfs /tmp:exec
  -e HOME=/tmp/home
)

status=0

if [ "$want" = both ] || [ "$want" = root ]; then
  echo "==> privileged pass (root: every socket's owner is visible)"
  docker run "${common[@]}" --cap-add=NET_ADMIN "$image" || status=$?
fi

if [ "$want" = both ] || [ "$want" = user ]; then
  echo "==> unprivileged pass (nobody: the attribution story users actually get)"
  docker run "${common[@]}" --user 65534:65534 "$image" || status=$?
fi

exit "$status"
