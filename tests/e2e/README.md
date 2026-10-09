# End-to-end tests

    make test-e2e            # both passes
    tests/e2e/run.sh root    # privileged only
    tests/e2e/run.sh user    # unprivileged only

## Why a container

Not for isolation from the developer's machine — for the **private network
namespace**.

Every test in `tests/` learns about the socket table from a mock. That proves
the code handles the records it is given; it cannot prove the records resemble
what the kernel produces. That gap is where this project's bugs have actually
lived: UDP listeners were missing from the "attack surface" tab for a whole
release because a mocked `status` field said `LISTEN` and a real one says
`NONE`. No amount of mock-based testing would ever have caught it.

On a real host `psutil.net_connections()` returns ~200 sockets belonging to a
browser, a sync client and whatever else is running, so a test can only assert
"our port is in there somewhere". Inside `--network none` the same call returns
**exactly what the test opened** — so `test_the_only_listeners_are_the_ones_we_opened`
is an assertion that can be written at all.

Nothing here is mocked. The tests open real sockets, move real bytes, and read
them back out of `/proc` and `ss`.

## What the container gets

| flag | why |
| --- | --- |
| `--network none` | private netns; the socket table is only ours |
| `--cap-add=NET_ADMIN` | `ip addr add 203.0.113.9/32 dev lo`, so there is a non-loopback peer to talk to (contacts to `127.0.0.0/8` are dropped by design) |
| `-v $root:/app:ro` | the project read-only, which also proves the app writes nothing into its own install directory |
| `-e HOME=/tmp/home` | config and history have somewhere to go |
| `--user 65534` (2nd pass) | the attribution story an unprivileged user actually gets |

## What it covers

- the namespace really is ours, and the inventory is exactly our sockets
- real TCP listener → `WORLD` + the sensitive-port note; real **UDP** bind is
  reported at all; a loopback bind is `LOCAL`
- listener diffing across a real open and a real close
- a real established connection becoming a contact, with its state
- **real byte counters**: transfer exactly 200,000 bytes, assert `ss`-derived
  RX/TX matches to the byte and reaches the contact table
- `SCOPE` under root vs unprivileged
- `--json` against a real socket table; `--watch` exit codes `0`/`3`/`4` driven
  by real sockets, including a port opening mid-run; `--duration 0` not stopping
  on its own and handling SIGINT
- history and config surviving a process restart, including a `SIGKILL`
- the full `MainWindow` starting, scanning and shutting its threads down

## What it does not cover, and why

- **Beacon detection** — needs 7 sightings ≥45s apart, so a faithful test would
  run for six minutes. It stays a unit test over synthetic timestamps.
- **GeoIP / ASN** — the MaxMind databases are not redistributable. A synthetic
  `.mmdb` would make this testable; until then the geo path is unit-tested.
- **Online geolocation and OSM tiles** — they are network calls by definition,
  and `--network none` is the point. Leaflet itself is vendored and unit-tested.
- **The system tray and `QtWebEngine`** — no tray in a container, and WebEngine
  would drag Chromium into the image for a tab these tests do not touch.

## Outside the container

`tests/e2e` is deliberately **not** a package, so `unittest discover -s tests`
cannot reach it; every test is additionally guarded by `BLACKICE_E2E=1`. The
normal suite is unaffected whether or not Docker is installed.
