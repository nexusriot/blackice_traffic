# Hermetic end-to-end environment for BLACK ICE.
#
# The point of the container is not isolation from the developer's machine —
# it is the private network namespace. On a real host psutil reports some two
# hundred sockets belonging to a browser, a sync client and whatever else is
# running, so a test can only ever assert "our port is somewhere in there".
# In here the socket table contains exactly what the test created, which is
# what makes assertions like "these are the only listeners" possible at all.
FROM python:3.12-slim-bookworm

# iproute2  — `ss`, for the per-socket byte counters, and `ip`, to put a
#             non-loopback address on lo so contacts are not filtered out
# libglib/gl/egl/xkbcommon/dbus/fontconfig — what PyQt6 needs even offscreen
RUN apt-get update && apt-get install -y --no-install-recommends \
      iproute2 \
      procps \
      libglib2.0-0 \
      libgl1 \
      libegl1 \
      libxkbcommon0 \
      libdbus-1-3 \
      libfontconfig1 \
      fonts-dejavu-core \
 && rm -rf /var/lib/apt/lists/*

# Pinned to match requirements.txt. PyQt6-WebEngine, geoip2 and pyinstaller
# are deliberately absent: the first drags in Chromium for a tab these tests
# do not exercise, and the other two have nothing to do without a MaxMind
# database or a build.
RUN pip install --no-cache-dir psutil==7.1.3 PyQt6==6.7.1

ENV QT_QPA_PLATFORM=offscreen \
    PYTHONDONTWRITEBYTECODE=1 \
    BLACKICE_E2E=1

WORKDIR /app
# tests/e2e is deliberately not a package, which is also why
# `unittest discover -s tests` cannot reach these from the normal suite.
CMD ["python3", "-m", "unittest", "discover", "-s", "tests/e2e", "-t", "tests/e2e", "-v"]
