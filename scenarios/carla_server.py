#!/usr/bin/env python3
"""Start CARLA in a chosen town, by rebooting it rather than load_world().

Why this exists
---------------
`client.load_world()` is unreliable on this install. UI.py has said so
since it was written — `set_boot_map`'s docstring reads "In-band
load_world() segfaults on this install — the boot-map ini is the only
reliable way" — and the scenario harness went and used it anyway, once
per site, because that was the obvious way to make a multi-site matrix
work.

Measured consequences, all of them attributed to the wrong thing at
first:

* A sweep that reached Town12 after Town03/04/10HD took the CARLA server
  down mid-load. Nine cells of twenty-seven had run (DEBUG §58).
* Every site group after the FIRST in a process produced no steering at
  all: camera frames kept flowing at ~14 fps and `/Car_1/cmd_steer` went
  stale for the whole run. Twenty-five of twenty-seven cells in the
  overnight sweep measured the harness's fallback controller rather than
  the LKAS, and read as lane-keeping results (DEBUG §59).

Both go away if the server is restarted into the new town instead. That
is what the operator was doing by hand — pick the town, Restart CARLA,
then run — and it is what this module automates.

The ADAS stack does NOT need restarting with it: a scenario process that
loads one town and runs works fine against a stack that has been up for
hours. What breaks is the second world inside one server session.
"""

from __future__ import annotations

import os
import re
import subprocess
import time
from pathlib import Path

# Same defaults UI.py uses. Kept here rather than imported because UI.py
# pulls in tkinter at module scope, and a headless scenario run should not
# need a display library to reboot a simulator.
CARLA_DIR = Path('/home/sirius/CARLA_0.9.16')
CARLA_INI = CARLA_DIR / 'CarlaUE4' / 'Config' / 'DefaultEngine.ini'
CARLA_SERVER = './CarlaUE4.sh'
# Large Maps stream their tiles in and take minutes on a cold cache.
BOOT_TIMEOUT_S = 420.0
LARGE_MAPS = ('Town11', 'Town12', 'Town13', 'Town15')


MAPS_DIR = CARLA_DIR / 'CarlaUE4' / 'Content' / 'Carla' / 'Maps'


def boot_map_value(town: str, maps_dir: Path = MAPS_DIR) -> str:
    """The /Game path CARLA needs in the ini for `town`.

    Stock towns are a single umap: Maps/Town04.umap -> /Game/Carla/Maps/
    Town04.Town04. Large Maps are a directory of streaming tiles with a
    master umap inside it: Maps/Town12/Town12.umap -> /Game/Carla/Maps/
    Town12/Town12.Town12.

    Writing the flat form for a Large Map does not fail loudly — the
    engine starts, cannot find the map, and exits. From the outside that
    is a server that never comes up, which is how a sweep sat waiting for
    Town12 with nothing but a defunct CarlaUE4.sh to show for it
    (DEBUG §60). Decided from the filesystem rather than a hardcoded
    list, so a newly installed Large Map works without editing this.
    """
    if (maps_dir / town / f'{town}.umap').exists():
        return f'/Game/Carla/Maps/{town}/{town}.{town}'
    return f'/Game/Carla/Maps/{town}.{town}'


def set_boot_map(town: str, ini_path: Path = CARLA_INI) -> str:
    """Point CARLA's three *.Map ini entries at `town` for the next boot.

    Mirrors UI.py's `set_boot_map` (UI.py:213) deliberately: the two
    processes both need it, and the alternative — importing UI.py — drags
    tkinter into a headless run. If the ini format ever changes, both
    copies need the same edit. UI.py's copy writes the flat form
    unconditionally — latent rather than broken, because its TOWNS list
    stops at Town10HD. Adding a Large Map to that dropdown needs this
    function's rule copied across.
    """
    if not ini_path.exists():
        raise RuntimeError(f"{ini_path} not found — cannot set the boot map")
    value = boot_map_value(town)
    text = ini_path.read_text()
    new_text, n = re.subn(
        r'(EditorStartupMap|GameDefaultMap|ServerDefaultMap)=.+',
        rf'\1={value}', text)
    if not n:
        raise RuntimeError(f"no *.Map entries found in {ini_path}")
    ini_path.write_text(new_text)
    os.sync()
    observed = re.search(r'ServerDefaultMap=(.+)', ini_path.read_text())
    got = observed.group(1).strip() if observed else '<missing>'
    if got != value:
        raise RuntimeError(
            f"{ini_path} did not retain the boot map: wrote {value}, "
            f"read back {got}")
    return value


def loaded_map(host: str = 'localhost', port: int = 2000,
               timeout_s: float = 5.0) -> str | None:
    """Short name of the map the running server has, or None if no server."""
    try:
        import carla
        client = carla.Client(host, port)
        client.set_timeout(timeout_s)
        return client.get_world().get_map().name.split('/')[-1]
    except Exception:
        return None


def is_town(map_name: str | None, town: str) -> bool:
    """Town12 reports as 'Town12/Town12' (Large Maps nest), the rest as
    'Town04'. Compare on the last path element either way."""
    return bool(map_name) and map_name.split('/')[-1] == town


def kill(wait_s: float = 5.0) -> None:
    for pattern in ('CarlaUE4-Linux-Shipping', 'CarlaUE4.sh'):
        subprocess.run(['pkill', '-f', pattern],
                       capture_output=True, check=False)
    deadline = time.time() + wait_s
    while time.time() < deadline:
        out = subprocess.run(['pgrep', '-f', 'CarlaUE4-Linux-Shipping'],
                             capture_output=True, text=True, check=False)
        if not out.stdout.strip():
            return
        time.sleep(0.5)


SERVER_LOG = Path('/tmp/carla_server_harness.log')


def start(town: str, port: int = 2000, quality: str = 'Epic',
          carla_dir: Path = CARLA_DIR) -> subprocess.Popen:
    """Boot a server that comes up already in `town`.

    Output goes to SERVER_LOG rather than /dev/null: when the boot map is
    wrong the engine says so and exits, and discarding that turned a
    one-line diagnosis into a silent wait.
    """
    cmd = [CARLA_SERVER, '-RenderOffScreen', f'-carla-rpc-port={port}',
           f'-quality-level={quality}']
    log = open(SERVER_LOG, 'wb')
    # start_new_session so a Ctrl-C in the harness does not take the
    # server with it — the run's own teardown decides that.
    return subprocess.Popen(cmd, cwd=str(carla_dir), start_new_session=True,
                            stdout=log, stderr=subprocess.STDOUT)


def wait_until_ready(town: str, host: str = 'localhost', port: int = 2000,
                     timeout_s: float = BOOT_TIMEOUT_S,
                     verbose: bool = True, proc=None) -> None:
    """Block until the server answers AND reports the expected town.

    `proc` is the handle from start(). A server that exits during boot is
    detected immediately rather than after the full timeout — the whole
    point of a timeout is the case where something is slow, not the case
    where it is already dead.
    """
    deadline = time.time() + timeout_s
    last = None
    while time.time() < deadline:
        if proc is not None and proc.poll() is not None:
            tail = ''
            try:
                tail = '\n    '.join(
                    SERVER_LOG.read_text(errors='replace').splitlines()[-8:])
            except Exception:
                pass
            raise RuntimeError(
                f"CARLA exited during boot into {town} (rc="
                f"{proc.returncode}). Boot map was set to "
                f"{boot_map_value(town)}.\n    {tail}")
        name = loaded_map(host, port, timeout_s=5.0)
        if is_town(name, town):
            if verbose:
                print(f"[carla] {town} ready after "
                      f"{timeout_s - (deadline - time.time()):.0f} s",
                      flush=True)
            return
        if verbose and name != last:
            last = name
            print(f"[carla] waiting for {town} "
                  f"(server says {name or 'not up yet'})", flush=True)
        time.sleep(3.0)
    raise RuntimeError(
        f"CARLA did not come up in {town} within {timeout_s:.0f} s. "
        f"{'Large Maps take minutes on a cold cache; ' if town in LARGE_MAPS else ''}"
        f"start it by hand and re-run with --no-carla-restart.")


def free_memory_gb() -> float:
    try:
        with open('/proc/meminfo') as fh:
            for line in fh:
                if line.startswith('MemAvailable:'):
                    return int(line.split()[1]) / 1024.0 / 1024.0
    except Exception:
        pass
    return float('inf')


# Free memory a Large Map needs, measured by watching one take the server
# down mid-load (DEBUG §58).
LARGE_MAP_MIN_FREE_GB = 8.0


def ensure_town(town: str, host: str = 'localhost', port: int = 2000,
                quality: str = 'Epic', force: bool = False,
                verbose: bool = True) -> bool:
    """Make the running server be in `town`, restarting it if it is not.

    Returns True if a restart happened. A server already in the right
    town is left alone — the point is to avoid load_world(), not to
    reboot for its own sake.
    """
    current = loaded_map(host, port)
    if not force and is_town(current, town):
        return False
    if verbose:
        print(f"[carla] restarting into {town} "
              f"(currently {current or 'no server'}) — in-band load_world "
              f"is not safe on this install", flush=True)
    # Checked before the reboot, not after: once the old server is down,
    # a machine that cannot hold the new map leaves no server at all.
    if town in LARGE_MAPS:
        free = free_memory_gb()
        if free < LARGE_MAP_MIN_FREE_GB:
            raise RuntimeError(
                f"{town} is a Large Map and needs about "
                f"{LARGE_MAP_MIN_FREE_GB:.0f} GB free; {free:.1f} GB is "
                f"available. Close what you can, or run the {town} sites "
                f"in their own session.")
    set_boot_map(town)
    # The caller has just closed its adapter; give any client threads it
    # released a moment to actually go away before the server does.
    time.sleep(1.0)
    kill()
    proc = start(town, port=port, quality=quality)
    wait_until_ready(town, host=host, port=port, verbose=verbose, proc=proc)
    return True
