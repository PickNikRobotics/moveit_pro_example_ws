# Localization analysis harness

Offline analysis and stress tooling for `hangar_sim`'s localization stack. These are run **from
the source tree** with `python3 <tool>.py` / `./<tool>`; they are not installed ROS programs and
nothing in the running stack depends on them.

They exist because a localization number from this simulator is meaningless without the
real-time factor it was measured at, and because the divergence they were built to chase is
triggered by the RTF *changing* rather than being low — see the "Simulated sensors, controllers
and state estimation" section of the repository's `AGENTS.md`.

## The tools

| Tool | What it is for |
| --- | --- |
| `recorder4.py` | Runs inside the runtime container and records the localization state (truth, `/odom`, `/odom_filtered`, `map -> odom`, the particle cloud, `/initialpose`) to a JSONL file. Every analysis tool below parses its output. |
| `navloop_ab.py` | Drives the shipped navigation Objectives back to back over `/do_objective`, answering the UI prompts headlessly, alternating seed-covariance arms start by start inside one continuous session. |
| `abrun` | Host-side wrapper: copies `recorder4.py` and `navloop_ab.py` into the runtime container, starts the recorder, runs the drive loop, then pulls the recording back out. |
| `load` | Puts host CPU load on the box so the simulator sits at a chosen RTF. `./load <n_burners>` / `./load off`. |
| `loadstepper` | Watches the counter `navloop_ab.py` writes before each start and toggles the burner count, producing a deliberate load *step* per start. |
| `rtf.py` | Recovers the real-time factor from a recording alone, by integrating the wheel-odometry path against the ground-truth path. |
| `armsumm.py` | Splits a recording into per-start episodes and summarizes each arm. A dependency of `adjudicate.py`. |
| `hfresh.py` | Composes `map -> base` from `map -> odom` and `/odom_filtered` at one instant, instead of a `tf` lookup that comes back stale under load. A dependency of `adjudicate.py`. |
| `settled.py` | Reports what the filter holds once converged — sampled stopped, but well after both a re-seed and some driving, because `beluga` only resamples while the robot moves. |
| `adjudicate.py` | Classifies every heading excursion as a genuine particle-filter divergence or a `fuse` estimator stall. |

`armsumm.py` and `recorder4.py` are dependencies, not optional extras: `adjudicate.py` imports
`episodes`/`load` from `armsumm` and `annotate` from `hfresh`, and every analysis tool parses
`recorder4.py`'s output format.

## Load-stepping recipe

1. Put the box at the real-time factor you are chasing and keep it there:

   ```bash
   ./load 6
   ```

2. Run the host-side stepper, which watches a counter the driver writes and toggles burners.
   It prints the absolute path it is watching -- check that line:

   ```bash
   LO=6 HI=24 ./loadstepper &
   # loadstepper: watching <workspace>/log/.loadstep, toggling 6 <-> 24 burners
   ```

3. Drive the shipped Objectives back to back with arms interleaved, stepping load before each.
   `abrun` prints the container-side and host-side paths for the same counter, and the driver
   prints the path it writes on the first step -- they must be the same file:

   ```bash
   LOADSTEP=1 WARMUP=60 RESCUE=2.0 ./abrun <label> 60 baseline,tight
   # == load-step counter: container writes <USER_WS>/log/.loadstep ==
   # == load-step counter: loadstepper must watch <workspace>/log/.loadstep (same file, host side) ==
   ```

4. Adjudicate every episode as a real divergence or an estimator stall, and always report the
   RTF you measured at:

   ```bash
   THRESH=20 python3 adjudicate.py runs/<label>.jsonl
   python3 rtf.py runs/<label>.jsonl
   ```

`navloop_ab.py` drives the A/B by rewriting the real Objective files in place: a `baseline` start
runs the pre-fix seed (no variance ports) and a `tight` start runs the committed one, which is the
experiment itself. So that a run cannot leave the pre-fix seed behind, it reads both files' exact
bytes at startup before writing anything and restores that snapshot on every exit path -- normal
completion, an exception, and SIGINT/SIGTERM -- announcing on the first line that it is doing so.
The snapshot is deliberately not a `git checkout`: this runs inside the runtime container against a
bind-mounted worktree whose `.git` points at a host path that does not exist there, on an image that
need not ship git. Restoring the startup bytes also means a tree with uncommitted edits is safe --
your own edits come back, not someone's idea of the committed content.

The stepper runs on the **host** while the driver runs **inside the runtime container**, so the
two name the same bind-mounted counter by different paths: `<workspace>/log/.loadstep` on the host,
`$USER_WS/log/.loadstep` in the container. `abrun` reads the container's `USER_WS` and passes the
container-side path in explicitly so they cannot drift; override both with `LOADSTEP_FILE` (host)
and `CONTAINER_LOADSTEP` (container) if your layout differs. A stepper pointed at the wrong file
never fires and produces a steady-load run that looks like a stepped one, so `loadstepper` warns
loudly if nothing writes its file within `GRACE` seconds (default 180).

`navloop_ab.py`'s `--load-settle` defaults to 20 s because the observed divergences appeared
26-29 s after their load step. `RESCUE=2.0` re-seeds the filter on truth when it has been left
more than 2 m out, with a covariance no arm uses, so the analysis cannot confuse a rescue seed
with an Objective's own seed.

The honest limit: a 4x load step produced divergences at roughly 25-33% of starts, a 1.6x step
produced none, and it still does not fire on demand. Plan for a run long enough to catch several,
and report the RTF alongside whatever you conclude.

## The captured session is not in this repository

The live operator session that motivated this work is deliberately not committed here. It is
private session data, and project memory is the wrong home for it. What the project keeps instead
is the instrument -- the recorder, the analysis tools, and the load-stepping recipe above -- which
is what lets anyone produce and adjudicate their own capture of the failure rather than take a
single recording on trust.

## Paths

The hardcoded paths are gone; each tool takes its locations from the environment:

- `abrun` — `HARNESS_DIR` (defaults to the script's own directory), `OUT_DIR` (defaults to
  `./runs`), `CYCLONEDDS_URI`, `INST`, plus `CONTAINER_WS` (defaults to the runtime container's
  own `USER_WS`) and `CONTAINER_LOADSTEP` for the container side of the load-step counter.
- `loadstepper` — `HARNESS_DIR`, `WORKSPACE` (defaults to the workspace root derived from the
  harness directory, **not** `$PWD`, so running it from here still watches
  `<workspace>/log/.loadstep`), `LOADSTEP_FILE`, `LO`/`HI`, and `GRACE` (seconds before it warns
  that nothing has written the counter; default 180).
- `navloop_ab.py` — `USER_WS` (the workspace root the container image already exports), which
  supplies the defaults for `OBJ_DIR` and `LOADSTEP_FILE`; either can be overridden directly, and
  `LOADSTEP_FILE` also has a `--loadstep-file` flag.

Recordings land in `runs/` beside these tools, which is gitignored on purpose — see the decision
above: a capture is private session data and must not be committed here.
