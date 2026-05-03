# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

MuJoCo Warp (MJWarp) is a GPU-optimized reimplementation of MuJoCo physics on top of [NVIDIA Warp](https://github.com/NVIDIA/warp). It targets the same simulation pipeline as MuJoCo / MJX but runs as Warp kernels for batched parallel worlds. The package is consumed both as a standalone library and as a backend for MJX and Newton.

## Common commands

Environment uses `uv`. The lockfile is committed and is verified by a pre-commit hook (`uv-lock`).

```bash
uv sync --all-extras                 # install dev deps from uv.lock
uv run pytest -n 8                   # full test suite (parallel)
uv run pytest mujoco_warp/_src/solver_test.py            # one file
uv run pytest mujoco_warp/_src/solver_test.py -k <name>  # one test
uv run pytest --cpu                  # force CPU device (default is GPU if available)
uv run pytest -k io_test --debug_mode   # debug-mode kernel compilation (CI runs this for io_test)
uvx ruff format .                    # format
uvx ruff check .                     # lint
uvx pre-commit run -a                # full pre-commit incl. kernel-analyzer + uv-lock
```

Custom pytest flags are defined in `mujoco_warp/conftest.py`: `--cpu`, `--verify_cuda`, `--lineinfo`, `--optimization_level`, `--debug_mode`, `--kernel_cache_dir`. They map directly onto `wp.config.*` toggles.

Console scripts (installed by the package):

```bash
mjwarp-viewer benchmarks/humanoid/humanoid.xml
mjwarp-testspeed benchmarks/humanoid/humanoid.xml --event_trace=True
```

Benchmark suite scripts live under `benchmarks/` (`run.sh` reads `config.txt`).

## Kernel analyzer (required, runs in CI)

`contrib/kernel_analyzer/` is a custom static analyzer that validates Warp kernel signatures against the field declarations in `mujoco_warp/_src/types.py`. It runs both as a pre-commit hook and as a CI job and **will block PRs**. Run locally with:

```bash
python contrib/kernel_analyzer/kernel_analyzer/cli.py mujoco_warp/_src/*.py --types mujoco_warp/_src/types.py
```

When adding/editing a `@wp.kernel`, keep its parameter names, ordering, and dtypes consistent with the canonical groupings used elsewhere (see neighboring kernels) — the analyzer enforces this convention. A VS Code plugin is shipped as `kernel-analyzer-*.vsix` in the same directory.

## Source layout

All implementation lives in a single flat module: `mujoco_warp/_src/`. The top-level `mujoco_warp/__init__.py` is the public API and re-exports a curated set of symbols from `_src` — when adding a new public function, also re-export it there.

Tests are colocated: every `foo.py` has a sibling `foo_test.py`. `pyproject.toml` sets `testpaths = ["mujoco_warp"]` and explicitly excludes `benchmarks/` and `contrib/` from test collection.

XML fixtures and meshes live in `mujoco_warp/test_data/` and are packaged with the wheel (`tool.setuptools.package-data`).

## Architecture

The pipeline mirrors MuJoCo's stages so anyone reading MuJoCo's C source can map files 1:1. Key modules:

- `types.py` — `Model`, `Data`, `Option`, `Constraint`, `Contact`, `State`, `Statistic`, plus all the MuJoCo-style enums (`JointType`, `GeomType`, `IntegratorType`, `SolverType`, `DisableBit`, …). The `Model`/`Data` dataclasses describe Warp array shapes; `types.py` is the single source of truth for kernel signature validation. `BlockDim` here centralizes `wp.launch_tiled` block-dim tuning constants.
- `io.py` — bridge between `mujoco.MjModel`/`MjData` and MJWarp's `Model`/`Data`. `put_model`, `put_data`, `make_data`, `reset_data`, `get_data_into` (write results back to an `MjData`), `set_const*`, `set_length_range`. This is where host arrays become Warp arrays.
- `forward.py` — `step`, `step1`, `step2`, `forward`, `fwd_position`, `fwd_velocity`, `fwd_acceleration`, `fwd_actuation`, integrators (`euler`, `rungekutta4`, `implicit`). `step` is the main entry.
- `smooth.py` — smooth dynamics: `kinematics`, `com_pos`, `com_vel`, `crb`, `factor_m`, `solve_m`, `rne`, `rne_postconstraint`, `transmission`, `tendon`, `flex`, `camlight`, `subtree_vel`.
- `constraint.py` / `solver.py` / `derivative.py` — constraint Jacobian assembly, the constraint solver, and analytic derivatives for the implicit integrator (see `docs/implicit_derivation_notes.md`).
- `inverse.py` — inverse dynamics.
- `passive.py`, `sensor.py`, `support.py` — passive forces; `sensor_pos`/`vel`/`acc` + `energy_*`; helpers (`jac`, `mul_m`, `xfrc_accumulate`, `contact_force`, `get_state`/`set_state`).
- Collision stack: `collision_driver.py` orchestrates broadphase + narrowphase. `bvh.py` provides BVH construction/refit. Narrowphase is split by primitive family: `collision_primitive*.py`, `collision_convex.py`, `collision_gjk.py` (GJK/EPA), `collision_sdf.py`, `collision_flex.py`. `collision_core.py` holds shared helpers.
- `island.py` — constraint islanding (graph partitioning of the constraint problem).
- `ray.py`, `render.py`, `render_util.py` — ray queries and the GPU batch renderer (uses Warp's BVH ray-tracing API; outputs RGB / depth / segmentation).
- `block_cholesky.py`, `math.py`, `util_misc.py`, `util_pkg.py`, `warp_util.py` — numeric kernels, math helpers, package/version utilities, and `event_scope` instrumentation that powers `--event_trace` in `testspeed`.

`testspeed.py` and `viewer.py` are not under `_src/` because they are user-facing entry points (the `mjwarp-testspeed` / `mjwarp-viewer` console scripts).

## Conventions worth knowing

- 2-space indent, line length 128, ruff-format enforced. Google docstring convention (`D100`/`D103` ignored — module/function docstrings optional).
- `wp.set_module_options({"enable_backward": False})` is set per-module in compute files; differentiability through Warp is **not** supported.
- Kernels deliberately do not use closures over module-level state; everything must come through kernel arguments matching the conventions enforced by the kernel analyzer.
- Sparse and dense mass-matrix paths and SAP/NXN broadphase variants coexist — many kernels have `_sparse`/`_dense` or `_tile`/`_segmented` siblings selected via enums in `types.py`. When editing one, check whether the sibling needs the same change.
- Public re-exports in `mujoco_warp/__init__.py` use the `from x import y as y` pattern to satisfy ruff's unused-import lint; preserve it when adding exports.
