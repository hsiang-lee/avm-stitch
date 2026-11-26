# Known debt

Refactors that were consciously deferred during the C++ → Rust migration and are
still open. Nothing here is a correctness bug; each entry names the smell, why it
was left alone, and the condition that should trigger the work.

## `BowlParams` is a large, multi-purpose value object

- **Where**: `crates/avm-core/src/bowl/params.rs` — 9-argument `new()` (:42-65),
  `for_car_size` with unnamed scale constants (:67-82), mesh generation
  (:143-195) and `compute_bowl_normals` (:261-328), view helpers (:197-215),
  clip planes (:217-236).
- **Why deferred**: it is a faithful port of the reference geometry and every
  field is public API; splitting it changes the shape of the algorithm crate for
  no behavioural gain at a point where parity was the goal.
- **When to handle**: when a new bowl parameter or a second platform shape is
  added, or when an external consumer needs to build bowl parameters without
  `for_car_size`.

## Camera-model parameters are data clumps and diverge per model

- **Where**: `crates/avm-core/src/camera/model.rs:40-56` (`CameraModelBase::new`),
  `:118-123` (`project_points` triple), `:189-273` (`KbParams` / `LutParams` /
  `OcamParams`). Each model repeats `(width, height, cx, cy)` with a different
  field order and the batch projections repeat the `(points, uv, valid)` triple.
- **Why deferred**: the three models mirror the reference behaviour exactly and
  the public structs are deserialised from calibration JSON; a shared geometry
  struct is a schema decision, not a mechanical refactor.
- **When to handle**: when a fourth model or a new geometry field (pixel size,
  gate radius) is introduced, or when the calibration schema is versioned.

## The three projection skeletons are duplicated

- **Where**: `crates/avm-core/src/camera/model.rs:118` plus
  `crates/avm-core/src/camera/{kb.rs:61,lut.rs:49,ocam.rs:40}` — the same
  equal-length checks, loop and UV normalisation around a per-model core.
- **Why deferred**: the shared skeleton lives behind a trait and each model's
  inner kernel differs; extracting it now would only move three size checks.
- **When to handle**: when a fourth model lands or the batch path gains
  chunking/parallelism.

## Camera model type is a string

- **Where**: `crates/avm-core/src/camera/model.rs` (`make_camera_model("kb" |
  "table" | "ocam", …)`).
- **Why deferred**: the strings come straight from the calibration files and the
  error path already names the offending kind; a typed enum is a schema change
  that belongs with JSON versioning.
- **When to handle**: when the schema is versioned or a model type is added.

## Long render functions

- **Where**: `crates/avm-render/src/offscreen.rs:59-` (setup, target creation and
  the per-view loop in one function) and
  `crates/avm-viewer/src/windowed.rs:224` / `:501` (draw + present, capture +
  encode + save).
- **Why deferred**: both are linear, single-purpose sequences around GPU calls
  that are hard to unit test; splitting them would mostly move lines.
- **When to handle**: when MSAA, a second capture format, or a second render
  target is added.

## `configure_surface` takes five arguments and a mode flag

- **Where**: `crates/avm-viewer/src/windowed.rs:408` —
  `(window, surface, adapter, device, want_copy: bool)`, where the bool both
  changes the surface usage and can fail with a screenshot-domain error.
- **Why deferred**: there is one caller; the flag only means "this run wants a
  screenshot".
- **When to handle**: when a second copy consumer (recording, export) appears or
  the surface is configured from more than one place.

## Test helpers are duplicated across integration-test binaries

- **Where**: `crates/avm-render/tests/parity.rs:35` and
  `crates/avm-render/tests/offscreen.rs` (`to_render_basis`, default-view and
  clip construction), `crates/avm-viewer/tests/windowed_smoke.rs` (calibration →
  `BowlModel` → `SceneData` → render), and the `count_vivid` helpers in
  `avm-engine/tests/engine.rs`.
- **Why deferred**: integration tests in different crates cannot share a module
  without a new test-utility crate, and adding one crate for four helpers is
  more structure than the duplication costs today.
- **When to handle**: when a helper changes twice, or a third consumer appears.

## `avm-engine` exposes test-only accessors

- **Where**: `crates/avm-engine/src/lib.rs:197` (`is_configured`) and `:201`
  (`num_cameras`); their only callers are `crates/avm-engine/tests/engine.rs`.
- **Why deferred**: they are harmless read-only queries and removing them means
  rewriting those tests around `render_frame` errors.
- **When to handle**: when the engine gains external consumers, or the next
  public-API review.

## The two binaries duplicate the data-directory default and argument parsing

- **Where**: `crates/avm-cli/src/main.rs:7` and
  `crates/avm-viewer/src/bin/avm-viewer.rs:9` (`DEFAULT_DATA_DIR`), plus each
  binary's positional/flag loop.
- **Why deferred**: the two CLIs deliberately differ (one rejects `--windowed`
  with a pointer to the other), and the shared surface is a single constant.
- **When to handle**: when a third entry point appears or the flags grow beyond
  `--frames`/`--screenshot`.
