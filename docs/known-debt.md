# Known debt

Refactors that were consciously deferred during the C++ → Rust migration and are
still open. Nothing here is a correctness bug; each entry names the smell, why it
was left alone, and the condition that should trigger the work.

## `BowlParams` is a large, multi-purpose value object

- **Where**: `src/bowl/params.rs` — 9-argument `new()` (:42-65),
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

- **Where**: `src/camera/model.rs:40-56` (`CameraModelBase::new`),
  `:118-123` (`project_points` triple), `:189-273` (`KbParams` / `LutParams` /
  `OcamParams`). Each model repeats `(width, height, cx, cy)` with a different
  field order and the batch projections repeat the `(points, uv, valid)` triple.
- **Why deferred**: the three models mirror the reference behaviour exactly and
  the public structs are deserialised from calibration JSON; a shared geometry
  struct is a schema decision, not a mechanical refactor.
- **When to handle**: when a fourth model or a new geometry field (pixel size,
  gate radius) is introduced, or when the calibration schema is versioned.

## The three projection skeletons are duplicated

- **Where**: `src/camera/model.rs:118` plus
  `src/camera/{kb.rs:61,lut.rs:49,ocam.rs:40}` — the same
  equal-length checks, loop and UV normalisation around a per-model core.
- **Why deferred**: the shared skeleton lives behind a trait and each model's
  inner kernel differs; extracting it now would only move three size checks.
- **When to handle**: when a fourth model lands or the batch path gains
  chunking/parallelism.

## Camera model type is a string

- **Where**: `src/camera/model.rs` (`make_camera_model("kb" |
  "table" | "ocam", …)`).
- **Why deferred**: the strings come straight from the calibration files and the
  error path already names the offending kind; a typed enum is a schema change
  that belongs with JSON versioning.
- **When to handle**: when the schema is versioned or a model type is added.

## Long render functions

- **Where**: `src/render/offscreen.rs:59-` (setup, target creation and
  the per-view loop in one function) and
  `src/render/windowed.rs:224` / `:501` (draw + present, capture +
  encode + save).
- **Why deferred**: both are linear, single-purpose sequences around GPU calls
  that are hard to unit test; splitting them would mostly move lines.
- **When to handle**: when MSAA, a second capture format, or a second render
  target is added.

## `configure_surface` takes five arguments and a mode flag

- **Where**: `src/render/windowed.rs:408` —
  `(window, surface, adapter, device, want_copy: bool)`, where the bool both
  changes the surface usage and can fail with a screenshot-domain error.
- **Why deferred**: there is one caller; the flag only means "this run wants a
  screenshot".
- **When to handle**: when a second copy consumer (recording, export) appears or
  the surface is configured from more than one place.

## Test helpers are duplicated across integration-test binaries

- **Where**: `tests/parity.rs:35` and
  `tests/offscreen.rs` (`to_render_basis`, default-view and
  clip construction), `tests/windowed_smoke.rs` (calibration →
  `BowlModel` → `SceneData` → render), and the `count_vivid` helpers in
  `tests/engine.rs`.
- **Why deferred**: the helpers were written per binary while the tests lived in
  five crates. `tests/common/mod.rs` can now be included from every test binary,
  but moving them changes four suites at once and B5 was a pure move; the
  duplication is stable and cheap in the meantime.
- **When to handle**: when a helper changes twice, or a third consumer appears.

## The VCS → render-basis remap is written four times

- **Where**: `src/render/vertex.rs:42-47` (`position` and `normal`),
  `src/render/controller.rs:11-14`, `src/engine.rs:98-100`, plus the copies in
  `tests/parity.rs` and `tests/offscreen.rs`. All implement
  `(x, y, z) -> (x, z, -y)`; `controller.rs` even carries a comment saying it is
  "the same remap as the renderer".
- **Why deferred**: before B5 these lived in three different crates, which was
  the reason for the copies. The merge makes one shared function possible, but
  two of the copies return `f32` and one `f64`, and touching the render path in
  a move-only batch would have mixed concerns.
- **When to handle**: the next change to the render basis or vertex packing; at
  that point fold all four into one function with a documented unit.

## The engine exposes test-only accessors

- **Where**: `src/engine.rs:197` (`is_configured`) and `:201`
  (`num_cameras`); their only callers are `tests/engine.rs`. Same shape:
  `src/bowl/model.rs:79` (`has_visible_vertices`), used only by
  `tests/bowl_model.rs` and `tests/calibration.rs`.
- **Why deferred**: they are harmless read-only queries and removing them means
  rewriting those tests around other observations.
- **When to handle**: when the engine gains external consumers, or the next
  public-API review.

## The engine's error text still uses reference-implementation names

- **Where**: `src/engine.rs:14-40` — `StitchEngine::Configure` (the method is
  `configure`), `StitchEngine::RenderFrame`, `Camera.model is null` (Rust has
  `Option`), `bowl_params.GridStep` (the accessor is `grid_step()`).
- **Why deferred**: the strings are asserted by the engine tests and originate
  in the reference implementation; changing them is a user-visible change, not
  a move.
- **When to handle**: with the next user-visible message pass, or when the
  engine gets a CLI front end.

## `naga` is a `render` dependency but only the tests use it

- **Where**: `Cargo.toml:11` and `:21`; `naga` has no production caller, only
  `tests/shader_validation.rs`. It was a dev-dependency before B5.
- **Why deferred**: the feature set is fixed by the migration plan, and moving
  `naga` to `[dev-dependencies]` would put it into every `cargo tree` closure
  (dev-dependencies cannot be feature-gated), weakening the CPU-only guard.
- **When to handle**: when `cargo-udeps`/`cargo-machete` joins CI, or when the
  shader validation moves into a build script.

## No configure-time validation runs without the `render` feature

- **Where**: `tests/engine.rs` begins with `#![cfg(feature = "render")]`
  because `StitchEngine` needs `SceneData`; the twelve `configure_*` cases never
  create a device but still disappear under `--no-default-features`
  (262 → 186 tests).
- **Why deferred**: separating validation from scene assembly is a behaviour-
  neutral refactor of `src/engine.rs`, not part of a move-only batch.
- **When to handle**: when validation grows a rule that a GPU-less consumer
  (a calibration checker, the CLI) should enforce.

## The two binaries duplicate the data-directory default and argument parsing

- **Where**: `src/bin/avm-cli.rs:7` and
  `examples/avm_viewer.rs:9` (`DEFAULT_DATA_DIR`), plus each
  binary's positional/flag loop.
- **Why deferred**: the two CLIs deliberately differ (one rejects `--windowed`
  with a pointer to the other), and the shared surface is a single constant.
- **When to handle**: when a third entry point appears or the flags grow beyond
  `--frames`/`--screenshot`.
