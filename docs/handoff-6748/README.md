# #6748 handoff, laptop -> 3090 (2026-11-12 evening). Drop this branch; never merge it.

PR #9245 is OUT OF DRAFT, head = `fix-6748-develop` @ df756ee77 on h0witzer/solidcraft
(fast-forward from 729f3cc7c; the three old WIP handoff commits are gone, upstream SHAs kept).
This branch = that + this folder + two solidcore reproducers (untracked on the laptop).

## Waiting on
- A maintainer (jyviko) must add `approved-for-ci` to #9245 and approve the fork's workflow run.
  CI runs only on Blacksmith runners; the fork cannot run it. Asked on #9245 (comment 6838509558).
- Hold request for flat-model sheet merges posted on #6748 (comment 6836473314).
  Heads-ups posted on #15365 (unusuallyhandsome) and #13906 (cvengels, foldable print: rebase
  onto the surface converter after #9245).

## Do next (in order)
1. `git fetch origin && git checkout fix-6748-develop` (NOT wip/6748-main-merge: stale, has WIP).
2. Step f on the 3090 (was stopped on the laptop part-way):
   - fmt: done and pushed. Note `cargo fmt -p` hits Windows os error 206; rustfmt files directly.
   - clippy -D warnings (doc, engine, develop, ui-egui, kernel): PASSED on the laptop.
   - xtask layers, licences, ui-containers, skills: passed. `xtask licences` REWRITES
     THIRD-PARTY-LICENSES.txt (+2784/-1918 lines here); reverted on the laptop, unverified:
     check whether CI wants it regenerated after the main merge.
   - xtask mesh-decisions: fixed and pushed (7 stale entries removed).
   - xtask lists: FAILS. Main no longer takes new kernel cases in tests/kernel-cases/; our
     branch adds `sheet-metal/7278-inside-flange-along-the-slanted-edge-a-cut-left` and
     `.../7278-outside-...`. Move them to solidcore/crates/solidcore/tests/kernel_cases/<area>/
     (`cargo xtask kernel-cases --area <area> --export DIR`) or make them engine tests; then
     delete the drafts. See the xtask message.
   - Not run yet: full engine lib, kernel, UI (`-- sheet dialogs_all_tests command_help
     workspace i18n --skip progressive_image_controls_route_to_the_registered_host`),
     scenarios `sheet_metal`, solidcore clippy + lib tests, release CLI, sheet-metal kernel
     cases, oracle. Laptop green so far: engine `sheet`+`loft` 436 pass / 3 ignored, doc 662,
     develop 35, solidcore `surface::` 736, workspace check clean.
   - Known non-sheet engine failures from main (not ours): the list in the old kit's
     baselines/known-nonsheet-engine-fail.txt.
3. Push fixes to fix-6748-develop (fast-forward), post the local result on #9245.

## Follow-ups after landing
- Ball-on-roll cuts (2 tests #[ignore]d): solidcore trace leaves coincident control points at a
  sphere pole lying on a cylinder. Reproduces in `crates/kernel/tests/ball_on_a_rolls_face.rs`
  (set BALL_DUMP_DIR to write .smb operands) and replays in solidcore with
  `solidcore/crates/solidcore/tests/ball_operands_from_solidcraft.rs` (BALL_ROLL_SMB,
  BALL_BALL_SMB, BALL_FIX_OFF, BALL_SKIP_UPDATE). Bisect done: not any single preset fix, not the
  tolerance update, not WR-751's split_parameter. `ssi_pole_point_keeps_its_tangent` and
  `keep_operand_uv_trims` off change the failure: look at the trace at the pole.
- Convert free-form (B-spline) faces: `Shape::of` in sheet_surface_convert.rs refuses them
  ("not supported yet: a free-form face"); develop already has `Surface::Spline`. Derek's demo
  target: `Pumpkin (1).step` (Downloads on the laptop; re-export from Onshape): 56 ruled (3,1)
  and 12 bicubic faces; convert only the curved (outer) faces of "Pumpkin" and "Part 2".
- `on_grid` 1 nm snap: replace with shared corners computed once in the develop (after landing).
- Replay reuse: guarded on any surface sheet for now; Derek wants stable identities to cover
  rebuilds instead.
- solidcore 2D boolean is asymmetric on near-coincident curves (near-far empty, far-near errors);
  `prism_curve.rs` `arc()` accepts a spline as an arc within body tolerance (rule conflict).

## Decisions (Derek) — copies of the laptop memory are in this folder
solidcraft-6748-decisions-2026-11.md (desktop) and solidcraft-6748-landing-calls.md (today).

Standing rules: tests are the spec (change only flat-first behavior, numbers by hand); no flat
code; exact forms only; no snapping/welding; no index/order/coordinate identity, no base panel;
thickness and radii from the rule; never "skin"; US English; ask before outward actions
(Derek said "whatever gets us in fastest" for landing #9245); one builder runs cargo.
