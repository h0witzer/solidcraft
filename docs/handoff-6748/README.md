# #6748 handoff: main merge in progress (drop this folder before merging)

Branch `wip/6748-main-merge` on h0witzer/solidcraft. `fix-6748-develop` is still 729f3cc7c (the
takeover push, fully verified). This branch = that + the merge with main + the work below. It is
NOT verified yet: do not push it to `fix-6748-develop` until the suites are green.

## On this branch (in order)
1. Merge of upstream/main at 01c86d9cb, then again at b532aa94b (both resolved to our side where
   main touched the flat model). Main now vendors solidcore in-tree (`solidcore/`).
2. Main's split-edge flange (13f2c2aa7) ported to surfaces (`flanges_on`, `segment_runs`,
   junction `slits`, `in_material` clips them); main's 22 segment tests pass.
3. Exact geometry: every sheet box/extent/containment read from curves (`seg_box`, `extremes`,
   `loop_box(_in)`, `holds`, `region_holds`, folded-zone `bent_box`, exact `extent_across`,
   develop `band_extent`); no more `polyline(1e-3)` bounds.
4. Through-all extrudes record their interval: text cut through a sheet keeps the font's curves
   (fixes the two DXF text tests).
5. Merge fixes: solidcore `eval_d1` (main moved pcurve into solidcore), typed replies, TOUCH_AREA.
6. Holes keep their exact cylinder when main's run-on trims a tool (#10695).
7. Flange inputs keyed by the edge's stored persistent name, never pick-point bits or distances
   (TNP proposal change 1); `edge_pick_tolerance`/`keys_match` deleted. Main's edit tests pass.
8. Bend radius 0 refused everywhere (`KNIFE_EDGE`): rule, fold, flange, hem gap 0. Main's #15288
   test replaced by `a_bend_radius_of_0_is_refused_wherever_it_is_given`.
9. Contour flange = generator of surfaces (line plane, arc roll at its own radius, other curves
   exact spline extrusion; seams at tangents, rule bends at sharp corners). Main's #15287 test
   ported (bend -> roll, same numbers).
10. Main's loft hopper test runs `sheet.recognize` (thin solid).

## Not yet on the branch
- `flat-delete.patch` here: flat model deleted (step 4), code-only, NOT compiled. Apply onto the
  branch with `git apply --3way docs/handoff-6748/flat-delete.patch` (made against the tree
  before the second main merge: where it conflicts in `crates/doc/src/sheet.rs`, take the
  patch's whole file, 825 lines). Then delete `crates/doc/src/sheet_flat_lineage.rs` (its `mod`
  line is already gone from lib.rs) and delete any flat items main added since
  (`contour_flange_path`, `SHARP_BEND`, empty-base checks in sheet.rs).
- `cargo fmt` (15 diffs across doc, engine, geom, io) once the code settles.

## Failing on the merged tree (engine), with causes
- `holes_in_a_contour_flanges_end_panels...` and the two `schema::sheet::...replies_match...`:
  a Hole near a wall / into a rim falls off `surface_hole_cut` onto the 3D path. Keeping the
  cylinder (item 6) was not it. Next: instrument `surface_hole_cut`'s bail-outs (eprintln at each
  `return Ok(Vec::new())`) and run those three tests; main's new hole code (`hole_run_on`,
  `kept_inside`, `hole_spots(doc,..)`) changed the tools.
- `a_ball_centred_on_a_rolls_face...`, `a_ball_stepped_through_a_rolls_face...`: main's newer
  kernel check refuses a B-spline with coincident control points from the sphere cut. Kernel
  side (now in-tree under `solidcore/`); passed before the merge.
- `round_cut_outs_cross_a_fold_across_slanted_edges`: solidcore 2D boolean status 1001, old.
- Not yet run on this head: the contour generator's tests, the loft test, UI, scenarios, kernel
  cases, oracle, clippy, xtask checks (layers, lists, mesh-decisions, licences, ui-containers,
  skills).
- Main's own non-sheet failures: the 11 in `known-nonsheet-engine-fail.txt` (handoff kit).

## Open decisions (Derek)
- Latent since step 1: `Feature::reads` / `change_from` used to refuse replaying a feature's
  change when the design had (flat) sheets; nothing checks `surface_sheets`, and `with_change`
  takes the sheet records from the new base, so a replayed cut/fillet/move on a sheet's body may
  keep a stale sheet record. Recommended: guard on `!st.surface_sheets.is_empty()` (safe).
- `sheet_surface::on_grid` snaps points to a 1 nm grid: conflicts with the no-snapping rule.
- Kernel case 296 (165 mm3 short) still undiagnosed.

## Decisions made this session (save to the laptop's memory)
- Takeover of fork PRs #5, #10, #11, #7 (done, pushed 729f3cc7c; #10 closed with a comment).
- Upstream flat-model sheet PRs (#15287, #15288) are not ported; our merge replaces them.
- Bend radius 0 refused: unbuildable (end wall joins a line to an arc), naming has no
  `bend.inner`, physics (-100% inside strain; flat vs folded volume off unless K = 1/2).
- The principle: sheet metal = surfaces + edges with attributes + rules, built in 3D; features
  are generators feeding one builder. Posted on #6748 (comment 6816245987).
- Contour flange arcs are rolls at their own radius (bend vs roll is provenance bookkeeping).
- Flange edge identity: the stored persistent edge name; inputs keyed by it (TNP change 1).

## Workflow
`merge-worker.sh` (T=<tree> for the target), `mergerun.sh` (engine + doc on wt-release with
CARGO_TARGET_DIR=target-rel), `takerun.sh` (probe, clippy, UI, scenarios, release CLI, kernel
cases, oracle) are copied here. UI runs skip `progressive_image_controls_route_to_the_registered_host`
(grows without bound).

## Main since b532aa94b (to take in the next merge, upstream/main 1f74b6834 or later)
- #15364 (977c7c753): contour curves listed out of order build (`ss.chain_order(curves)` before
  joining). TAKE IT in our generator: call `ss.chain_order(curves)?` before `contour_profile`
  in the contour arm of crates/doc/src/eval_more.rs; its test then runs on our generator. Its
  "lone folded panel's perforations" part is flat-model only (ours are on surfaces); its path
  flange help text change is fine.
- #15365 (a4e0a2c11): refines the zero-radius special case in the flat model. Not taken (radius 0
  is refused); replace its test as #15288's was.
- #15346 (7c1f81886, khanahmad4527): an Extrude Cut a hair off a flange's corner builds (107
  lines of flat-model code, not taken). Its 55-line test is main's spec: run it on our surface
  cuts and fix there if it fails.
- Open on main, all flat-model: #15357 (segment junction reliefs), #15296 and #14758 (flat
  pattern cut-out booleans).
