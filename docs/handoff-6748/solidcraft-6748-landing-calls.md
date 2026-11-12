---
name: solidcraft-6748-landing-calls
description: "#9245 landing calls 2026-11-12 (laptop): grazing cuts refused, ball cuts ignored + fixed after, replay guarded on any sheet, on_grid after landing; future: stable identities should cover rebuilds"
metadata:
  node_type: memory
  type: project
  originSessionId: 66d31e50-57b4-4f5c-ba3c-89ecc95d6ef2
  modified: 2026-11-12T20:09:32.815Z
---

Calls Derek made on 2026-11-12 while landing SolidCraft #6748 / PR #9245 (see
[[solidcraft-6748-decisions-2026-11]], [[sheet-surfaces-are-the-definition]]):

- #15346 (cut a hair off a flange corner): no snapping. An extrude cut passing within the body's
  working tolerance (max(size*1e-6, 1e-5), the sew's) of a sheet corner but not through it is
  refused by name (`surface_corner_graze_refused` in eval_more.rs).
- Bend shadow volumes (no exact NURBS form in the flat): tests bound the error by
  T x outline length x the shadow's read sag, worked by hand; investigate if it doesn't fit.
- Ball-on-roll cuts (sphere pole on a cylinder): solidcore trace bug since main's kernel merge;
  land with both tests #[ignore] naming the issue, fix in solidcore after landing.
- Replay reuse: guard on any surface sheet (as the flat model did). Derek's direction: in future,
  stable identities should cover rebuilds instead of turning reuse off.
- `on_grid` (1 nm snap of flat pieces so shared corners coincide): leave until after landing,
  "potentially never"; the clean fix is each shared corner computed once in the develop.

**Why:** made to get #9245 out before more flat-model PRs land upstream.
**How to apply:** don't re-ask; after landing, the follow-ups are the ball trace, on_grid, and
per-feature replay via stable identities.
