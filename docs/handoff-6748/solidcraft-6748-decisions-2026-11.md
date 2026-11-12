---
name: solidcraft-6748-decisions-2026-11
description: "#6748 decisions made on the desktop (handoff README, 2026-11-11): PR takeovers, upstream flat PRs not ported, radius 0 refused, principle, contour arcs = rolls, edge identity = persistent name"
metadata:
  node_type: memory
  type: project
  originSessionId: 66d31e50-57b4-4f5c-ba3c-89ecc95d6ef2
  modified: 2026-11-12T14:27:53.220Z
---

Decisions Derek made on the desktop session for SolidCraft #6748 / PR #9245 (from
docs/handoff-6748/README.md on h0witzer/solidcraft branch wip/6748-main-merge, picked up on the
laptop 2026-11-12). See [[solidcraft-sheet-metal-rfc]], [[sheet-surfaces-are-the-definition]].

- Fork PRs #5, #10, #11, #7 taken over and pushed as fix-6748-develop 729f3cc7c; #10 closed with a comment.
- Upstream's flat-model sheet PRs (#15287, #15288, later #15365, #15346's code) are NOT ported; our
  merge replaces them, and they get rebased over the flat deletion. Their tests are the spec: port
  or replace them (#15288's test -> `a_bend_radius_of_0_is_refused_wherever_it_is_given`).
- Bend radius 0 is refused everywhere (`KNIFE_EDGE`): rule, fold, flange, hem gap 0. Reasons:
  unbuildable (end wall joins a line to an arc), its bend has no `bend.inner` face to name, and it
  isn't physical (-100% inside strain; flat vs folded volume off unless K = 1/2).
- The principle: sheet metal = surfaces + edges with attributes + rules, built in 3D; every
  feature is a generator feeding one builder. Posted on #6748 (comment 6816245987).
- Contour flange arcs are rolls at their own radius (bend vs roll is provenance bookkeeping).
- A flange edge's identity is its stored persistent edge name; inputs are keyed by it, never by
  pick coordinates or distances (TNP proposal change 1; `edge_pick_tolerance`/`keys_match` deleted).

**Why:** made on the desktop, existed only there.
**How to apply:** don't re-ask these; refuse radius 0, never key flange inputs by geometry, and
treat upstream flat-model sheet PRs as tests to port, not code to take.
