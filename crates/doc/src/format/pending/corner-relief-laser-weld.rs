//! A sheet-metal rule's `corner_relief_shape`, and an edge flange's own corner relief shape, may
//! be `laser_weld`: the bends close tight against the seam between their flanges, as at a tear,
//! and a round hole the rule's `corner_relief` across is cut where the bends cross (#12651). An
//! older design never has it and builds as it did. The last release opens a design that has it
//! with a newer-format warning, and a feature whose rule names it fails with "unknown corner
//! relief shape `laser_weld`" until the rule is set to a shape that release knows.
