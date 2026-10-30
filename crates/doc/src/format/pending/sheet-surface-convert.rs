//! Convert to Sheet Metal converts a body from its faces (#6748): a new feature type,
//! `sheet_surface_convert`, saves the body, its bend edges, the faces it skips as `skip` and the
//! material's side as `direction` (inside unless saved), with no fixed face. The model cache keeps
//! the converted body's flat pattern as `surface_flats`. An older design has neither and converts
//! as before (its conversions are Convert's thicken mode). The last release refuses the newer
//! design format instead of dropping them.
