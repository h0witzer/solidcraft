//! World-frame metadata at a timeline body ownership boundary.
use solidcore_record::Plane;
use solidcraft_geom::prelude::*;

use crate::{DocError, Mat, ModelState, Result, apply_point, apply_vector};

pub(crate) fn remap_sheet_metadata(st: &mut ModelState, names: &[String], transform: &Mat) -> Result<()> {
    if names.len() > 1000 || st.sheets.len().saturating_add(st.threads.len()) > 100_000 || !transform.iter().flatten().all(|v| v.is_finite()) {
        return Err(DocError::Invalid("body transfer metadata exceeds its finite processing budget".into()));
    }
    let names: std::collections::BTreeSet<&str> = names.iter().map(String::as_str).collect();
    if !st.sheets.iter().any(|s| names.contains(s.body.as_str()))
        && !st.surface_sheets.iter().any(|s| names.contains(s.body.as_str()))
        && !st.threads.iter().any(|t| names.contains(t.body.as_str()))
    {
        return Ok(());
    }
    if !crate::valid_transfer_matrix(transform) {
        return Err(DocError::Invalid("sheet and thread transfer requires a rigid placement".into()));
    }
    let frames: Vec<_> = st
        .sheets
        .iter()
        .enumerate()
        .filter(|(_, s)| names.contains(s.body.as_str()))
        .map(|(i, s)| {
            let frame = Plane::new(apply_point(transform, s.frame.origin), apply_vector(transform, s.frame.x), apply_vector(transform, s.frame.y))
                .ok_or_else(|| DocError::Invalid("invalid transferred sheet frame".into()))?;
            Ok((i, frame))
        })
        .collect::<Result<_>>()?;
    let threads: Vec<_> = st
        .threads
        .iter()
        .enumerate()
        .filter(|(_, t)| names.contains(t.body.as_str()))
        .map(|(i, t)| {
            let point = apply_point(transform, t.axis_point);
            let axis = apply_vector(transform, *t.axis).unit().ok_or_else(|| DocError::Invalid("invalid transferred thread axis".into()))?;
            if !point.is_finite() {
                return Err(DocError::Invalid("invalid transferred thread placement".into()));
            }
            Ok((i, point, axis))
        })
        .collect::<Result<_>>()?;
    // A sheet made from surfaces (#6748): where each of its panels lies moves with its body, so
    // that the sheet still describes the body it builds.
    let surface: Vec<_> = st
        .surface_sheets
        .iter()
        .enumerate()
        .filter(|(_, s)| names.contains(s.body.as_str()))
        .map(|(i, s)| {
            let places = s
                .places
                .iter()
                .map(|place| match place {
                    None => Ok(None),
                    Some(p) => {
                        let moved = solidcraft_develop::Rigid3 {
                            x: apply_vector(transform, p.x),
                            y: apply_vector(transform, p.y),
                            z: apply_vector(transform, p.z),
                            shift: apply_point(transform, p.shift),
                        };
                        let finite = [moved.x, moved.y, moved.z, moved.shift].iter().all(|v| v.is_finite());
                        finite.then_some(Some(moved)).ok_or_else(|| DocError::Invalid("invalid transferred sheet place".into()))
                    }
                })
                .collect::<Result<Vec<_>>>()?;
            Ok((i, places))
        })
        .collect::<Result<_>>()?;
    for (i, frame) in frames {
        if let Some(s) = st.sheets.get_mut(i) {
            s.frame = frame;
        }
    }
    for (i, places) in surface {
        if let Some(s) = st.surface_sheets.get_mut(i) {
            s.places = places;
        }
    }
    for (i, point, axis) in threads {
        if let Some(t) = st.threads.get_mut(i) {
            t.axis_point = point;
            t.axis = axis;
        }
    }
    Ok(())
}
