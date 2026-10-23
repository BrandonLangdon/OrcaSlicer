#pragma once

#include "../ExPolygon.hpp"
#include "../Polyline.hpp"
#include "../libslic3r.h"

namespace Slic3r {

// Orca: Arc overhangs, after Steven McCulloch's arc-overhang script (github.com/stmcculloch/arc-overhang).
// An area with nothing below it is filled with concentric arcs grown out of a supported region. Each arc is laid
// one line spacing beyond the arcs printed before it, so it fuses to material that is already solid; printed
// slowly and with strong cooling, such a layer spans the area without any support underneath.
struct ArcOverhangParams
{
    // Distance between neighbouring arcs, scaled.
    coord_t line_spacing { 0 };
    // Largest deviation of an arc's polyline from its circle, scaled.
    coord_t resolution { scaled<coord_t>(0.0125) };
    // Arc fragments shorter than this are dropped, scaled.
    coord_t min_arc_length { 0 };
    // Cap on the number of centres the arcs are grown from once the anchors are done.
    int     max_centers { 64 };
};

// Fills `area` with arcs grown from `anchors`, the regions of the layer that rest on something. Arcs grow first
// around the centroid of each anchor, then from points on the edge of the filled region, until the area is covered
// or no further arc can be laid against what is filled. An arc is only laid where it lies within about half a line
// spacing of the anchors or of the arcs before it, and the arcs keep half a line spacing inside the area.
// The arcs are returned in print order: each rests only on the anchors and on the arcs before it.
Polylines generate_arc_overhangs(const ExPolygons &area, const ExPolygons &anchors, const ArcOverhangParams &params);

} // namespace Slic3r
