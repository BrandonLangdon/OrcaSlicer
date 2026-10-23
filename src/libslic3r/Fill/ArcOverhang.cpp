#include "ArcOverhang.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

#include "../BoundingBox.hpp"
#include "../ClipperUtils.hpp"
#include "../Point.hpp"
#include "../Polygon.hpp"

namespace Slic3r {

namespace {

// A circle as a polyline that ends where it starts, so that clipping it yields open arcs.
Polyline circle_polyline(const Point &center, double radius, double resolution)
{
    // A chord of the circle strays from it by radius * (1 - cos(step / 2)).
    const double step  = 2. * std::acos(std::clamp(1. - resolution / radius, -1., 1.));
    const size_t count = std::max<size_t>(8, size_t(std::ceil(2. * M_PI / std::max(step, 1e-3))));
    Polyline     circle;
    circle.points.reserve(count + 1);
    for (size_t i = 0; i < count; ++i) {
        const double angle = 2. * M_PI * double(i) / double(count);
        circle.points.emplace_back(center.x() + coord_t(std::lround(radius * std::cos(angle))),
                                   center.y() + coord_t(std::lround(radius * std::sin(angle))));
    }
    circle.points.emplace_back(circle.points.front());
    return circle;
}

// The arcs of a circle that lie inside `allowed`, in the order they run around the circle.
Polylines clip_circle(const Point &center, double radius, const ExPolygons &allowed, const ArcOverhangParams &params)
{
    Polylines arcs = intersection_pl(Polylines{ circle_polyline(center, radius, double(params.resolution)) }, allowed);
    if (arcs.empty())
        return arcs;
    // Clipping loses the order and the direction of the arcs; run them all counter-clockwise, sorted by where they start.
    // restore_source_path_order() does not fit, as the circle's first and last points are one and the same.
    auto start_angle = [&center](const Polyline &arc) {
        const Vec2d v     = (arc.first_point() - center).cast<double>();
        const double angle = std::atan2(v.y(), v.x());
        return angle < 0. ? angle + 2. * M_PI : angle;
    };
    for (Polyline &arc : arcs)
        if (arc.size() > 1 && cross2((arc.points[0] - center).cast<double>(), (arc.points[1] - arc.points[0]).cast<double>()) < 0.)
            arc.reverse();
    std::sort(arcs.begin(), arcs.end(), [&start_angle](const Polyline &a, const Polyline &b) { return start_angle(a) < start_angle(b); });
    // Clipping splits the circle at its first point; an arc running through that point is one arc.
    if (arcs.size() > 1 && (arcs.back().last_point() - arcs.front().first_point()).cast<double>().norm() <= SCALED_EPSILON) {
        arcs.back().points.insert(arcs.back().points.end(), arcs.front().points.begin() + 1, arcs.front().points.end());
        arcs.erase(arcs.begin());
    }
    if (params.min_arc_length > 0)
        arcs.erase(std::remove_if(arcs.begin(), arcs.end(),
                                  [&params](const Polyline &arc) { return arc.length() < double(params.min_arc_length); }),
                   arcs.end());
    return arcs;
}

bool any_contains(const ExPolygons &expolygons, const Point &point)
{
    return std::any_of(expolygons.begin(), expolygons.end(), [&point](const ExPolygon &expoly) { return expoly.contains(point); });
}

} // namespace

Polylines generate_arc_overhangs(const ExPolygons &area, const ExPolygons &anchors, const ArcOverhangParams &params)
{
    Polylines out;
    if (params.line_spacing <= 0 || area.empty() || anchors.empty())
        return out;

    const double spacing = double(params.line_spacing);
    // Half a line in from the edge, so the extrusions do not spill over the area.
    const ExPolygons  printable = offset_ex(area, -0.5f * float(spacing));
    const BoundingBox bbox      = get_extents(printable);
    if (printable.empty())
        return out;
    // An arc rests on what is filled when the gap between its centre line and the filled region is no more than half
    // its width; the margin absorbs the error of the polyline circles.
    const float support_reach = float(0.6 * spacing);
    // Gaps smaller than a square of one line spacing are left unfilled.
    const double min_gap_area = spacing * spacing;

    ExPolygons filled = union_ex(anchors);
    // Gaps that no arc could be grown into.
    ExPolygons unreachable;

    // Lays concentric arcs around `center` for as long as each larger circle finds room beside the filled region.
    // The first circle has a radius of half a line spacing, so that around a point on the edge of the filled region
    // the whole of its outer half rests on that edge, and every further circle rests on the one before it.
    // Circles smaller than `inner_radius` may come out empty, as they still lie inside the region the growth starts
    // from; past it, the first circle without room ends the growth.
    auto grow_from = [&](const Point &center, double inner_radius) {
        double max_radius = 0.;
        for (const Point &corner : { bbox.min, bbox.max, Point(bbox.min.x(), bbox.max.y()), Point(bbox.max.x(), bbox.min.y()) })
            max_radius = std::max(max_radius, (corner - center).cast<double>().norm());
        bool grown = false;
        for (double radius = 0.5 * spacing; radius <= max_radius + spacing; radius += spacing) {
            const ExPolygons allowed = intersection_ex(diff_ex(printable, filled), offset_ex(filled, support_reach));
            Polylines        arcs    = clip_circle(center, radius, allowed, params);
            if (arcs.empty()) {
                if (grown || radius > inner_radius)
                    break;
                continue;
            }
            grown  = true;
            filled = union_ex(filled, offset(arcs, float(0.5 * spacing + SCALED_EPSILON)));
            append(out, std::move(arcs));
        }
        return grown;
    };

    for (const ExPolygon &anchor : anchors) {
        const Point center       = anchor.contour.centroid();
        double      inner_radius = 0.;
        for (const Point &point : anchor.contour.points)
            inner_radius = std::max(inner_radius, (point - center).cast<double>().norm());
        grow_from(center, inner_radius + spacing);
    }

    for (int i = 0; i < params.max_centers; ++i) {
        ExPolygons gaps = diff_ex(printable, union_ex(filled, unreachable));
        gaps.erase(std::remove_if(gaps.begin(), gaps.end(), [min_gap_area](const ExPolygon &gap) { return gap.area() < min_gap_area; }),
                   gaps.end());
        if (gaps.empty())
            break;
        const ExPolygon &gap =
            *std::max_element(gaps.begin(), gaps.end(), [](const ExPolygon &a, const ExPolygon &b) { return a.area() < b.area(); });

        // The next centre is the point on the edge of the filled region nearest the middle of the largest gap, so
        // the arcs fan out into the gap from the side that faces it.
        const Point          aim       = gap.contour.centroid();
        const ExPolygons     gap_reach = offset_ex(gap, support_reach);
        std::optional<Point> center;
        double               best_distance = 0.;
        for (const Polygon &edge : to_polygons(filled))
            for (const Point &candidate : edge.equally_spaced_points(0.5 * spacing))
                if (any_contains(gap_reach, candidate)) {
                    const double distance = (candidate - aim).cast<double>().squaredNorm();
                    if (!center || distance < best_distance) {
                        center        = candidate;
                        best_distance = distance;
                    }
                }
        if (!center || !grow_from(*center, spacing))
            unreachable = union_ex(unreachable, ExPolygons{ gap });
    }
    return out;
}

} // namespace Slic3r
