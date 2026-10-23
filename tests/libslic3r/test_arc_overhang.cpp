#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Fill/ArcOverhang.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/libslic3r.h"

using namespace Slic3r;

namespace {

constexpr double spacing_mm = 0.4;

ExPolygon disc(const Vec2d &center, double radius)
{
    std::vector<Vec2d> points;
    for (int i = 0; i < 64; ++i) {
        const double angle = 2. * M_PI * i / 64.;
        points.emplace_back(center + radius * Vec2d(std::cos(angle), std::sin(angle)));
    }
    return ExPolygon(Polygon::new_scale(points));
}

ExPolygon rectangle(double x0, double y0, double x1, double y1)
{
    return ExPolygon(Polygon::new_scale({ { x0, y0 }, { x1, y0 }, { x1, y1 }, { x0, y1 } }));
}

ArcOverhangParams params()
{
    ArcOverhangParams params;
    params.line_spacing = scaled<coord_t>(spacing_mm);
    return params;
}

// Share of the printable part of the area, half a line in from its edge, covered by the anchors and the arc extrusions.
double coverage(const ExPolygons &area, const ExPolygons &anchors, const Polylines &arcs)
{
    const ExPolygons printable = offset_ex(area, -0.5f * float(scaled<double>(spacing_mm)));
    const ExPolygons covered   = union_ex(anchors, offset(arcs, 0.5f * float(scaled<double>(spacing_mm)) + float(SCALED_EPSILON)));
    double printable_area = 0.;
    for (const ExPolygon &expoly : printable)
        printable_area += expoly.area();
    double covered_area = 0.;
    for (const ExPolygon &expoly : intersection_ex(printable, covered))
        covered_area += expoly.area();
    return covered_area / printable_area;
}

} // namespace

TEST_CASE("Arcs grown from a central anchor cover a square area", "[ArcOverhang]")
{
    const ExPolygons area    = { rectangle(-8., -8., 8., 8.) };
    const ExPolygons anchors = { disc({ 0., 0. }, 2.) };

    const Polylines arcs = generate_arc_overhangs(area, anchors, params());

    REQUIRE_FALSE(arcs.empty());
    CHECK(coverage(area, anchors, arcs) > 0.95);
}

TEST_CASE("Arcs fill an L-shaped area around its inner corner", "[ArcOverhang]")
{
    // The anchor sits at the end of one leg, so the arcs have to turn the corner to reach the other leg.
    const ExPolygons area    = union_ex(ExPolygons{ rectangle(0., 0., 20., 6.), rectangle(0., 0., 6., 20.) });
    const ExPolygons anchors = { disc({ 17., 3. }, 1.5) };

    const Polylines arcs = generate_arc_overhangs(area, anchors, params());

    CHECK(coverage(area, anchors, arcs) > 0.9);
}

TEST_CASE("Every arc rests on the anchors or on the arcs printed before it", "[ArcOverhang]")
{
    const ExPolygons area    = union_ex(ExPolygons{ rectangle(0., 0., 20., 6.), rectangle(0., 0., 6., 20.) });
    const ExPolygons anchors = { disc({ 17., 3. }, 1.5) };
    const float      half    = 0.5f * float(scaled<double>(spacing_mm));
    // An arc touches what it rests on when its centre line comes within half a line width of it; allow for the
    // polyline approximation of the circles on top of the generator's own reach.
    const float      reach   = 1.3f * half;

    const Polylines arcs = generate_arc_overhangs(area, anchors, params());
    REQUIRE_FALSE(arcs.empty());

    // Footprints of the anchors and of the arcs printed so far; each arc is checked against those near it only.
    std::vector<std::pair<BoundingBox, Polygons>> solid;
    for (const ExPolygon &anchor : anchors)
        solid.emplace_back(get_extents(anchor), to_polygons(anchor));
    for (size_t i = 0; i < arcs.size(); ++i) {
        const BoundingBox near = get_extents(arcs[i]).inflated(coord_t(reach) + scaled<coord_t>(spacing_mm));
        Polygons          below;
        for (const auto &[bbox, footprint] : solid)
            if (bbox.overlap(near))
                append(below, footprint);
        const Polylines floating = diff_pl(Polylines{ arcs[i] }, offset(below, reach));
        INFO("arc " << i << " of " << arcs.size());
        CHECK(unscaled(total_length(floating)) < 0.05);
        Polygons footprint = offset(arcs[i], half + float(SCALED_EPSILON));
        solid.emplace_back(get_extents(footprint), std::move(footprint));
    }
}

TEST_CASE("Arcs stay half a line spacing inside the area", "[ArcOverhang]")
{
    const ExPolygons area    = { rectangle(-8., -8., 8., 8.) };
    const ExPolygons anchors = { disc({ 0., 0. }, 2.) };

    const Polylines arcs = generate_arc_overhangs(area, anchors, params());

    const ExPolygons inside  = offset_ex(area, -0.5f * float(scaled<double>(spacing_mm)) + float(scaled<double>(0.01)));
    CHECK(unscaled(total_length(diff_pl(arcs, inside))) < 0.01);
}

TEST_CASE("No arcs are generated without an anchor to grow from", "[ArcOverhang]")
{
    const ExPolygons area = { rectangle(-8., -8., 8., 8.) };

    CHECK(generate_arc_overhangs(area, {}, params()).empty());
}
