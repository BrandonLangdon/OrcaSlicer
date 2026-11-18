#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <limits>
#include <math.h>
#include <utility>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"

using namespace Slic3r;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

// A 30 x 30 x 20 mm block holding a stepped part: a 10 mm square from 5 to 10 mm, under a 4 mm square from 10 to 15 mm.
// The part's wide step overhangs its narrow top, so a pocket that only followed the part would close over the step
// and trap it. The part is created before or after the block, which sets the volumes' order.
Print &stepped_part_in_block(Print &print, Model &model, bool fixture, bool part_first)
{
    ModelObject *object = model.add_object();
    object->name        = "fixture.stl";
    auto add_part = [object]() {
        TriangleMesh wide = make_cube(10., 10., 5.);
        wide.translate(10.f, 10.f, 5.f);
        object->add_volume(std::move(wide), ModelVolumeType::NEGATIVE_VOLUME, false);
        TriangleMesh narrow = make_cube(4., 4., 5.);
        narrow.translate(13.f, 13.f, 10.f);
        object->add_volume(std::move(narrow), ModelVolumeType::NEGATIVE_VOLUME, false);
    };
    if (part_first)
        add_part();
    object->add_volume(make_cube(30., 30., 20.), ModelVolumeType::MODEL_PART, false);
    if (! part_first)
        add_part();
    object->add_instance();
    object->ensure_on_bed();

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "layer_height", 0.2 },
        { "initial_layer_print_height", 0.2 },
        { "solid_fixture", fixture ? 1 : 0 },
        { "solid_fixture_clearance", 0.2 },
    });
    print.auto_assign_extruders(object);
    print.apply(model, config);
    print.validate();
    print.set_status_silent();
    print.process();
    return print;
}

// The printed area of the layer printed at `z`, in square millimetres.
double area_at(const Print &print, double z)
{
    const Layer *nearest = nullptr;
    for (const Layer *layer : print.objects().front()->layers())
        if (nearest == nullptr || std::abs(layer->print_z - z) < std::abs(nearest->print_z - z))
            nearest = layer;
    REQUIRE(nearest != nullptr);
    double area = 0.;
    for (const ExPolygon &expolygon : nearest->lslices)
        area += expolygon.area();
    return area * SCALING_FACTOR * SCALING_FACTOR;
}

// The block less a square of `side` grown by the 0.2 mm clearance, with its rounded corners.
double block_less(double side)
{
    const double clearance = 0.2;
    return 30. * 30. - (side * side + 4. * side * clearance + M_PI * clearance * clearance);
}

// A 10 mm cube 5 mm up in the middle of an 80 x 80 x 20 mm solid fixture, with the mounting holes `mount` sets.
Print &centred_part_in_block(Print &print, Model &model, std::initializer_list<ConfigBase::SetDeserializeItem> mount)
{
    ModelObject *object = model.add_object();
    object->name        = "fixture.stl";
    object->add_volume(make_cube(80., 80., 20.), ModelVolumeType::MODEL_PART, false);
    TriangleMesh part = make_cube(10., 10., 10.);
    part.translate(35.f, 35.f, 5.f);
    object->add_volume(std::move(part), ModelVolumeType::NEGATIVE_VOLUME, false);
    object->add_instance();
    object->ensure_on_bed();

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "layer_height", 0.2 },
        { "initial_layer_print_height", 0.2 },
        { "solid_fixture", 1 },
        { "solid_fixture_clearance", 0.2 },
    });
    config.set_deserialize_strict(mount);
    print.auto_assign_extruders(object);
    print.apply(model, config);
    print.validate();
    print.set_status_silent();
    print.process();
    return print;
}

// The holes in the layer printed at `z`.
size_t holes_at(const Print &print, double z)
{
    const Layer *nearest = nullptr;
    for (const Layer *layer : print.objects().front()->layers())
        if (nearest == nullptr || std::abs(layer->print_z - z) < std::abs(nearest->print_z - z))
            nearest = layer;
    REQUIRE(nearest != nullptr);
    size_t holes = 0;
    for (const ExPolygon &expolygon : nearest->lslices)
        holes += expolygon.holes.size();
    return holes;
}

} // namespace

TEST_CASE("A solid fixture's pocket rises from the part's underside through the top of the block", "[SolidFixture]")
{
    const bool part_first = GENERATE(false, true);
    CAPTURE(part_first);
    Print print;
    Model model;
    stepped_part_in_block(print, model, true, part_first);

    // Under the part, more than the clearance below it, the block is solid: the floor the part rests on.
    CHECK_THAT(area_at(print, 4.), WithinAbs(30. * 30., 1.));
    // Beside the wide step, the pocket is the step and its clearance.
    CHECK_THAT(area_at(print, 7.), WithinAbs(block_less(10.), 1.));
    // Beside the narrow top and above the part, the pocket stays as wide as the step, so the part lifts out.
    CHECK_THAT(area_at(print, 12.), WithinAbs(block_less(10.), 1.));
    CHECK_THAT(area_at(print, 18.), WithinAbs(block_less(10.), 1.));
}

TEST_CASE("Negative volumes outside a solid fixture cut only their own shape", "[SolidFixture]")
{
    Print print;
    Model model;
    stepped_part_in_block(print, model, false, false);

    // The ordinary negative volume: the narrow top's hole, with no clearance, and nothing above the part.
    CHECK_THAT(area_at(print, 12.), WithinAbs(30. * 30. - 4. * 4., 1.));
    CHECK_THAT(area_at(print, 18.), WithinAbs(30. * 30., 1.));
}

TEST_CASE("A solid fixture's mounting holes run through the block on the workspace's grid", "[SolidFixture]")
{
    // The M6 breadboard's 25 mm grid, centred on the 80 mm block, falls at 15, 40 and 65 mm each way: nine holes, less
    // the middle one in the pocket. The 30 x 20 mm custom grid falls at 10, 40 and 70 mm by 0, 20, 40, 60 and 80 mm,
    // and keeps the 8 of them that are clear of the block's edge and the pocket.
    const auto [pattern, diameter] = GENERATE(table<const char *, double>({ { "breadboard_m6", 6.6 }, { "custom", 5. } }));
    CAPTURE(pattern);
    Print print;
    Model model;
    centred_part_in_block(print, model, { { "solid_fixture_mount_pattern", pattern },
                                          { "solid_fixture_mount_pitch_x", 30 },
                                          { "solid_fixture_mount_pitch_y", 20 },
                                          { "solid_fixture_mount_hole", 5 } });

    // Under the part the floor holds only the mounting holes; above it they surround the pocket.
    CHECK(holes_at(print, 2.) == 8);
    CHECK(holes_at(print, 18.) == 9);
    // Each hole is the full diameter: the floor is the block less 8 of them.
    const double hole_area = M_PI * 0.25 * diameter * diameter;
    CHECK_THAT(area_at(print, 2.), WithinRel(80. * 80. - 8. * hole_area, 0.002));
}

TEST_CASE("A solid fixture has no mounting holes unless a pattern is chosen", "[SolidFixture]")
{
    Print print;
    Model model;
    centred_part_in_block(print, model, { { "solid_fixture_mount_pattern", "none" } });
    CHECK(holes_at(print, 2.) == 0);
}
