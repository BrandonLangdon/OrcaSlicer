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
#include <vector>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"

using namespace Slic3r;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

// A solid fixture made of a `block` x `block` x 20 mm block, or `block` x `block_y` mm, and the part `pieces` make up,
// sliced at 0.2 mm with a 0.2 mm clearance and the settings `items` add. The part is created before or after the block,
// which sets the volumes' order.
Print &fixture(Print &print, Model &model, double block, std::vector<TriangleMesh> pieces,
               std::initializer_list<ConfigBase::SetDeserializeItem> items, bool part_first = false, double block_y = 0.)
{
    ModelObject *object = model.add_object();
    object->name        = "fixture.stl";
    auto add_part = [object, &pieces]() {
        for (TriangleMesh &piece : pieces)
            object->add_volume(std::move(piece), ModelVolumeType::NEGATIVE_VOLUME, false);
    };
    if (part_first)
        add_part();
    object->add_volume(make_cube(block, block_y > 0. ? block_y : block, 20.), ModelVolumeType::MODEL_PART, false);
    if (! part_first)
        add_part();
    object->add_instance();
    object->ensure_on_bed();

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "layer_height", 0.2 },
        { "initial_layer_print_height", 0.2 },
        { "solid_fixture", 1 },
        { "solid_fixture_clearance", 0.2 },
    });
    config.set_deserialize_strict(items);
    print.auto_assign_extruders(object);
    print.apply(model, config);
    print.validate();
    print.set_status_silent();
    print.process();
    return print;
}

TriangleMesh cube_at(double size_x, double size_y, double size_z, double x, double y, double z)
{
    TriangleMesh cube = make_cube(size_x, size_y, size_z);
    cube.translate(float(x), float(y), float(z));
    return cube;
}

// A stepped part in a 30 mm block: a 10 mm square from 5 to 10 mm, under a 4 mm square from 10 to 15 mm. The wide step
// overhangs the narrow top, so a pocket that only followed the part would close over the step and trap it.
std::vector<TriangleMesh> stepped_part()
{
    return { cube_at(10., 10., 5., 10., 10., 5.), cube_at(4., 4., 5., 13., 13., 10.) };
}

// A 10 mm cube 5 mm up in the middle of an 80 mm block.
std::vector<TriangleMesh> centred_cube() { return { cube_at(10., 10., 10., 35., 35., 5.) }; }

// The layer printed nearest `z`.
const Layer &layer_at(const Print &print, double z)
{
    const Layer *nearest = nullptr;
    for (const Layer *layer : print.objects().front()->layers())
        if (nearest == nullptr || std::abs(layer->print_z - z) < std::abs(nearest->print_z - z))
            nearest = layer;
    REQUIRE(nearest != nullptr);
    return *nearest;
}

// The printed area of the layer printed at `z`, in square millimetres.
double area_at(const Print &print, double z)
{
    double area = 0.;
    for (const ExPolygon &expolygon : layer_at(print, z).lslices)
        area += expolygon.area();
    return area * SCALING_FACTOR * SCALING_FACTOR;
}

// The holes in the layer printed at `z`.
size_t holes_at(const Print &print, double z)
{
    size_t holes = 0;
    for (const ExPolygon &expolygon : layer_at(print, z).lslices)
        holes += expolygon.holes.size();
    return holes;
}

// A square of `side` grown by `by`, its corners kept square.
double grown_square(double side, double by) { return (side + 2. * by) * (side + 2. * by); }

} // namespace

TEST_CASE("A solid fixture's pocket rises from the part's underside through the top of the block", "[SolidFixture]")
{
    const bool part_first = GENERATE(false, true);
    CAPTURE(part_first);
    Print print;
    Model model;
    // A wall wide and tall enough to fill the block, so only the pocket is cut from it.
    fixture(print, model, 30., stepped_part(), { { "solid_fixture_wall_thickness", 40 }, { "solid_fixture_wall_height", 100 } }, part_first);

    // Under the part, more than the clearance below it, the block is solid: the floor the part rests on.
    CHECK_THAT(area_at(print, 4.), WithinAbs(30. * 30., 1.));
    // Beside the wide step, the pocket is the step and its clearance.
    CHECK_THAT(area_at(print, 7.), WithinAbs(30. * 30. - grown_square(10., 0.2), 1.));
    // Beside the narrow top and above the part, the pocket stays as wide as the step, so the part lifts out.
    CHECK_THAT(area_at(print, 12.), WithinAbs(30. * 30. - grown_square(10., 0.2), 1.));
    CHECK_THAT(area_at(print, 18.), WithinAbs(30. * 30. - grown_square(10., 0.2), 1.));
}

TEST_CASE("Negative volumes outside a solid fixture cut only their own shape", "[SolidFixture]")
{
    Print print;
    Model model;
    fixture(print, model, 30., stepped_part(), { { "solid_fixture", 0 } });

    // The ordinary negative volume: the narrow top's hole, with no clearance, and nothing above the part.
    CHECK_THAT(area_at(print, 12.), WithinAbs(30. * 30. - 4. * 4., 1.));
    CHECK_THAT(area_at(print, 18.), WithinAbs(30. * 30., 1.));
}

TEST_CASE("A solid fixture is a thin base and a locating wall, not a solid block", "[SolidFixture]")
{
    // A 10 mm cube 5 mm up, on a 2 mm base with a 4 mm margin round a 2 mm wall 5 mm high: the wall reaches 10 mm.
    Print print;
    Model model;
    fixture(print, model, 80., centred_cube(),
            { { "solid_fixture_base_thickness", 2 }, { "solid_fixture_margin", 4 }, { "solid_fixture_wall_thickness", 2 },
              { "solid_fixture_wall_height", 5 }, { "solid_fixture_mount_pattern", "none" } });

    // The base reaches the clearance, the wall and the margin past the part, not the 80 mm block.
    CHECK_THAT(area_at(print, 1.), WithinRel(grown_square(10., 0.2 + 2. + 4.), 0.01));
    // Below the part, the wall's whole footprint holds the part up.
    CHECK_THAT(area_at(print, 3.6), WithinRel(grown_square(10., 0.2 + 2.), 0.01));
    // Beside the part, the wall is a ring round the pocket.
    CHECK_THAT(area_at(print, 8.), WithinRel(grown_square(10., 0.2 + 2.) - grown_square(10., 0.2), 0.02));
    // Nothing is printed above the wall: the cube's flat underside needs no cradle there.
    CHECK_THAT(print.objects().front()->layers().back()->print_z, WithinAbs(10., 0.21));
}

TEST_CASE("A solid fixture's cradle holds up an overhanging underside above the wall", "[SolidFixture]")
{
    // A T: a 4 mm stem from 5 to 10 mm under a 10 mm cap from 10 to 15 mm. The wall stops 2 mm up the stem, at 7 mm.
    Print print;
    Model model;
    fixture(print, model, 30., { cube_at(4., 4., 5., 13., 13., 5.), cube_at(10., 10., 5., 10., 10., 10.) },
            { { "solid_fixture_wall_height", 2 }, { "solid_fixture_mount_pattern", "none" } });

    // Above the wall and under the cap, the cradle fills the cap's outline round the stem's pocket.
    CHECK_THAT(area_at(print, 8.5), WithinRel(10. * 10. - grown_square(4., 0.2), 0.02));
}

TEST_CASE("A solid fixture's base reaches out in tabs to a mounting hole in each quadrant on every pattern", "[SolidFixture]")
{
    // A 10 mm cube in a block sized as "Create solid fixture" sizes it for the pattern: room for a grid hole in every
    // quadrant round the part, however coarse the grid.
    const char *pattern = GENERATE("breadboard_m6", "breadboard_1/4-20", "welding_system_16", "welding_system_22", "welding_system_28",
                                   "welding_5/8", "pegboard_1/4", "pegboard_1/8", "tslot_20", "tslot_30", "tslot_40", "tslot_8020_10",
                                   "tslot_8020_15", "custom");
    CAPTURE(pattern);
    const double clearance = 0.2, wall = 2., margin = 5.;
    DynamicPrintConfig settings;
    settings.set_deserialize_strict({ { "solid_fixture_mount_pattern", pattern } });
    const Vec2d half = fixture_block_half_size(Vec2d(5., 5.), clearance + wall, margin,
                                               fixture_mount_grid(settings.opt_enum<FixtureMountPattern>("solid_fixture_mount_pattern"), 30., 20., 5.));
    Print print;
    Model model;
    fixture(print, model, 2. * half.x(), { cube_at(10., 10., 10., half.x() - 5., half.y() - 5., 5.) },
            { { "solid_fixture_base_thickness", 2 }, { "solid_fixture_margin", margin }, { "solid_fixture_wall_thickness", wall },
              { "solid_fixture_wall_height", 5 }, { "solid_fixture_mount_pattern", pattern }, { "solid_fixture_mount_pitch_x", 30 },
              { "solid_fixture_mount_pitch_y", 20 }, { "solid_fixture_mount_hole", 5 } },
            false, 2. * half.y());

    // The base is one piece, its four tabs joined to the flange, each with its hole.
    REQUIRE(layer_at(print, 1.).lslices.size() == 1);
    CHECK(holes_at(print, 1.) == 4);
    // Above the base only the wall is printed, round the pocket: no mounting holes there.
    CHECK(holes_at(print, 8.) == 1);
}

TEST_CASE("A solid fixture has no mounting holes unless a pattern is chosen", "[SolidFixture]")
{
    Print print;
    Model model;
    fixture(print, model, 80., centred_cube(), { { "solid_fixture_mount_pattern", "none" } });
    CHECK(holes_at(print, 1.) == 0);
}
