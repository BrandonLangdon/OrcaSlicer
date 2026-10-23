#include <catch2/catch_all.hpp>

#include <algorithm>
#include <initializer_list>
#include <sstream>
#include <string>
#include <iterator>
#include <utility>
#include <vector>
#include "libslic3r/Config.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Point.hpp"
#include <cstddef>
#include "libslic3r/libslic3r.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>
#include "libslic3r/Layer.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include "test_helpers.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"

using namespace Slic3r::Test;
using namespace Slic3r;

namespace {

// The upper plate overhangs both the lower plate and open air, so branches land on the model and on
// the bed in the same slice.
TriangleMesh two_tier_mesh()
{
    TriangleMesh lower  = make_cube(30, 30, 3);
    TriangleMesh column = make_cube(8, 8, 15);
    TriangleMesh upper  = make_cube(50, 50, 3);
    // Each part overlaps the one below rather than resting on it; a coplanar join slices ambiguously.
    column.translate(11.f, 11.f, 2.f);
    upper.translate(-10.f, -10.f, 16.f);
    TriangleMesh mesh = lower;
    mesh.merge(column);
    mesh.merge(upper);
    return mesh;
}

// A downward pointing cone hangs from a plate held up by a column, so its tip floats 10 mm above the bed.
constexpr float floating_tip_z = 10.f;
TriangleMesh hanging_cone_mesh()
{
    TriangleMesh column = make_cube(6, 6, 31);
    TriangleMesh plate  = make_cube(30, 16, 3);
    TriangleMesh cone   = make_cone(8, 30.5 - floating_tip_z);
    plate.translate(0.f, -5.f, 30.f);
    cone.mirror_z();
    cone.translate(20.f, 3.f, 30.5f);
    TriangleMesh mesh = column;
    mesh.merge(plate);
    mesh.merge(cone);
    return mesh;
}

// Support layers below the floating tip that carry a canopy.
std::vector<const SupportLayer *> canopy_layers(const Slic3r::Print &print)
{
    std::vector<const SupportLayer *> layers;
    for (const SupportLayer *layer : print.objects().front()->support_layers())
        if (!layer->canopies().empty())
            layers.push_back(layer);
    return layers;
}

TriangleMesh scaled(TestMesh id, float scale)
{
    TriangleMesh mesh = Slic3r::Test::mesh(id);
    mesh.scale(scale);
    return mesh;
}

// `extra` is applied last, so a caller can add or override any key.
DynamicPrintConfig tree_support_config(const char *style, int threshold_angle = 30, int build_plate_only = 0, int raft_layers = 0,
                                       std::initializer_list<Slic3r::ConfigBase::SetDeserializeItem> extra = {})
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "enable_support",              1 },
        { "support_type",                "tree(auto)" },
        { "support_style",               style },
        { "support_on_build_plate_only", build_plate_only },
        { "support_threshold_angle",     threshold_angle },
        { "raft_layers",                 raft_layers },
        { "layer_height",                0.2 },
    });
    config.set_deserialize_strict(extra);
    return config;
}

void slice_with_tree_support(const TriangleMesh &mesh, Slic3r::Print &print, const char *style,
                             int threshold_angle = 30, int build_plate_only = 0, int raft_layers = 0,
                             std::initializer_list<Slic3r::ConfigBase::SetDeserializeItem> extra = {})
{
    Slic3r::Test::init_and_process_print({ mesh }, print, tree_support_config(style, threshold_angle, build_plate_only, raft_layers, extra));
}

// Slices the hanging cone in the middle of the bed. Arranging centres an object on the origin, where tree support
// is clipped to the part of it that lies on the bed.
void slice_hanging_cone(Slic3r::Print &print, std::initializer_list<Slic3r::ConfigBase::SetDeserializeItem> extra = {})
{
    TriangleMesh mesh = hanging_cone_mesh();
    mesh.translate(100.f, 100.f, 0.f);
    std::vector<TriangleMesh> meshes;
    meshes.push_back(std::move(mesh));
    Slic3r::Model model;
    Slic3r::Test::init_print(std::move(meshes), print, model, tree_support_config("tree_slim", 30, 0, 0, extra), nullptr, /*arrange=*/false);
    print.process();
}

Points support_points(const Slic3r::Print &print)
{
    Points points;
    for (const SupportLayer *layer : print.objects().front()->support_layers())
        layer->support_fills.collect_points(points);
    return points;
}

size_t support_point_count(const TriangleMesh &mesh, const char *style, int threshold_angle = 30,
                           int build_plate_only = 0)
{
    Slic3r::Print print;
    slice_with_tree_support(mesh, print, style, threshold_angle, build_plate_only);
    return support_points(print).size();
}

// Index of the first differing point, or the common length when they match. An index keeps a
// failure readable; comparing the vectors themselves dumps thousands of points.
size_t first_difference(const Points &a, const Points &b)
{
    const size_t common = std::min(a.size(), b.size());
    for (size_t i = 0; i < common; ++i)
        if (a[i] != b[i])
            return i;
    return common;
}

// Slice `mesh` twice and require an identical support point sequence. Point counts and total
// length are order insensitive, so the sequence is what a reordering shows up in.
void sliced_twice_matches(const TriangleMesh &mesh, int build_plate_only, const char *style = "tree_slim",
                          std::initializer_list<Slic3r::ConfigBase::SetDeserializeItem> extra = {})
{
    Slic3r::Print first_print, second_print;
    slice_with_tree_support(mesh, first_print, style, 30, build_plate_only, 0, extra);
    slice_with_tree_support(mesh, second_print, style, 30, build_plate_only, 0, extra);
    const Points first  = support_points(first_print);
    const Points second = support_points(second_print);
    REQUIRE(first.size() > 1000); // without support the comparison below passes vacuously
    REQUIRE(second.size() == first.size());
    REQUIRE(first_difference(first, second) == first.size());
}

} // namespace

TEST_CASE("Tree support is generated for an overhang and not for a plain cube", "[TreeSupport]")
{
    REQUIRE(support_point_count(scaled(TestMesh::overhang, 2.f), "tree_slim") > 1000);
    REQUIRE(support_point_count(Slic3r::Test::cube(20), "tree_slim") == 0);
}

TEST_CASE("Restricting tree support to the build plate changes what is generated", "[TreeSupport]")
{
    const TriangleMesh mesh = two_tier_mesh();
    const size_t anywhere    = support_point_count(mesh, "tree_slim", 30, 0);
    const size_t plate_only  = support_point_count(mesh, "tree_slim", 30, 1);
    REQUIRE(anywhere > 1000);
    REQUIRE(plate_only > 1000);
    // The upper plate overhangs the lower one, so some branches would land on the model.
    REQUIRE(plate_only != anywhere);
}

TEST_CASE("Tree support layers rise monotonically within the layer height limits", "[TreeSupport]")
{
    Slic3r::Print print;
    slice_with_tree_support(scaled(TestMesh::overhang, 2.f), print, "tree_slim");
    const double nozzle = print.config().nozzle_diameter.values.front();

    size_t checked = 0;
    double previous = 0;
    bool   previous_was_adjacent = false;
    for (const SupportLayer *layer : print.objects().front()->support_layers()) {
        if (layer->print_z <= 0 || layer->height <= 0) {
            // Layers with no nodes are left at zero. Skipping one leaves a hole, so the next pair
            // spans more than one layer and its gap says nothing about the layer height limit.
            previous_was_adjacent = false;
            continue;
        }
        if (previous > 0) {
            CAPTURE(previous, layer->print_z);
            REQUIRE(layer->print_z > previous);
            if (previous_was_adjacent)
                REQUIRE(layer->print_z - previous <= nozzle + EPSILON);
        }
        previous = layer->print_z;
        previous_was_adjacent = true;
        ++checked;
    }
    REQUIRE(checked > 10);
}

TEST_CASE("A raft is still generated under tree support", "[TreeSupport]")
{
    // The mesh supports itself, so a layer count alone passes with no raft at all.
    Slic3r::Print rafted, unrafted;
    slice_with_tree_support(scaled(TestMesh::overhang, 2.f), rafted, "tree_slim", 30, 0, 3);
    slice_with_tree_support(scaled(TestMesh::overhang, 2.f), unrafted, "tree_slim", 30, 0, 0);
    const PrintObject *rafted_object   = rafted.objects().front();
    const PrintObject *unrafted_object = unrafted.objects().front();
    REQUIRE(rafted_object->support_layers().size() > unrafted_object->support_layers().size());
    // The raft goes under the object.
    REQUIRE(rafted_object->layers().front()->print_z > unrafted_object->layers().front()->print_z);
}

// drop_nodes() decides the node merges and spawns the next layer's nodes in parallel. Every one of
// those decisions has to be applied in a fixed order, or the same model gives different branches on
// each slice.
TEST_CASE("Tree support toolpaths do not depend on thread scheduling", "[TreeSupport][Regression]")
{
    // Scaled up so that a layer holds enough nodes for the parallel range to be split. At stock
    // size it stays in one chunk and the order never varies.
    SECTION("overhang")            { sliced_twice_matches(scaled(TestMesh::overhang, 2.f), 0); }
    SECTION("bridge with hole")    { sliced_twice_matches(scaled(TestMesh::bridge_with_hole, 3.f), 0); }
    // Dropping every branch that cannot reach the bed leaves the survivors dense enough that the
    // neighbour merge fires in bulk.
    SECTION("on the build plate")  { sliced_twice_matches(scaled(TestMesh::overhang, 4.f), 1); }
    // Branches resting on the model are what put nodes in a part group other than 0, which is the
    // only way to reach the prune in the second pass. tree_hybrid additionally builds polygon
    // nodes, so it is the only style that exercises the overhang merge.
    SECTION("resting on the model") { sliced_twice_matches(two_tier_mesh(), 0); }
    SECTION("hybrid on the model")  { sliced_twice_matches(two_tier_mesh(), 0, "tree_hybrid"); }
}

// Prim breaks equal-distance ties by heap address. A 1 mm branch diameter puts neighbours close
// enough to tie, and an explicit line width pins max_move_dist, so the moved tie winner reaches
// the support toolpaths.
TEST_CASE("Tree support toolpaths do not depend on the MST tie order", "[TreeSupport][Regression]")
{
    sliced_twice_matches(two_tier_mesh(), 0, "tree_hybrid", {
        { "tree_support_branch_diameter", 1.0 },
        { "tree_support_branch_distance", 5.0 },
        { "tree_support_branch_angle",    40 },
        { "support_line_width",           0.4 },
    });
}

TEST_CASE("A floating tip gets a canopy right below it when canopies are on", "[TreeSupport][Canopy]")
{
    constexpr double margin = 2.;
    Slic3r::Print    print;
    slice_hanging_cone(print, {
        { "tree_support_canopy",        1 },
        { "tree_support_canopy_margin", margin },
    });

    const std::vector<const SupportLayer *> layers = canopy_layers(print);
    REQUIRE(layers.size() == 1);
    const SupportLayer &canopy = *layers.front();
    // One gap of support_top_z_distance below the tip, give or take a layer.
    CHECK(canopy.print_z < floating_tip_z);
    CHECK(canopy.print_z > floating_tip_z - 3 * 0.2);
    // The canopy reaches the margin beyond the tip all around, so it spans at least the margin's disc.
    double canopy_area = 0.;
    for (const ExPolygon &expoly : canopy.canopies())
        canopy_area += unscaled(unscaled(expoly.area()));
    CHECK(canopy_area > 0.9 * PI * margin * margin);
    CHECK_FALSE(canopy.support_fills.empty());

    // The canopy stands on a single trunk rather than on a cluster of branches. Support of the plate around the cone
    // may pass under the canopy too, so only what lies within the canopy counts as standing on it.
    const auto &support_layers = print.objects().front()->support_layers();
    const auto  it             = std::find(support_layers.begin(), support_layers.end(), &canopy);
    REQUIRE(it != support_layers.begin());
    size_t trunks = 0;
    for (const ExPolygon &island : (*std::prev(it))->base_areas)
        if (diff_ex(ExPolygons{ island }, canopy.canopies()).empty())
            ++trunks;
    CHECK(trunks == 1);
}

TEST_CASE("No canopy is generated while canopies are off", "[TreeSupport][Canopy]")
{
    Slic3r::Print print;
    slice_hanging_cone(print);

    REQUIRE_FALSE(support_points(print).empty());
    CHECK(canopy_layers(print).empty());
}

TEST_CASE("A canopy prints at its own speed and fan speed", "[TreeSupport][Canopy]")
{
    constexpr double canopy_speed = 4.;
    constexpr int    canopy_fan   = 90;
    Slic3r::Print    print;
    slice_hanging_cone(print, {
        { "tree_support_canopy",      1 },
        { "support_canopy_speed",     canopy_speed },
        { "support_canopy_fan_speed", canopy_fan },
    });
    const std::string gcode = Slic3r::Test::gcode(print);

    // Walk the G-code, keeping the modal feedrate and fan speed, and look at the extrusions inside the canopy.
    std::istringstream input(gcode);
    std::string        line;
    bool               in_canopy        = false;
    double             feedrate         = 0.;
    int                fan              = 0;
    size_t             canopy_moves     = 0;
    double             fastest_feedrate = 0.;
    int                slowest_fan      = 255;
    while (std::getline(input, line)) {
        if (line.rfind(";TYPE:", 0) == 0)
            in_canopy = line == ";TYPE:Support canopy";
        else if (line.rfind("M106", 0) == 0 && line.find(" S") != std::string::npos)
            fan = std::stoi(line.substr(line.find(" S") + 2));
        else if (line.rfind("M107", 0) == 0)
            fan = 0;
        else if (line.rfind("G1 ", 0) == 0 || line.rfind("G2 ", 0) == 0 || line.rfind("G3 ", 0) == 0) {
            if (const size_t f = line.find(" F"); f != std::string::npos)
                feedrate = std::stod(line.substr(f + 2));
            if (in_canopy && line.find(" E") != std::string::npos) {
                ++canopy_moves;
                fastest_feedrate = std::max(fastest_feedrate, feedrate);
                slowest_fan      = std::min(slowest_fan, fan);
            }
        }
    }
    REQUIRE(canopy_moves > 0);
    // Feedrates are in mm/min. Cooling may slow the canopy down further, never speed it up.
    CHECK(fastest_feedrate <= canopy_speed * 60. + EPSILON);
    // M106 takes the fan speed on a 0-255 scale, as GCodeWriter::set_fan() scales it.
    CHECK(slowest_fan == int(255.5 * canopy_fan / 100.));
}
