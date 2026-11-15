#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cfloat>
#include <map>
#include <set>
#include <string>

#include <boost/filesystem/operations.hpp>

#include "libslic3r/Config.hpp"
#include "libslic3r/Format/CMB.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include "test_helpers.hpp"

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

// Applies a shipped Stratasys preset onto `config`, its `inherits` chain first.
void apply_stratasys_preset(DynamicPrintConfig &config, const std::string &type, const std::string &name)
{
    const std::string path = std::string(PROFILES_DIR) + "/Stratasys/" + type + "/" + name + ".json";
    REQUIRE(boost::filesystem::exists(path));
    DynamicPrintConfig                 preset;
    std::map<std::string, std::string> key_values;
    ConfigSubstitutionContext          substitutions { ForwardCompatibilitySubstitutionRule::Enable };
    std::string                        reason;
    preset.load_from_json(path, substitutions, false, key_values, reason);
    if (const auto parent = key_values.find("inherits"); parent != key_values.end() && !parent->second.empty())
        apply_stratasys_preset(config, type, parent->second);
    config.apply(preset, true);
}

// The F170 with T14 tips, as shipped, with ABS on head 1 and QSR on head 2.
DynamicPrintConfig f170_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    apply_stratasys_preset(config, "machine", "Stratasys F170 T14 model T14 support");
    apply_stratasys_preset(config, "process", "0.254mm Standard @Stratasys F170 T14-T14");
    Test::size_flush_to_nozzles(config);
    config.set_deserialize_strict({
        { "filament_diameter", "1.78,1.78" },
        { "filament_notes", "\"CMB_MATERIAL=ABS\";\"CMB_MATERIAL=QSR\"" },
        { "filament_is_support", "0,1" },
        { "filament_soluble", "0,1" },
        { "filament_shrink", "99.41%,100%" },
        { "filament_shrinkage_compensation_z", "99.51%,100%" },
        // The purge tower is left out: it needs complete material presets on both heads, which the app has and this
        // config does not.
        { "enable_prime_tower", "0" },
    });
    return config;
}

} // namespace

TEST_CASE("A cube sliced for the F170 exports as a CMB job laid out like GrabCAD Print's", "[CMB]")
{
    const DynamicPrintConfig config = f170_config();
    // A 20 mm cube overhanging the bed by nothing, in the middle of the F170's 254 mm tray.
    TriangleMesh cube(its_make_cube(20., 20., 20.));
    cube.translate(117.f, 117.f, 0.f);
    Print print;
    Model model;
    std::vector<TriangleMesh> meshes { cube };
    Test::init_print(std::move(meshes), print, model, config, nullptr, /*arrange=*/false);
    // The default bridge width, 100% of the 0.356 mm tip, computes as a hair over the tip and must still be accepted.
    CHECK(print.validate().string == "");
    GCodeProcessorResult toolpaths;
    Test::gcode(print, &toolpaths);

    const CmbJob job  = cmb_job_from_toolpaths(toolpaths, cmb_export_settings(print.full_print_config(), "cube", 0));
    const CmbJob back = parse_cmb(serialize_cmb(job));
    REQUIRE(back.opaque.writable);
    CHECK(back.header.part_material == "ABS");
    CHECK(back.header.support_material == "QSR");
    CHECK(back.header.part_tip == "T14");
    CHECK(back.header.packs == std::vector<std::string> { "cube" });

    // The two-layer support base starts at 0.014 in; the part above it is about 20 mm, 0.79 in, at 0.010 in a layer,
    // plus the shrinkage compensation.
    REQUIRE(back.layers.size() > 80);
    CHECK_THAT(back.layers.front().min.z(), WithinAbs(0.014, 1e-3));
    std::set<uint8_t> first_layer, all;
    for (const CmbMove &move : back.layers.front().moves)
        first_layer.insert(move.mode);
    for (const CmbLayer &layer : back.layers)
        for (const CmbMove &move : layer.moves)
            all.insert(move.mode);
    // Only support material on the sheet, and the F170's modes throughout: base, support and part surface.
    for (uint8_t mode : first_layer)
        CHECK(cmb_mode_is_support(mode));
    CHECK(first_layer.count(0xC9) == 1);
    CHECK(all.count(0x66) == 1);
    CHECK(all.count(0xC8) == 1);

    // As in GrabCAD Print's jobs, the toolpaths are scaled by the model material's shrink factors: the part surface of
    // a middle layer runs 20 mm times 1.0059, less one 0.51 mm bead, from side to side.
    const CmbLayer &middle = back.layers[back.layers.size() / 2];
    float           x_min = FLT_MAX, x_max = -FLT_MAX;
    for (const CmbMove &move : middle.moves)
        if (move.mode == 0x66 && move.kind == CmbMove::Kind::Line)
            x_min = std::min(x_min, move.position.x()), x_max = std::max(x_max, move.position.x());
    CHECK_THAT((x_max - x_min) * 25.4, WithinAbs(20. * 100. / 99.41 - 0.51, 0.02));
    // From the first layer to the last one's lift: the second base layer, the part, and the 0.06 in lift.
    CHECK_THAT(back.header.bounding_box.size().z() * 25.4, WithinAbs(0.254 + 20. * 100. / 99.51 + 25.4 * 0.06, 0.3));
}
