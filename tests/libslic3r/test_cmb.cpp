#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstring>
#include <set>
#include <sstream>
#include <string>

#include <boost/filesystem/operations.hpp>
#include <zlib.h>

#include "libslic3r/Exception.hpp"
#include "libslic3r/Format/CMB.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"

using namespace Slic3r;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

// Builds a job in the layout GrabCAD Print writes for the F170.
class CmbWriter
{
public:
    std::string data;

    void bytes(std::initializer_list<uint8_t> b) { for (uint8_t v : b) data.push_back(char(v)); }
    void i32(int32_t v) { append(&v, 4); }
    void f32(float v) { append(&v, 4); }
    void str(const std::string &s) { i32(int32_t(s.size())); data += s; }
    void move(uint8_t sub, std::initializer_list<float> values) { bytes({ 0x0B, sub }); for (float v : values) f32(v); }

private:
    void append(const void *p, size_t n) { data.append(static_cast<const char *>(p), n); }
};

constexpr float bead_area = 0.0002f; // 0.02 in wide at 0.01 in thick

// One layer: a 0.1 in square of part surface, then 0.05 in more along its first side.
std::string square_job()
{
    CmbWriter w;
    w.data = "3 3 ";
    w.bytes({ 0x0E, 0x09, 0x06, 0x01 });
    w.f32(0.01f); w.f32(0.5f); w.f32(0.1f);
    w.str("ABS"); w.str("QSR");
    for (float v : { 1.f, 1.f, 0.f, 1.1f, 1.1f, 0.01f }) w.f32(v);
    for (int i = 0; i < 12; ++i) w.f32(0.f);
    w.i32(1);
    const size_t table_field = w.data.size();
    w.i32(0);
    w.str("Written by GrabCAD Print!!!!!");
    w.i32(1); w.str("square");
    w.str("26.10 (7632)"); w.str("dorado1"); w.str("T14"); w.str("T14");
    w.i32(60);
    // GrabCAD Print's signature, job id and quality, seven bytes of unknown meaning, then two section addresses,
    // which point past the layer table.
    w.str("none"); w.str("job"); w.str("normal");
    w.bytes({ 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00 });
    const size_t section_field = w.data.size();
    w.i32(0);
    w.i32(0);

    const int32_t layer_start = int32_t(w.data.size());
    w.bytes({ 0x05 }); for (float v : { 1.f, 1.f, 0.01f }) w.f32(v);
    w.bytes({ 0x06 }); for (float v : { 1.1f, 1.1f, 0.05f, 0.01f }) w.f32(v);
    w.bytes({ 0x0D, 0x66, 0x00 });
    w.move(0x00, { 1.f, 1.f, 0.05f }); w.i32(0);
    w.move(0x02, { 1.f, 1.f, 0.01f });
    w.move(0x03, { 1.1f, 1.f, bead_area });
    w.move(0x01, { 1.1f, 1.1f });
    w.move(0x01, { 1.f, 1.1f });
    w.move(0x01, { 1.f, 1.f });
    w.move(0x03, { 1.05f, 1.f, 0.f });
    w.move(0x02, { 1.05f, 1.f, 0.05f });
    w.bytes({ 0x07, 0x03 });

    const int32_t table = int32_t(w.data.size());
    w.i32(layer_start);
    std::memcpy(&w.data[table_field], &table, 4);
    const int32_t section = int32_t(w.data.size());
    std::memcpy(&w.data[section_field], &section, 4);
    w.bytes({ 0x11, 0x01, 0x02, 0x03 });
    const int32_t thumbnail = int32_t(w.data.size());
    std::memcpy(&w.data[section_field + 4], &thumbnail, 4);
    w.bytes({ 0x15, 0x04, 0x05 });
    return w.data;
}

} // namespace

TEST_CASE("A CMB job's header and toolpaths are read", "[CMB]")
{
    const CmbJob job = parse_cmb(square_job());

    CHECK(job.header.part_material == "ABS");
    CHECK(job.header.support_material == "QSR");
    CHECK(job.header.machine == "dorado1");
    CHECK(job.header.part_tip == "T14");
    CHECK(job.header.build_time_s == 60);
    CHECK_THAT(job.header.slice_height, WithinAbs(0.01, 1e-6));

    REQUIRE(job.layers.size() == 1);
    const CmbLayer &layer = job.layers.front();
    CHECK_THAT(layer.thickness, WithinAbs(0.01, 1e-6));
    REQUIRE(layer.moves.size() == 8);
    CHECK(layer.moves[0].kind == CmbMove::Kind::Rapid);
    CHECK(layer.moves[2].kind == CmbMove::Kind::SetArea);
    CHECK_THAT(layer.moves[2].area, WithinAbs(bead_area, 1e-9));
    // A horizontal move keeps the height the head was lowered to and the bead it was given.
    CHECK(layer.moves[3].kind == CmbMove::Kind::Line);
    CHECK_THAT(layer.moves[3].position.z(), WithinAbs(0.01, 1e-6));
    CHECK_THAT(layer.moves[3].area, WithinAbs(bead_area, 1e-9));
    for (const CmbMove &move : layer.moves)
        CHECK(move.mode == 0x66);
}

TEST_CASE("A CMB job converts to G-code that carries its roles and bead volume", "[CMB]")
{
    const bool bbl = GCodeProcessor::s_IsBBLPrinter;
    GCodeProcessor::s_IsBBLPrinter = false;
    std::ostringstream out;
    cmb_to_gcode(parse_cmb(square_job()), out);
    GCodeProcessor::s_IsBBLPrinter = bbl;
    const std::string gcode = out.str();

    // Mode 0x66 is the part surface.
    CHECK(gcode.find(";TYPE:Outer wall") != std::string::npos);

    // The square and the closing 0.05 in are extruded at the bead area, nothing else.
    double             e = 0.;
    std::istringstream lines(gcode);
    for (std::string line; std::getline(lines, line);)
        if (const size_t at = line.find(" E"); at != std::string::npos)
            e += std::stod(line.substr(at + 2));
    const double inch           = 25.4;
    const double volume_mm3     = bead_area * 0.45 * inch * inch * inch;
    const double filament_area  = M_PI * 0.875 * 0.875;
    CHECK_THAT(e * filament_area, WithinRel(volume_mm3, 1e-3));
}

TEST_CASE("A CMB job is written back byte for byte", "[CMB]")
{
    const std::string data = square_job();
    const CmbJob      job  = parse_cmb(data);
    REQUIRE(job.opaque.writable);
    CHECK(serialize_cmb(job) == data);
}

TEST_CASE("A CMB job keeps its sections addressed when its toolpaths change", "[CMB]")
{
    CmbJob job = parse_cmb(square_job());
    job.layers.front().moves.pop_back();
    const CmbJob edited = parse_cmb(serialize_cmb(job));
    REQUIRE(edited.opaque.writable);
    CHECK(edited.layers.front().moves.size() == job.layers.front().moves.size());
    CHECK(edited.opaque.trailer == job.opaque.trailer);
}

TEST_CASE("A truncated or foreign file is rejected as a CMB job", "[CMB]")
{
    const std::string job = square_job();
    CHECK_THROWS_AS(parse_cmb(job.substr(0, job.size() - 10)), Slic3r::RuntimeError);
    CHECK_THROWS_AS(parse_cmb("; G-code\nG1 X0\n"), Slic3r::RuntimeError);
}

// The sample jobs live outside tests/data because they include a customer part; the test only runs where they are
// present. They cover the F170 with and without a purge tower and the F900 across tips and materials.
TEST_CASE("GrabCAD Print jobs for the F170 and F900 are read in full and written back unchanged", "[CMB]")
{
    namespace fs = boost::filesystem;
    const fs::path dir = fs::path(std::string(TEST_DATA_DIR)) / ".." / "fortus_TestFiles";
    if (!fs::is_directory(dir))
        SKIP("The Fortus sample jobs are not present");

    size_t jobs = 0;
    for (const fs::directory_entry &entry : fs::directory_iterator(dir)) {
        const std::string path = entry.path().string();
        if (!is_cmb_file(path))
            continue;
        ++jobs;
        DYNAMIC_SECTION(entry.path().filename().string())
        {
            const CmbJob job = read_cmb(path);
            CHECK(job.header.comment == "Written by GrabCAD Print!!!!!");
            CHECK(job.layers.size() == size_t(job.header.layer_count));

            // The whole job is accounted for: written back, it is the file GrabCAD Print wrote.
            REQUIRE(job.opaque.writable);
            gzFile      file = gzopen(path.c_str(), "rb");
            std::string data;
            char        buffer[1 << 16];
            for (int n; (n = gzread(file, buffer, sizeof(buffer))) > 0;)
                data.append(buffer, size_t(n));
            gzclose(file);
            // Compared as a flag: Catch2 cannot print a binary string of hundreds of kilobytes.
            const std::string written   = serialize_cmb(job);
            const bool        identical = written == data;
            CHECK(written.size() == data.size());
            CHECK(identical);
        }
    }
    CHECK(jobs > 0);
}

namespace {

using Move = GCodeProcessorResult::MoveVertex;

Move toolpath_move(EMoveType type, ExtrusionRole role, int head, float x, float y, float z)
{
    Move move;
    move.type           = type;
    move.extrusion_role = role;
    move.extruder_id    = (unsigned char) head;
    move.position       = Vec3f(x, y, z);
    move.width          = 0.51f;
    move.height         = 0.254f;
    return move;
}

// A 10 mm square on head 1 for each layer, over a support base line of head 2 on the first layer, at the F170's
// 0.356 mm first layer and 0.254 mm layers.
void square_toolpaths(GCodeProcessorResult &result, float first_layer_z = 0.3556f)
{
    auto &m = result.moves;
    m.push_back(toolpath_move(EMoveType::Travel, erNone, 1, 100.f, 90.f, first_layer_z));
    m.push_back(toolpath_move(EMoveType::Extrude, erSupportMaterial, 1, 120.f, 90.f, first_layer_z));
    for (float z : { first_layer_z, first_layer_z + 0.254f }) {
        m.push_back(toolpath_move(EMoveType::Travel, erNone, 0, 100.f, 100.f, z));
        for (auto [x, y] : { std::pair { 110.f, 100.f }, { 110.f, 110.f }, { 100.f, 110.f }, { 100.f, 100.f } })
            m.push_back(toolpath_move(EMoveType::Extrude, erExternalPerimeter, 0, x, y, z));
    }
}

CmbExportSettings f170_settings()
{
    CmbExportSettings s;
    s.machine          = "dorado1";
    s.model_tip        = "T14";
    s.support_tip      = "T14";
    s.model_material   = "ABS";
    s.support_material = "QSR";
    s.shrink           = Vec3f(1.0059f, 1.0059f, 1.0049f);
    s.layer_height     = 0.254;
    s.bed              = BoundingBoxf(Vec2d::Zero(), Vec2d(254., 254.));
    s.max_print_height = 254.;
    s.job_name         = "square";
    s.build_time_s     = 600;
    return s;
}

} // namespace

TEST_CASE("Sliced toolpaths convert to a CMB job that reads back as written", "[CMB]")
{
    GCodeProcessorResult toolpaths;
    square_toolpaths(toolpaths);
    const CmbJob      job  = cmb_job_from_toolpaths(toolpaths, f170_settings());
    const std::string data = serialize_cmb(job);
    const CmbJob      back = parse_cmb(data);

    REQUIRE(back.opaque.writable);
    CHECK(serialize_cmb(back) == data);
    CHECK(back.header.machine == "dorado1");
    CHECK(back.header.part_material == "ABS");
    CHECK(back.header.support_material == "QSR");
    REQUIRE(back.layers.size() == 2);
    // The first layer sits at 0.014 in, the next one slice height higher, and the head lifts above each.
    CHECK_THAT(back.layers[0].min.z(), WithinAbs(0.014, 1e-4));
    CHECK_THAT(back.layers[1].min.z() - back.layers[0].min.z(), WithinAbs(0.01, 1e-4));
    CHECK(back.layers[0].z_top > back.layers[0].min.z());

    // F170 modes: the support base, the model's first layer, then the part surface.
    std::set<uint8_t> first, second;
    float             widest = 0.f;
    for (const CmbMove &move : back.layers[0].moves)
        first.insert(move.mode), widest = std::max(widest, move.area);
    for (const CmbMove &move : back.layers[1].moves)
        second.insert(move.mode);
    CHECK(first == std::set<uint8_t> { 0xC9, 0x6B });
    CHECK(second == std::set<uint8_t> { 0x66 });
    // A bead is counted as width times layer thickness.
    CHECK_THAT(widest, WithinRel(0.51 * 0.254 / (25.4 * 25.4), 1e-4));

    // Every bead starts and ends with a SetArea move, the end one at no area.
    const std::vector<CmbMove> &moves = back.layers[1].moves;
    auto                        ends  = std::count_if(moves.begin(), moves.end(),
                                                      [](const CmbMove &m) { return m.kind == CmbMove::Kind::SetArea && m.area == 0.f; });
    CHECK(ends == 1);
}

TEST_CASE("A thin wall tapering below the narrowest bead Stratasys jobs carry is written without its tip", "[CMB]")
{
    GCodeProcessorResult toolpaths;
    square_toolpaths(toolpaths);
    // On the second layer, a wall 0.2525 mm wide, just under the 0.254 mm layer, ending in a 0.1 mm tip: GrabCAD Print
    // writes beads down to half the layer thickness, 0.127 mm.
    const float z = toolpaths.moves.back().position.z();
    toolpaths.moves.push_back(toolpath_move(EMoveType::Travel, erNone, 0, 130.f, 100.f, z));
    for (auto [x, width] : { std::pair { 135.f, 0.2525f }, { 136.f, 0.1f } }) {
        Move move  = toolpath_move(EMoveType::Extrude, erExternalPerimeter, 0, x, 100.f, z);
        move.width = width;
        toolpaths.moves.push_back(move);
    }

    const CmbJob back = parse_cmb(serialize_cmb(cmb_job_from_toolpaths(toolpaths, f170_settings())));
    REQUIRE(back.layers.size() == 2);
    float narrowest = FLT_MAX, east = -FLT_MAX;
    for (const CmbMove &move : back.layers[1].moves)
        if (move.area > 0.f)
            narrowest = std::min(narrowest, move.area), east = std::max(east, move.position.x());
    CHECK_THAT(narrowest, WithinRel(0.2525 * 0.254 / (25.4 * 25.4), 1e-4));
    // The wall's bead ends by x = 135 mm, short of its tip.
    CHECK(east * 25.4f < 135.01f);
}

TEST_CASE("Toolpaths a Stratasys printer would not take are refused", "[CMB]")
{
    // A first layer of 0.254 mm instead of the 0.356 mm every job starts with.
    GCodeProcessorResult low;
    square_toolpaths(low, 0.254f);
    CHECK_THROWS_AS(cmb_job_from_toolpaths(low, f170_settings()), Slic3r::RuntimeError);

    GCodeProcessorResult toolpaths;
    square_toolpaths(toolpaths);

    // A square off the edge of a 100 mm bed.
    CmbExportSettings small_bed = f170_settings();
    small_bed.bed               = BoundingBoxf(Vec2d::Zero(), Vec2d(100., 100.));
    CHECK_THROWS_AS(cmb_job_from_toolpaths(toolpaths, small_bed), Slic3r::RuntimeError);

    // A bead 1.5 mm wide on a 0.254 mm layer, wider than any Stratasys job lays.
    GCodeProcessorResult wide;
    square_toolpaths(wide);
    wide.moves.back().width = 1.5f;
    CHECK_THROWS_AS(cmb_job_from_toolpaths(wide, f170_settings()), Slic3r::RuntimeError);

    // A machine Orca has no layout for.
    CmbExportSettings unknown = f170_settings();
    unknown.machine           = "uprint";
    CHECK_THROWS_AS(cmb_job_from_toolpaths(toolpaths, unknown), Slic3r::RuntimeError);
}
