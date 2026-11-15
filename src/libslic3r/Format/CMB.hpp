#pragma once

#include <array>
#include <cstdint>
#include <ostream>
#include <string>
#include <vector>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Point.hpp"

namespace Slic3r {

// Orca: Stratasys CMB toolpath jobs, as written by Insight, CatalystEX and GrabCAD Print for Fortus, Dimension and
// F-series printers. A job is a gzipped little-endian binary: a header, then per layer a stream of 1-byte opcodes,
// then a table of where each layer starts, then sections the header points at (outlines, a thumbnail). Coordinates
// are in inches. Nothing about the format is published; the layout follows
// https://azttm.wordpress.com/2012/09/22/inside-the-stratasys-dimension-catalyst-cmb-file-format/
// and GrabCAD Print 26.10 jobs for the F170.

struct CmbHeader
{
    // The four bytes after the magic, such as 0E 09 06 01 from GrabCAD Print for the F170.
    std::array<uint8_t, 4>  format {};
    float                   slice_height { 0.f };
    // Cubic inches.
    float                   part_volume { 0.f };
    float                   support_volume { 0.f };
    std::string             part_material;
    std::string             support_material;
    BoundingBoxf3           bounding_box;
    // Bead area, width and aspect ratio limits for the part and support heads.
    std::array<float, 12>   limits {};
    int                     layer_count { 0 };
    // The fields below follow in the order shown; a file whose layout differs leaves them empty.
    std::string             comment;
    std::vector<std::string> packs;
    std::string             version;
    std::string             machine;
    std::string             part_tip;
    std::string             support_tip;
    int                     build_time_s { 0 };
};

struct CmbMove
{
    enum class Kind : uint8_t {
        // 0B 00: positioning move between paths, nothing extruded.
        Rapid,
        // 0B 02: vertical move into or out of a layer, nothing extruded.
        ZMove,
        // 0B 01: horizontal move along a path, extruding the current bead area.
        Line,
        // 0B 03: horizontal move that starts, ends or changes a bead; `area` is the new bead area.
        SetArea,
    };
    Kind    kind;
    // The deposition mode in effect (0D), such as 0x66 part surface or 0xC8 support.
    uint8_t mode;
    // Whether the job sets the mode (0D) right before this move, which it may do without changing it.
    bool    sets_mode { false };
    // Inches. Z is the current height for a Line or SetArea.
    Vec3f   position;
    // Bead cross-section in square inches after this move, which is width times layer thickness.
    float   area;
};

struct CmbLayer
{
    // Inches: the extent of the layer's toolpaths, whose top is the height above the layer the head moves at.
    Vec3f                min { Vec3f::Zero() };
    Vec2f                max_xy { Vec2f::Zero() };
    float                z_top { 0.f };
    float                thickness { 0.f };
    // GrabCAD Print stores the thickness with the extent; CatalystEX does not.
    bool                 stores_thickness { true };
    // GrabCAD Print's 0x13 record after the extent, which F900 jobs and F170 jobs without a purge tower carry on
    // every layer. Its meaning is unknown: a type (0, 2 or 3), then values that may be a time or a temperature.
    std::string          record13;
    std::vector<CmbMove> moves;
};

// The parts of a GrabCAD Print job Orca does not interpret, kept so that a job reads back and writes out unchanged.
// They include each material's shrink factors (X, Y, Z): GrabCAD Print scales the toolpaths by them before writing
// the job, and records them in the header.
struct CmbOpaque
{
    // False when the job's header is laid out differently; such a job is shown but cannot be written.
    bool                writable { false };
    // From the end of the build time to the first layer: the job id, the pack outlines' and thumbnail's
    // addresses, shrink factors, tip names and pack placement.
    std::string         header_tail;
    // Where the addresses sit in `header_tail`.
    std::vector<size_t> address_fields;
    // After the layer table: the pack outlines and the thumbnail.
    std::string         trailer;
    // Where the trailer started in the file read, which the addresses are relative to.
    int32_t             trailer_start { 0 };
};

struct CmbJob
{
    CmbHeader             header;
    std::vector<CmbLayer> layers;
    CmbOpaque             opaque;
};

bool is_cmb_file(const std::string &path);

// Throws Slic3r::RuntimeError on a malformed job. `read_cmb` accepts a gzipped or a plain job.
CmbJob parse_cmb(const std::string &data);
CmbJob read_cmb(const std::string &path);

// The job's bytes, uncompressed; and the job written to a file, gzipped as F-series printers load it or plain as
// Fortus printers do. Throws Slic3r::RuntimeError when the job is not writable.
std::string serialize_cmb(const CmbJob &job);
void        write_cmb(const CmbJob &job, const std::string &path, bool gzipped);

// Whether a deposition mode lays support material rather than model material.
bool cmb_mode_is_support(uint8_t mode);

struct GCodeProcessorResult;
class DynamicPrintConfig;

// What a job converted from sliced toolpaths takes from the printer and its materials. The printer and material
// notes name what the printer knows them by, one KEY=value per line: CMB_MACHINE (dorado1 for the F170, f900),
// CMB_MODEL_TIP and CMB_SUPPORT_TIP in the printer notes, CMB_MATERIAL in each material's notes.
struct CmbExportSettings
{
    std::string machine;
    std::string model_tip;
    std::string support_tip;
    std::string model_material;
    std::string support_material;
    // The model material's shrink factors, which the toolpaths are already scaled by; the job records them.
    Vec3f       shrink { Vec3f::Ones() };
    // Millimetres.
    double      layer_height { 0. };
    BoundingBoxf bed;
    double      max_print_height { 0. };
    std::string job_name;
    int         build_time_s { 0 };
};

// Reads the export settings from a full print config. Throws Slic3r::RuntimeError naming what is missing.
CmbExportSettings cmb_export_settings(const DynamicPrintConfig &config, const std::string &job_name, int build_time_s);

// Converts sliced toolpaths, head 1 laying the model and head 2 the support, into a job for the printer, laid out as
// GrabCAD Print lays out its own. Throws Slic3r::RuntimeError, without writing anything, when the toolpaths fall
// outside what GrabCAD Print jobs for the printer have been seen to contain: another first layer height or layer
// pitch, another head, a bead too wide for the layer, or a move off the bed. A bead narrower than half the layer
// thickness, narrower than any GrabCAD Print writes, is left out.
CmbJob cmb_job_from_toolpaths(const GCodeProcessorResult &toolpaths, const CmbExportSettings &settings);

// Writes the job as G-code in millimetres, tagged with the extrusion roles, widths and layer heights the G-code
// preview reads, so a job can be inspected there like a sliced print. Stratasys printers set speeds from the mode,
// so the feedrates are nominal and the preview's time estimate is not the printer's.
void cmb_to_gcode(const CmbJob &job, std::ostream &out);

} // namespace Slic3r
