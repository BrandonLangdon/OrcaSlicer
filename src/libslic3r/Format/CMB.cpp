#include "CMB.hpp"

#include <cfloat>
#include <cmath>
#include <cstring>
#include <random>
#include <sstream>
#include <zlib.h>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/format.hpp>

#include "libslic3r/Exception.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/I18N.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/format.hpp"

namespace Slic3r {

namespace {

enum Opcode : uint8_t {
    OpEndOfProgram = 0x03,
    OpLayerMin     = 0x05,
    OpLayerMax     = 0x06,
    OpEndOfLayer   = 0x07,
    OpMove         = 0x0B,
    OpMode         = 0x0D,
    OpRecord13     = 0x13,
};

// GrabCAD Print's header holds two section addresses, whether or not the job has the purge tower that the sections
// mostly describe.
constexpr int SectionAddresses = 2;
constexpr int Record13Size     = 16;

// Bounds-checked little-endian reader. Every platform Orca builds for is little-endian, as the format is.
class Reader
{
public:
    explicit Reader(const std::string &data) : m_data(data) {}

    size_t pos() const { return m_pos; }
    void   seek(size_t pos) { require(pos, 0); m_pos = pos; }
    void   skip(size_t n) { require(m_pos, n); m_pos += n; }

    uint8_t peek() const { require(m_pos, 1); return uint8_t(m_data[m_pos]); }
    uint8_t byte() { uint8_t v = peek(); ++m_pos; return v; }
    int32_t i32() { return read<int32_t>(); }
    float   f32() { return read<float>(); }

    std::string str()
    {
        const int32_t n = i32();
        if (n < 0)
            fail("a string of negative length");
        require(m_pos, size_t(n));
        std::string s = m_data.substr(m_pos, size_t(n));
        m_pos += size_t(n);
        return s;
    }

    [[noreturn]] void fail(const std::string &what) const
    {
        // TRN %1% describes the problem in English, such as "unknown command 0x0c"; %2% is a byte offset in the file.
        throw Slic3r::RuntimeError(format(_u8L("The Stratasys CMB job is damaged or of an unsupported version: %1% at byte %2%."), what, m_pos));
    }

private:
    template<typename T> T read()
    {
        require(m_pos, sizeof(T));
        T v;
        std::memcpy(&v, m_data.data() + m_pos, sizeof(T));
        m_pos += sizeof(T);
        return v;
    }

    void require(size_t pos, size_t n) const
    {
        if (pos > m_data.size() || n > m_data.size() - pos)
            fail("unexpected end of file");
    }

    const std::string &m_data;
    size_t             m_pos { 0 };
};

// Little-endian writer, the counterpart of Reader.
class Writer
{
public:
    std::string data;

    size_t size() const { return data.size(); }
    void   byte(uint8_t v) { data.push_back(char(v)); }
    void   i32(int32_t v) { append(&v, sizeof(v)); }
    void   f32(float v) { append(&v, sizeof(v)); }
    void   str(const std::string &s) { i32(int32_t(s.size())); data += s; }
    void   i32_at(size_t pos, int32_t v) { std::memcpy(&data[pos], &v, sizeof(v)); }

private:
    void append(const void *p, size_t n) { data.append(static_cast<const char *>(p), n); }
};

// Reads the header up to the build time, and returns where the GrabCAD Print fields after it start, or 0 when the
// header is laid out differently.
size_t read_header(Reader &in, CmbHeader &header, int32_t &layer_table, CmbOpaque &opaque)
{
    char magic[4];
    for (char &c : magic)
        c = char(in.byte());
    if (std::memcmp(magic, "3 3 ", 4) != 0)
        in.fail("not a CMB job");
    for (uint8_t &b : header.format)
        b = in.byte();
    header.slice_height     = in.f32();
    header.part_volume      = in.f32();
    header.support_volume   = in.f32();
    header.part_material    = in.str();
    header.support_material = in.str();
    Vec3d min, max;
    for (int i = 0; i < 3; ++i) min[i] = in.f32();
    for (int i = 0; i < 3; ++i) max[i] = in.f32();
    header.bounding_box = BoundingBoxf3(min, max);
    for (float &limit : header.limits)
        limit = in.f32();
    header.layer_count = in.i32();
    layer_table        = in.i32();

    // Descriptive fields, which some versions lay out differently; the toolpaths do not depend on them.
    try {
        header.comment = in.str();
        const int32_t packs = in.i32();
        if (packs < 0 || packs > 10000)
            in.fail("an implausible pack count");
        for (int32_t i = 0; i < packs; ++i)
            header.packs.emplace_back(in.str());
        header.version      = in.str();
        header.machine      = in.str();
        header.part_tip     = in.str();
        header.support_tip  = in.str();
        header.build_time_s = in.i32();
    } catch (const Slic3r::RuntimeError &) {
        header.comment.clear();
        header.packs.clear();
        header.version.clear();
        header.machine.clear();
        header.part_tip.clear();
        header.support_tip.clear();
        header.build_time_s = 0;
        return 0;
    }

    // GrabCAD Print: a signature ("none"), the job id and a quality name, then seven bytes of unknown meaning (2 0 0 0
    // 2 0 0 with a purge tower, zeros without), then the addresses of the sections after the layer table.
    const size_t tail = in.pos();
    try {
        in.str();
        in.str();
        in.str();
        in.skip(7);
        for (int i = 0; i < SectionAddresses; ++i) {
            opaque.address_fields.emplace_back(in.pos() - tail);
            in.i32();
        }
    } catch (const Slic3r::RuntimeError &) {
        opaque.address_fields.clear();
        return 0;
    }
    return tail;
}

} // namespace

bool is_cmb_file(const std::string &path)
{
    return boost::iends_with(path, ".cmb") || boost::iends_with(path, ".cmb.gz");
}

bool cmb_mode_is_support(uint8_t mode)
{
    return mode >= 0xC8 && mode <= 0xCF;
}

CmbJob parse_cmb(const std::string &data)
{
    CmbJob       job;
    Reader       in(data);
    int32_t      layer_table = 0;
    const size_t tail        = read_header(in, job.header, layer_table, job.opaque);

    // The layer table holds the address of every layer, the first one included, so the commands are found without
    // depending on the layout of the header between.
    if (job.header.layer_count <= 0)
        return job;
    in.seek(size_t(layer_table));
    const size_t commands = size_t(in.i32());
    in.seek(commands);

    uint8_t  mode      = 0;
    bool     sets_mode = false;
    float    area      = 0.f;
    Vec3f    pos       = Vec3f::Zero();
    CmbLayer layer;
    for (;;) {
        const uint8_t op = in.byte();
        if (op == OpEndOfProgram) {
            if (!layer.moves.empty())
                job.layers.emplace_back(std::move(layer));
            break;
        }
        switch (op) {
        case OpLayerMin:
            layer.min.x() = in.f32();
            layer.min.y() = in.f32();
            layer.min.z() = in.f32();
            break;
        case OpLayerMax:
            layer.max_xy.x() = in.f32();
            layer.max_xy.y() = in.f32();
            layer.z_top      = in.f32();
            // GrabCAD Print adds the layer thickness, which CatalystEX does not; a command follows right away there.
            if (const uint8_t next = in.peek(); next == OpMove || next == OpMode || next == OpEndOfLayer || next == OpRecord13) {
                layer.thickness        = job.header.slice_height;
                layer.stores_thickness = false;
            } else
                layer.thickness = in.f32();
            break;
        case OpRecord13:
            layer.record13 = data.substr(in.pos(), Record13Size);
            in.skip(Record13Size);
            break;
        case OpEndOfLayer:
            job.layers.emplace_back(std::move(layer));
            layer = CmbLayer();
            break;
        case OpMode:
            mode      = in.byte();
            sets_mode = true;
            in.skip(1);
            break;
        case OpMove: {
            const uint8_t sub  = in.byte();
            CmbMove       move { CmbMove::Kind::Line, mode, sets_mode, pos, area };
            // One statement per coordinate: the order in which function arguments are evaluated is unspecified.
            const auto read_xy = [&in, &move]() {
                move.position.x() = in.f32();
                move.position.y() = in.f32();
            };
            switch (sub) {
            case 0x00:
                move.kind = CmbMove::Kind::Rapid;
                read_xy();
                move.position.z() = in.f32();
                in.skip(4);
                break;
            case 0x01: read_xy(); break;
            case 0x02:
                move.kind = CmbMove::Kind::ZMove;
                read_xy();
                move.position.z() = in.f32();
                break;
            case 0x03:
                move.kind = CmbMove::Kind::SetArea;
                read_xy();
                move.area = in.f32();
                break;
            default: in.fail((boost::format("unknown move type %1$#04x") % int(sub)).str());
            }
            pos       = move.position;
            area      = move.area;
            sets_mode = false;
            layer.moves.emplace_back(move);
            break;
        }
        default: in.fail((boost::format("unknown command %1$#04x") % int(op)).str());
        }
    }

    // Only a job laid out as GrabCAD Print writes it, every part of it accounted for, can be written back: the
    // commands run from the header up to the layer table, and the sections the header points at follow the table.
    const size_t trailer = size_t(layer_table) + 4 * size_t(job.header.layer_count);
    bool         known   = tail != 0 && commands >= tail && in.pos() == size_t(layer_table) && trailer <= data.size() &&
                 job.layers.size() == size_t(job.header.layer_count);
    for (size_t field : job.opaque.address_fields) {
        int32_t address;
        if (!known)
            break;
        std::memcpy(&address, data.data() + tail + field, sizeof(address));
        known = size_t(address) >= trailer && size_t(address) <= data.size();
    }
    if (known) {
        job.opaque.writable      = true;
        job.opaque.header_tail   = data.substr(tail, commands - tail);
        job.opaque.trailer       = data.substr(trailer);
        job.opaque.trailer_start = int32_t(trailer);
    } else
        job.opaque = CmbOpaque();
    return job;
}

CmbJob read_cmb(const std::string &path)
{
    // gzread passes a file that is not gzipped through unchanged.
    gzFile file = gzopen(path.c_str(), "rb");
    if (file == nullptr)
        throw Slic3r::RuntimeError(format(_u8L("Cannot open \"%1%\"."), path));
    std::string data;
    char        buffer[1 << 16];
    int         n;
    while ((n = gzread(file, buffer, sizeof(buffer))) > 0)
        data.append(buffer, size_t(n));
    const bool failed = n < 0;
    gzclose(file);
    if (failed)
        throw Slic3r::RuntimeError(format(_u8L("Cannot read \"%1%\"."), path));
    return parse_cmb(data);
}

std::string serialize_cmb(const CmbJob &job)
{
    if (!job.opaque.writable)
        throw Slic3r::RuntimeError(_u8L("This Stratasys CMB job cannot be written, because its layout is not one Orca knows."));

    const CmbHeader &h = job.header;
    Writer           out;
    out.data = "3 3 ";
    for (uint8_t b : h.format)
        out.byte(b);
    out.f32(h.slice_height);
    out.f32(h.part_volume);
    out.f32(h.support_volume);
    out.str(h.part_material);
    out.str(h.support_material);
    for (int i = 0; i < 3; ++i) out.f32(float(h.bounding_box.min[i]));
    for (int i = 0; i < 3; ++i) out.f32(float(h.bounding_box.max[i]));
    for (float limit : h.limits)
        out.f32(limit);
    out.i32(int32_t(job.layers.size()));
    const size_t table_field = out.size();
    out.i32(0);
    out.str(h.comment);
    out.i32(int32_t(h.packs.size()));
    for (const std::string &pack : h.packs)
        out.str(pack);
    out.str(h.version);
    out.str(h.machine);
    out.str(h.part_tip);
    out.str(h.support_tip);
    out.i32(h.build_time_s);
    const size_t tail = out.size();
    out.data += job.opaque.header_tail;

    std::vector<int32_t> layer_starts;
    for (const CmbLayer &layer : job.layers) {
        layer_starts.emplace_back(int32_t(out.size()));
        out.byte(OpLayerMin);
        out.f32(layer.min.x());
        out.f32(layer.min.y());
        out.f32(layer.min.z());
        out.byte(OpLayerMax);
        out.f32(layer.max_xy.x());
        out.f32(layer.max_xy.y());
        out.f32(layer.z_top);
        if (layer.stores_thickness)
            out.f32(layer.thickness);
        if (!layer.record13.empty()) {
            out.byte(OpRecord13);
            out.data += layer.record13;
        }
        for (const CmbMove &move : layer.moves) {
            if (move.sets_mode) {
                out.byte(OpMode);
                out.byte(move.mode);
                out.byte(0);
            }
            out.byte(OpMove);
            switch (move.kind) {
            case CmbMove::Kind::Rapid: out.byte(0x00); break;
            case CmbMove::Kind::Line: out.byte(0x01); break;
            case CmbMove::Kind::ZMove: out.byte(0x02); break;
            case CmbMove::Kind::SetArea: out.byte(0x03); break;
            }
            out.f32(move.position.x());
            out.f32(move.position.y());
            if (move.kind == CmbMove::Kind::Rapid || move.kind == CmbMove::Kind::ZMove)
                out.f32(move.position.z());
            if (move.kind == CmbMove::Kind::Rapid)
                out.i32(0);
            else if (move.kind == CmbMove::Kind::SetArea)
                out.f32(move.area);
        }
        out.byte(OpEndOfLayer);
    }
    out.byte(OpEndOfProgram);

    out.i32_at(table_field, int32_t(out.size()));
    for (int32_t start : layer_starts)
        out.i32(start);
    const int32_t trailer_start = int32_t(out.size());
    for (size_t field : job.opaque.address_fields) {
        int32_t address;
        std::memcpy(&address, job.opaque.header_tail.data() + field, sizeof(address));
        out.i32_at(tail + field, address - job.opaque.trailer_start + trailer_start);
    }
    out.data += job.opaque.trailer;
    return std::move(out.data);
}

void write_cmb(const CmbJob &job, const std::string &path, bool gzipped)
{
    const std::string data = serialize_cmb(job);
    // zlib's transparent mode ("T") writes the bytes as they are.
    gzFile            file = gzopen(path.c_str(), gzipped ? "wb9" : "wbT");
    if (file == nullptr)
        throw Slic3r::RuntimeError(format(_u8L("Cannot open \"%1%\"."), path));
    const bool written = gzwrite(file, data.data(), unsigned(data.size())) == int(data.size());
    if (gzclose(file) != Z_OK || !written)
        throw Slic3r::RuntimeError(format(_u8L("Cannot write \"%1%\"."), path));
}

// The extrusion role a deposition mode is shown as. Modes marked provisional are guessed from where and how they
// are laid in sample jobs and still need checking against GrabCAD Print's preview.
static ExtrusionRole cmb_mode_role(uint8_t mode)
{
    switch (mode) {
    case 0x64: return erWipeTower;            // purge tower, model material
    case 0x65: return erPerimeter;            // part interior contour
    case 0x66: return erExternalPerimeter;    // part surface
    case 0x67: return erInternalInfill;       // sparse raster
    case 0x68: return erSolidInfill;          // solid raster
    case 0x6B: return erBottomSurface;        // part first layer
    case 0x6D: return erPerimeter;            // provisional, in F900 jobs in place of 0x65
    case 0x6E: return erExternalPerimeter;    // part first layer perimeter
    case 0x6F: return erBridgeInfill;         // provisional
    case 0x71: return erGapFill;              // provisional
    case 0x72: return erGapFill;              // provisional
    case 0xCA: return erSupportMaterialInterface;
    case 0xCB: return erSupportTransition;    // sparse bridge, provisional
    case 0xCC: return erWipeTower;            // purge tower, support material
    default: return cmb_mode_is_support(mode) ? erSupportMaterial : erCustom;
    }
}

void cmb_to_gcode(const CmbJob &job, std::ostream &out)
{
    constexpr double inch = 25.4;
    // E is written for 1.75 mm filament, which the preview assumes when a G-code carries no settings, so the volumes
    // it reports are the job's.
    const double filament_area = M_PI * 0.875 * 0.875;
    const auto   tag           = [](GCodeProcessor::ETags t) { return GCodeProcessor::reserved_tag(t); };

    const CmbHeader &h = job.header;
    out << "; Stratasys CMB job converted for preview\n"
        << "; machine: " << h.machine << ", version: " << h.version << "\n"
        << "; part: " << h.part_material << " " << h.part_tip << ", support: " << h.support_material << " " << h.support_tip << "\n"
        << "; build time reported by the job: " << h.build_time_s << " s\n"
        << "G21\nG90\nM83\n";

    Vec3d   pos   = Vec3d::Zero();
    float   area  = 0.f;
    uint8_t mode  = 0;
    bool    first = true;
    auto    xy    = [&out](const Vec3d &p) { out << " X" << p.x() << " Y" << p.y(); };
    for (const CmbLayer &layer : job.layers) {
        out << ";" << tag(GCodeProcessor::ETags::Layer_Change) << "\n;Z:" << layer.z_top * inch << "\n;"
            << tag(GCodeProcessor::ETags::Height) << layer.thickness * inch << "\n";
        for (const CmbMove &move : layer.moves) {
            if (first || move.mode != mode) {
                mode  = move.mode;
                first = false;
                out << ";" << tag(GCodeProcessor::ETags::Role) << ExtrusionEntity::role_to_string(cmb_mode_role(mode))
                    << "\n; CMB mode " << (boost::format("%1$#04x") % int(mode)).str() << "\n";
            }
            const Vec3d to = move.position.cast<double>() * inch;
            // A SetArea move ramps the bead between the old and the new area; it is shown at the larger one.
            const float bead = move.kind == CmbMove::Kind::SetArea ? std::max(area, move.area) :
                               move.kind == CmbMove::Kind::Line    ? move.area : 0.f;
            if (bead > 0.f) {
                const double e = bead * inch * inch * (to - pos).norm() / filament_area;
                out << "G1";
                xy(to);
                out << " E" << e << " F1800\n";
            } else {
                out << "G0";
                xy(to);
                if (to.z() != pos.z())
                    out << " Z" << to.z();
                out << " F6000\n";
            }
            area = move.area;
            pos  = to;
        }
    }
    out << "; end of CMB job\n";
}

namespace {

constexpr double MmPerInch = 25.4;

// What GrabCAD Print writes the same way into every job for a machine.
struct CmbMachine
{
    const char            *id;
    std::array<uint8_t, 4> format;
    // The last of the six floats after the shrink factors; its meaning is unknown.
    float                  header_constant;
    // The F900 writes its support tip together with the support material, then three empty strings.
    bool                   support_tip_with_material;
    // F-series jobs print a support base under the part; Fortus jobs lay the part straight on the sheet.
    bool                   support_base;
};

const CmbMachine CmbMachines[] = {
    { "dorado1", { 0x0E, 0x09, 0x06, 0x01 }, 0.118f, false, true },
    { "f900",    { 0x0E, 0x09, 0x09, 0x01 }, 0.068f, true,  false },
};

// Every job has its first layer this high, in inches, whatever its slice height.
constexpr double FirstLayerZ = 0.014;
// The head rises this far, in inches, plus a slice height, between paths. GrabCAD Print also writes lower lifts.
constexpr double LiftHeight  = 0.05;
// The narrowest and widest beads, as multiples of the slice height, seen in GrabCAD Print jobs for the F170.
constexpr double MinBeadAspect = 0.5;
constexpr double MaxBeadAspect = 5.0;

// The deposition mode a toolpath of `role` on `head` (0 model, 1 support) is laid in, as GrabCAD Print jobs for the
// machine lay it.
uint8_t cmb_mode(const CmbMachine &machine, int head, ExtrusionRole role, bool first_layer)
{
    const bool f_series = machine.support_base;
    if (head == 1) {
        // The support base covers everything on the sheet, the purge tower's foot included.
        if (f_series && first_layer) return 0xC9;
        if (role == erWipeTower) return 0xCC;
        if (role == erSupportMaterialInterface) return 0xCA;
        if (f_series && role == erSupportTransition) return 0xCB;
        return f_series ? 0xC8 : 0xCD;
    }
    if (role == erWipeTower) return 0x64;
    if (first_layer) return 0x6B;
    switch (role) {
    case erExternalPerimeter: return 0x66;
    case erPerimeter:
    case erOverhangPerimeter: return f_series ? 0x65 : 0x6D;
    case erInternalInfill: return 0x67;
    case erGapFill: return 0x71;
    default: return 0x68;
    }
}

std::string note_value(const std::string &notes, const std::string &key)
{
    std::istringstream lines(notes);
    for (std::string line; std::getline(lines, line);) {
        boost::trim(line);
        if (boost::starts_with(line, key + "="))
            return line.substr(key.size() + 1);
    }
    return {};
}

std::string random_uuid()
{
    std::random_device rd;
    std::mt19937_64    gen((uint64_t(rd()) << 32) ^ rd());
    std::uniform_int_distribution<int> hex(0, 15);
    std::string uuid = "xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx";
    for (char &c : uuid) {
        if (c == 'x') c = "0123456789abcdef"[hex(gen)];
        else if (c == 'y') c = "89ab"[hex(gen) & 3];
    }
    return uuid;
}

// A white image of the size GrabCAD Print's thumbnails have, until Orca renders one.
std::string blank_png(uint32_t width, uint32_t height)
{
    auto be32 = [](std::string &s, uint32_t v) { for (int i = 3; i >= 0; --i) s.push_back(char((v >> (8 * i)) & 0xFF)); };
    auto chunk = [&be32](std::string &png, const char *type, const std::string &body) {
        be32(png, uint32_t(body.size()));
        std::string typed = std::string(type, 4) + body;
        png += typed;
        be32(png, uint32_t(crc32(0, reinterpret_cast<const Bytef *>(typed.data()), uInt(typed.size()))));
    };
    std::string raw;
    for (uint32_t y = 0; y < height; ++y) {
        raw.push_back(0);
        raw.append(size_t(width) * 4, char(0xFF));
    }
    uLongf      size = compressBound(uLong(raw.size()));
    std::string deflated(size, '\0');
    compress2(reinterpret_cast<Bytef *>(&deflated[0]), &size, reinterpret_cast<const Bytef *>(raw.data()), uLong(raw.size()), 9);
    deflated.resize(size);
    std::string png = "\x89PNG\r\n\x1a\n";
    std::string ihdr;
    be32(ihdr, width);
    be32(ihdr, height);
    ihdr += std::string("\x08\x06\x00\x00\x00", 5);
    chunk(png, "IHDR", ihdr);
    chunk(png, "IDAT", deflated);
    chunk(png, "IEND", {});
    return png;
}

[[noreturn]] void refuse(const std::string &why)
{
    // TRN %1% says what in the sliced toolpaths a Stratasys printer would not take.
    throw Slic3r::RuntimeError(format(_u8L("The Stratasys CMB job was not written: %1%"), why));
}

} // namespace

CmbExportSettings cmb_export_settings(const DynamicPrintConfig &config, const std::string &job_name, int build_time_s)
{
    CmbExportSettings s;
    const std::string printer_notes = config.opt_string("printer_notes");
    s.machine     = note_value(printer_notes, "CMB_MACHINE");
    s.model_tip   = note_value(printer_notes, "CMB_MODEL_TIP");
    s.support_tip = note_value(printer_notes, "CMB_SUPPORT_TIP");
    const auto *filament_notes = config.option<ConfigOptionStrings>("filament_notes");
    if (filament_notes != nullptr && !filament_notes->values.empty()) {
        s.model_material   = note_value(filament_notes->get_at(0), "CMB_MATERIAL");
        s.support_material = note_value(filament_notes->get_at(1), "CMB_MATERIAL");
    }
    if (s.machine.empty() || s.model_tip.empty() || s.support_tip.empty())
        refuse(_u8L("the printer's notes do not name its CMB_MACHINE, CMB_MODEL_TIP and CMB_SUPPORT_TIP."));
    if (s.model_material.empty() || s.support_material.empty())
        refuse(_u8L("the notes of the materials on both heads must name their CMB_MATERIAL."));

    // A shrinkage of 99.41% scales the model by 1 / 0.9941, the factor the job records.
    const auto *shrink_xy = config.option<ConfigOptionPercents>("filament_shrink");
    const auto *shrink_z  = config.option<ConfigOptionPercents>("filament_shrinkage_compensation_z");
    if (shrink_xy != nullptr && shrink_xy->get_at(0) > 0.)
        s.shrink.x() = s.shrink.y() = float(100. / shrink_xy->get_at(0));
    if (shrink_z != nullptr && shrink_z->get_at(0) > 0.)
        s.shrink.z() = float(100. / shrink_z->get_at(0));

    s.layer_height = config.opt_float("layer_height");
    for (const Vec2d &p : config.option<ConfigOptionPoints>("printable_area")->values)
        s.bed.merge(p);
    s.max_print_height = config.opt_float("printable_height");
    s.job_name         = job_name;
    s.build_time_s     = build_time_s;
    return s;
}

CmbJob cmb_job_from_toolpaths(const GCodeProcessorResult &toolpaths, const CmbExportSettings &settings)
{
    const CmbMachine *machine = nullptr;
    for (const CmbMachine &m : CmbMachines)
        if (settings.machine == m.id)
            machine = &m;
    if (machine == nullptr)
        refuse(format(_u8L("Orca does not know the machine \"%1%\"."), settings.machine));

    const double slice = settings.layer_height / MmPerInch;
    const double lift  = LiftHeight + slice;

    // The toolpaths grouped into layers and paths, in inches. A path runs while the same head extrudes in one mode
    // without leaving the point it reached.
    struct Path
    {
        uint8_t            mode;
        int                head;
        bool               tower;
        std::vector<Vec2d> points;
        std::vector<float> areas;
    };
    struct Layer
    {
        double            z;
        std::vector<Path> paths;
    };
    std::vector<Layer> layers;
    Vec3d              prev       = Vec3d::Zero();
    bool               continuous = false;
    for (const GCodeProcessorResult::MoveVertex &move : toolpaths.moves) {
        const Vec3d pos = move.position.cast<double>();
        // Stratasys beads are counted as rectangles, width times layer thickness.
        const float area = float(double(move.width) * move.height / (MmPerInch * MmPerInch));
        // A bead narrower than any GrabCAD Print writes, such as the tapered end of a thin wall, is left out.
        if (move.type == EMoveType::Extrude && area >= MinBeadAspect * slice * slice && (pos - prev).head<2>().norm() > EPSILON) {
            const double z = pos.z() / MmPerInch;
            if (layers.empty() || std::abs(z - layers.back().z) > 0.25 * slice) {
                if (!layers.empty() && z < layers.back().z)
                    refuse(_u8L("the toolpaths return to a lower layer."));
                layers.push_back({ z, {} });
                continuous = false;
            }
            if (move.extruder_id > 1)
                refuse(_u8L("a Stratasys printer has two heads, and the toolpaths use a third."));
            const int     head  = move.extruder_id;
            const uint8_t mode  = cmb_mode(*machine, head, move.extrusion_role, layers.size() == 1);
            std::vector<Path> &paths = layers.back().paths;
            if (!continuous || paths.empty() || paths.back().mode != mode || paths.back().head != head)
                paths.push_back({ mode, head, move.extrusion_role == erWipeTower, { prev.head<2>() / MmPerInch }, {} });
            paths.back().points.emplace_back(pos.head<2>() / MmPerInch);
            paths.back().areas.emplace_back(area);
            continuous = true;
        } else if ((pos - prev).head<2>().norm() > EPSILON)
            continuous = false;
        prev = pos;
    }
    if (layers.empty())
        refuse(_u8L("there is nothing to print."));

    // The checks: the job must look like the jobs GrabCAD Print writes for the machine.
    if (std::abs(layers.front().z - FirstLayerZ) > 0.0015)
        refuse(format(_u8L("its first layer must be %1% mm high, as Stratasys jobs start, and it is %2% mm."),
                      FirstLayerZ * MmPerInch, layers.front().z * MmPerInch));
    for (size_t i = 1; i < layers.size(); ++i)
        if (std::abs(layers[i].z - layers[i - 1].z - slice) > 0.1 * slice)
            refuse(format(_u8L("its layers must be %1% mm apart, and there are %2% mm between layers at %3% mm."),
                          settings.layer_height, (layers[i].z - layers[i - 1].z) * MmPerInch, layers[i - 1].z * MmPerInch));
    if ((layers.back().z + lift) * MmPerInch > settings.max_print_height)
        refuse(_u8L("it is taller than the printer's build volume."));

    CmbJob     job;
    CmbHeader &h = job.header;
    h.format           = machine->format;
    h.slice_height     = float(slice);
    h.part_material    = settings.model_material;
    h.support_material = settings.support_material;
    h.comment          = "Written by OrcaSlicer!!!!!";
    h.version          = "26.10 (7632)";
    h.machine          = machine->id;
    h.part_tip         = settings.model_tip;
    h.support_tip      = settings.support_tip;
    h.build_time_s     = settings.build_time_s;

    // Bead area, width and aspect limits as [part min, part max, support min, support max] per quantity.
    std::array<float, 2> area_min { FLT_MAX, FLT_MAX }, area_max { 0.f, 0.f };
    BoundingBoxf         footprint, tower_footprint, part_footprint;
    bool                 tower = false;
    const BoundingBoxf   bed(settings.bed.min / MmPerInch, settings.bed.max / MmPerInch);
    for (Layer &layer : layers) {
        CmbLayer out;
        out.z_top     = float(layer.z + lift);
        out.thickness = float(slice);
        BoundingBoxf extent;
        Vec2d        at       = Vec2d::Zero();
        uint8_t      mode     = 0;
        bool         first    = true;
        auto         add_move = [&out, &mode](CmbMove::Kind kind, uint8_t m, const Vec2d &xy, float z, float area) {
            CmbMove move { kind, m, m != mode, Vec3f(float(xy.x()), float(xy.y()), z), area };
            mode = m;
            out.moves.push_back(move);
        };
        const float zp = float(layer.z), zl = float(layer.z + lift);
        for (const Path &path : layer.paths) {
            for (size_t i = 0; i < path.points.size(); ++i) {
                const Vec2d &p = path.points[i];
                if (!bed.contains(p))
                    refuse(format(_u8L("a toolpath at (%1%, %2%) mm is off the bed."), p.x() * MmPerInch, p.y() * MmPerInch));
                extent.merge(p);
                footprint.merge(p);
                (path.tower ? tower_footprint : part_footprint).merge(p);
            }
            tower |= path.tower;
            for (float a : path.areas) {
                const double aspect = a / slice / slice;
                if (aspect > MaxBeadAspect)
                    refuse(format(_u8L("a bead is %1% times as wide as the layer is thick, and Stratasys jobs keep it under %2%."), aspect, MaxBeadAspect));
                area_min[path.head] = std::min(area_min[path.head], a);
                area_max[path.head] = std::max(area_max[path.head], a);
            }

            // Approach: rapid in over the first path, travel at the lift height between the others, then plunge.
            const Vec2d &start = path.points.front();
            if (first)
                add_move(CmbMove::Kind::Rapid, path.mode, start, zl, 0.f);
            else {
                add_move(CmbMove::Kind::ZMove, mode, at, zl, 0.f);
                add_move(CmbMove::Kind::Line, path.mode, start, zl, 0.f);
            }
            first = false;
            add_move(CmbMove::Kind::ZMove, path.mode, start, zp, 0.f);

            // The bead starts over the first segment and ramps down over the end of the last one, as in GrabCAD
            // Print's jobs; a change of bead area mid-path is a SetArea move as well.
            const size_t n     = path.areas.size();
            const Vec2d  end   = path.points.back();
            const Vec2d  last  = path.points[n - 1];
            const double ramp  = std::min(0.5 * (end - last).norm(), double(path.areas.back()) / slice);
            const Vec2d  taper = end - (end - last).normalized() * ramp;
            float        area  = 0.f;
            for (size_t i = 1; i <= n; ++i) {
                const Vec2d p = i == n ? taper : path.points[i];
                const float a = path.areas[i - 1];
                add_move(a != area ? CmbMove::Kind::SetArea : CmbMove::Kind::Line, path.mode, p, zp, a);
                area = a;
            }
            add_move(CmbMove::Kind::SetArea, path.mode, end, zp, 0.f);
            at = end;
        }
        add_move(CmbMove::Kind::ZMove, mode, at, zl, 0.f);
        // GrabCAD Print sets the mode at the start of every layer.
        out.moves.front().sets_mode = true;
        out.min    = Vec3f(float(extent.min.x()), float(extent.min.y()), zp);
        out.max_xy = extent.max.cast<float>();
        job.layers.emplace_back(std::move(out));
    }

    // Volumes, from the moves as written.
    std::array<double, 2> volume { 0., 0. };
    for (const CmbLayer &layer : job.layers) {
        Vec2f from = Vec2f::Zero();
        float area = 0.f;
        for (const CmbMove &move : layer.moves) {
            const Vec2f to = move.position.head<2>();
            const float a  = move.kind == CmbMove::Kind::SetArea ? std::max(area, move.area) :
                             move.kind == CmbMove::Kind::Line    ? area : 0.f;
            volume[cmb_mode_is_support(move.mode) ? 1 : 0] += double(a) * (to - from).norm();
            if (move.kind == CmbMove::Kind::SetArea)
                area = move.area;
            from = to;
        }
    }
    h.part_volume    = float(volume[0]);
    h.support_volume = float(volume[1]);
    h.bounding_box   = BoundingBoxf3(Vec3d(footprint.min.x(), footprint.min.y(), layers.front().z),
                                     Vec3d(footprint.max.x(), footprint.max.y(), layers.back().z + lift));
    for (int head = 0; head < 2; ++head) {
        if (area_max[head] == 0.f)
            area_min[head] = 0.f;
        h.limits[2 * head]         = area_min[head];
        h.limits[2 * head + 1]     = area_max[head];
        h.limits[4 + 2 * head]     = float(area_min[head] / slice);
        h.limits[4 + 2 * head + 1] = float(area_max[head] / slice);
        h.limits[8 + 2 * head]     = float(area_min[head] / slice / slice);
        h.limits[8 + 2 * head + 1] = float(area_max[head] / slice / slice);
    }
    if (tower)
        h.packs.emplace_back("purgeTower");
    h.packs.emplace_back(settings.job_name);

    // The header tail and the sections after the layer table, laid out as in GrabCAD Print's jobs.
    Writer tail;
    tail.str("none");
    tail.str(random_uuid());
    tail.str("normal");
    for (uint8_t b : tower ? std::array<uint8_t, 7> { 2, 0, 0, 0, 2, 0, 0 } : std::array<uint8_t, 7> {})
        tail.byte(b);
    Writer sections;
    auto   rectangle = [&footprint](Writer &out, const BoundingBoxf &box) {
        out.byte(0x11);
        out.i32(4);
        const Vec2d lo = box.min - footprint.min, hi = box.max - footprint.min;
        for (const Vec2d &p : { lo, Vec2d(hi.x(), lo.y()), hi, Vec2d(lo.x(), hi.y()) }) {
            out.byte(0x12);
            out.f32(float(p.x()));
            out.f32(float(p.y()));
        }
    };
    sections.byte(0x11);
    sections.i32(tower ? 2 : 1);
    if (tower)
        rectangle(sections, tower_footprint);
    rectangle(sections, part_footprint);
    const int32_t thumbnail = int32_t(sections.size());
    sections.byte(0x15);
    sections.i32(1);
    for (uint8_t b : { 1, 0, 1, 0 })
        sections.byte(b);
    sections.i32(600);
    sections.i32(400);
    sections.str(settings.job_name);
    const std::string png = blank_png(600, 400);
    sections.str(png);

    job.opaque.address_fields = { tail.size(), tail.size() + 4 };
    tail.i32(0);
    tail.i32(thumbnail);
    tail.str("not specified");
    for (float f : { settings.shrink.x(), settings.shrink.y(), settings.shrink.z(), 0.f, 0.f, machine->header_constant })
        tail.f32(f);
    tail.str(settings.model_tip);
    if (machine->support_tip_with_material) {
        tail.str(settings.support_tip + settings.support_material);
        for (int i = 0; i < 3; ++i)
            tail.str({});
    } else
        tail.str(settings.support_tip);
    tail.byte(0x10);
    tail.i32(6);
    // A key/value block of unknown meaning, the same in every job seen.
    const std::pair<int32_t, int32_t> constants[] = { { 17, 4 }, { 18, 8 }, { 19, 16 }, { 20, 4 }, { 21, -1 }, { 22, 8 } };
    for (const auto &[key, value] : constants) {
        tail.i32(key);
        tail.i32(value);
    }
    tail.byte(0x11);
    tail.i32(1);
    rectangle(tail, footprint);

    job.opaque.writable      = true;
    job.opaque.header_tail   = std::move(tail.data);
    job.opaque.trailer       = std::move(sections.data);
    job.opaque.trailer_start = 0;
    h.layer_count            = int(job.layers.size());
    return job;
}

} // namespace Slic3r
