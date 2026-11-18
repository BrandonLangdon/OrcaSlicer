# Stratasys CMB jobs

OrcaSlicer reads and writes the toolpath jobs that Stratasys FDM printers load: `.cmb` files, gzipped as
`.cmb.gz` for the F-series. It can open a job in the G-code preview to inspect it, and it can export a sliced plate
as a job for the F170, F370 and F900 (F900 export is not enabled yet). The format is not published. Everything here comes
from GrabCAD Print's jobs, from the printers' own logs, and from
[a public write-up of the older CatalystEX format](https://azttm.wordpress.com/2012/09/22/inside-the-stratasys-dimension-catalyst-cmb-file-format/).

| Where | What |
| --- | --- |
| `src/libslic3r/Format/CMB.{hpp,cpp}` | Reader, writer, conversion from sliced toolpaths, conversion to preview G-code |
| `output_file_format` (printer setting) | `gcode`, `cmb` (F900) or `cmb.gz` (F-series); picks the exporter and the file extension |
| `src/slic3r/GUI/BackgroundSlicingProcess.cpp` | Writes the job instead of G-code when the printer's format is CMB |
| `src/slic3r/GUI/Plater.cpp` (`load_gcode`) | Opens a job by converting it to tagged G-code for the preview |
| `resources/profiles/Stratasys/` | Printer, process and material profiles |
| `tests/libslic3r/test_cmb.cpp`, `tests/fff_print/test_cmb.cpp` | Format and export tests |

## What the job decides and what the printer decides

A CMB job is geometry. The printer does not run it directly: it compiles every job on board into its own motion
code, and everything about *how* the material goes down comes from parameter sets stored in the printer.

| In the job | Decided by the printer |
| --- | --- |
| Toolpaths: every bead as a path with a cross-section area | Head and chamber temperatures |
| Deposition mode of each path (model wall, sparse fill, support base, purge…) | Speeds and accelerations for each mode |
| Layer heights, the first layer at 0.014 in, lift moves between paths | Extrusion rate (filament counts per cubic inch) |
| Model and support material names, tip names, slice height | Which parameter set to use: chosen by material, tip and slice height |
| Material shrink factors, already applied to the toolpaths | Calibration and part placement offsets on the tray |
| Part and support volumes, build time, bounding box (informational) | The build time and material use it shows and logs |

On the F-series printers the job goes through these steps:

1. A job picked from USB (or sent over the network) is copied into the printer's queue.
2. The printer's move compiler reads the header: the slice height, the materials and tips. It selects the
   parameter set `{model material, tip}`, `{support material, tip}`, `sliceN`, where `N` is the slice height in
   thousandths of an inch (`slice5`, `slice7`, `slice10`, `slice13`).
3. It compiles the paths with that parameter set and the machine's calibration into a motion file, computes the
   build time and material use, and builds from it.

So:

- **A job must use a slice height the printer has a parameter set for**, with materials and tips it knows.
  Orca's profiles only offer those slice heights.
- **Orca's speed, acceleration and temperature settings have no effect** on a Stratasys print, and the print
  time Orca estimates is not the printer's.
- **Shrink compensation is in the job.** The printer does not scale the toolpaths, so the exporter must scale them
  as GrabCAD Print does (see below).
- **The printer accepts plain and gzipped jobs.** The F-series takes `.cmb.gz` and `.cmb`; Fortus printers take
  `.cmb`. Orca writes the form GrabCAD Print writes for each printer.

## File layout

Little-endian binary, coordinates in inches. Strings are a 32-bit length followed by the bytes.

**Header:** magic `3 3 `, four format bytes (`0E 09 06 01` for the F-series, `0E 09 09 01` for the F900), slice
height, part and support volumes (cubic inches), model and support material names, bounding box (six floats),
twelve bead limits (minimum and maximum area, width and width-over-slice ratio, for each head), layer count, layer
table address, comment, pack names (the parts on the tray, and `purgeTower` when there is one), producer version,
machine name (`dorado1` F170, `dorado3` F370, `f900`), model and support tip, build time in seconds.

**GrabCAD Print header tail:**

- the signature `none`, a job UUID and a quality name (`normal`, `none`, `allModel`);
- seven bytes: `02 00 00 00 02 00 00` with a purge tower, zeros without one, and the second byte `01` for
  adaptive slicing;
- two addresses into the trailer, and a colour name;
- six floats: the model material's X, Y and Z shrink factors, two zeros, and a value that depends on tip and
  slice height (see below);
- model tip, support tip;
- on the F900, the support material and three empty strings;
- a key/value block (`0x10`) and the tray footprint (`0x11`).

**Layers:**

| Opcode | Meaning |
| --- | --- |
| `05` | Layer start: minimum X, Y, Z |
| `06` | Layer extent: maximum X, Y, the lift height, and the layer thickness (GrabCAD Print only) |
| `13` | 16-byte record of unknown meaning, present on every layer of F900 jobs and of F-series jobs without a purge tower |
| `0D` | Deposition mode (one byte and a pad byte) |
| `0B 00` | Rapid move: X, Y, Z and four zero bytes |
| `0B 01` | Line at the current bead area: X, Y |
| `0B 02` | Vertical move: X, Y, Z |
| `0B 03` | Line that changes the bead area: X, Y, area in square inches |
| `07` | End of layer |
| `03` | End of job |

A path is: a rapid move at the lift height, a vertical move down, an area-setting move that starts the bead, lines, an
area-setting move to zero that ends it, and a vertical move back up. Between paths in a layer the head travels with
lines at the lift height and zero area.

**Trailer:** the layer table (the address of every layer, the first included), then the pack outlines (`0x11`/`0x12`
polygons) and a thumbnail (`0x15`: a 600 × 400 PNG and the job name).

**Deposition modes**, model head: `64` purge, `65` inner wall (F-series), `66` outer wall, `67` sparse fill, `68`
solid fill, `6B` first model layer, `6C` model base layer (jobs without support), `6D` inner wall (F900), `71` and
`72` small fills.

Support head: `C8` support, `C9` first support base layer, `CA` interface, `CB` transition, `CC` support purge,
`CD` support (F900).

## Writing a job

`cmb_job_from_toolpaths` converts the sliced toolpaths of a two-head plate (head 1 model, head 2 support) into a job laid
out as GrabCAD Print lays out its own. `cmb_export_settings` reads what it needs from the configuration:

- **Printer:** the printer notes name the machine and the tips, as `CMB_MACHINE=`, `CMB_MODEL_TIP=` and
  `CMB_SUPPORT_TIP=`.
- **Materials:** each material's notes name the material the printer knows it by, as `CMB_MATERIAL=`.
- **Shrink:** the model material's `filament_shrink` and `filament_shrinkage_compensation_z` give the shrink
  factors (shrink factor = 100 / percentage).

The layout the exporter writes:

- **First layer and layer pitch:** the first layer is at 0.014 in whatever the slice height. Each layer after it is
  one slice height higher.
- **Support base on the F-series:** two layers of support material, `C9` then `C8` with `CC`, before the part.
  Orca's raft provides them. For CMB printers, raft layers take the layer height instead of Orca's usual raft
  heights (`Slicing.cpp`).
- **Shrinkage:** GrabCAD Print scales the whole job by the model material's factors. For CMB printers Orca does
  the same, even when the support material has no shrink (`PrintApply.cpp`).
- **Bead area:** each bead's cross-section is its width times the layer thickness. A path ends by ramping down over
  its last segment.
- **Lift:** the head lifts 0.05 in plus one slice height between paths. GrabCAD Print writes both this and a lower
  lift (0.026 in plus one slice height) in different jobs.

**The tip and slice height table.** The last of the six floats after the shrink factors, and the quality name,
depend on the tip and the slice height, not on the printer model:

| Tip | Slice height | F-series value | F900 value |
| --- | --- | --- | --- |
| T10 | 0.005 in | | 0.063 |
| T12 | 0.007 in | | 0.063 |
| T14 | 0.005 in | 0.093 | |
| T14 | 0.007 in | 0.105 | |
| T14 | 0.010 in | 0.118 | |
| T14 | 0.013 in | 0.119 | |
| T16 | 0.010 in | 0.078 | 0.068 |
| T20 | 0.013 in | | 0.067 |

`CmbSliceStyles` lists the combinations the exporter writes, and it refuses any other. The F370 rows come from its
jobs. The F170's 0.007 and 0.013 in rows reuse the F370's, because the two printers' jobs agree wherever both have
been seen.

**Refusals.** The exporter only writes jobs that look like the ones GrabCAD Print writes for the machine. It
refuses, with a message naming the reason:

- an unknown machine, or a tip and slice height not in the table;
- a first layer other than 0.014 in, or a layer pitch other than the slice height;
- a third head;
- a move off the bed or above the build height;
- a bead wider than 0.084 in. That is GrabCAD Print's widest, a first-layer bead at 0.005 in. A bead's width is
  its area over the slice height, so the thick first layer counts as wide at thin slices.

Beads narrower than half the slice height, such as the pointed ends of thin walls, are left out: GrabCAD Print
never writes them.

**Things the exporter does not produce yet:**
- **Thin slices:** the F370's 0.005 in layout lays the part's first layer straight on the sheet with no support
  base, as the F900 does at every slice height.
- **Adaptive slicing:** GrabCAD Print's adaptive jobs mix layers of two slice heights.
- **Thumbnail:** the exporter writes a blank one.

## Reading a job

`parse_cmb` reads a job into a header and layers of moves. It also keeps every byte it does not interpret: the
header tail, the 0x13 records and the trailer. So a GrabCAD Print job writes back byte for byte, which the tests
check against every sample job available. `cmb_to_gcode` writes the job as G-code tagged with extrusion roles,
widths and layer heights, which is how the preview shows it. Its feedrates are nominal, since the printer sets
the speeds.

## Unknowns

- **The header float that varies with tip and slice height:** its meaning is unknown.
- **The `0x13` per-layer record:** its meaning is unknown.
- **Mode mappings:** `6C`, `6F`, `71`, `72`, `CB` and `CE` are mapped from where they appear in GrabCAD Print's jobs.
  Their exact meaning is unconfirmed.
- **Lift height:** what decides between GrabCAD Print's two lift heights is unknown.

## Sample jobs

The reverse-engineering samples (GrabCAD Print jobs for the F170, F370 and F900) are kept locally in
`tests/fortus_TestFiles/`. That folder is ignored by git because some samples are customer parts. The round-trip
test runs every sample it finds there, and skips itself when the folder is absent.
