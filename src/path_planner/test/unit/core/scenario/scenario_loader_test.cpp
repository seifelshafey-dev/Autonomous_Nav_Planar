#include <gtest/gtest.h>

#include <unistd.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "core/scenario/scenario.hpp"
#include "core/scenario/scenario_loader.hpp"

/*
Trinary image-occupancy decode tests for loadScenario (occupancyFromImage, reached through the public
loadScenario API). The decode maps each grayscale pixel to a darkness fraction p = (255 - px) / 255 and
buckets it against two thresholds (defaults occupied_thresh=0.65, free_thresh=0.196):
  p > occupied_thresh  -> OCCUPIED (100)
  p < free_thresh      -> FREE (0)
  otherwise            -> UNKNOWN (-1)
The image is read top-row-first; the grid is row-major with row 0 at the BOTTOM, so the loader flips
vertically (img_r = rows - 1 - r). These tests generate a tiny grayscale PNG at test time (cv::imwrite
into TempDir) plus a minimal scenario YAML that points at it, then assert the resolved spec.occupancy.
No binary fixture is committed to the corpus.
*/

namespace {

// Write `content` to a uniquely-named file under the gtest temp dir and return its absolute path.
std::string writeTemp(const std::string & name, const std::string & content) {
    const std::string path = std::string(testing::TempDir()) + name;
    std::ofstream out(path);
    out << content;
    out.close();
    return path;
}

// Write a single-column grayscale PNG whose rows (image top -> bottom) take `pixels` 0-255 values.
// Returns the PNG path. rows = pixels.size(), cols = 1.
std::string writeColumnImage(const std::string & name, const std::vector<uint8_t> & pixels) {
    const std::string path = std::string(testing::TempDir()) + name;
    cv::Mat img(static_cast<int>(pixels.size()), 1, CV_8UC1);
    for (int r = 0; r < static_cast<int>(pixels.size()); ++r) {
        img.at<uint8_t>(r, 0) = pixels[static_cast<std::size_t>(r)];
    }
    EXPECT_TRUE(cv::imwrite(path, img)) << "failed to write fixture PNG: " << path;
    return path;
}

// Build a minimal-but-complete scenario YAML whose occupancy comes from `image_path`. cols/rows must
// match the image dimensions (the loader CHECKs this). `image_extra` injects sibling keys under the
// image node (e.g. thresholds); empty for the default-threshold cases.
std::string scenarioYaml(int cols, int rows, const std::string & image_path,
                         const std::string & image_extra = "") {
    return
        "name: image_decode_fixture\n"
        "input:\n"
        "  grid:\n"
        "    cols: " + std::to_string(cols) + "\n"
        "    rows: " + std::to_string(rows) + "\n"
        "    resolution: 0.1\n"
        "    origin: [0.0, 0.0]\n"
        "  image:\n"
        "    path: \"" + image_path + "\"\n"
        + image_extra +
        "  robot:\n"
        "    vertices: [[0.1, 0.1], [-0.1, 0.1], [-0.1, -0.1], [0.1, -0.1]]\n"
        "  diff_drive:\n"
        "    rmin: 0.5\n"
        "    vmax: 1.0\n"
        "  cost_policy:\n"
        "    clearance_radius: 0.5\n"
        "    clearance_decay: 15.0\n"
        "    max_clearance_cost: 30.0\n"
        "    unknown_cost_factor: 1.0\n"
        "  start: [0.05, 0.05, 0.0]\n"
        "  goal:  [0.05, 0.25, 0.0]\n";
}

// As scenarioYaml, but additionally injects an `a_star` block (a sibling of `image` under `input`)
// from `a_star_body`. Empty `a_star_body` omits the block entirely so the loader applies struct
// defaults. Used to exercise the a_star override path (defaults + per-field CHECK) independently of
// the image decode above.
std::string scenarioYamlWithAStar(int cols, int rows, const std::string & image_path,
                                  const std::string & a_star_body) {
    const std::string base = scenarioYaml(cols, rows, image_path);
    if (a_star_body.empty()) { return base; }
    // Insert the a_star block right after the closing of the image node, before `robot`. The image
    // node ends just before "  robot:"; splice the a_star block there so it nests under `input`.
    const std::string anchor = "  robot:\n";
    const std::size_t pos = base.find(anchor);
    return base.substr(0, pos) + "  a_star:\n" + a_star_body + base.substr(pos);
}

// Decoded occupancy values (not the global unscoped CellState enum, which also names FREE/OCCUPIED/
// UNKNOWN). spec.occupancy is the int8_t buffer the loader produces: 0=FREE, 100=OCCUPIED, -1=UNKNOWN.
constexpr int8_t OCC = 100;
constexpr int8_t FRE = 0;
constexpr int8_t UNK = -1;

// Build a uniquely-prefixed bundle root under the gtest temp dir, with `maps/` and `bundle/`
// subdirectories, and return the bundle root path (no trailing separator). Each call uses a fresh
// prefix so reruns don't collide. The two subdirs are SIBLINGS: a scenario in `bundle/` referring to
// `../maps/m.png` only resolves correctly if the loader anchors the relative path at the YAML's own
// parent directory.
std::filesystem::path makeBundle(const std::string & prefix) {
    const std::filesystem::path root =
        std::filesystem::path(testing::TempDir()) / (prefix + "_bundle");
    std::filesystem::remove_all(root);  // clean any stale artifact from a previous run
    std::filesystem::create_directories(root / "maps");
    std::filesystem::create_directories(root / "bundle");
    return root;
}

// Write a single-column grayscale PNG (rows top->bottom = `pixels`) at an absolute path.
void writeColumnImageAt(const std::filesystem::path & path, const std::vector<uint8_t> & pixels) {
    cv::Mat img(static_cast<int>(pixels.size()), 1, CV_8UC1);
    for (int r = 0; r < static_cast<int>(pixels.size()); ++r) {
        img.at<uint8_t>(r, 0) = pixels[static_cast<std::size_t>(r)];
    }
    ASSERT_TRUE(cv::imwrite(path.string(), img)) << "failed to write fixture PNG: " << path;
}

// Write `content` to an absolute path.
void writeFileAt(const std::filesystem::path & path, const std::string & content) {
    std::ofstream out(path);
    out << content;
    out.close();
}

// RAII chdir guard: switches the process CWD on construction and restores it on destruction so the
// CWD-independence assertion below cannot leak into sibling tests. The relative-path resolution under
// test must NOT depend on this CWD.
class CwdGuard {
public:
    explicit CwdGuard(const std::filesystem::path & to) : prev_(std::filesystem::current_path()) {
        std::filesystem::current_path(to);
    }
    ~CwdGuard() { std::filesystem::current_path(prev_); }
    CwdGuard(const CwdGuard &) = delete;
    CwdGuard & operator=(const CwdGuard &) = delete;

private:
    std::filesystem::path prev_;
};

}  // namespace

// Pins the trinary bucketing at the default thresholds: a near-black pixel (px=0 -> p=1.0) decodes
// OCCUPIED, a mid-gray pixel (px=128 -> p~=0.498, between 0.196 and 0.65) decodes UNKNOWN, and a
// white pixel (px=255 -> p=0.0) decodes FREE.
TEST(ScenarioLoaderImageDecode, TrinaryBucketsAtDefaultThresholds) {
    // Image rows top->bottom: [white(FREE), gray(UNKNOWN), black(OCCUPIED)].
    const std::string png = writeColumnImage("img_trinary.png", {255, 128, 0});
    const std::string yaml = writeTemp("scn_trinary.yaml", scenarioYaml(1, 3, png));

    ScenarioSpec spec = loadScenario(yaml);

    ASSERT_EQ(spec.occupancy.size(), 3u);
    // Grid is bottom-origin; the loader flips. Image bottom row (black) -> grid row 0; image top row
    // (white) -> grid row 2. So buffer order (row 0..2) is [OCCUPIED, UNKNOWN, FREE].
    EXPECT_EQ(spec.occupancy[0], OCC) << "image-bottom black pixel must land at grid row 0";
    EXPECT_EQ(spec.occupancy[1], UNK) << "mid-gray pixel must decode to UNKNOWN (-1)";
    EXPECT_EQ(spec.occupancy[2], FRE) << "image-top white pixel must land at grid row 2 (top)";
}

// Vertical-flip correctness guard: a vertically-asymmetric image (dark at image-TOP, light at
// image-BOTTOM) must place the image-top pixel at the HIGHEST grid row index, not row 0.
TEST(ScenarioLoaderImageDecode, FlipsImageTopToHighestGridRow) {
    // Image rows top->bottom: [black, white]. cols=1, rows=2.
    const std::string png = writeColumnImage("img_flip.png", {0, 255});
    const std::string yaml = writeTemp("scn_flip.yaml", scenarioYaml(1, 2, png));

    ScenarioSpec spec = loadScenario(yaml);
    const int cols = static_cast<int>(spec.meta.cols);
    const int rows = static_cast<int>(spec.meta.rows);
    ASSERT_EQ(cols, 1);
    ASSERT_EQ(rows, 2);
    ASSERT_EQ(spec.occupancy.size(), 2u);

    // Image TOP (black -> OCCUPIED) must be at the highest grid row index (rows-1), NOT row 0.
    EXPECT_EQ(spec.occupancy[static_cast<std::size_t>(rows - 1) * cols + 0], OCC)
        << "image top row must flip to the highest grid row";
    // Image BOTTOM (white -> FREE) must be at grid row 0.
    EXPECT_EQ(spec.occupancy[0], FRE) << "image bottom row must flip to grid row 0";
}

// Rejection path: the loader CHECKs free_thresh < occupied_thresh and throws std::invalid_argument
// when violated (here free_thresh == occupied_thresh).
TEST(ScenarioLoaderImageDecode, ThrowsWhenFreeThreshNotBelowOccupiedThresh) {
    const std::string png = writeColumnImage("img_reject.png", {128});
    const std::string extra = "    occupied_thresh: 0.5\n    free_thresh: 0.5\n";
    const std::string yaml = writeTemp("scn_reject.yaml", scenarioYaml(1, 1, png, extra));

    EXPECT_THROW(loadScenario(yaml), std::invalid_argument);
}

// Pins the relative-image-path resolution seam: a RELATIVE
// `input.image.path` is resolved against the scenario YAML's OWN parent directory, NOT the process
// CWD. The PNG lives in `<tmp>/<p>_bundle/maps/m.png`; the YAML lives in the SIBLING
// `<tmp>/<p>_bundle/bundle/scn.yaml` and points at `../maps/m.png`. We chdir to a neutral, unrelated
// directory first, so a CWD-relative resolution would look for `<neutral>/../maps/m.png` and fail to
// decode -- the test passing therefore proves YAML-dir anchoring, not coincidental CWD agreement.
TEST(ScenarioLoaderImageDecode, ResolvesRelativeImagePathAgainstYamlDir) {
    const std::filesystem::path root = makeBundle("relpath");
    const std::filesystem::path png  = root / "maps" / "m.png";
    const std::filesystem::path yaml = root / "bundle" / "scn.yaml";

    // Image rows top->bottom: [white(FREE), gray(UNKNOWN), black(OCCUPIED)]; same trinary decode as
    // the default-threshold case, so a correctly-resolved image yields a known cell pattern.
    writeColumnImageAt(png, {255, 128, 0});
    writeFileAt(yaml, scenarioYaml(1, 3, "../maps/m.png"));

    // Make CWD-relative resolution provably wrong: cd into a directory with no `../maps/m.png`.
    CwdGuard cwd(testing::TempDir());

    ScenarioSpec spec = loadScenario(yaml.string());

    // Decode succeeded (would have thrown on an unreadable image) and dims + a known trinary cell
    // match -> the loader read the PNG via the YAML's parent dir, not the CWD.
    ASSERT_EQ(spec.meta.cols, 1.0);
    ASSERT_EQ(spec.meta.rows, 3.0);
    ASSERT_EQ(spec.occupancy.size(), 3u);
    EXPECT_EQ(spec.occupancy[0], OCC) << "image-bottom black pixel must land at grid row 0";
    EXPECT_EQ(spec.occupancy[1], UNK) << "mid-gray pixel must decode to UNKNOWN (-1)";
    EXPECT_EQ(spec.occupancy[2], FRE) << "image-top white pixel must land at grid row 2 (top)";
}

// Complements the relative case: an ABSOLUTE `input.image.path` is passed through unchanged (the
// loader only rewrites relative paths). The YAML lives in a different directory than the image, so an
// (erroneous) YAML-dir join would corrupt the absolute path and fail to decode.
TEST(ScenarioLoaderImageDecode, AbsoluteImagePathLoadsUnchanged) {
    const std::filesystem::path root = makeBundle("abspath");
    const std::filesystem::path png  = root / "maps" / "m.png";
    const std::filesystem::path yaml = root / "bundle" / "scn.yaml";

    writeColumnImageAt(png, {0, 255});  // top->bottom: [black, white]
    // Absolute path embedded directly; std::filesystem makes it absolute on this platform.
    writeFileAt(yaml, scenarioYaml(1, 2, std::filesystem::absolute(png).string()));

    CwdGuard cwd(testing::TempDir());

    ScenarioSpec spec = loadScenario(yaml.string());

    ASSERT_EQ(spec.occupancy.size(), 2u);
    // Image top (black -> OCCUPIED) flips to highest grid row; image bottom (white -> FREE) to row 0.
    EXPECT_EQ(spec.occupancy[1], OCC);
    EXPECT_EQ(spec.occupancy[0], FRE);
}

// ---- a_star override: closed_cell_size default + CHECK ---------------------------------------
// The scenario loader's a_star block is optional and mirrors loadAStarConfig: an omitted field
// falls back to the AStarConfig struct default, and a non-positive closed_cell_size is rejected.
// These pin the new dedup-cell key on the scenario path (separate codepath from config_loader).

// With no `a_star` block at all, closed_cell_size resolves to the struct default (0.15 m).
TEST(ScenarioLoaderAStar, ClosedCellSizeDefaultsWhenAStarBlockOmitted) {
    const std::string png  = writeColumnImage("ccz_def_block.png", {255, 0});
    const std::string yaml = writeTemp("scn_ccz_def_block.yaml", scenarioYaml(1, 2, png));
    ScenarioSpec spec = loadScenario(yaml);
    EXPECT_DOUBLE_EQ(spec.a_star.closed_cell_size, 0.15);
}

// With an a_star block present but closed_cell_size omitted, the single field still defaults.
TEST(ScenarioLoaderAStar, ClosedCellSizeDefaultsWhenFieldOmitted) {
    const std::string png  = writeColumnImage("ccz_def_field.png", {255, 0});
    const std::string yaml = writeTemp("scn_ccz_def_field.yaml",
        scenarioYamlWithAStar(1, 2, png, "    max_expansions: 1234\n"));
    ScenarioSpec spec = loadScenario(yaml);
    EXPECT_EQ(spec.a_star.max_expansions, 1234);          // the override took effect...
    EXPECT_DOUBLE_EQ(spec.a_star.closed_cell_size, 0.15); // ...and the omitted field still defaults.
}

// An explicit closed_cell_size overrides the default.
TEST(ScenarioLoaderAStar, ClosedCellSizeOverrideParsed) {
    const std::string png  = writeColumnImage("ccz_ovr.png", {255, 0});
    const std::string yaml = writeTemp("scn_ccz_ovr.yaml",
        scenarioYamlWithAStar(1, 2, png, "    closed_cell_size: 0.25\n"));
    ScenarioSpec spec = loadScenario(yaml);
    EXPECT_DOUBLE_EQ(spec.a_star.closed_cell_size, 0.25);
}

// closed_cell_size <= 0 is rejected by the scenario SCENARIO_CHECK (throws std::invalid_argument).
TEST(ScenarioLoaderAStar, ThrowsOnNonPositiveClosedCellSize) {
    const std::string png0 = writeColumnImage("ccz_zero.png", {255, 0});
    const std::string yz   = writeTemp("scn_ccz_zero.yaml",
        scenarioYamlWithAStar(1, 2, png0, "    closed_cell_size: 0.0\n"));
    EXPECT_THROW(loadScenario(yz), std::invalid_argument);

    const std::string pngn = writeColumnImage("ccz_neg.png", {255, 0});
    const std::string yn   = writeTemp("scn_ccz_neg.yaml",
        scenarioYamlWithAStar(1, 2, pngn, "    closed_cell_size: -0.1\n"));
    EXPECT_THROW(loadScenario(yn), std::invalid_argument);
}

// ---- expectations.unknown_poses_max ---------------------------------------------------------

// A non-negative unknown_poses_max loads into spec.expect; omitted stays unset.
TEST(ScenarioLoaderExpectations, UnknownPosesMaxParsed) {
    const std::string png = writeColumnImage("upm_ok.png", {255, 0});
    const std::string base = scenarioYaml(1, 2, png);
    ScenarioSpec none = loadScenario(writeTemp("scn_upm_none.yaml", base));
    EXPECT_FALSE(none.expect.unknown_poses_max.has_value());
    ScenarioSpec spec = loadScenario(
        writeTemp("scn_upm_ok.yaml", base + "expectations:\n  expect_found: true\n  unknown_poses_max: 3\n"));
    ASSERT_TRUE(spec.expect.unknown_poses_max.has_value());
    EXPECT_EQ(*spec.expect.unknown_poses_max, 3);
}

// A negative unknown_poses_max is rejected.
TEST(ScenarioLoaderExpectations, ThrowsOnNegativeUnknownPosesMax) {
    const std::string png = writeColumnImage("upm_neg.png", {255, 0});
    EXPECT_THROW(loadScenario(writeTemp("scn_upm_neg.yaml", scenarioYaml(1, 2, png) +
                 "expectations:\n  unknown_poses_max: -1\n")), std::invalid_argument);
}

// ---- cost_policy.unknown_cost_factor --------------------------------------------------------

namespace {
std::string replaceFirst(std::string s, const std::string & from, const std::string & to) {
    const auto at = s.find(from);
    s.replace(at, from.size(), to);
    return s;
}
}  // namespace

// A factor below 1 and the former unknown_cost key are both rejected.
TEST(ScenarioLoaderCostPolicy, RejectsFactorBelowOneAndFormerKey) {
    const std::string png = writeColumnImage("ucf.png", {255, 0});
    const std::string base = scenarioYaml(1, 2, png);
    EXPECT_NO_THROW(loadScenario(writeTemp("scn_ucf_ok.yaml", base)));
    EXPECT_THROW(loadScenario(writeTemp("scn_ucf_lt1.yaml",
        replaceFirst(base, "unknown_cost_factor: 1.0", "unknown_cost_factor: 0.9"))), std::invalid_argument);
    EXPECT_THROW(loadScenario(writeTemp("scn_ucf_old.yaml",
        replaceFirst(base, "unknown_cost_factor: 1.0", "unknown_cost: 2.0"))), std::invalid_argument);
}

// ---- input.image node forms ------------------------------------------------------------------

// `image: <file>` (scalar) loads the same occupancy as `image: {path: <file>}`.
TEST(ScenarioLoaderImageForm, ScalarFormEqualsMappingForm) {
    const std::string png = writeColumnImage("img_form.png", {255, 128, 0});
    const std::string map_yaml = scenarioYaml(1, 3, png);
    const std::string scalar_yaml =
        replaceFirst(map_yaml, "  image:\n    path: \"" + png + "\"\n", "  image: \"" + png + "\"\n");
    ASSERT_NE(map_yaml, scalar_yaml);

    const ScenarioSpec a = loadScenario(writeTemp("scn_form_map.yaml", map_yaml));
    const ScenarioSpec b = loadScenario(writeTemp("scn_form_scalar.yaml", scalar_yaml));
    EXPECT_EQ(a.occupancy, b.occupancy);
}

// A mapping with no `path` key is rejected with the field named.
TEST(ScenarioLoaderImageForm, MappingWithoutPathThrowsNamingField) {
    const std::string png = writeColumnImage("img_nopath.png", {128});
    const std::string yaml = replaceFirst(scenarioYaml(1, 1, png), "    path: \"" + png + "\"\n",
                                          "    occupied_thresh: 0.65\n");
    try {
        loadScenario(writeTemp("scn_nopath.yaml", yaml));
        FAIL() << "expected std::invalid_argument";
    } catch (const std::invalid_argument & e) {
        EXPECT_NE(std::string(e.what()).find("input.image.path"), std::string::npos) << e.what();
    }
}

// A sequence is rejected with the field named.
TEST(ScenarioLoaderImageForm, SequenceThrowsNamingField) {
    const std::string png = writeColumnImage("img_seq.png", {128});
    const std::string yaml = replaceFirst(scenarioYaml(1, 1, png),
                                          "  image:\n    path: \"" + png + "\"\n",
                                          "  image: [a, b]\n");
    try {
        loadScenario(writeTemp("scn_seq.yaml", yaml));
        FAIL() << "expected std::invalid_argument";
    } catch (const std::invalid_argument & e) {
        EXPECT_NE(std::string(e.what()).find("input.image.path"), std::string::npos) << e.what();
    }
}
