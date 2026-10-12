/**
 * @file scenario_loader.hpp
 * @author Seif ElShafey
 * @brief Data-file loader and corpus discovery for the shared planning-scenario corpus.
 * @version 0.1
 * @date 2026-06-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <string>
#include <vector>

#include "core/scenario/scenario.hpp"

/**
 * @brief Parse a single scenario YAML file into a fully resolved ScenarioSpec.
 *
 * The file has two top-level sections, `input:` (map geometry, occupancy, robot, planner config,
 * and the start/goal query) and `expectations:` (the coarse design-intent oracle). See the package
 * `scenarios/` corpus for annotated examples.
 *
 * Occupancy is resolved from EITHER inline `obstacles`/`unknown` rectangles painted over a base fill
 * (default FREE) OR an `image:` path (grayscale, loaded via OpenCV). When `image` is present it is the
 * sole source of occupancy and the inline rectangles must be omitted.
 *
 * @param path Filesystem path to the scenario YAML file.
 * @return ScenarioSpec Fully resolved scenario: `meta`, `occupancy` (sized `rows*cols`), robot,
 *         planner configs, start/goal, and `expect`.
 *
 * @throws std::invalid_argument if a required field is missing or invalid, if the resolved occupancy
 *         buffer size does not equal `cols*rows`, or if `image` is requested but cannot be read. The
 *         message is formatted `file:field` to point at the offending entry.
 */
ScenarioSpec loadScenario(const std::string & path);

/**
 * @brief List the scenario data files directly inside a directory, sorted lexicographically.
 *
 * Non-recursive: only entries directly in `dir` with a `.yaml` or `.yml` extension are returned. The
 * sorted order makes the resulting list deterministic so callers (e.g. the integration test that
 * runtime-globs the corpus) produce stable, reproducible case ordering across runs.
 *
 * @param dir Directory to scan (typically the package `scenarios/` corpus dir).
 * @return std::vector<std::string> Absolute-or-relative paths (as joined onto `dir`) to each matching
 *         file, sorted ascending. Empty if `dir` does not exist or contains no matching files.
 */
std::vector<std::string> discoverScenarioFiles(const std::string & dir);
