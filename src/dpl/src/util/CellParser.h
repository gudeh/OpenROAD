// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2021-2025, The OpenROAD Authors

#pragma once

#include <boost/json.hpp>
#include <optional>
#include <string>
#include <vector>

namespace dpl {

// Structure to represent pin mapping for multibit cells
struct PinMapping
{
  std::string from;
  std::string to;
};

// Structure to represent multibit configuration
struct MultibitConfig
{
  int LSb;
  int MSb;
  std::vector<PinMapping> pinMap;
};

// Structure to represent a cell configuration
struct CellConfig
{
  std::string function;
  std::optional<std::vector<std::vector<std::pair<std::string, std::string>>>>
      pinSwaps;
  std::optional<std::vector<std::vector<std::string>>> pinPermutes;
  std::optional<MultibitConfig> multibitMap;
};

class CellParser
{
 public:
  CellParser() = default;
  ~CellParser() = default;

  // Parse the JSON file and return a map of cell names to their configurations
  std::unordered_map<std::string, CellConfig> parse(
      const std::string& filename);

 private:
  // Helper functions to parse specific parts of the JSON
  CellConfig parseCellConfig(const boost::json::object& obj);
  std::optional<MultibitConfig> parseMultibitMap(
      const boost::json::value& value);
  std::vector<PinMapping> parsePinMap(const boost::json::array& arr);
};

}  // namespace dpl