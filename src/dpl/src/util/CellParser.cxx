// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2021-2025, The OpenROAD Authors

#include "CellParser.h"

#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>

namespace dpl {

namespace json = boost::json;

std::vector<PinMapping> CellParser::parsePinMap(const json::array& arr)
{
  std::vector<PinMapping> pinMappings;
  for (const auto& mapping : arr) {
    const auto& mapArr = mapping.as_array();
    PinMapping pm;
    pm.from = json::value_to<std::string>(mapArr[0]);
    pm.to = json::value_to<std::string>(mapArr[1]);
    pinMappings.push_back(pm);
  }
  return pinMappings;
}

std::optional<MultibitConfig> CellParser::parseMultibitMap(
    const json::value& value)
{
  if (value.is_null()) {
    return std::nullopt;
  }

  const auto& obj = value.as_object();
  MultibitConfig config;
  config.LSb = json::value_to<int>(obj.at("LSb"));
  config.MSb = json::value_to<int>(obj.at("MSb"));
  config.pinMap = parsePinMap(obj.at("PinMap").as_array());
  return config;
}

CellConfig CellParser::parseCellConfig(const json::object& obj)
{
  CellConfig config;
  config.function = json::value_to<std::string>(obj.at("Function"));

  // Parse PinSwaps
  if(obj.contains("PinSwaps")) {
    const auto& pinSwaps = obj.at("PinSwaps");
    if (!pinSwaps.is_null()) {
      std::vector<std::vector<std::pair<std::string, std::string>>> swaps;
      for (const auto& swapGroup : pinSwaps.as_array()) {
        std::vector<std::pair<std::string, std::string>> group;
        for (const auto& swap : swapGroup.as_array()) {
          const auto& swapArr = swap.as_array();
          group.emplace_back(json::value_to<std::string>(swapArr[0]),
                             json::value_to<std::string>(swapArr[1]));
        }
        swaps.push_back(std::move(group));
      }
      config.pinSwaps = std::move(swaps);
    }
  } else {
    config.pinSwaps = std::nullopt;
  }

  // Parse PinPermutes
  if(obj.contains("PinPermutes")) {
    const auto& pinPermutes = obj.at("PinPermutes");
    if (!pinPermutes.is_null()) {
      std::vector<std::vector<std::string>> permutes;
      for (const auto& permute : pinPermutes.as_array()) {
        std::vector<std::string> group;
        for (const auto& pin : permute.as_array()) {
          group.push_back(json::value_to<std::string>(pin));
        }
        permutes.push_back(std::move(group));
      }
      config.pinPermutes = std::move(permutes);
    }
  } else {
    config.pinPermutes = std::nullopt;
  }

  // Parse MultibitMap
  if(obj.contains("MultibitMap")) {
    config.multibitMap = parseMultibitMap(obj.at("MultibitMap"));
  } else {
    config.multibitMap = std::nullopt;
  }

  return config;
}

std::unordered_map<std::string, CellConfig> CellParser::parse(
    const std::string& filename)
{
  std::unordered_map<std::string, CellConfig> cells;

  try {
    // Read the entire file into a string
    std::ifstream file(filename);
    if (!file.is_open()) {
      throw std::runtime_error("Could not open file: " + filename);
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string jsonStr = buffer.str();

    // Parse JSON
    std::error_code ec;
    json::value jv = json::parse(jsonStr, ec);
    if (ec) {
      throw std::runtime_error("Failed to parse JSON: " + ec.message());
    }

    const auto& obj = jv.as_object();
    for (const auto& [name, value] : obj) {
      cells[name.data()] = parseCellConfig(value.as_object());
    }
  } catch (const std::exception& e) {
    std::cerr << "Error parsing cells file: " << e.what() << std::endl;
    throw;
  }

  return cells;
}
}  // namespace dpl