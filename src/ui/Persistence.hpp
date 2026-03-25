#pragma once

#include "core/Grid.hpp"
#include "ui/ViewSettings.hpp"

#include <string>

namespace pathsim::persistence {

// Capture current state into a string for storage
std::string serialize(const Grid& grid, const ViewSettings& view);

// Restore state from a stored string. Returns false if data is invalid.
bool deserialize(const std::string& data, Grid& grid, ViewSettings& view);

// Platform storage are the following: localStorage on web and .dat file for desktop
// todo: Add tabs? Send/share current grid content with others? Add json support?
void save_to_storage(const std::string& data);
std::string load_from_storage();

} // namespace pathsim::persistence