#ifndef FAST_LIVO_RUNTIME_OUTPUT_H
#define FAST_LIVO_RUNTIME_OUTPUT_H

#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace fast_livo_runtime
{

inline std::filesystem::path default_output_directory()
{
  const char * state_home = std::getenv("XDG_STATE_HOME");
  if (state_home != nullptr && state_home[0] != '\0')
  {
    return std::filesystem::path(state_home) / "fast_livo";
  }
  const char * user_home = std::getenv("HOME");
  if (user_home == nullptr || user_home[0] == '\0')
  {
    throw std::runtime_error(
      "output_dir is empty and neither XDG_STATE_HOME nor HOME is available");
  }
  return std::filesystem::path(user_home) / ".local" / "state" / "fast_livo";
}

inline std::filesystem::path & output_directory_storage()
{
  static std::filesystem::path output_directory;
  return output_directory;
}

inline void set_output_directory(const std::string & configured)
{
  std::filesystem::path selected = configured.empty() ?
    default_output_directory() : std::filesystem::path(configured);
  if (!selected.is_absolute())
  {
    throw std::runtime_error("output_dir must be an absolute path: " + selected.string());
  }
  selected = selected.lexically_normal();
  std::error_code error;
  std::filesystem::create_directories(selected, error);
  if (error || !std::filesystem::is_directory(selected))
  {
    throw std::runtime_error(
      "cannot create output_dir " + selected.string() + ": " + error.message());
  }
  output_directory_storage() = selected;
}

inline const std::filesystem::path & output_directory()
{
  if (output_directory_storage().empty())
  {
    set_output_directory("");
  }
  return output_directory_storage();
}

inline std::filesystem::path path(const std::filesystem::path & relative)
{
  if (relative.is_absolute())
  {
    throw std::runtime_error("runtime output path must be relative");
  }
  return output_directory() / relative;
}

}  // namespace fast_livo_runtime

#endif
