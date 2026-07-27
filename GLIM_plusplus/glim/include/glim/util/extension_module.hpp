#pragma once

#include <cstdint>
#include <memory>
#include <optional>

namespace glim {

/**
 * @brief Extension module to be dynamically loaded via dynamic linking
 */
class ExtensionModule {
public:
  ExtensionModule() {}
  virtual ~ExtensionModule() {}

  /**
   * @brief Check if the module is behind the main mapping process.
   */
  virtual bool needs_wait() const { return false; }

  /**
   * @brief Check if the module is alive. (If it returns false, the system will be shutdown)
   */
  virtual bool ok() const { return true; }

  /**
   * @brief Number of global constraints this module has CONFIRMED into the
   *        optimizer's factor graph.
   *
   * Returns std::nullopt when the module does not contribute global
   * constraints (the default), which lets a caller distinguish "no such
   * module is loaded" from "the module is loaded and delivered nothing".
   * A module that answers must count only constraints it has verified
   * actually reached the graph -- NOT messages it published or factors it
   * queued, since neither proves the optimizer accepted them.
   */
  virtual std::optional<std::uint64_t> delivered_global_constraints() const { return std::nullopt; }

  /**
   * @brief Called when the system is quitting.
   */
  virtual void at_exit(const std::string& dump_path) {}

  /**
   * @brief Load an extension module from a dynamic library
   * @param so_name  Dynamic library name
   * @return         Loaded extension module
   */
  static std::shared_ptr<ExtensionModule> load_module(const std::string& so_name);

  /**
   * @brief Export classes (factors) from a dynamic library
   * @param so_name  Dynamic library name
   */
  static void export_classes(const std::string& so_name);
};

}  // namespace glim