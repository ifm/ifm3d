/*
 * Copyright (C) 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_RTSP_PLUGIN_MANAGER_H
#define IFM3D_RTSP_PLUGIN_MANAGER_H

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <ifm3d/rtsp/plugin.h>

namespace ifm3d {

/**
 * Discovers, loads, and manages video decoder plugins at runtime.
 *
 * Search path resolution (in order):
 *   1. Directories passed to the constructor.
 *   2. Paths from the IFM3D_PLUGIN_PATH environment variable
 *      (colon-separated on POSIX, semicolon-separated on Windows).
 *
 * Plugin naming convention:
 *   Linux/macOS : libifm3d_plugin_<name>.so
 *   Windows     : ifm3d_plugin_<name>.dll
 *
 * Plugin selection for a codec (deterministic):
 *   - If preferred_plugin_name is non-empty and a loaded plugin whose
 *     filename contains that string supports the codec, it is used.
 *   - Otherwise the first plugin (alphabetically by filename) that
 *     advertises support for the codec is used.
 *
 * Lifetime:
 *   A plugin is never unloaded while a decoder instance from it is alive.
 *   Destroy all decoders before destroying the PluginManager.
 */
class PluginManager
{
public:
  /**
   * @param paths                 Directories to scan for plugins.
   * @param preferred_plugin_name Optional filename substring of the preferred
   *                              plugin (e.g. "ffmpeg").
   */
  explicit PluginManager(const std::vector<std::string>& paths,
                         const std::string& preferred_plugin_name = {});

  ~PluginManager();

  /* Non-copyable, non-movable — owns OS library handles. */
  PluginManager(const PluginManager&) = delete;
  PluginManager& operator=(const PluginManager&) = delete;
  PluginManager(PluginManager&&) = delete;
  PluginManager& operator=(PluginManager&&) = delete;

  /**
   * Scan all configured directories and load matching shared libraries.
   * Load failures are logged and non-fatal.
   * Should be called once after construction.
   */
  void LoadPlugins();

  /**
   * Create a decoder for the given codec using the best available plugin.
   * Returns nullptr if no loaded plugin supports the codec.
   * The returned pointer MUST be passed to DestroyDecoder — do not delete
   * it directly.
   */
  VideoDecoder* CreateDecoder(VideoCodec codec);

  /**
   * Destroy a decoder previously returned by CreateDecoder.
   * Logs a warning and does nothing if the pointer is unrecognised.
   */
  void DestroyDecoder(VideoDecoder* decoder);

  /**
   * Return the VideoPluginAPI table for the plugin currently selected for
   * the given codec, or nullptr if no plugin supports it.
   * The pointer is valid for the lifetime of this PluginManager.
   * Intended for use by DecoderHost and tests.
   */
  const VideoPluginAPI* GetPluginApi(VideoCodec codec) const;

private:
  struct LoadedPlugin
  {
    void*              handle;  /* OS library handle */
    const VideoPluginAPI* api;
    std::string        path;    /* Full path, kept for logging */
  };

  std::vector<std::string> CollectSearchPaths() const;
  void ScanDirectory(const std::string& path);
  bool LoadPlugin(const std::string& path);

  std::vector<std::string> config_paths_;
  std::string              preferred_plugin_name_;
  std::vector<LoadedPlugin> loaded_plugins_;

  /* First plugin (by load order = alphabetical) supporting each codec.
   * Preferred plugin, when set and capable, overrides this. */
  std::unordered_map<VideoCodec, const VideoPluginAPI*> codec_support_;

  /* Tracks which API owns each live decoder so DestroyDecoder is safe. */
  std::mutex                                             decoder_map_mutex_;
  std::unordered_map<VideoDecoder*, const VideoPluginAPI*> decoder_owners_;
};

} // namespace ifm3d

#endif /* IFM3D_RTSP_PLUGIN_MANAGER_H */
