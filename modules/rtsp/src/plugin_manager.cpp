/**
 * Copyright (C) 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/common/logging/log.h>
#include <ifm3d/rtsp/plugin.h>
#include <ifm3d/rtsp/plugin_manager.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#ifdef _WIN32
#  include <windows.h>
#else
#  include <dlfcn.h>
#endif

// ---------------------------------------------------------------------------
// Platform-abstraction helpers (file-local)
// ---------------------------------------------------------------------------

namespace
{

  void*
  open_library(const std::string& path)
  {
#ifdef _WIN32
    return static_cast<void*>(LoadLibraryA(path.c_str()));
#else
    return dlopen(path.c_str(), RTLD_LAZY | RTLD_LOCAL);
#endif
  }

  void
  close_library(void* handle)
  {
#ifdef _WIN32
    FreeLibrary(static_cast<HMODULE>(handle));
#else
    dlclose(handle);
#endif
  }

#ifdef _WIN32
  using proc_handle_t = FARPROC;
#else
  using proc_handle_t = void*;
#endif

  proc_handle_t
  get_symbol(void* handle, const char* name)
  {
#ifdef _WIN32
    return GetProcAddress(static_cast<HMODULE>(handle), name);
#else
    return dlsym(handle, name);
#endif
  }

  std::string
  last_dl_error()
  {
#ifdef _WIN32
    DWORD err = GetLastError();
    char buf[256] = {};
    FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
                   nullptr,
                   err,
                   0,
                   buf,
                   sizeof(buf),
                   nullptr);
    return buf;
#else
    const char* e = dlerror();
    return e ? e : "(unknown error)";
#endif
  }

  /* Returns true if the filename follows the ifm3d plugin naming convention.
   */
  bool
  matches_plugin_name(const std::filesystem::path& p)
  {
    const std::string stem = p.stem().string();
#ifdef _WIN32
    return stem.rfind("ifm3d_plugin_", 0) == 0 && p.extension() == ".dll";
#else
    return stem.rfind("libifm3d_plugin_", 0) == 0 && p.extension() == ".so";
#endif
  }

} // anonymous namespace

// ---------------------------------------------------------------------------
// ifm3d::PluginManager
// ---------------------------------------------------------------------------

namespace ifm3d
{

  PluginManager::PluginManager(std::vector<std::string> paths,
                               std::string preferred_plugin_name)
    : config_paths_(std::move(paths)),
      preferred_plugin_name_(std::move(preferred_plugin_name))
  {}

  PluginManager::~PluginManager()
  {
    {
      const std::lock_guard<std::mutex> lock(decoder_map_mutex_);
      if (!decoder_owners_.empty())
        {
          LOG_WARNING("[PluginManager] WARNING: {} decoder(s) still alive at "
                      "PluginManager shutdown.",
                      decoder_owners_.size());
        }
    }
    for (auto& plugin : loaded_plugins_)
      {
        close_library(plugin.handle);
      }
  }

  std::vector<std::string>
  PluginManager::CollectSearchPaths() const
  {
    std::vector<std::string> paths = config_paths_;

    /* Append paths from IFM3D_PLUGIN_PATH environment variable. */
    const char* env = std::getenv("IFM3D_PLUGIN_PATH");
    if (env)
      {
        std::istringstream ss(env);
        std::string token;
#ifdef _WIN32
        const char sep = ';';
#else
        const char sep = ':';
#endif
        while (std::getline(ss, token, sep))
          {
            if (!token.empty())
              {
                paths.push_back(token);
              }
          }
      }
    return paths;
  }

  void
  PluginManager::LoadPlugins()
  {
    for (const auto& path : CollectSearchPaths())
      {
        ScanDirectory(path);
      }
  }

  void
  PluginManager::ScanDirectory(const std::string& path)
  {
    std::error_code ec;
    const std::filesystem::directory_iterator it(path, ec);
    if (ec)
      {
        LOG_ERROR("[PluginManager] Cannot scan directory: {} ({})",
                  path,
                  ec.message());
        return;
      }

    /* Collect matching files then sort alphabetically for deterministic
     * load order (first-wins selection per codec). */
    std::vector<std::filesystem::path> candidates;
    for (const auto& entry : it)
      {
        if (entry.is_regular_file() && matches_plugin_name(entry.path()))
          {
            candidates.push_back(entry.path());
          }
      }
    std::sort(candidates.begin(), candidates.end());

    for (const auto& candidate : candidates)
      {
        LoadPlugin(candidate.string());
      }
  }

  bool
  PluginManager::LoadPlugin(const std::string& path)
  {
    if (std::any_of(loaded_plugins_.begin(),
                    loaded_plugins_.end(),
                    [&path](const LoadedPlugin& p) { return p.path == path; }))
      {
        LOG_WARNING("[PluginManager] Plugin already loaded: {}", path);
        return true;
      }
    void* handle = open_library(path);
    if (!handle)
      {
        LOG_ERROR("[PluginManager] Failed to open: {} ({})",
                  path,
                  last_dl_error());
        return false;
      }

    proc_handle_t sym = get_symbol(handle, "video_plugin_init");
    if (!sym)
      {
        LOG_ERROR(
          "[PluginManager] Missing symbol 'video_plugin_init' in: {} ({})",
          path,
          last_dl_error());
        close_library(handle);
        return false;
      }

    auto init_fn = reinterpret_cast<video_plugin_init_fn>(sym);

    const VideoPluginAPI* api = nullptr;
    if (init_fn(VIDEO_PLUGIN_ABI_VERSION, &api) != 0 || !api)
      {
        LOG_ERROR("[PluginManager] video_plugin_init failed for: {}", path);
        close_library(handle);
        return false;
      }

    if (api->abi_version != VIDEO_PLUGIN_ABI_VERSION)
      {
        LOG_ERROR(
          "[PluginManager] ABI version mismatch in: {} (plugin={}, host={})",
          path,
          api->abi_version,
          VIDEO_PLUGIN_ABI_VERSION);
        close_library(handle);
        return false;
      }

    if (!api->create_decoder || !api->destroy_decoder || !api->send_packet ||
        !api->receive_frame)
      {
        LOG_ERROR(
          "[PluginManager] Plugin missing required function pointers: {}",
          path);
        close_library(handle);
        return false;
      }

    /* Probe H264 support: create a decoder and immediately destroy it. */
    VideoDecoder* probe = api->create_decoder(VIDEO_CODEC_H264);
    if (!probe)
      {
        LOG_WARNING(
          "[PluginManager] Plugin does not support H264 (probe failed): {}",
          path);
        /* Still register the plugin — it may support future codecs. */
      }
    else
      {
        api->destroy_decoder(probe);

        /* Register codec support: preferred plugin overrides first-found. */
        const std::string stem = std::filesystem::path(path).stem().string();
        auto it = codec_support_.find(VIDEO_CODEC_H264);
        const bool register_as_primary =
          (it == codec_support_.end()) ||
          (!preferred_plugin_name_.empty() &&
           stem.find(preferred_plugin_name_) != std::string::npos);

        if (register_as_primary)
          {
            codec_support_[VIDEO_CODEC_H264] = api;
          }
      }

    loaded_plugins_.push_back({handle, api, path});
    std::cout << "[PluginManager] Loaded plugin: " << path << "\n";
    return true;
  }

  const VideoPluginAPI*
  PluginManager::GetPluginApi(VideoCodec codec) const
  {
    auto it = codec_support_.find(codec);
    return (it != codec_support_.end()) ? it->second : nullptr;
  }

  VideoDecoder*
  PluginManager::CreateDecoder(VideoCodec codec)
  {
    auto it = codec_support_.find(codec);
    if (it == codec_support_.end())
      {
        return nullptr;
      }
    VideoDecoder* dec = it->second->create_decoder(codec);
    if (dec)
      {
        const std::lock_guard<std::mutex> lock(decoder_map_mutex_);
        decoder_owners_[dec] = it->second;
      }
    return dec;
  }

  void
  PluginManager::DestroyDecoder(VideoDecoder* decoder)
  {
    if (!decoder)
      {
        return;
      }

    const VideoPluginAPI* api = nullptr;
    {
      const std::lock_guard<std::mutex> lock(decoder_map_mutex_);
      auto it = decoder_owners_.find(decoder);
      if (it == decoder_owners_.end())
        {
          LOG_ERROR(
            "[PluginManager] DestroyDecoder: unrecognised decoder pointer");
          return;
        }
      api = it->second;
      decoder_owners_.erase(it);
    }

    api->destroy_decoder(decoder);
  }

} // namespace ifm3d
