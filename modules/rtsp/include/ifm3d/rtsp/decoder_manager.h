/*
 * Copyright (C) 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_RTSP_DECODER_MANAGER_H
#define IFM3D_RTSP_DECODER_MANAGER_H

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <ifm3d/rtsp/video_decoder.h>

namespace ifm3d
{

  /**
   * Lists and selects the video decoder implementations that are compiled into
   * the RTSP module.
   *
   * Each implementation (e.g. the ffmpeg decoder and the always-available
   * "null" decoder) is registered here. Implementations that depend on
   * optional runtime libraries report their own availability -- the ffmpeg
   * decoder, for example, resolves libavcodec/libavutil at runtime and is
   * unavailable when no compatible FFmpeg is installed.
   *
   * Selection for a codec (deterministic):
   *   - If preferred_decoder is the "null" name, the null decoder is used
   *     (frames are discarded, NAL-only).
   *   - Else if preferred_decoder is set and an available decoder whose name
   *     contains that string supports the codec, it is used.
   *   - Otherwise the first available decoder (excluding "null") that supports
   *     the codec is used.
   *   - If none is available, the "null" decoder is used as a fallback. A
   *     warning is logged whenever the null decoder is selected.
   */
  class DecoderManager
  {
  public:
    /**
     * @param preferred_decoder Optional decoder name substring (e.g.
     * "ffmpeg"), or the "null" decoder name.
     */
    explicit DecoderManager(std::optional<std::string> preferred_decoder = {});

    ~DecoderManager();

    /**
     * Describes a registered decoder implementation and whether it can be used
     * on the current system.
     */
    struct DecoderInfo
    {
      /* Stable decoder name (e.g. "ffmpeg", "null"). */
      std::string name;
      /* True when the decoder can be used on the current system (e.g. the
       * ffmpeg decoder found a usable libavcodec). */
      bool available{false};
      /* True when the decoder advertises H.264 decode support. */
      bool supports_h264{false};
      /* Human-readable reason when available is false; empty otherwise. */
      std::string error;
    };

    /**
     * List the decoder implementations and report whether each is usable on
     * the current system.
     *
     * This is a diagnostic helper: it does not affect the state of any
     * DecoderManager instance.
     */
    static std::vector<DecoderInfo> DiscoverDecoders();

    /* Non-copyable, non-movable. */
    DecoderManager(const DecoderManager&) = delete;
    DecoderManager& operator=(const DecoderManager&) = delete;
    DecoderManager(DecoderManager&&) = delete;
    DecoderManager& operator=(DecoderManager&&) = delete;

    /**
     * Resolve which decoder serves each codec, honouring preferred_decoder and
     * falling back to the null decoder. Should be called once after
     * construction.
     */
    void LoadDecoders();

    /**
     * Create a decoder for the given codec using the selected implementation.
     * Returns nullptr if LoadDecoders() has not run or no decoder is
     * available.
     */
    std::unique_ptr<ifm3d::rtsp::VideoDecoder> CreateDecoder(
      ifm3d::rtsp::VideoCodec codec);

    /**
     * @return the name of the decoder selected for @p codec, or an empty
     * string if none has been selected. Intended for diagnostics/tests.
     */
    [[nodiscard]] std::string SelectedDecoderName(
      ifm3d::rtsp::VideoCodec codec) const;

  private:
    void select_decoders();

    std::optional<std::string> _preferred_decoder;

    /* Registered decoder implementations, in preference order (real decoders
     * first, the null decoder last). */
    std::vector<std::unique_ptr<ifm3d::rtsp::VideoDecoderFactory>> _factories;

    /* Decoder selected for each codec (may be the null decoder). Empty until
     * LoadDecoders() runs. */
    std::unordered_map<ifm3d::rtsp::VideoCodec,
                       ifm3d::rtsp::VideoDecoderFactory*>
      _selected;
  };

} // namespace ifm3d

#endif /* IFM3D_RTSP_DECODER_MANAGER_H */
