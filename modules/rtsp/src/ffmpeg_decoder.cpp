/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 *
 * FFmpeg-based reference H.264 decoder for ifm3d RTSP. Exposes decoded frames
 * as YUV420P-compatible planes. libavcodec/libavutil are resolved at runtime
 * through ffmpeg_lib.hpp, so this file links against neither.
 */

#include "ffmpeg_decoder.hpp"

#include <array>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

#include <ifm3d/rtsp/video_decoder.h>

#include "ffmpeg_lib.hpp"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavcodec/codec.h>
#include <libavcodec/codec_id.h>
#include <libavutil/error.h>
#include <libavutil/frame.h>
#include <libavutil/pixfmt.h>
}

namespace ifm3d::rtsp
{
  namespace
  {
    bool
    is_supported_output_format(int pix_fmt)
    {
      return pix_fmt == AV_PIX_FMT_YUV420P || pix_fmt == AV_PIX_FMT_YUVJ420P;
    }

    class FfmpegDecoder : public VideoDecoder
    {
    public:
      FfmpegDecoder() : _api(ffmpeg_api()) {}

      FfmpegDecoder(const FfmpegDecoder&) = delete;
      FfmpegDecoder& operator=(const FfmpegDecoder&) = delete;
      FfmpegDecoder(FfmpegDecoder&&) = delete;
      FfmpegDecoder& operator=(FfmpegDecoder&&) = delete;

      ~FfmpegDecoder() override
      {
        if (_frame != nullptr)
          {
            _api.av_frame_free(&_frame);
          }
        if (_packet != nullptr)
          {
            _api.av_packet_free(&_packet);
          }
        if (_codec_ctx != nullptr)
          {
            _api.avcodec_free_context(&_codec_ctx);
          }
      }

      /* Allocate and open the decoder. Returns false on failure. */
      bool
      Init()
      {
        const AVCodec* codec = _api.avcodec_find_decoder(AV_CODEC_ID_H264);
        if (codec == nullptr)
          {
            _last_error = "Failed to locate H264 decoder";
            return false;
          }

        _codec_ctx = _api.avcodec_alloc_context3(codec);
        if (_codec_ctx == nullptr)
          {
            _last_error = "Failed to allocate AVCodecContext";
            return false;
          }

        const int rc = _api.avcodec_open2(_codec_ctx, codec, nullptr);
        if (rc < 0)
          {
            set_error(rc, "avcodec_open2 failed");
            return false;
          }

        _packet = _api.av_packet_alloc();
        _frame = _api.av_frame_alloc();
        if (_packet == nullptr || _frame == nullptr)
          {
            _last_error = "Failed to allocate AVPacket/AVFrame";
            return false;
          }

        return true;
      }

      int
      SendPacket(const std::uint8_t* data, int size) override
      {
        if (_codec_ctx == nullptr || _packet == nullptr || data == nullptr ||
            size <= 0)
          {
            _last_error = "Invalid decoder or packet input";
            return -1;
          }

        _api.av_packet_unref(_packet);

        int rc = _api.av_new_packet(_packet, size);
        if (rc < 0)
          {
            set_error(rc, "av_new_packet failed");
            return rc;
          }

        std::memcpy(_packet->data, data, static_cast<std::size_t>(size));

        rc = _api.avcodec_send_packet(_codec_ctx, _packet);
        _api.av_packet_unref(_packet);

        if (rc < 0)
          {
            set_error(rc, "avcodec_send_packet failed");
            return rc;
          }

        return 0;
      }

      int
      ReceiveFrame(VideoFrame& out) override
      {
        if (_codec_ctx == nullptr || _frame == nullptr)
          {
            _last_error = "Invalid decoder or output frame";
            return -1;
          }

        const int rc = _api.avcodec_receive_frame(_codec_ctx, _frame);
        if (rc == AVERROR(EAGAIN) || rc == AVERROR_EOF)
          {
            return 0;
          }

        if (rc < 0)
          {
            set_error(rc, "avcodec_receive_frame failed");
            return rc;
          }

        if (!is_supported_output_format(_frame->format))
          {
            _last_error = "Unsupported pixel format from decoder";
            return -2;
          }

        for (int i = 0; i < 4; ++i)
          {
            out.planes[i] = _frame->data[i];
            out.linesize[i] = _frame->linesize[i];
          }
        out.width = _frame->width;
        out.height = _frame->height;
        out.format = VIDEO_FORMAT_YUV420P;

        return 1;
      }

      int
      Flush() override
      {
        if (_codec_ctx == nullptr)
          {
            _last_error = "Invalid decoder in flush";
            return -1;
          }

        const int rc = _api.avcodec_send_packet(_codec_ctx, nullptr);
        if (rc < 0 && rc != AVERROR_EOF && rc != AVERROR(EAGAIN))
          {
            set_error(rc, "flush failed");
            return rc;
          }

        return 0;
      }

      [[nodiscard]] std::string
      LastError() const override
      {
        return _last_error;
      }

    private:
      void
      set_error(int errnum, const char* prefix)
      {
        std::array<char, AV_ERROR_MAX_STRING_SIZE> errbuf{};
        _api.av_strerror(errnum, errbuf.data(), errbuf.size());
        _last_error = std::string(prefix) + ": " + errbuf.data();
      }

      const FfmpegApi& _api;
      AVCodecContext* _codec_ctx = nullptr;
      AVPacket* _packet = nullptr;
      AVFrame* _frame = nullptr;
      std::string _last_error;
    };

    class FfmpegDecoderFactory : public VideoDecoderFactory
    {
    public:
      [[nodiscard]] std::string
      Name() const override
      {
        return FFMPEG_DECODER_NAME;
      }

      [[nodiscard]] bool
      IsAvailable() const override
      {
        const FfmpegApi& api = ffmpeg_api();
        return api.available &&
               api.avcodec_find_decoder(AV_CODEC_ID_H264) != nullptr;
      }

      [[nodiscard]] bool
      SupportsCodec(VideoCodec codec) const override
      {
        return codec == VIDEO_CODEC_H264 && IsAvailable();
      }

      std::unique_ptr<VideoDecoder>
      CreateDecoder(VideoCodec codec) override
      {
        if (codec != VIDEO_CODEC_H264 || !IsAvailable())
          {
            return nullptr;
          }
        auto decoder = std::make_unique<FfmpegDecoder>();
        if (!decoder->Init())
          {
            return nullptr;
          }
        return decoder;
      }

      [[nodiscard]] std::string
      AvailabilityError() const override
      {
        return ffmpeg_api().error;
      }
    };
  } // namespace

  std::unique_ptr<VideoDecoderFactory>
  make_ffmpeg_decoder_factory()
  {
    return std::make_unique<FfmpegDecoderFactory>();
  }

} // namespace ifm3d::rtsp
