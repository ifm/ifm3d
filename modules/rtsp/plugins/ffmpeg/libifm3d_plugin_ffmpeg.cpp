/*
 * FFmpeg-based reference decoder plugin for ifm3d RTSP.
 *
 * This plugin implements VideoPluginAPI v1 for H264 decode and exposes
 * decoded frames as YUV420P-compatible planes.
 */

#include <ifm3d/rtsp/plugin.h>

#include <array>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavcodec/codec.h>
#include <libavcodec/codec_id.h>
#include <libavcodec/packet.h>
#include <libavutil/error.h>
#include <libavutil/frame.h>
#include <libavutil/pixfmt.h>
}

struct VideoDecoder
{
  struct CodecContextDeleter
  {
    void
    operator()(AVCodecContext* p) const
    {
      if (p)
        {
          avcodec_free_context(&p);
        }
    }
  };

  struct PacketDeleter
  {
    void
    operator()(AVPacket* p) const
    {
      if (p)
        {
          av_packet_free(&p);
        }
    }
  };

  struct FrameDeleter
  {
    void
    operator()(AVFrame* p) const
    {
      if (p)
        {
          av_frame_free(&p);
        }
    }
  };

  std::unique_ptr<AVCodecContext, CodecContextDeleter> codec_ctx;
  std::unique_ptr<AVPacket, PacketDeleter> packet;
  std::unique_ptr<AVFrame, FrameDeleter> frame;
  std::string last_error;
};

namespace
{

  void
  set_error(VideoDecoder* decoder, int errnum, const char* prefix)
  {
    if (!decoder)
      {
        return;
      }

    std::array<char, AV_ERROR_MAX_STRING_SIZE> errbuf{};
    av_strerror(errnum, errbuf.data(), errbuf.size());
    decoder->last_error = std::string(prefix) + ": " + errbuf.data();
  }

  bool
  is_supported_output_format(int pix_fmt)
  {
#ifdef AV_PIX_FMT_YUVJ420P
    return pix_fmt == AV_PIX_FMT_YUV420P || pix_fmt == AV_PIX_FMT_YUVJ420P;
#else
    return pix_fmt == AV_PIX_FMT_YUV420P;
#endif
  }

} // anonymous namespace

extern "C" VideoDecoder*
create_decoder(VideoCodec codec)
{
  if (codec != VIDEO_CODEC_H264)
    {
      return nullptr;
    }

  std::unique_ptr<VideoDecoder> decoder(new VideoDecoder());

  const AVCodec* avcodec = avcodec_find_decoder(AV_CODEC_ID_H264);
  if (!avcodec)
    {
      decoder->last_error = "Failed to locate H264 decoder";
      return nullptr;
    }

  decoder->codec_ctx.reset(avcodec_alloc_context3(avcodec));
  if (!decoder->codec_ctx)
    {
      decoder->last_error = "Failed to allocate AVCodecContext";
      return nullptr;
    }

  const int rc = avcodec_open2(decoder->codec_ctx.get(), avcodec, nullptr);
  if (rc < 0)
    {
      set_error(decoder.get(), rc, "avcodec_open2 failed");
      return nullptr;
    }

  decoder->packet.reset(av_packet_alloc());
  decoder->frame.reset(av_frame_alloc());

  if (!decoder->packet || !decoder->frame)
    {
      decoder->last_error = "Failed to allocate AVPacket/AVFrame";
      return nullptr;
    }

  return decoder.release();
}

extern "C" void
destroy_decoder(VideoDecoder* decoder)
{
  const std::unique_ptr<VideoDecoder> owned(decoder);
}

extern "C" int
send_packet(VideoDecoder* decoder, const uint8_t* data, int size)
{
  if (!decoder || !decoder->codec_ctx || !decoder->packet || !data ||
      size <= 0)
    {
      if (decoder)
        {
          decoder->last_error = "Invalid decoder or packet input";
        }
      return -1;
    }

  av_packet_unref(decoder->packet.get());

  int rc = av_new_packet(decoder->packet.get(), size);
  if (rc < 0)
    {
      set_error(decoder, rc, "av_new_packet failed");
      return rc;
    }

  std::memcpy(decoder->packet->data, data, static_cast<size_t>(size));

  rc = avcodec_send_packet(decoder->codec_ctx.get(), decoder->packet.get());
  av_packet_unref(decoder->packet.get());

  if (rc < 0)
    {
      set_error(decoder, rc, "avcodec_send_packet failed");
      return rc;
    }

  return 0;
}

extern "C" int
receive_frame(VideoDecoder* decoder, VideoFrame* out)
{
  if (!decoder || !decoder->codec_ctx || !decoder->frame || !out)
    {
      if (decoder)
        {
          decoder->last_error = "Invalid decoder or output frame";
        }
      return -1;
    }

  const int rc =
    avcodec_receive_frame(decoder->codec_ctx.get(), decoder->frame.get());
  if (rc == AVERROR(EAGAIN) || rc == AVERROR_EOF)
    {
      return 0;
    }

  if (rc < 0)
    {
      set_error(decoder, rc, "avcodec_receive_frame failed");
      return rc;
    }

  if (!is_supported_output_format(decoder->frame->format))
    {
      decoder->last_error = "Unsupported pixel format from decoder";
      return -2;
    }

  for (int i = 0; i < 4; ++i)
    {
      out->planes[i] = decoder->frame->data[i];
      out->linesize[i] = decoder->frame->linesize[i];
    }
  out->width = decoder->frame->width;
  out->height = decoder->frame->height;
  out->format = VIDEO_FORMAT_YUV420P;

  return 1;
}

extern "C" int
flush(VideoDecoder* decoder)
{
  if (!decoder || !decoder->codec_ctx)
    {
      if (decoder)
        {
          decoder->last_error = "Invalid decoder in flush";
        }
      return -1;
    }

  const int rc = avcodec_send_packet(decoder->codec_ctx.get(), nullptr);
  if (rc < 0 && rc != AVERROR_EOF && rc != AVERROR(EAGAIN))
    {
      set_error(decoder, rc, "flush failed");
      return rc;
    }

  return 0;
}

extern "C" const char*
last_error(VideoDecoder* decoder)
{
  if (!decoder || decoder->last_error.empty())
    {
      return nullptr;
    }
  return decoder->last_error.c_str();
}

extern "C" int
video_plugin_init(uint32_t host_abi, const VideoPluginAPI** out_api)
{
  if (!out_api || host_abi != VIDEO_PLUGIN_ABI_VERSION)
    {
      return -1;
    }

  static const VideoPluginAPI API = {
    VIDEO_PLUGIN_ABI_VERSION,
    create_decoder,
    destroy_decoder,
    send_packet,
    receive_frame,
    flush,
    last_error,
  };

  *out_api = &API;
  return 0;
}
