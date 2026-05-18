/*
 * Reference sample plugin — compiled as a shared library.
 * Naming must match the convention: libifm3d_plugin_sample.so / ifm3d_plugin_sample.dll
 *
 * The plugin is responsible for defining struct VideoDecoder.
 * It is an intentionally incomplete/opaque type in plugin.h so the host
 * cannot dereference it. Only the plugin that allocates it knows its layout.
 */
#include <iostream>
#include <ifm3d/rtsp/plugin.h>

/* Concrete definition — visible only inside this translation unit.
 * The host holds a VideoDecoder* but never looks inside it. */
struct VideoDecoder {
    VideoCodec codec;
};

extern "C" VideoDecoder* create_decoder(VideoCodec codec) {
    if (codec != VIDEO_CODEC_H264) {
        return nullptr;
    }
    return new VideoDecoder{codec};
}

extern "C" void destroy_decoder(VideoDecoder* decoder) {
    delete decoder;
}

extern "C" int send_packet(VideoDecoder* decoder, const uint8_t* data, int size) {
    return 0;
}

extern "C" int receive_frame(VideoDecoder* decoder, VideoFrame* out) {
    return 0;
}

extern "C" int flush(VideoDecoder* decoder) {
    return 0;
}

extern "C" const char* last_error(VideoDecoder* decoder) {
    return nullptr;
}

extern "C" int video_plugin_init(uint32_t host_abi, const VideoPluginAPI** out_api) {
    if (host_abi != VIDEO_PLUGIN_ABI_VERSION) {
        return -1;
    }

    static VideoPluginAPI api = {
        .abi_version = VIDEO_PLUGIN_ABI_VERSION,
        .create_decoder = create_decoder,
        .destroy_decoder = destroy_decoder,
        .send_packet = send_packet,
        .receive_frame = receive_frame,
        .flush = flush,
        .last_error = last_error,
    };

    *out_api = &api;
    return 0;
}