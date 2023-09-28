// USE THIS RESOURCE
// https://developer.mozilla.org/en-US/docs/Web/API/RTCPeerConnection/
// https://developer.mozilla.org/en-US/docs/Web/API/RTCPeerConnection/createOffer
// IT WILL BE REALLY HELPFUL


#define GST_USE_UNSTABLE_API
#define GST_DISABLE_REGISTRY 1

#include <glib-unix.h>
#include <glib.h>
#include <gst/app/app.h>
#include <gst/gst.h>
#include <gst/sdp/sdp.h>
#include <gst/webrtc/icetransport.h>
#include <gst/webrtc/webrtc.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <map>
#include <thread>

#include "absl/strings/str_format.h"
#include "flatbuffers/flatbuffers.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "aos/events/glib_main_loop.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/network/web_proxy_generated.h"
#include "aos/seasocks/seasocks_logger.h"
#include "frc971/vision/vision_generated.h"
#include "internal/Embedded.h"
#include "seasocks/Server.h"
#include "seasocks/StringUtil.h"
#include "seasocks/WebSocket.h"

// Link plugins to GStreamer
extern "C" {
GST_PLUGIN_STATIC_DECLARE(app);
GST_PLUGIN_STATIC_DECLARE(coreelements);
GST_PLUGIN_STATIC_DECLARE(dtls);
GST_PLUGIN_STATIC_DECLARE(nice);
GST_PLUGIN_STATIC_DECLARE(rtp);
GST_PLUGIN_STATIC_DECLARE(rtpmanager);
GST_PLUGIN_STATIC_DECLARE(srtp);
GST_PLUGIN_STATIC_DECLARE(webrtc);
GST_PLUGIN_STATIC_DECLARE(video4linux2);
GST_PLUGIN_STATIC_DECLARE(videoconvert);
GST_PLUGIN_STATIC_DECLARE(videoparsersbad);
GST_PLUGIN_STATIC_DECLARE(videorate);
GST_PLUGIN_STATIC_DECLARE(videoscale);
GST_PLUGIN_STATIC_DECLARE(videotestsrc);
GST_PLUGIN_STATIC_DECLARE(x264);
}

// A bunch of define macros
DEFINE_string(config, "aos_config.json",
              "Name of the config file to replay using.");
DEFINE_string(device, "/dev/video0",
              "Camera fd. Ignored if reading from channel");
DEFINE_string(data_dir, "image_streamer_www",
              "Directory to serve data files from");
DEFINE_int32(width, 400, "Image width");
DEFINE_int32(height, 300, "Image height");
DEFINE_int32(framerate, 25, "Framerate (FPS)");
DEFINE_int32(brightness, 50, "Camera brightness");
DEFINE_int32(exposure, 300, "Manual exposure");
DEFINE_int32(bitrate, 500000, "H264 encode bitrate");
DEFINE_int32(streaming_port, 1180, "Port to stream images on with seasocks");
DEFINE_int32(min_port, 5800, "Min rtp port");
DEFINE_int32(max_port, 5810, "Max rtp port");
DEFINE_string(listen_on, "",
              "Channel on which to receive frames from. Used in place of "
              "internal V4L2 reader. Note: width and height MUST match the "
              "expected size of channel images.");

// Forward declare a connection class
class Connection;

using aos::web_proxy::Payload;
using aos::web_proxy::SdpType;
using aos::web_proxy::WebSocketIce;
using aos::web_proxy::WebSocketMessage;
using aos::web_proxy::WebSocketSdp;

// Create a super class Sample Source
class GstSampleSource {
 public:
  GstSampleSource() = default;

  virtual ~GstSampleSource() = default;

 private:
  GstSampleSource(const GstSampleSource &) = delete;
};

// Create a "v42l" source
// V4L2 is https://en.wikipedia.org/wiki/Video4Linux
// This is a library for realtime video capture on linux
// This represents a video source from that library
// It has a GST sampling pipline
// A pointer app src, which allows us to pull data from the pipeline
// And a callback function to do stuff whenever we get a sample
class V4L2Source : public GstSampleSource {
 public:
    // A V4L2 source has a callback function that puts data into a pointer sample
  V4L2Source(std::function<void(GstSample *)> callback)
      : callback_(std::move(callback)) {
    GError *error = NULL;

    // Create pipeline to read from camera, pack into rtp stream, and dump
    // stream to callback. v4l2 device should already be configured with correct
    // bitrate from v4l2-ctl. do-timestamp marks the time the frame was taken to
    // track when it should be dropped under latency.

    // With the Pi's hardware encoder, we can encode and package the stream once
    // and the clients will jump in at any point unsynchronized. With the stream
    // from x264enc this doesn't seem to work. For now, just reencode for each
    // client since we don't expect more than 1 or 2.

    // Parse a launch?
    pipeline_ = gst_parse_launch(
        absl::StrFormat("v4l2src device=%s do-timestamp=true "
                        "extra-controls=\"c,brightness=%d,auto_exposure=1,"
                        "exposure_time_absolute=%d\" ! "
                        "video/x-raw,width=%d,height=%d,framerate=%d/"
                        "1,format=YUY2 ! appsink "
                        "name=appsink "
                        "emit-signals=true sync=false async=false "
                        "caps=video/x-raw,format=YUY2",
                        FLAGS_device, FLAGS_brightness, FLAGS_exposure,
                        FLAGS_width, FLAGS_height, FLAGS_framerate)
            .c_str(),
        &error);

    // If there is an error
    if (error != NULL) {
      LOG(FATAL) << "Could not create v4l2 pipeline: " << error->message;
    }

    // Get a binary named "app sink"
    appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "appsink");
    if (appsink_ == NULL) {
      LOG(FATAL) << "Could not get appsink";
    }

    // Connect the binary as some callback function?
    g_signal_connect(appsink_, "new-sample",
                     G_CALLBACK(V4L2Source::OnSampleCallback),
                     static_cast<gpointer>(this));

    // Set the pipeline state as "playing"
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  }

      // Destructor
  ~V4L2Source() {
    if (pipeline_ != NULL) {
      gst_element_set_state(GST_ELEMENT(pipeline_), GST_STATE_NULL);
      gst_object_unref(GST_OBJECT(pipeline_));
      gst_object_unref(GST_OBJECT(appsink_));
    }
  }


 private:
  // Set gpointer to point at the function "on sample"
  // That function is probably called whenever we get a sample
  static GstFlowReturn OnSampleCallback(GstElement *, gpointer user_data) {
    static_cast<V4L2Source *>(user_data)->OnSample();
    return GST_FLOW_OK;
  }

  // Pull the sample from the GST app sink
  void OnSample() {
    GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink_));
    if (sample == NULL) {
      LOG(WARNING) << "Received null sample";
      return;
    }
    // Call the callback function on the sample
    callback_(sample);
    // remove some reference to the sample
    gst_sample_unref(sample);
  }

  GstElement *pipeline_;
  GstElement *appsink_;

  std::function<void(GstSample *)> callback_;
};

// This represents a source from a "Channel" (whatever that is)
// It accesses the event loop, and whenever it gets an image
// It does some preperation and then gives data to a callback function
class ChannelSource : public GstSampleSource {
  // Create a Channel source from the event loop and a callback function
 public:
  ChannelSource(aos::ShmEventLoop *event_loop, std::function<void(GstSample *)> callback)
      : callback_(std::move(callback)) {
    // Make a watcher for the event loop
    // This listens to when we get an image from the camera and does something with it
    event_loop->MakeWatcher(
        FLAGS_listen_on,
        [this](const frc971::vision::CameraImage &image) { OnImage(image); });
  }

 private:
  // Does something with an image!
  void OnImage(const frc971::vision::CameraImage &image) {
    // If the image doesn't have data, skip it
    if (!image.has_rows() || !image.has_cols() || !image.has_data()) {
      VLOG(2) << "Skipping CameraImage with no data";
      return;
    }
    // Make sure that we're in the right aspect ratio
    CHECK_EQ(image.rows(), FLAGS_height);
    CHECK_EQ(image.cols(), FLAGS_width);

    // Create a buffer from the image data
    GBytes *bytes = g_bytes_new(image.data()->data(), image.data()->size());
    GstBuffer *buffer = gst_buffer_new_wrapped_bytes(bytes);

    // Get a timestamp ???
    GST_BUFFER_PTS(buffer) = image.monotonic_timestamp_ns();

    // Short for capabilities, describes media types
    // Allows our code to understand how the image is formatted
    GstCaps *caps = CHECK_NOTNULL(gst_caps_new_simple(
        "video/x-raw", "width", G_TYPE_INT, image.cols(), "height", G_TYPE_INT,
        image.rows(), "format", G_TYPE_STRING, "YUY2", nullptr));

    // Create a new sample
    GstSample *sample = gst_sample_new(buffer, caps, nullptr, nullptr);

    // do something with the sample
    callback_(sample);

    // Release refereces to things
    gst_sample_unref(sample);
    gst_caps_unref(caps);
    gst_buffer_unref(buffer);
    g_bytes_unref(bytes);
  }

  std::function<void(GstSample *)> callback_;
};

// Basic class that handles receiving new websocket connections. Creates a new
// Connection to manage the rest of the negotiation and data passing. When the
// websocket closes, it deletes the Connection.
class WebsocketHandler : public ::seasocks::WebSocket::Handler {
 public:
  WebsocketHandler(aos::ShmEventLoop *event_loop, ::seasocks::Server *server);
  ~WebsocketHandler() override = default;

  void onConnect(::seasocks::WebSocket *sock) override;
  void onData(::seasocks::WebSocket *sock, const uint8_t *data,
              size_t size) override;
  void onDisconnect(::seasocks::WebSocket *sock) override;

 private:
  void OnSample(GstSample *sample);

  std::map<::seasocks::WebSocket *, std::unique_ptr<Connection>> connections_;
  ::seasocks::Server *server_;
  std::unique_ptr<GstSampleSource> source_;

  aos::Sender<frc971::vision::CameraImage> sender_;
};

// Seasocks requires that sends happen on the correct thread. This class takes a
// detached buffer to send on a specific websocket connection and sends it when
// seasocks is ready.
class UpdateData : public ::seasocks::Server::Runnable {
 public:
  UpdateData(::seasocks::WebSocket *websocket,
             flatbuffers::DetachedBuffer &&buffer)
      : sock_(websocket), buffer_(std::move(buffer)) {}
  ~UpdateData() override = default;
  UpdateData(const UpdateData &) = delete;
  UpdateData &operator=(const UpdateData &) = delete;

  void run() override { sock_->send(buffer_.data(), buffer_.size()); }

 private:
  ::seasocks::WebSocket *sock_;
  const flatbuffers::DetachedBuffer buffer_;
};

// Connection was forward declared, now we define it
// A Connection is made of a server, a WebSocket, a streaming pipeline
// A "web rtc bin (binary?)"
// And an "App Source"
class Connection {
 public:
  Connection(::seasocks::WebSocket *sock, ::seasocks::Server *server);

  ~Connection();

  void HandleWebSocketData(const uint8_t *data, size_t size);

  void OnSample(GstSample *sample);

 private:
  static void OnOfferCreatedCallback(GstPromise *promise, gpointer user_data) {
    static_cast<Connection *>(user_data)->OnOfferCreated(promise);
  }

  static void OnNegotiationNeededCallback(GstElement *, gpointer user_data) {
    static_cast<Connection *>(user_data)->OnNegotiationNeeded();
  }

  static void OnIceCandidateCallback(GstElement *, guint mline_index,
                                     gchar *candidate, gpointer user_data) {
    static_cast<Connection *>(user_data)->OnIceCandidate(mline_index,
                                                         candidate);
  }

  void OnOfferCreated(GstPromise *promise);
  void OnNegotiationNeeded();
  void OnIceCandidate(guint mline_index, gchar *candidate);

  ::seasocks::WebSocket *sock_;
  ::seasocks::Server *server_;

  GstElement *pipeline_;
  GstElement *webrtcbin_;
  GstElement *appsrc_;

  bool first_sample_ = true;
};

// This creates a handler, using either V4L2 source of Channel source
// In either case, the callback function is just WebsocketHandler::OnSample
WebsocketHandler::WebsocketHandler(aos::ShmEventLoop *event_loop,
                                   ::seasocks::Server *server)
    : server_(server) {
  // If we aren't told what to listen for
  if (FLAGS_listen_on.empty()) {
    // Make a sender, which gives us camera data
    sender_ = event_loop->MakeSender<frc971::vision::CameraImage>("/camera");
    // Use V4L2
    source_ = std::make_unique<V4L2Source>([this](auto sample) { OnSample(sample); });
  } 
  else {
    // Use Channel Source
    source_ = std::make_unique<ChannelSource>( event_loop, [this](auto sample) { OnSample(sample); });
  }
}

// Just makes a connection to the server, and maps the given websocket to the new connection
void WebsocketHandler::onConnect(::seasocks::WebSocket *sock) {
  std::unique_ptr<Connection> conn =
      std::make_unique<Connection>(sock, server_);
  connections_.insert({sock, std::move(conn)});
}

// When we get data, pass it to Connections::HandleWebSocketData
void WebsocketHandler::onData(::seasocks::WebSocket *sock, const uint8_t *data,
                              size_t size) {
  connections_[sock]->HandleWebSocketData(data, size);
}

// For connection in connections, call some OnSample function
// If we have a valid sender, build an image from the sample data
// and sender it with the sender builder class
void WebsocketHandler::OnSample(GstSample *sample) {
  for (auto iter = connections_.begin(); iter != connections_.end(); ++iter) {
    iter->second->OnSample(sample);
  }

  // If we have a valid sender ... 
  if (sender_.valid()) {
    // Get the image format and the structure
    const GstCaps *caps = CHECK_NOTNULL(gst_sample_get_caps(sample));
    CHECK_GT(gst_caps_get_size(caps), 0U);
    const GstStructure *str = gst_caps_get_structure(caps, 0);

    gint width;
    gint height;

    // Set the width and height variables (somehow?)
    CHECK(gst_structure_get_int(str, "width", &width));
    CHECK(gst_structure_get_int(str, "height", &height));

    // Get a buffer (?) from the sample
    GstBuffer *buffer = CHECK_NOTNULL(gst_sample_get_buffer(sample));

    // Get the buffer size 
    const gsize size = gst_buffer_get_size(buffer);

    // Make an image builder
    auto builder = sender_.MakeBuilder();

    // Create a vector out of a pointer
    uint8_t *image_data;
    auto image_offset =
        builder.fbb()->CreateUninitializedVector(size, &image_data);
    // Extract the data from the buffer to the pointer
    gst_buffer_extract(buffer, 0, image_data, size);

    // Make an Image builder from the size and data
    auto image_builder = builder.MakeBuilder<frc971::vision::CameraImage>();
    image_builder.add_rows(height);
    image_builder.add_cols(width);
    image_builder.add_data(image_offset);

    // Finish the builder and send it
    builder.CheckOk(builder.Send(image_builder.Finish()));
  }
}


void WebsocketHandler::onDisconnect(::seasocks::WebSocket *sock) {
  connections_.erase(sock);
}

// Create a connection
// To do this, we 
// Uhh
// Come back later :)
Connection::Connection(::seasocks::WebSocket *sock, ::seasocks::Server *server)
    : sock_(sock), server_(server) {
  GError *error = NULL;

  // Build pipeline to read data from application into pipeline, place in
  // webrtcbin group, and stream.

  // Create a pipeline(?)
  pipeline_ = gst_parse_launch(
      // aggregate-mode should be zero-latency but this drops the stream on
      // bitrate spikes for some reason - probably the weak CPU on the pi.

      // What the h*ll is this namespace?
      absl::StrFormat(
          "webrtcbin name=webrtcbin appsrc "
          "name=appsrc block=false "
          "is-live=true "
          "format=3 max-buffers=0 leaky-type=2 "
          "caps=video/x-raw,width=%d,height=%d,format=YUY2 ! videoconvert ! "
          "x264enc bitrate=%d speed-preset=ultrafast "
          "tune=zerolatency key-int-max=15 sliced-threads=true ! "
          "video/x-h264,profile=constrained-baseline ! h264parse ! "
          "rtph264pay "
          "config-interval=-1 name=payloader aggregate-mode=none ! "
          "application/"
          "x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000 !"
          "webrtcbin. ",
          FLAGS_width, FLAGS_height, FLAGS_bitrate / 1000)
          .c_str(),
      &error);

  // Fail on error
  if (error != NULL) {
    LOG(FATAL) << "Could not create WebRTC pipeline: " << error->message;
  }

  // Get the web rtc binary
  webrtcbin_ = gst_bin_get_by_name(GST_BIN(pipeline_), "webrtcbin");
  if (webrtcbin_ == NULL) {
    LOG(FATAL) << "Could not initialize webrtcbin";
  }

  // Get the App Source
  // App Src puts sampling data into the pipeline
  appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "appsrc");
  if (appsrc_ == NULL) {
    LOG(FATAL) << "Could not initialize appsrc";
  }

  {
    // Create a pointer to an array of transceivers

    GArray *transceivers;
    // Get the transceivers somehow 
    g_signal_emit_by_name(webrtcbin_, "get-transceivers", &transceivers);
    if (transceivers == NULL || transceivers->len <= 0) {
      LOG(FATAL) << "Could not initialize transceivers";
    }

    // I have no idea what this does
    GstWebRTCRTPTransceiver *trans =
        g_array_index(transceivers, GstWebRTCRTPTransceiver *, 0);
    g_object_set(trans, "direction",
                 GST_WEBRTC_RTP_TRANSCEIVER_DIRECTION_SENDONLY, nullptr);

    g_array_unref(transceivers);
  }

  {
    // What is an ice-agent
    GstObject *ice = nullptr;
    g_object_get(G_OBJECT(webrtcbin_), "ice-agent", &ice, nullptr);
    CHECK_NOTNULL(ice);

    g_object_set(ice, "min-rtp-port", FLAGS_min_port, "max-rtp-port",
                 FLAGS_max_port, nullptr);

    // We don't need upnp on a local network.
    {
      // What is this
      GstObject *nice = nullptr;
      g_object_get(ice, "agent", &nice, nullptr);
      CHECK_NOTNULL(nice);

      g_object_set(nice, "upnp", false, nullptr);
      g_object_unref(nice);
    }

    gst_object_unref(ice);
  }

  // Configures Web rtc to use a call back whenever a "negotiation" is needed
  g_signal_connect(webrtcbin_, "on-negotiation-needed",
                   G_CALLBACK(Connection::OnNegotiationNeededCallback),
                   static_cast<gpointer>(this));

  // Configures web rtc to use a call back whenever an "ice candidate" is recieved
  g_signal_connect(webrtcbin_, "on-ice-candidate",
                   G_CALLBACK(Connection::OnIceCandidateCallback),
                   static_cast<gpointer>(this));

  // Set the pipeline to ready and playing
  gst_element_set_state(pipeline_, GST_STATE_READY);
  gst_element_set_state(pipeline_, GST_STATE_PLAYING);
}

Connection::~Connection() {
  if (pipeline_ != NULL) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);

    gst_object_unref(GST_OBJECT(webrtcbin_));
    gst_object_unref(GST_OBJECT(pipeline_));
    gst_object_unref(GST_OBJECT(appsrc_));
  }
}

// When we get a sample, put it into the stream
// And do some other garbage idk
void Connection::OnSample(GstSample *sample) {
  // When we get a sample, try to push it into the stream
  GstFlowReturn response =
      gst_app_src_push_sample(GST_APP_SRC(appsrc_), sample);
  if (response != GST_FLOW_OK) {
    LOG(WARNING) << "Sample pushed, did not receive OK";
  }

  // Since the stream is already running (the camera turns on with
  // image_streamer) we need to tell the new appsrc where
  // we are starting in the stream so it can catch up immediately.
  if (first_sample_) {
    GstPad *src = gst_element_get_static_pad(appsrc_, "src");
    if (src == NULL) {
      return;
    }

    GstSegment *segment = gst_sample_get_segment(sample);
    GstBuffer *buffer = gst_sample_get_buffer(sample);

    guint64 offset = gst_segment_to_running_time(segment, GST_FORMAT_TIME,
                                                 GST_BUFFER_PTS(buffer));
    LOG(INFO) << "Fixing offset " << offset;
    gst_pad_set_offset(src, -offset);

    gst_object_unref(GST_OBJECT(src));
    first_sample_ = false;
  }
}

// Handles an SDP offer 
// This offer is how a WebRTC connection is started
// The SDP includes information about the media in the WebRTC
// Codec, and ICE candidates
void Connection::OnOfferCreated(GstPromise *promise) {
  LOG(INFO) << "OnOfferCreated";

  // Get the session description?
  GstWebRTCSessionDescription *offer = NULL;
  gst_structure_get(gst_promise_get_reply(promise), "offer",
                    GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &offer, NULL);
  gst_promise_unref(promise);

  {
    // Set the local description?
    std::unique_ptr<GstPromise, decltype(&gst_promise_unref)>
        local_desc_promise(gst_promise_new(), &gst_promise_unref);
    g_signal_emit_by_name(webrtcbin_, "set-local-description", offer,
                          local_desc_promise.get());
    gst_promise_interrupt(local_desc_promise.get());
  }

  // Create a string from an "sdp" message
  GstSDPMessage *sdp_msg = offer->sdp;
  std::string sdp_str(gst_sdp_message_as_text(sdp_msg));

  LOG(INFO) << "Negotiation offer created:\n" << sdp_str;

  // Create a buffer and an answer message
  flatbuffers::FlatBufferBuilder fbb(512);
  flatbuffers::Offset<WebSocketSdp> sdp_fb =
      CreateWebSocketSdpDirect(fbb, SdpType::OFFER, sdp_str.c_str());
  flatbuffers::Offset<WebSocketMessage> answer_message =
      CreateWebSocketMessage(fbb, Payload::WebSocketSdp, sdp_fb.Union());
  fbb.Finish(answer_message);

  // ???
  server_->execute(std::make_shared<UpdateData>(sock_, fbb.Release()));
}

void Connection::OnNegotiationNeeded() {
  LOG(INFO) << "OnNegotiationNeeded";

  GstPromise *promise;
  promise = gst_promise_new_with_change_func(Connection::OnOfferCreatedCallback,
                                             static_cast<gpointer>(this), NULL);
  g_signal_emit_by_name(G_OBJECT(webrtcbin_), "create-offer", NULL, promise);
}

void Connection::OnIceCandidate(guint mline_index, gchar *candidate) {
  LOG(INFO) << "OnIceCandidate";

  flatbuffers::FlatBufferBuilder fbb(512);

  auto ice_fb_builder = WebSocketIce::Builder(fbb);
  ice_fb_builder.add_sdp_m_line_index(mline_index);
  ice_fb_builder.add_sdp_mid(fbb.CreateString("video0"));
  ice_fb_builder.add_candidate(
      fbb.CreateString(static_cast<char *>(candidate)));
  flatbuffers::Offset<WebSocketIce> ice_fb = ice_fb_builder.Finish();

  flatbuffers::Offset<WebSocketMessage> ice_message =
      CreateWebSocketMessage(fbb, Payload::WebSocketIce, ice_fb.Union());
  fbb.Finish(ice_message);

  server_->execute(std::make_shared<UpdateData>(sock_, fbb.Release()));

  g_signal_emit_by_name(webrtcbin_, "add-ice-candidate", mline_index,
                        candidate);
}

void Connection::HandleWebSocketData(const uint8_t *data, size_t /* size*/) {
  LOG(INFO) << "HandleWebSocketData";

  const WebSocketMessage *message =
      flatbuffers::GetRoot<WebSocketMessage>(data);

  switch (message->payload_type()) {
    case Payload::WebSocketSdp: {
      const WebSocketSdp *offer = message->payload_as_WebSocketSdp();
      if (offer->type() != SdpType::ANSWER) {
        LOG(WARNING) << "Expected SDP message type \"answer\"";
        break;
      }
      const flatbuffers::String *sdp_string = offer->payload();

      LOG(INFO) << "Received SDP:\n" << sdp_string->c_str();

      GstSDPMessage *sdp;
      GstSDPResult status = gst_sdp_message_new(&sdp);
      if (status != GST_SDP_OK) {
        LOG(WARNING) << "Could not create SDP message";
        break;
      }

      status = gst_sdp_message_parse_buffer((const guint8 *)sdp_string->c_str(),
                                            sdp_string->size(), sdp);

      if (status != GST_SDP_OK) {
        LOG(WARNING) << "Could not parse SDP string";
        break;
      }

      std::unique_ptr<GstWebRTCSessionDescription,
                      decltype(&gst_webrtc_session_description_free)>
          answer(gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_ANSWER,
                                                    sdp),
                 &gst_webrtc_session_description_free);
      std::unique_ptr<GstPromise, decltype(&gst_promise_unref)> promise(
          gst_promise_new(), &gst_promise_unref);
      g_signal_emit_by_name(webrtcbin_, "set-remote-description", answer.get(),
                            promise.get());
      gst_promise_interrupt(promise.get());

      break;
    }
    case Payload::WebSocketIce: {
      const WebSocketIce *ice = message->payload_as_WebSocketIce();
      if (!ice->has_candidate() || ice->candidate()->size() == 0) {
        LOG(WARNING) << "Received ICE message without candidate";
        break;
      }

      const gchar *candidate =
          static_cast<const gchar *>(ice->candidate()->c_str());
      guint mline_index = ice->sdp_m_line_index();

      LOG(INFO) << "Received ICE candidate with mline index " << mline_index
                << "; candidate: " << candidate;

      g_signal_emit_by_name(webrtcbin_, "add-ice-candidate", mline_index,
                            candidate);

      break;
    }
    default:
      break;
  }
}

// Tells GST that we want to use some plugins
void RegisterPlugins() {
  GST_PLUGIN_STATIC_REGISTER(app); // Relates to Connection's appsrc (probably)
  GST_PLUGIN_STATIC_REGISTER(coreelements);
  GST_PLUGIN_STATIC_REGISTER(dtls);
  GST_PLUGIN_STATIC_REGISTER(nice);
  GST_PLUGIN_STATIC_REGISTER(rtp);
  GST_PLUGIN_STATIC_REGISTER(rtpmanager);
  GST_PLUGIN_STATIC_REGISTER(srtp);
  GST_PLUGIN_STATIC_REGISTER(webrtc);
  GST_PLUGIN_STATIC_REGISTER(video4linux2);
  GST_PLUGIN_STATIC_REGISTER(videoconvert);
  GST_PLUGIN_STATIC_REGISTER(videoparsersbad);
  GST_PLUGIN_STATIC_REGISTER(videorate);
  GST_PLUGIN_STATIC_REGISTER(videoscale);
  GST_PLUGIN_STATIC_REGISTER(videotestsrc);
  GST_PLUGIN_STATIC_REGISTER(x264);
}

int main(int argc, char **argv) {
  // Connect to Google Chrome
  // This sets some flags maybe?
  aos::InitGoogle(&argc, &argv);

  // ? 
  findEmbeddedContent("");

  // Config for OpenSSL
  // OpenSSL is Open Secure socket Layer, a WebSocket connection library
  std::string openssl_env = "OPENSSL_CONF=\"\"";
  // Put something in  the environment variables?
  putenv(const_cast<char *>(openssl_env.c_str()));

  // Disable to registry
  putenv(const_cast<char *>("GST_REGISTRY_DISABLE=yes"));

  // Initiallize GST
  gst_init(&argc, &argv);
  // Register all the plugins
  RegisterPlugins();

  // Create a "buffer" from config data
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);
  
  // Create an event loop based on the config data
  // What does "Shm" mean?
  aos::ShmEventLoop event_loop(&config.message());

  {
    // Initialize the main loop
    aos::GlibMainLoop main_loop(&event_loop);

    // Create a seasocks Server
    seasocks::Server server(::std::shared_ptr<seasocks::Logger>(
        new ::aos::seasocks::SeasocksLogger(seasocks::Logger::Level::Info)));

    // Log info
    LOG(INFO) << "Serving from " << FLAGS_data_dir;

    // Create a websocket handle to the server
    auto websocket_handler =
        std::make_shared<WebsocketHandler>(&event_loop, &server);
    // Add the handler to the server
    server.addWebSocketHandler("/ws", websocket_handler);

    // Start listing from a specific port
    server.startListening(FLAGS_streaming_port);
    // ???
    server.setStaticPath(FLAGS_data_dir.c_str());

    // Poll for messages
    aos::internal::EPoll *epoll = event_loop.epoll();

    // Whenever there is a a readable message
    epoll->OnReadable(server.fd(), [&server] {
      // Do something?
      CHECK(::seasocks::Server::PollResult::Continue == server.poll(0));
    });

  // Run the event loop
    event_loop.Run();

    // Once the loop is over, delete the file descriptor
    epoll->DeleteFd(server.fd());
    // Terminate the server
    server.terminate();
  }

  // End GST
  gst_deinit();

  return 0;
}
