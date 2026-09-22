#include <array>
#include <asio.hpp> // NOLINT(misc-include-cleaner), provides asio::io_service
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <fmt/format.h> // NOLINT(misc-header-include-cycle), fmt self-include
#include <future>
#include <gtest/gtest.h>
#include <ifm3d/common/err.h>
#include <ifm3d/device/device.h>
#include <ifm3d/fg/buffer_id.h>
#include <ifm3d/fg/frame_grabber.h>
#include <ifm3d/fg/organizer.h>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

// ASIO exposes these APIs through its umbrella header, which the include
// cleaner cannot map to the individual declarations.
// NOLINTBEGIN(misc-include-cleaner)
namespace
{
  using namespace std::chrono_literals;

  class HeartbeatDevice : public ifm3d::Device
  {
  public:
    HeartbeatDevice() : Device("127.0.0.1") {}

    DeviceFamily
    WhoAmI() override
    {
      return DeviceFamily::O3D;
    }

    bool
    AmI(DeviceFamily family) override
    {
      return family == WhoAmI();
    }
  };

  class HeartbeatOrganizer : public ifm3d::Organizer
  {
  public:
    Result
    Organize(const std::vector<std::uint8_t>&,
             const std::set<ifm3d::buffer_id>&,
             bool) override
    {
      return {{}, {}, 0};
    }
  };

  class HeartbeatPeer
  {
  public:
    explicit HeartbeatPeer(std::string event_ticket = {},
                           std::string reply = "03 01 02 03")
      : _acceptor(
          _service,
          asio::ip::tcp::endpoint(asio::ip::address_v4::loopback(), 0)),
        _socket(_service),
        _event_ticket(std::move(event_ticket)),
        _reply(std::move(reply))
    {
      accept_connection();
      _thread = std::thread([this]() { _service.run(); });
    }

    ~HeartbeatPeer()
    {
      _service.stop();
      _thread.join();
    }

    HeartbeatPeer(const HeartbeatPeer&) = delete;
    HeartbeatPeer(HeartbeatPeer&&) = delete;
    HeartbeatPeer& operator=(const HeartbeatPeer&) = delete;
    HeartbeatPeer& operator=(HeartbeatPeer&&) = delete;

    [[nodiscard]] std::uint16_t
    Port() const
    {
      return _acceptor.local_endpoint().port();
    }

    void
    SetReplyEnabled(bool enabled)
    {
      _reply_enabled = enabled;
    }

    bool
    WaitForHeartbeats(std::size_t count,
                      std::chrono::milliseconds timeout = 2s)
    {
      std::unique_lock<std::mutex> lock(_mutex);
      return _changed.wait_for(lock, timeout, [this, count]() {
        return _heartbeat_times.size() >= count;
      });
    }

    std::vector<std::chrono::steady_clock::time_point>
    HeartbeatTimes()
    {
      const std::lock_guard<std::mutex> lock(_mutex);
      return _heartbeat_times;
    }

  private:
    static std::string
    packet(const std::string& ticket, const std::string& payload)
    {
      return fmt::format("{0}L{1:09}\r\n{0}{2}\r\n",
                         ticket,
                         payload.size() + 6,
                         payload);
    }

    void
    accept_connection()
    {
      _socket.close();
      _acceptor.async_accept(_socket, [this](const asio::error_code& error) {
        if (!error)
          {
            read_ticket();
          }
      });
    }

    void
    read_ticket()
    {
      asio::async_read(
        _socket,
        asio::buffer(_ticket),
        [this](const asio::error_code& error, std::size_t) {
          if (error)
            {
              accept_connection();
              return;
            }
          EXPECT_EQ(_ticket.at(4), 'L');
          EXPECT_EQ(std::string(_ticket.data() + 14, 2), "\r\n");
          _payload.resize(std::stoul(std::string(_ticket.data() + 5, 9)));
          asio::async_read(
            _socket,
            asio::buffer(_payload),
            [this](const asio::error_code& payload_error, std::size_t) {
              if (payload_error)
                {
                  accept_connection();
                  return;
                }
              handle_command();
              read_ticket();
            });
        });
    }

    void
    handle_command()
    {
      const std::string ticket(_ticket.data(), 4);
      EXPECT_EQ(std::string(_payload.data(), 4), ticket);
      EXPECT_EQ(std::string(_payload.end() - 2, _payload.end()), "\r\n");
      const std::string command(_payload.begin() + 4, _payload.end() - 2);
      std::string response;
      if (command == "V?")
        {
          bool first = false;
          {
            const std::lock_guard<std::mutex> lock(_mutex);
            first = _heartbeat_times.empty();
            _heartbeat_times.push_back(std::chrono::steady_clock::now());
          }
          _changed.notify_all();
          if (first && !_event_ticket.empty())
            {
              response += packet(
                _event_ticket,
                _event_ticket == "0000" ? "starstop" : "000000001:payload");
            }
          if (_reply_enabled)
            {
              response += packet(ticket, _reply);
            }
        }
      else
        {
          EXPECT_TRUE(command == "t" || command.front() == 'p');
          response = packet(ticket, "*");
        }
      asio::error_code error;
      asio::write(_socket, asio::buffer(response), error);
      EXPECT_FALSE(error) << error.message();
    }

    asio::io_service _service;
    asio::ip::tcp::acceptor _acceptor;
    asio::ip::tcp::socket _socket;
    std::array<char, 16> _ticket{};
    std::vector<char> _payload;
    std::string _event_ticket;
    std::string _reply;
    std::atomic<bool> _reply_enabled{true};
    std::mutex _mutex;
    std::condition_variable _changed;
    std::vector<std::chrono::steady_clock::time_point> _heartbeat_times;
    std::thread _thread;
  };

  TEST(FrameGrabberHeartbeat, DefaultsTo200msAndRejectsNegativeInterval)
  {
    HeartbeatPeer peer;
    ifm3d::FrameGrabber grabber(std::make_shared<HeartbeatDevice>(),
                                peer.Port());
    EXPECT_THROW(grabber.SetHeartbeatInterval(-1ms), std::invalid_argument);
    auto started = grabber.Start({});
    ASSERT_EQ(started.wait_for(2s), std::future_status::ready);
    ASSERT_NO_THROW(started.get());
    ASSERT_TRUE(peer.WaitForHeartbeats(3));
    const auto times = peer.HeartbeatTimes();
    EXPECT_GE(times.at(1) - times.at(0), 180ms);
    EXPECT_LT(times.at(1) - times.at(0), 1000ms);
    EXPECT_GE(times.at(2) - times.at(1), 180ms);
    EXPECT_LT(times.at(2) - times.at(1), 1000ms);
  }

  TEST(FrameGrabberHeartbeat, PeriodicRepliesDoNotInterfereWithTriggers)
  {
    HeartbeatPeer peer;
    std::atomic<int> errors{0};
    ifm3d::FrameGrabber grabber(std::make_shared<HeartbeatDevice>(),
                                peer.Port());
    grabber.OnError([&errors](const auto&) { ++errors; });
    grabber.SetHeartbeatInterval(100ms);
    auto started = grabber.Start({});
    ASSERT_EQ(started.wait_for(2s), std::future_status::ready);
    ASSERT_NO_THROW(started.get());
    ASSERT_TRUE(peer.WaitForHeartbeats(3));
    auto triggered = grabber.SWTrigger();
    ASSERT_EQ(triggered.wait_for(2s), std::future_status::ready);
    EXPECT_NO_THROW(triggered.get());
    const auto times = peer.HeartbeatTimes();
    EXPECT_GE(times.at(1) - times.at(0), 80ms);
    EXPECT_GE(times.at(2) - times.at(1), 80ms);
    ASSERT_EQ(grabber.Stop().wait_for(2s), std::future_status::ready);
    EXPECT_EQ(errors, 0);
  }

  TEST(FrameGrabberHeartbeat, CanChangeIntervalAndDisableWhileRunning)
  {
    HeartbeatPeer peer;
    ifm3d::FrameGrabber grabber(std::make_shared<HeartbeatDevice>(),
                                peer.Port());
    auto started = grabber.Start({});
    ASSERT_EQ(started.wait_for(2s), std::future_status::ready);
    ASSERT_NO_THROW(started.get());
    grabber.SetHeartbeatInterval(100ms);
    ASSERT_TRUE(peer.WaitForHeartbeats(1));
    grabber.SetHeartbeatInterval(300ms);
    EXPECT_FALSE(peer.WaitForHeartbeats(2, 150ms));
    ASSERT_TRUE(peer.WaitForHeartbeats(2));
    grabber.SetHeartbeatInterval(0ms);
    EXPECT_FALSE(peer.WaitForHeartbeats(3, 400ms));
    EXPECT_EQ(grabber.WaitForFrame().wait_for(0ms),
              std::future_status::timeout);
  }

  TEST(FrameGrabberHeartbeat, MissingReplyFailsWaitersAndCanRestart)
  {
    HeartbeatPeer peer;
    peer.SetReplyEnabled(false);
    std::promise<int> error;
    auto failed = error.get_future();
    ifm3d::FrameGrabber grabber(std::make_shared<HeartbeatDevice>(),
                                peer.Port());
    grabber.OnError(
      [&error](const auto& failure) { error.set_value(failure.code()); });
    grabber.SetHeartbeatInterval(100ms);
    auto started = grabber.Start({});
    ASSERT_EQ(started.wait_for(2s), std::future_status::ready);
    ASSERT_NO_THROW(started.get());
    auto frame = grabber.WaitForFrame();
    ASSERT_EQ(failed.wait_for(2s), std::future_status::ready);
    EXPECT_EQ(failed.get(), IFM3D_NETWORK_ERROR);
    ASSERT_EQ(frame.wait_for(2s), std::future_status::ready);
    EXPECT_THROW(frame.get(), ifm3d::Error);
    ASSERT_EQ(grabber.Stop().wait_for(2s), std::future_status::ready);
    EXPECT_THROW(grabber.WaitForFrame().get(), ifm3d::Error);

    peer.SetReplyEnabled(true);
    started = grabber.Start({});
    ASSERT_EQ(started.wait_for(2s), std::future_status::ready);
    ASSERT_NO_THROW(started.get());
    EXPECT_TRUE(peer.WaitForHeartbeats(3));
    ASSERT_EQ(grabber.Stop().wait_for(2s), std::future_status::ready);
  }

  class FrameGrabberHeartbeatCallbacks
    : public ::testing::TestWithParam<std::string>
  {
  };

  TEST_P(FrameGrabberHeartbeatCallbacks, SlowCallbackDoesNotCauseTimeout)
  {
    HeartbeatPeer peer(GetParam());
    std::atomic<int> errors{0};
    std::atomic<int> callbacks{0};
    ifm3d::FrameGrabber grabber(std::make_shared<HeartbeatDevice>(),
                                peer.Port());
    grabber.SetOrganizer(std::make_unique<HeartbeatOrganizer>());
    const auto slow_callback = [&callbacks](const auto&...) {
      ++callbacks;
      std::this_thread::sleep_for(300ms);
    };
    grabber.OnNewFrame(slow_callback);
    grabber.OnAsyncError(slow_callback);
    grabber.OnAsyncNotification(slow_callback);
    grabber.OnError([&errors](const auto&) { ++errors; });
    grabber.SetHeartbeatInterval(100ms);
    auto started = grabber.Start({});
    ASSERT_EQ(started.wait_for(2s), std::future_status::ready);
    ASSERT_NO_THROW(started.get());
    EXPECT_TRUE(peer.WaitForHeartbeats(3));
    ASSERT_EQ(grabber.Stop().wait_for(2s), std::future_status::ready);
    EXPECT_EQ(callbacks, 1);
    EXPECT_EQ(errors, 0);
  }

  TEST_P(FrameGrabberHeartbeatCallbacks,
         MissingReplyStillFailsAfterSlowCallback)
  {
    HeartbeatPeer peer(GetParam());
    peer.SetReplyEnabled(false);
    std::promise<int> error;
    auto failed = error.get_future();
    ifm3d::FrameGrabber grabber(std::make_shared<HeartbeatDevice>(),
                                peer.Port());
    grabber.SetOrganizer(std::make_unique<HeartbeatOrganizer>());
    const auto slow_callback = [](const auto&...) {
      std::this_thread::sleep_for(300ms);
    };
    grabber.OnNewFrame(slow_callback);
    grabber.OnAsyncError(slow_callback);
    grabber.OnAsyncNotification(slow_callback);
    grabber.OnError(
      [&error](const auto& failure) { error.set_value(failure.code()); });
    grabber.SetHeartbeatInterval(100ms);
    auto started = grabber.Start({});
    ASSERT_EQ(started.wait_for(2s), std::future_status::ready);
    ASSERT_NO_THROW(started.get());
    ASSERT_EQ(failed.wait_for(2s), std::future_status::ready);
    EXPECT_EQ(failed.get(), IFM3D_NETWORK_ERROR);
    ASSERT_EQ(grabber.Stop().wait_for(2s), std::future_status::ready);
    EXPECT_THROW(grabber.WaitForFrame().get(), ifm3d::Error);
  }

  INSTANTIATE_TEST_SUITE_P(Events,
                           FrameGrabberHeartbeatCallbacks,
                           ::testing::Values("0000", "0001", "0010"));

  class FrameGrabberHeartbeatReplies
    : public ::testing::TestWithParam<std::string>
  {
  };

  TEST_P(FrameGrabberHeartbeatReplies, RejectsMalformedVersionList)
  {
    const HeartbeatPeer peer({}, GetParam());
    std::promise<int> error;
    auto failed = error.get_future();
    ifm3d::FrameGrabber grabber(std::make_shared<HeartbeatDevice>(),
                                peer.Port());
    grabber.OnError(
      [&error](const auto& failure) { error.set_value(failure.code()); });
    grabber.SetHeartbeatInterval(50ms);
    auto started = grabber.Start({});
    ASSERT_EQ(started.wait_for(2s), std::future_status::ready);
    ASSERT_NO_THROW(started.get());
    ASSERT_EQ(failed.wait_for(2s), std::future_status::ready);
    EXPECT_EQ(failed.get(), IFM3D_PCIC_BAD_REPLY);
    ASSERT_EQ(grabber.Stop().wait_for(2s), std::future_status::ready);
  }

  INSTANTIATE_TEST_SUITE_P(
    Versions,
    FrameGrabberHeartbeatReplies,
    ::testing::Values("*", "?", "3 03", "03x03", "03 0A", "03 03 "));
}
// NOLINTEND(misc-include-cleaner)