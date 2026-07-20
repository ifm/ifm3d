#include "../src/xmlrpc.hpp"

#include <chrono>
#include <cstdint>
#include <gtest/gtest.h>
#include <httplib.h>
#include <ifm3d/common/err.h>
#include <string>
#include <thread>

TEST(XMLRPCTest, UsesProvidedTimeout)
{
  httplib::Server server;
  server.Post(ifm3d::XMLRPC_MAIN,
              [](const httplib::Request&, httplib::Response& response) {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                response.set_content(
                  "<?xml version=\"1.0\"?>"
                  "<methodResponse><params><param><value><string>ok</string>"
                  "</value></param></params></methodResponse>",
                  "text/xml");
              });

  int const port = server.bind_to_any_port("127.0.0.1");
  ASSERT_GT(port, 0);

  std::thread server_thread([&server]() { server.listen_after_bind(); });
  server.wait_until_ready();

  ifm3d::XMLRPC client("127.0.0.1", static_cast<std::uint16_t>(port));
  try
    {
      client.XCallMainTimeout("test", 50);
      ADD_FAILURE() << "Expected the XML-RPC request to time out";
    }
  catch (const ifm3d::Error& error)
    {
      EXPECT_EQ(error.code(), IFM3D_CURL_TIMEOUT);
      EXPECT_STREQ(error.message(),
                   "Timed out after 50 ms while waiting for the response to "
                   "XML-RPC method 'test'.");
    }

  server.stop();
  server_thread.join();
}
