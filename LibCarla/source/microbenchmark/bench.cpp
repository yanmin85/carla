#include <benchmark/benchmark.h>

#include <carla/streaming/Client.h>
#include <carla/streaming/Server.h>

static void WriteToStream(benchmark::State& state) {
  carla::streaming::Server srv("localhost", 2345);
  srv.AsyncRun(2u);
  auto stream = srv.MakeStream();
  carla::streaming::Client client;
  client.AsyncRun(2u);
  client.Subscribe(stream.token(), [](auto){});
  const unsigned char c = 'a';
  for (auto _ : state) {
    stream.Write(carla::Buffer(&c, sizeof(c)));
  }
}
BENCHMARK(WriteToStream);

static void WriteToMultiStream(benchmark::State& state) {
  carla::streaming::Server srv("localhost", 2345);
  srv.AsyncRun(2u);
  auto stream = srv.MakeMultiStream();
  carla::streaming::Client c1;
  c1.AsyncRun(2u);
  c1.Subscribe(stream.token(), [](auto){});
  carla::streaming::Client c2;
  c2.AsyncRun(2u);
  c2.Subscribe(stream.token(), [](auto){});
  carla::streaming::Client c3;
  c3.AsyncRun(2u);
  c3.Subscribe(stream.token(), [](auto){});
  const unsigned char c = 'a';
  for (auto _ : state) {
    stream.Write(carla::Buffer(&c, sizeof(c)));
  }
}
BENCHMARK(WriteToMultiStream);

#ifdef LIBCARLA_NO_EXCEPTIONS

#include <carla/Exception.h>
#include <carla/Logging.h>

#include <exception>

namespace carla {

  void throw_exception(const std::exception &e) {
    log_critical("carla::throw_exception:", e.what());
    log_critical("calling std::terminate because exceptions are disabled.");
    std::terminate();
  }

} // namespace carla

#endif // LIBCARLA_NO_EXCEPTIONS
