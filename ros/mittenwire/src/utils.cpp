// (c) 2023 Philipp Ruppel

#include <utils.hpp>

#include <log.hpp>
#include <thread>
#include <sstream>

namespace mittenwire {

void set_current_thread_name(const char* name) {
  std::stringstream strm;
  strm << std::hex << pthread_self();

  MTW_LOG_INFO("thread " << strm.str() << " " << name);
  pthread_setname_np(pthread_self(), name);
}

}  // namespace mittenwire