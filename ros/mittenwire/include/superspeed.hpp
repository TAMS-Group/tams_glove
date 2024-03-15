// (c) 2023-2024 Philipp Ruppel

#pragma once

#include "object.hpp"

#include <stdint.h>

#include <functional>
#include <memory>
#include <mutex>
#include <vector>
#include <set>
#include <condition_variable>

namespace mittenwire {

namespace superspeed {

enum class FIFOClock : uint8_t {
  Clock100MHz,
  Clock66MHz,
  Clock50MHz,
  Clock40MHz,
};

enum class FIFOMode : uint8_t {
  Mode245,
  Mode600,
};

enum class ChannelConfig : uint8_t {
  QuadChannel,
  DoubleChannel,
  SingleChannel,
  SingleChannelOutPipe,
  SingleChannelInPipe,
};

struct StringDescriptors {
  std::string manufacturer, product_description, serial_number;
};

struct alignas(1) EncodedStringDescriptors {
  uint8_t data[128] = {0};
  std::vector<std::string> decode_array() const;
  StringDescriptors decode() const;
};

struct Config {
  uint16_t vendor_id = 0;
  uint16_t product_id = 0;
  EncodedStringDescriptors string_descriptors;

  uint8_t reserved_1 = 0;
  uint8_t power_attributes = 0xE0;
  uint16_t power_consumption = 0x60;

  uint8_t reserved_2 = 0;
  FIFOClock fifo_clock = FIFOClock::Clock100MHz;
  FIFOMode fifo_mode = FIFOMode::Mode600;
  ChannelConfig channel_config = ChannelConfig::QuadChannel;

  uint16_t optional_feature_support = 0x0;
  uint8_t battery_charging_gpio_config = 0xE4;
  uint8_t flash_eeprom_detection = 0;

  uint32_t msio_control = 0x10800;
  uint32_t gpio_control = 0x0;
};

class ContextImpl;
class EventLoop;
class Context : public Object<Context> {
  std::shared_ptr<ContextImpl> _impl;

 public:
  Context();
  void close();
  friend class DeviceImpl;
  friend class EventLoop;
};

class DeviceImpl;

class DeviceBase : public Object<DeviceBase> {
 private:
  std::mutex __impl_mutex;
  std::shared_ptr<DeviceImpl> __impl_ptr;

 protected:
  void set_impl(const std::shared_ptr<DeviceImpl> &impl);

 public:
  std::shared_ptr<DeviceImpl> impl_or_null_unsafe();
  friend class Reader;
};

class DeviceLock {
  std::shared_ptr<DeviceImpl> _impl;
  std::unique_lock<std::mutex> _lock;

 public:
  DeviceLock(DeviceBase *device);
  DeviceImpl *operator->() {
    if (auto *ptr = _impl.get()) {
      return ptr;
    } else {
      throw std::runtime_error("device already deleted");
    }
  }
  operator bool() { return bool(_impl); }
};

class Device : public DeviceBase {
 public:
  Device(const Device &) = delete;
  Device &operator=(const Device &) = delete;
  Device();
  void open_vid_pid(const Context &context, uint16_t vid, uint16_t pid);
  void close();
  void start();
  Config read_config();
  void write_config(const Config &config);
  void set_gpio_directions(uint8_t mask, uint8_t direction);
  void set_gpio_levels(uint8_t mask, uint8_t value);
  void set_gpio_direction(int pin, int direction);
  void set_gpio_level(int pin, int level);
  void write(const void *data, size_t size);
  void write(const std::vector<uint8_t> &data);
  void write(const std::vector<int8_t> &data);
  void write(const std::string &data);
  std::vector<uint8_t> read(ssize_t count = -1, int timeout = -1);
  void reset();
  void write_async(const std::string &data);
  friend class Reader;
};

class EventLoop : public Object<EventLoop> {
  std::shared_ptr<Context> _context;
  std::vector<std::shared_ptr<Device>> _devices;
  std::thread _thread;
  int _event_fd = -1;

 public:
  EventLoop(const std::shared_ptr<Context> &context,
            const std::vector<std::shared_ptr<Device>> &devices);
  ~EventLoop();
};

struct ReaderImpl;

class Reader : public Object<Reader> {
  std::shared_ptr<ReaderImpl> _impl;

 public:
  Reader(const std::shared_ptr<Device> &device,
         const std::function<void(const void *, size_t)> &callback,
         size_t buffer_count, size_t buffer_size);
};

class Writer : public Object<Writer> {
  std::shared_ptr<Device> _device;
  std::thread _thread;
  std::mutex _mutex;
  std::condition_variable _condition;
  bool _ok = true;
  bool _new = false;
  std::function<std::string()> _callback;

 public:
  Writer(const std::shared_ptr<Device> &device,
         const std::function<std::string()> &callback, double interval);
  ~Writer();
  void update();
};

}  // namespace superspeed

}  // namespace mittenwire