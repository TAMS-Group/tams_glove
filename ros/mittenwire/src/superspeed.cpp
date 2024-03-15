// (c) 2023-2024 Philipp Ruppel

#include <superspeed.hpp>

#include <log.hpp>
#include <utils.hpp>

#include <libusb.h>
#include <string.h>
#include <atomic>
#include <iostream>
#include <poll.h>
#include <sys/eventfd.h>
#include <unistd.h>
#include <condition_variable>
#include <chrono>

#define V_USB_STR(x) #x

#define V_USB_E(status, error) \
  if (status == error) throw std::runtime_error(V_USB_STR(error))

namespace mittenwire {
namespace superspeed {

struct ContextImpl : Object<ContextImpl> {
  std::shared_ptr<libusb_context> usb_context;
  ContextImpl() {
    libusb_context *context = nullptr;
    if (LIBUSB_SUCCESS != libusb_init(&context)) {
      throw std::runtime_error("failed to init LibUSB");
    }
    usb_context =
        std::shared_ptr<libusb_context>(context, [](libusb_context *context) {
          MTW_LOG_INFO("libusb_exit begin " << context);
          libusb_exit(context);
          MTW_LOG_INFO("libusb_exit done");
        });
  }
  ~ContextImpl() {}
};

Context::Context() { _impl = std::make_shared<ContextImpl>(); }

void Context::close() { _impl.reset(); }

struct DeviceImpl : Object<DeviceImpl> {
  std::shared_ptr<ContextImpl> context;
  std::shared_ptr<libusb_context> usb_context;
  std::shared_ptr<libusb_device_handle> usb_device;
  std::atomic<size_t> counter;
  int completed = 0;
  int timeout = 2000;
  bool started = false;
  std::mutex mutex;

  DeviceImpl(const Context &context, uint16_t vid, uint16_t pid) {
    this->context = context._impl;

    usb_context = context._impl->usb_context;

    usb_device = std::shared_ptr<libusb_device_handle>(
        libusb_open_device_with_vid_pid(usb_context.get(), vid, pid),
        [](libusb_device_handle *handle) {
          MTW_LOG_INFO("libusb_close device_handle begin " << handle);
          libusb_close(handle);
          MTW_LOG_INFO("libusb_close device_handle done");
        });
    if (!usb_device) {
      throw std::runtime_error("failed to open usb device");
    }
  }

  ~DeviceImpl() { MTW_LOG_INFO("~DeviceImpl"); }
};

DeviceLock::DeviceLock(DeviceBase *device) {
  _impl = device->impl_or_null_unsafe();
  if (_impl) {
    _lock = std::unique_lock<std::mutex>(_impl->mutex);
  }
}

EventLoop::EventLoop(const std::shared_ptr<Context> &context,
                     const std::vector<std::shared_ptr<Device>> &devices)
    : _context(context), _devices(devices) {
  _event_fd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  if (_event_fd < 0) {
    throw std::runtime_error("failed to create eventfd");
  }
  auto *usb_context = context->_impl->usb_context.get();
  libusb_lock_events(usb_context);
  _thread = std::thread([this, context, usb_context]() {
    set_current_thread_name("event loop thread");
    auto **usb_pollfds = libusb_get_pollfds(usb_context);
    for (auto **p = usb_pollfds; *p; p++) {
      MTW_LOG_INFO("pollfd " << (*p)->fd << " " << (*p)->events);
    }
    std::vector<pollfd> pfd;
    bool ok = true;
    while (ok) {
      if (!libusb_event_handling_ok(usb_context)) {
        MTW_LOG_INFO("exit libusb");
        break;
      }
      pfd.clear();
      {
        pollfd d = {0};
        d.fd = _event_fd;
        d.events = POLLIN;
        pfd.push_back(d);
      }
      for (auto **p = usb_pollfds; *p; p++) {
        pollfd d = {0};
        d.fd = (*p)->fd;
        d.events = (*p)->events;
        pfd.push_back(d);
      }
      poll(pfd.data(), pfd.size(), 30 * 1000);
      bool act = false;
      for (auto &d : pfd) {
        if (d.revents != 0) {
          if (d.fd != _event_fd) {
            act = true;
          } else {
            uint64_t v = 0;
            ssize_t n = read(_event_fd, &v, sizeof(v));
            if (n == sizeof(v) && v > 0) {
              MTW_LOG_INFO("shutting down event loop");
              ok = false;
            }
          }
        }
      }
      if (act && ok) {
        timeval zero_tv = {0};
        libusb_handle_events_locked(usb_context, &zero_tv);
      }
    }
    MTW_LOG_INFO("event loop exit");
    libusb_unlock_events(usb_context);
    libusb_free_pollfds(usb_pollfds);
    MTW_LOG_INFO("event loop return");
  });
}

static void raise_event_loop_error() {
  throw std::runtime_error("event write failed");
}

EventLoop::~EventLoop() {
  {
    uint64_t v = 1;
    ssize_t n = write(_event_fd, &v, sizeof(v));
    if (n != sizeof(v)) {
      raise_event_loop_error();
    }
  }
  _thread.join();
  close(_event_fd);
}

std::vector<std::string> EncodedStringDescriptors::decode_array() const {
  constexpr size_t size = sizeof(data);
  static_assert(size == 128);
  constexpr int state_len = 0;
  constexpr int state_type = 1;
  constexpr int state_data = 2;
  int state = state_len;
  size_t len = 0;
  uint8_t type = 0;
  std::vector<std::string> ret;
  size_t ichr = 0;
  while (true) {
    if (ichr >= size) {
      break;
      throw std::runtime_error("Failed to decode string descriptors");
    }
    uint8_t chr = data[ichr];
    switch (state) {
      case state_len:
        len = chr - 2;
        if (len == 0) {
          break;
        }
        state = state_type;
        break;
      case state_type:
        type = chr;
        if (type != 3) {
          break;
        }
        ret.emplace_back();
        state = state_data;
        break;
      case state_data:
        if (len % 2 == 0) ret.back().push_back((char)chr);
        len--;
        if (len == 0) state = state_len;
        break;
    }
    ichr++;
  }
  return ret;
}

StringDescriptors EncodedStringDescriptors::decode() const {
  std::vector<std::string> array = decode_array();
  array.resize(3);
  StringDescriptors ret;
  ret.manufacturer = array.at(0);
  ret.product_description = array.at(1);
  ret.serial_number = array.at(2);
  return ret;
}

Device::Device() {}

void Device::open_vid_pid(const Context &context, uint16_t vid, uint16_t pid) {
  set_impl(std::make_shared<DeviceImpl>(context, vid, pid));
}

void Device::close() {
  MTW_LOG_INFO("device close begin");
  set_impl(nullptr);
  MTW_LOG_INFO("device close ready");
}

std::shared_ptr<DeviceImpl> DeviceBase::impl_or_null_unsafe() {
  std::shared_ptr<DeviceImpl> ret;
  {
    std::unique_lock<std::mutex> lock(__impl_mutex);
    ret = __impl_ptr;
  }
  return ret;
}

void Device::reset() {
  MTW_LOG_DEBUG("a");
  DeviceLock _impl(this);
  MTW_LOG_DEBUG("b");
  libusb_reset_device(_impl->usb_device.get());
  MTW_LOG_DEBUG("c");
}

Config Device::read_config() {
  DeviceLock _impl(this);
  Config config;
  auto ret = libusb_control_transfer(
      _impl->usb_device.get(),
      LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_IN,
      0xcf, 1, 0, (unsigned char *)&config, sizeof(config), _impl->timeout);
  if (ret != sizeof(config)) {
    throw std::runtime_error("failed to read config");
  }
  return config;
}

void Device::write_config(const Config &config) {
  DeviceLock _impl(this);
  auto ret = libusb_control_transfer(
      _impl->usb_device.get(),
      LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE |
          LIBUSB_ENDPOINT_OUT,
      0xcf, 0, 0, (unsigned char *)&config, sizeof(config), _impl->timeout);
  if (ret != sizeof(config)) {
    throw std::runtime_error("failed to write config");
  }
}

static uint16_t encode_gpios(uint8_t mask, uint8_t value) {
  uint16_t encoded = 0;
  for (size_t i = 0; i < 2; i++) {
    if ((mask >> i) & 1) {
      encoded |= (((value >> i) & 1) << (i * 3));
      encoded |= (4 << (i * 3));
    }
  }
  return encoded;
}

void DeviceBase::set_impl(const std::shared_ptr<DeviceImpl> &impl) {
  std::shared_ptr<DeviceImpl> prev_impl;
  {
    std::unique_lock<std::mutex> lock(__impl_mutex);
    prev_impl = __impl_ptr;
    __impl_ptr = impl;
  }
}

void Device::set_gpio_directions(uint8_t mask, uint8_t direction) {
  DeviceLock _impl(this);
  uint16_t encoded = encode_gpios(mask, direction);
  auto ret = libusb_control_transfer(_impl->usb_device.get(), 0x40, 2, 2, 0,
                                     (uint8_t *)&encoded, 2, _impl->timeout);
  if (ret != 2) {
    throw std::runtime_error("usb transfer failed");
  }
}

void Device::set_gpio_levels(uint8_t mask, uint8_t value) {
  DeviceLock _impl(this);
  uint16_t encoded = encode_gpios(mask, value);
  auto ret = libusb_control_transfer(_impl->usb_device.get(), 0x40, 2, 1, 0,
                                     (uint8_t *)&encoded, 2, _impl->timeout);
  if (ret != 2) {
    throw std::runtime_error("usb transfer failed");
  }
}

void Device::set_gpio_direction(int pin, int direction) {
  direction = ((direction != 0) ? 1 : 0);
  set_gpio_directions(1 << pin, direction << pin);
}

void Device::set_gpio_level(int pin, int level) {
  level = ((level != 0) ? 1 : 0);
  set_gpio_levels(1 << pin, level << pin);
}

void Device::start() {
  DeviceLock _impl(this);
  if (!_impl->started) {
    if (LIBUSB_SUCCESS != libusb_claim_interface(_impl->usb_device.get(), 0)) {
      throw std::runtime_error("failed to claim usb interface 0");
    }
    if (LIBUSB_SUCCESS != libusb_claim_interface(_impl->usb_device.get(), 1)) {
      throw std::runtime_error("failed to claim usb interface 1");
    }
    {
      std::vector<uint8_t> req = {0x00, 0x00, 0x00, 0x00, 0x82, 0x02, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      int written = 0;
      int ret = libusb_bulk_transfer(_impl->usb_device.get(), 0x01, req.data(),
                                     req.size(), &written, _impl->timeout);
      if (ret != LIBUSB_SUCCESS || written != req.size()) {
        throw std::runtime_error("failed to init streaming mode");
      }
    }
    _impl->started = true;
  }
}

void Device::write(const void *data, size_t size) {
  DeviceLock _impl(this);
  int transferred = 0;
  int status =
      libusb_bulk_transfer(_impl->usb_device.get(), 0x02, (uint8_t *)data, size,
                           &transferred, _impl->timeout);
  if (status != LIBUSB_SUCCESS) {
    throw std::runtime_error("write failed with code " +
                             std::to_string(status));
  }
  if (transferred != size) {
    throw std::runtime_error("write incomplete");
  }
}

struct ReaderImpl : Object<ReaderImpl> {
  std::shared_ptr<Device> device;
  std::function<void(const void *, size_t)> callback;

  std::vector<std::vector<uint8_t>> buffers;

  std::vector<std::shared_ptr<libusb_transfer>> transfers;

  size_t count_value = 0;
  std::mutex count_mutex;
  std::condition_variable count_condition;

  size_t callback_running = 0;

  volatile bool exit_flag = false;

  void note_transfer_canceled() {
    std::unique_lock<std::mutex> lock(count_mutex);
    count_value--;
    count_condition.notify_all();
  }

  ReaderImpl(const std::shared_ptr<Device> &device,
             const std::function<void(const void *, size_t)> &callback,
             size_t buffer_count, size_t buffer_size)
      : device(device), callback(callback) {
    buffers.resize(buffer_count);
    for (auto &buffer : buffers) buffer.resize(buffer_size, 0);

    auto impl = device->impl_or_null_unsafe();
    if (!impl) {
      throw std::runtime_error("reader device impl null");
    }

    for (size_t i = 0; i < buffer_count; i++) {
      auto *transfer = libusb_alloc_transfer(0);
      if (!transfer) {
        throw std::runtime_error("reader libusb_alloc_transfer failed");
      }
      transfers.emplace_back(transfer, [](libusb_transfer *transfer) {
        libusb_free_transfer(transfer);
      });
      libusb_fill_bulk_transfer(
          transfer, impl->usb_device.get(), 0x82, buffers.at(i).data(),
          buffer_size,
          [](libusb_transfer *transfer) {
            auto *thiz = (ReaderImpl *)transfer->user_data;
            switch (transfer->status) {
              case LIBUSB_TRANSFER_COMPLETED:
                thiz->callback_running++;
                thiz->callback(transfer->buffer, transfer->actual_length);
                thiz->callback_running--;
              case LIBUSB_TRANSFER_TIMED_OUT:
                if (thiz->exit_flag ||
                    (0 != libusb_submit_transfer(transfer))) {
                  thiz->note_transfer_canceled();
                }
                break;
              default:
                thiz->note_transfer_canceled();
                break;
            }
          },
          this, 5000);
    }

    MTW_LOG_INFO("reader starting usb transfers");
    for (auto &transfer : transfers) {
      MTW_LOG_INFO("reader libusb_submit_transfer");
      auto ret = libusb_submit_transfer(transfer.get());
      if (ret != 0) {
        throw std::runtime_error("reader failed to submit transfer");
      }
      {
        std::unique_lock<std::mutex> lock(count_mutex);
        count_value++;
      }
    }
    MTW_LOG_INFO("reader usb transfers started");
  }
  void print_shutdown_status() {
    MTW_LOG_INFO("waiting for " << count_value << " transfers "
                                << callback_running << " callbacks ");
  }
  ~ReaderImpl() {
    print_shutdown_status();

    MTW_LOG_INFO("reader canceling transfers");
    exit_flag = true;
    for (auto &t : transfers) {
      libusb_cancel_transfer(t.get());
    }
    MTW_LOG_INFO("reader transfers canceled");

    MTW_LOG_INFO("reader waiting for completion");
    {
      std::unique_lock<std::mutex> lock(count_mutex);
      while (count_value > 0) {
        print_shutdown_status();
        count_condition.wait_for(lock, std::chrono::duration<double>(0.5));
      }
    }
    print_shutdown_status();
    MTW_LOG_INFO("reader completed");

    MTW_LOG_INFO("reader freeing transfers");
    transfers.clear();
    MTW_LOG_INFO("reader transfers freed");
  }
};

Reader::Reader(const std::shared_ptr<Device> &device,
               const std::function<void(const void *, size_t)> &callback,
               size_t buffer_count, size_t buffer_size) {
  _impl =
      std::make_shared<ReaderImpl>(device, callback, buffer_count, buffer_size);
}

void Device::write_async(const std::string &data) {
  auto transfer = std::shared_ptr<libusb_transfer>(
      libusb_alloc_transfer(0), [](libusb_transfer *transfer) {
        if (transfer) {
          libusb_free_transfer(transfer);
        }
      });
  if (!transfer) {
    throw std::runtime_error("failed to alloc transfer");
  }

  auto impl = impl_or_null_unsafe();
  if (!impl) {
    throw std::runtime_error("reader device impl null");
  }

  struct Context {
    bool finished = false;
    std::mutex mutex;
    std::condition_variable condition;
  };
  Context context;

  libusb_fill_bulk_transfer(
      transfer.get(), impl->usb_device.get(), 0x02,
      (unsigned char *)data.data(), data.size(),
      [](libusb_transfer *transfer) {
        auto *context = (Context *)transfer->user_data;
        std::unique_lock<std::mutex> lock(context->mutex);
        context->finished = true;
        context->condition.notify_all();
      },
      &context, 5000);

  {
    int ret = libusb_submit_transfer(transfer.get());
    if (ret != 0) {
      throw std::runtime_error("reader failed to submit transfer");
    }
  }

  {
    std::unique_lock<std::mutex> lock(context.mutex);
    while (!context.finished) {
      context.condition.wait(lock);
    }
  }

  transfer.reset();
}

std::vector<uint8_t> Device::read(ssize_t count, int timeout) {
  DeviceLock _impl(this);
  std::vector<uint8_t> ret;
  if (count > 0) {
    ret.resize(count);
  } else {
    ret.resize(32 * 1024);
  }
  int transferred = 0;
  if (timeout < 0) timeout = _impl->timeout;
  int status = libusb_bulk_transfer(_impl->usb_device.get(), 0x82, ret.data(),
                                    ret.size(), &transferred, timeout);

  V_USB_E(status, LIBUSB_ERROR_INVALID_PARAM);
  V_USB_E(status, LIBUSB_ERROR_NO_DEVICE);
  V_USB_E(status, LIBUSB_ERROR_PIPE);
  V_USB_E(status, LIBUSB_ERROR_OVERFLOW);
  V_USB_E(status, LIBUSB_ERROR_BUSY);

  ret.resize(transferred);

  return ret;
}

void Device::write(const std::vector<uint8_t> &data) {
  write(data.data(), data.size());
}

void Device::write(const std::vector<int8_t> &data) {
  write(data.data(), data.size());
}

void Device::write(const std::string &data) { write(data.data(), data.size()); }

Writer::Writer(const std::shared_ptr<Device> &device,
               const std::function<std::string()> &callback, double interval)
    : _device(device), _callback(callback) {
  _thread = std::thread([this, interval]() {
    set_current_thread_name("usb writer thread");
    auto last_time = std::chrono::steady_clock::now();
    while (true) {
      {
        std::unique_lock<std::mutex> lock(_mutex);
        while (true) {
          if (!_ok) {
            return;
          }
          if (_new || std::chrono::steady_clock::now() >=
                          last_time + std::chrono::duration<double>(interval)) {
            _new = false;
            break;
          }
          _condition.wait_until(lock, last_time + std::chrono::duration<double>(
                                                      std::min(0.1, interval)));
        }
      }
      MTW_LOG_INFO("begin writer callback");
      auto message = _callback();
      MTW_LOG_INFO("writer callback ended");
      if (!message.empty()) {
        _device->write_async(message);
      }
      MTW_LOG_INFO("device write ended");
      last_time = std::chrono::steady_clock::now();
    }
  });
}

Writer::~Writer() {
  MTW_LOG_INFO("destroying writer");
  {
    std::unique_lock<std::mutex> lock(_mutex);
    _ok = false;
    _condition.notify_all();
  }
  _thread.join();
  MTW_LOG_INFO("writer destroyed");
}

void Writer::update() {
  std::unique_lock<std::mutex> lock(_mutex);
  _new = true;
  _condition.notify_all();
}

}  // namespace superspeed
}  // namespace mittenwire