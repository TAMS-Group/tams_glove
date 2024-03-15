// (c) 2023-2024 Philipp Ruppel

#include <camera.hpp>
#include <messages.hpp>
#include <imagepublisher.hpp>
#include <master.hpp>
#include <packet.hpp>
#include <hub.hpp>
#include <log.hpp>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/photo/cuda.hpp>

namespace py = pybind11;

namespace mittenwire {

namespace superspeed {
void init_python(py::module &m) {
  py::enum_<FIFOClock>(m, "FIFOClock")
      .value("Clock100MHz", FIFOClock::Clock100MHz)
      .value("Clock66MHz", FIFOClock::Clock66MHz)
      .value("Clock50MHz", FIFOClock::Clock50MHz)
      .value("Clock40MHz", FIFOClock::Clock40MHz);

  py::enum_<FIFOMode>(m, "FIFOMode")
      .value("Mode245", FIFOMode::Mode245)
      .value("Mode600", FIFOMode::Mode600);

  py::enum_<ChannelConfig>(m, "ChannelConfig")
      .value("QuadChannel", ChannelConfig::QuadChannel)
      .value("DoubleChannel", ChannelConfig::DoubleChannel)
      .value("SingleChannel", ChannelConfig::SingleChannel)
      .value("SingleChannelOutPipe", ChannelConfig::SingleChannelOutPipe)
      .value("SingleChannelInPipe", ChannelConfig::SingleChannelInPipe);

  py::class_<StringDescriptors>(m, "StringDescriptors")
      .def(py::init<>())
      .def_readwrite("manufacturer", &StringDescriptors::manufacturer)
      .def_readwrite("product_description",
                     &StringDescriptors::product_description)
      .def_readwrite("serial_number", &StringDescriptors::serial_number);

  py::class_<Config>(m, "Config")
      .def(py::init<>())
      .def_readwrite("vendor_id", &Config::vendor_id)
      .def_readwrite("product_id", &Config::product_id)
      .def_property_readonly(
          "string_descriptors",
          [](const Config &cfg) { return cfg.string_descriptors.decode(); })
      .def_readwrite("power_attributes", &Config::power_attributes)
      .def_readwrite("power_consumption", &Config::power_consumption)
      .def_readwrite("fifo_clock", &Config::fifo_clock)
      .def_readwrite("fifo_mode", &Config::fifo_mode)
      .def_readwrite("channel_config", &Config::channel_config)
      .def_readwrite("optional_feature_support",
                     &Config::optional_feature_support)
      .def_readwrite("battery_charging_gpio_config",
                     &Config::battery_charging_gpio_config)
      .def_readwrite("flash_eeprom_detection", &Config::flash_eeprom_detection)
      .def_readwrite("msio_control", &Config::msio_control)
      .def_readwrite("gpio_control", &Config::gpio_control);

  py::class_<Context, std::shared_ptr<Context>>(m, "Context").def(py::init<>());

  py::class_<EventLoop>(m, "EventLoop")
      .def(py::init<const std::shared_ptr<Context> &,
                    const std::vector<std::shared_ptr<Device>> &>());

  py::class_<Writer>(m, "Writer")
      .def(py::init<const std::shared_ptr<Device> &,
                    const std::function<std::string()> &, double>())
      .def("update", &Writer::update);

  py::class_<Device, std::shared_ptr<Device>>(m, "Device")
      .def(py::init<>())
      .def("open_vid_pid", &Device::open_vid_pid)
      .def("close", &Device::close)
      .def("reset", &Device::reset)
      .def("read_config", &Device::read_config)
      .def("write_config", &Device::write_config)
      .def("set_gpio_direction", &Device::set_gpio_direction)
      .def("set_gpio_level", &Device::set_gpio_level)
      .def("set_gpio_directions", &Device::set_gpio_directions)
      .def("set_gpio_levels", &Device::set_gpio_levels)
      .def("start", &Device::start)
      .def("read",
           [](Device *me) {
             auto ret = me->read();
             return py::bytearray((const char *)ret.data(), ret.size());
           })
      .def("read",
           [](Device *me, ssize_t count) {
             auto ret = me->read(count);
             return py::bytearray((const char *)ret.data(), ret.size());
           })
      .def("test_read", [](Device *me) { return me->read().size(); })
      .def("write",
           [](Device *me, const std::string &data) { me->write(data); })
      .def("write_async", &Device::write_async);
}
}  // namespace superspeed

void init_python(py::module &m) {
  superspeed::init_python(m);

  struct AsyncSpinner : ros::AsyncSpinner {
    AsyncSpinner() : ros::AsyncSpinner(0) {}
  };

  py::class_<AsyncSpinner>(m, "AsyncSpinner")
      .def(py::init<>())
      .def("start", &AsyncSpinner::start);

  m.def("ros_ok", []() { return ros::ok(); });

  m.def("ros_init",
        [](const std::vector<std::string> &argv, const std::string &name) {
          std::vector<std::vector<char>> argv_vec(argv.size());
          std::vector<char *> argv_ptr(argv.size());
          for (size_t i = 0; i < argv.size(); i++) {
            argv_vec[i].assign(argv[i].begin(), argv[i].end());
            argv_vec[i].push_back(0);
            argv_ptr[i] = argv_vec[i].data();
          }
          int argc = argv_ptr.size();
          ros::init(argc, argv_ptr.data(), name);
        });

  m.def("ros_spin", []() { ros::spin(); });

  m.def("denoise", [](py::array_t<uint8_t> &img) {
    auto proxy = img.mutable_unchecked();
    cv::Mat mat(img.shape(0), img.shape(1), CV_8UC3,
                (unsigned char *)proxy.data());
    cv::cuda::fastNlMeansDenoisingColored(mat, mat, 2.0, 2.0);
  });

  py::class_<Packet>(m, "Packet")
      .def(py::init<>())
      .def_readonly("flags", &Packet::flags)
      .def_property_readonly("data",
                             [](const Packet *msg) {
                               std::vector<uint8_t> ret;
                               ret.resize(msg->data.size());
                               memcpy(ret.data(), msg->data.data(),
                                      msg->data.size());
                               return ret;
                             })
      .def_readonly("channel", &Packet::channel)
      .def_property_readonly("str", &Packet::str);

  py::class_<Master, std::shared_ptr<Master>>(m, "Master")
      .def(py::init<const std::shared_ptr<superspeed::Device> &, size_t,
                    size_t>());

  py::class_<Hub>(m, "Hub")
      .def(py::init<const std::shared_ptr<Master>>())
      .def("connect", &Hub::connect)
      .def("disconnect", &Hub::disconnect);

  py::class_<Node, std::shared_ptr<Node>>(m, "Node").def(
      py::init([](const std::function<void(const Packet &)> &callback) {
        struct LambdaNode : Node {
          std::function<void(const Packet &)> callback;
          LambdaNode(const std::function<void(const Packet &)> &callback)
              : callback(callback) {}
          virtual void process(const Packet &packet) override {
            callback(packet);
          };
        };
        return std::make_shared<LambdaNode>(callback);
      }));

  py::class_<Camera, std::shared_ptr<Camera>, Node>(m, "Camera")
      .def(py::init<const std::function<void(const ImageMessage &)> &>());

  py::class_<Message>(m, "Message").def_readonly("channel", &Message::channel);

  py::class_<ImageMessage, Message>(m, "ImageMessage")
      .def(py::init<>())
      .def_readonly("left", &ImageMessage::left)
      .def_readonly("top", &ImageMessage::top)
      .def_readonly("width", &ImageMessage::width)
      .def_readonly("height", &ImageMessage::height)
      .def_readonly("temperature", &ImageMessage::temperature)
      .def_readonly("valid", &ImageMessage::valid)
      .def_readonly("skip", &ImageMessage::skip)
      .def_readonly("request_timestamp", &ImageMessage::request_timestamp)
      .def_property_readonly("data", [](const ImageMessage *thiz) {
        auto arr = py::array_t<uint8_t>({thiz->height, thiz->width});
        auto proxy = arr.mutable_unchecked();
        for (size_t y = 0; y < thiz->height; y++) {
          memcpy(proxy.mutable_data(y, 0), thiz->data.data() + y * thiz->width,
                 thiz->width);
        }
        return arr;
      });

  py::class_<ros::NodeHandle>(m, "NodeHandle")
      .def(py::init([](const std::string &ns) { return ros::NodeHandle(ns); }));

#define PYFIELDSTR(name) #name

#define PYFIELD(type, name)                                   \
  def_property(                                               \
      PYFIELDSTR(name), [](const type &v) { return v.name; }, \
      [](type &v, const decltype(type::name) &x) { v.name = x; })

  py::class_<CameraMessage>(m, "CameraMessage")

      .def(py::init([]() { return CameraMessage{0}; }))

      .PYFIELD(CameraMessage, magic)

      .PYFIELD(CameraMessage, delay)

      .PYFIELD(CameraMessage, left)
      .PYFIELD(CameraMessage, top)

      .PYFIELD(CameraMessage, width)
      .PYFIELD(CameraMessage, height)

      .PYFIELD(CameraMessage, shutter)
      .PYFIELD(CameraMessage, binning)
      .PYFIELD(CameraMessage, blacklevel)

      .PYFIELD(CameraMessage, analog_gain_red)
      .PYFIELD(CameraMessage, analog_gain_green)
      .PYFIELD(CameraMessage, analog_gain_blue)
      .PYFIELD(CameraMessage, digital_gain)
      .PYFIELD(CameraMessage, double_gain)

      .PYFIELD(CameraMessage, timestamp)

      .PYFIELD(CameraMessage, skip)

      .def("pack",
           [](const CameraMessage &m) {
             return py::bytearray((const char *)&m, sizeof(m));
           })

      ;

  py::class_<HubMessage>(m, "HubMessage")

      .def(py::init([]() { return HubMessage{0}; }))

      .PYFIELD(HubMessage, frametime)
      .PYFIELD(HubMessage, port)
      .PYFIELD(HubMessage, type)

      .def("pack",
           [](const HubMessage &m) {
             return py::bytearray((const char *)&m, sizeof(m));
           })

      ;

  m.def("pack_sample", [](int32_t i, int32_t q) {
    uint32_t exp = 0;
    while ((((i & 0xC0000000) == 0xC0000000) || ((i & 0xC0000000) == 0)) &&
           (((q & 0xC0000000) == 0xC0000000) || ((q & 0xC0000000) == 0))) {
      i <<= 1;
      q <<= 1;
      exp++;
    }
    return exp | (uint32_t(i & 0xfff00000)) | (uint32_t(q & 0xfff00000) >> 12);
  });

  m.def("unpack_sample", [](uint32_t v) {
    uint32_t exp = (v & 0xff);
    int32_t i = (v & 0xfff00000);
    int32_t q = ((v << 12) & 0xfff00000);
    return std::make_pair(i >> exp, q >> exp);
  });
}

}  // namespace mittenwire

PYBIND11_MODULE(pymittenwire, m) { mittenwire::init_python(m); }
