// (c) 2023-2024 Philipp Ruppel

#include <object.hpp>

#include <log.hpp>

#include <iostream>

namespace mittenwire {

ObjectBaseImpl::ObjectBaseImpl(const std::type_info& type) : _type(type) {
  MTW_LOG_INFO("creating " << _type.name());
  _alive = true;
}

ObjectBaseImpl::~ObjectBaseImpl() {
  checkAlive();
  _alive = false;
  MTW_LOG_INFO("destroying " << _type.name() << " " << _thread << " "
                             << std::this_thread::get_id());
}

void ObjectBaseImpl::checkAlive() {
  if (!_alive) {
    MTW_LOG_ERROR("ERROR " << _type.name() << " already destroyed");
    throw std::runtime_error(std::string() + _type.name() +
                             " already destroyed");
  }
}

}  // namespace mittenwire