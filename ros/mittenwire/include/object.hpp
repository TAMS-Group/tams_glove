// (c) 2023-2024 Philipp Ruppel

#pragma once

#include <typeindex>
#include <thread>

namespace mittenwire {

class ObjectBaseImpl {
  bool _alive = false;
  std::type_index _type = typeid(void);
  std::thread::id _thread = std::this_thread::get_id();

 public:
  ObjectBaseImpl(const ObjectBaseImpl&) = delete;
  ObjectBaseImpl& operator=(const ObjectBaseImpl&) = delete;
  ObjectBaseImpl(const std::type_info& type);
  virtual ~ObjectBaseImpl();
  void checkAlive();
};

template <class Derived>
class Object : public ObjectBaseImpl {
 public:
  Object() : ObjectBaseImpl(typeid(Derived)) {}
};

}  // namespace mittenwire