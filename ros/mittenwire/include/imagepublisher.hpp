// (c) 2023-2024 Philipp Ruppel

#pragma once

#include "camera.hpp"
#include "object.hpp"

#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <condition_variable>
#include <mutex>
#include <thread>

namespace mittenwire {}  // namespace mittenwire