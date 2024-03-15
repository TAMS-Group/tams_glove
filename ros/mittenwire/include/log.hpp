// (c) 2023-2024 Philipp Ruppel

#pragma once

#include <iostream>

#define MTW_LOG_DEBUG(...)  // std::cerr << "D " << __VA_ARGS__ << std::endl;
#define MTW_LOG_INFO(...) std::cerr << "I " << __VA_ARGS__ << std::endl;
#define MTW_LOG_ERROR(...) std::cerr << "E " << __VA_ARGS__ << std::endl;