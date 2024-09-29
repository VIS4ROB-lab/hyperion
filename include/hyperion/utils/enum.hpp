/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <boost/container/flat_map.hpp>
#include <boost/preprocessor.hpp>

#define DEFINE_SERIALIZABLE_ENUM_WRITE_CASE(macro, name, element) \
  case name::element: return BOOST_PP_STRINGIZE(element);

#define DEFINE_SERIALIZABLE_ENUM_READ_CASE(macro, name, element) {BOOST_PP_STRINGIZE(element), name::element},

#define DEFINE_SERIALIZABLE_ENUM(name, elements)                                                           \
  enum class name { BOOST_PP_SEQ_ENUM(elements) };                                                         \
                                                                                                           \
  static auto name##ToString(const name& e)->const char* {                                                 \
    switch (e) {                                                                                           \
      BOOST_PP_SEQ_FOR_EACH(DEFINE_SERIALIZABLE_ENUM_WRITE_CASE, name, elements)                           \
      default: return "UNKNOWN";                                                                           \
    }                                                                                                      \
  }                                                                                                        \
                                                                                                           \
  static auto name##FromString(const std::string& str)->name {                                             \
    using Lookup = boost::container::flat_map<std::string, name>;                                          \
    static const Lookup lookup{BOOST_PP_SEQ_FOR_EACH(DEFINE_SERIALIZABLE_ENUM_READ_CASE, name, elements)}; \
    return lookup.at(str);                                                                                 \
  }
