#pragma once

#include <algorithm>
#include <limits>

#include "rapidjson/document.h"

namespace ppr::profiles {

inline void get_double(double& field,
                       rapidjson::Value const& doc,
                       char const* key) {
  if (doc.HasMember(key)) {
    auto const& val = doc[key];
    if (val.IsNumber()) {
      field = val.GetDouble();
    }
  }
}

template <typename T>
inline void get_int(T& field, rapidjson::Value const& doc, char const* key) {
  if (doc.HasMember(key)) {
    auto const& val = doc[key];
    if (val.IsInt()) {
      field = static_cast<T>(std::clamp(
          val.GetInt(), static_cast<int>(std::numeric_limits<T>::min()),
          static_cast<int>(std::numeric_limits<T>::max())));
    }
  }
}

template <typename T>
inline void get_double_as_int(T& field,
                              rapidjson::Value const& doc,
                              char const* key,
                              double const multiply_by) {
  if (doc.HasMember(key)) {
    auto const& val = doc[key];
    if (val.IsNumber()) {
      field = static_cast<T>(
          std::clamp(val.GetDouble() * multiply_by,
                     static_cast<double>(std::numeric_limits<T>::min()),
                     static_cast<double>(std::numeric_limits<T>::max())));
    }
  }
}

inline void get_bool(bool& field,
                     rapidjson::Value const& doc,
                     char const* key) {
  if (doc.HasMember(key)) {
    auto const& val = doc[key];
    if (val.IsBool()) {
      field = val.GetBool();
    }
  }
}

}  // namespace ppr::profiles
