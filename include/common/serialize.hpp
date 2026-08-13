#pragma once

#include <optional>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <memory>
#include <cstddef>
#include <string>
#include <cassert>
#include <iostream>

#include <Eigen/Dense>

// detection helper
template<typename T, typename = void>
struct has_serialize : std::false_type {};

template<typename T>
struct has_serialize<T, std::void_t<decltype(std::declval<T>().serialize(std::declval<std::vector<std::byte>&>()))>> : std::true_type {};

template<typename T, typename = void>
struct has_deserialize : std::false_type {};

template<typename T>
struct has_deserialize<T, std::void_t<decltype(std::declval<T>().deserialize(std::declval<const std::byte*&>()))>> : std::true_type {};


/** Forward declarations */
template<typename T>
std::enable_if_t<has_serialize<T>::value>
pack(std::vector<std::byte>& buf, const T& val);
template<typename T>
std::enable_if_t<has_deserialize<T>::value>
unpack(const std::byte*& cursor, T& var);


template<typename T>
std::enable_if_t<!has_serialize<T>::value && std::is_trivially_copyable_v<T>>
pack(std::vector<std::byte>& buf, const T& val);
template<typename T>
std::enable_if_t<!has_deserialize<T>::value && std::is_trivially_copyable_v<T>>
unpack(const std::byte*& cursor, T& var);

void pack(std::vector<std::byte>& buf, const std::string& str);
void unpack(const std::byte*& cursor, std::string& str);

template<typename T>
void pack(std::vector<std::byte>& buf, const std::vector<T>& vec);
template<typename T>
void unpack(const std::byte*& cursor, std::vector<T>& vec);

template<typename Derived>
std::enable_if_t<std::is_base_of_v<Eigen::MatrixBase<Derived>, Derived>>
pack(std::vector<std::byte>& buf, const Derived& mat);
template<typename Derived>
std::enable_if_t<std::is_base_of_v<Eigen::MatrixBase<Derived>, Derived>>
unpack(const std::byte*& cursor, Derived& mat);


/**
 * User-defined types with serialize and deserialize implemented
 */

template<typename T>
inline std::enable_if_t<has_serialize<T>::value>
pack(std::vector<std::byte>& buf, const T& val)
{
    val.serialize(buf);
}

template<typename T>
inline std::enable_if_t<has_deserialize<T>::value>
unpack(const std::byte*& cursor, T& var)
{
    var.deserialize(cursor);
}


/**
 * Trivially copyable types
 */
template<typename T>
inline std::enable_if_t<!has_serialize<T>::value && std::is_trivially_copyable_v<T>>
pack(std::vector<std::byte>& buf, const T& val)
{
    const auto* bytes = reinterpret_cast<const std::byte*>(&val);
    buf.insert(buf.end(), bytes, bytes + sizeof(T));
}

template<typename T>
inline std::enable_if_t<!has_deserialize<T>::value && std::is_trivially_copyable_v<T>>
unpack(const std::byte*& cursor, T& var)
{
    static_assert(std::is_trivially_copyable_v<T>);
    std::memcpy(&var, cursor, sizeof(T));
    cursor += sizeof(T);
}


/** std::string */

inline void pack(std::vector<std::byte>& buf, const std::string& str) {
    pack(buf, (int64_t)str.size());
    const auto* bytes = reinterpret_cast<const std::byte*>(str.data());
    buf.insert(buf.end(), bytes, bytes + str.size());
}

inline void unpack(const std::byte*& cursor, std::string& str) {
    int64_t size;
    unpack(cursor, size);
    str.assign(reinterpret_cast<const char*>(cursor), size);
    cursor += size;
}


/** std::vector */

template<typename T>
inline void pack(std::vector<std::byte>& buf, const std::vector<T>& vec)
{
    // pack size, then elements
    pack(buf, vec.size());
    for (const auto& v : vec)
        pack(buf, v);
}

template<typename T>
inline void unpack(const std::byte*& cursor, std::vector<T>& vec)
{
    // unpack size, then elements
    size_t size;
    unpack(cursor, size);
    vec.resize(size);
    for (auto& v : vec)
        unpack(cursor, v);
}

template<>
inline void unpack(const std::byte*& cursor, std::vector<bool>& vec)
{
    size_t size;
    unpack(cursor, size);
    vec.resize(size);
    for (size_t i = 0; i < size; i++)
    {
        bool val;
        unpack(cursor, val);
        vec[i] = val;
    }
}


/** dynamic Eigen types */

template<typename Derived>
inline std::enable_if_t<std::is_base_of_v<Eigen::MatrixBase<Derived>, Derived>>
pack(std::vector<std::byte>& buf, const Derived& mat)
{
    if constexpr (Derived::RowsAtCompileTime == Eigen::Dynamic ||
                  Derived::ColsAtCompileTime == Eigen::Dynamic)
    {
        pack(buf, (size_t)mat.rows());
        pack(buf, (size_t)mat.cols());
    }
    const auto* bytes = reinterpret_cast<const std::byte*>(mat.derived().data());
    buf.insert(buf.end(), bytes, bytes + mat.size() * sizeof(typename Derived::Scalar));
}

template<typename Derived>
inline std::enable_if_t<std::is_base_of_v<Eigen::MatrixBase<Derived>, Derived>>
unpack(const std::byte*& cursor, Derived& mat)
{
    if constexpr (Derived::RowsAtCompileTime == Eigen::Dynamic ||
                  Derived::ColsAtCompileTime == Eigen::Dynamic)
    {
        size_t rows, cols;
        unpack(cursor, rows);
        unpack(cursor, cols);
        mat.derived().resize(rows, cols);
    }
    std::memcpy(mat.derived().data(), cursor, mat.size() * sizeof(typename Derived::Scalar));
    cursor += mat.size() * sizeof(typename Derived::Scalar);
}