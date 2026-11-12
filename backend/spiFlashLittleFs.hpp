// ENTIRELY CLANKED

#pragma once

#include <cstddef>
#include <cstring>
#include <type_traits>

bool spiFlashLittleFsInit();

bool spiFlashWriteBytes(const char *path, const void *data, size_t size);
bool spiFlashReadBytes(
    const char *path,
    void *data,
    size_t capacity,
    size_t &bytesRead
);

namespace spiFlashDetail {
bool writeFileExact(const char *path, const void *data, size_t size);
bool readFileExact(const char *path, void *data, size_t size);
}

template <typename T>
bool spiFlashWriteFile(const char *path, const T &value) {
    static_assert(
        std::is_trivially_copyable_v<T>,
        "SPI flash files can only be written directly from trivially-copyable types"
    );

    return spiFlashDetail::writeFileExact(path, &value, sizeof(value));
}

// Leaves value unchanged unless the file exists, has exactly sizeof(T) bytes,
// and the complete file can be read successfully.
template <typename T>
bool spiFlashReadFile(const char *path, T &value) {
    static_assert(
        std::is_trivially_copyable_v<T>,
        "SPI flash files can only be loaded directly into trivially-copyable types"
    );

    T loaded;
    if (!spiFlashDetail::readFileExact(path, &loaded, sizeof(loaded))) {
        return false;
    }

    std::memcpy(&value, &loaded, sizeof(value));
    return true;
}
