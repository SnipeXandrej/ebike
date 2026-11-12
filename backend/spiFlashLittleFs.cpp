// ENTIRELY CLANKED

#include "spiFlashLittleFs.hpp"

extern "C" {
#include "ch.h"
#include "hal.h"
#include "lfs.h"
}

#include <cstdint>
#include <cstring>

namespace {

constexpr uint32_t EXTERNAL_FLASH_SIZE = 8U * 1024U * 1024U;
constexpr uint32_t ERASE_BLOCK_SIZE = 4096U;
constexpr uint32_t PAGE_SIZE = 256U;
constexpr uint32_t BLOCK_COUNT = EXTERNAL_FLASH_SIZE / ERASE_BLOCK_SIZE;

constexpr uint8_t CMD_READ_ID = 0x9FU;
constexpr uint8_t CMD_READ_STATUS = 0x05U;
constexpr uint8_t CMD_WRITE_ENABLE = 0x06U;
constexpr uint8_t CMD_READ_DATA = 0x03U;
constexpr uint8_t CMD_PAGE_PROGRAM = 0x02U;
constexpr uint8_t CMD_SECTOR_ERASE = 0x20U;

constexpr uint8_t STATUS_BUSY = 0x01U;

static const SPIConfig flashSpiConfig = {
    .circular = false,
    .slave = false,
    .data_cb = nullptr,
    .error_cb = nullptr,
    .ssport = GPIOD,
    .sspad = 6U,

    // SPI1 source is 52 MHz in the current clock configuration.
    // DIV2 gives 26 MHz.
    .cfg1 = SPI_CFG1_MBR_DIV2 | SPI_CFG1_DSIZE_VALUE(7),
    .cfg2 = 0U
};

static lfs_t littleFs;
static lfs_config littleFsConfig{};

CC_ALIGN_DATA(32)
static uint8_t readBuffer[PAGE_SIZE];

CC_ALIGN_DATA(32)
static uint8_t programBuffer[PAGE_SIZE];

CC_ALIGN_DATA(32)
static uint8_t lookaheadBuffer[32];

// SPI1 uses DMA. The main thread stack is in DTCM on this target, which DMA
// cannot access, so command and status buffers must have static AXI-SRAM
// storage rather than being local variables on the stack.
CC_ALIGN_DATA(32)
static uint8_t spiCommandBuffer[4];

CC_ALIGN_DATA(32)
static uint8_t spiResponseBuffer[3];

CC_ALIGN_DATA(32)
static uint8_t spiDataBuffer[PAGE_SIZE];

static void selectFlash() {
    spiSelect(&SPID1);
}

static void unselectFlash() {
    spiUnselect(&SPID1);
}

static void dmaSend(size_t size, const uint8_t *buffer) {
    cacheBufferFlush(const_cast<uint8_t *>(buffer), size);
    spiSend(&SPID1, size, buffer);
}

static void dmaReceive(size_t size, uint8_t *buffer) {
    // Remove any dirty/stale cache line before DMA writes RAM, then discard
    // the stale CPU copy after the transfer completes.
    cacheBufferFlush(buffer, size);
    spiReceive(&SPID1, size, buffer);
    cacheBufferInvalidate(buffer, size);
}

static uint8_t readStatus() {
    spiCommandBuffer[0] = CMD_READ_STATUS;
    spiResponseBuffer[0] = 0U;

    selectFlash();
    dmaSend(1U, spiCommandBuffer);
    dmaReceive(1U, spiResponseBuffer);
    unselectFlash();

    return spiResponseBuffer[0];
}

static bool waitUntilReady(uint32_t timeoutMs) {
    const systime_t start = chVTGetSystemTimeX();
    const sysinterval_t timeout = TIME_MS2I(timeoutMs);

    while ((readStatus() & STATUS_BUSY) != 0U) {
        if (chVTTimeElapsedSinceX(start) >= timeout) {
            return false;
        }

        chThdSleepMilliseconds(1);
    }

    return true;
}

static void writeEnable() {
    spiCommandBuffer[0] = CMD_WRITE_ENABLE;

    selectFlash();
    dmaSend(1U, spiCommandBuffer);
    unselectFlash();
}

static int littleFsFlashRead(
    const lfs_config *,
    lfs_block_t block,
    lfs_off_t offset,
    void *buffer,
    lfs_size_t size
) {
    const uint32_t address =
        static_cast<uint32_t>(block) * ERASE_BLOCK_SIZE + offset;

    if ((address + size) > EXTERNAL_FLASH_SIZE) {
        return LFS_ERR_INVAL;
    }

    auto *destination = static_cast<uint8_t *>(buffer);
    uint32_t currentAddress = address;
    lfs_size_t remaining = size;

    while (remaining != 0U) {
        const lfs_size_t chunk =
            remaining > PAGE_SIZE ? PAGE_SIZE : remaining;

        spiCommandBuffer[0] = CMD_READ_DATA;
        spiCommandBuffer[1] = static_cast<uint8_t>(currentAddress >> 16U);
        spiCommandBuffer[2] = static_cast<uint8_t>(currentAddress >> 8U);
        spiCommandBuffer[3] = static_cast<uint8_t>(currentAddress);

        selectFlash();
        dmaSend(sizeof(spiCommandBuffer), spiCommandBuffer);
        dmaReceive(chunk, spiDataBuffer);
        unselectFlash();

        std::memcpy(destination, spiDataBuffer, chunk);
        destination += chunk;
        currentAddress += chunk;
        remaining -= chunk;
    }

    return LFS_ERR_OK;
}

static int littleFsFlashProgram(
    const lfs_config *,
    lfs_block_t block,
    lfs_off_t offset,
    const void *buffer,
    lfs_size_t size
) {
    const uint32_t address =
        static_cast<uint32_t>(block) * ERASE_BLOCK_SIZE + offset;

    if ((address + size) > EXTERNAL_FLASH_SIZE ||
        size == 0U ||
        size > PAGE_SIZE ||
        ((address & (PAGE_SIZE - 1U)) + size) > PAGE_SIZE) {
        return LFS_ERR_INVAL;
    }

    if (!waitUntilReady(5000U)) {
        return LFS_ERR_IO;
    }

    writeEnable();

    std::memcpy(spiDataBuffer, buffer, size);

    spiCommandBuffer[0] = CMD_PAGE_PROGRAM;
    spiCommandBuffer[1] = static_cast<uint8_t>(address >> 16U);
    spiCommandBuffer[2] = static_cast<uint8_t>(address >> 8U);
    spiCommandBuffer[3] = static_cast<uint8_t>(address);

    selectFlash();
    dmaSend(sizeof(spiCommandBuffer), spiCommandBuffer);
    dmaSend(size, spiDataBuffer);
    unselectFlash();

    return waitUntilReady(5000U) ? LFS_ERR_OK : LFS_ERR_IO;
}

static int flashErase(
    const lfs_config *,
    lfs_block_t block
) {
    if (block >= BLOCK_COUNT) {
        return LFS_ERR_INVAL;
    }

    if (!waitUntilReady(5000U)) {
        return LFS_ERR_IO;
    }

    writeEnable();

    const uint32_t address =
        static_cast<uint32_t>(block) * ERASE_BLOCK_SIZE;

    spiCommandBuffer[0] = CMD_SECTOR_ERASE;
    spiCommandBuffer[1] = static_cast<uint8_t>(address >> 16U);
    spiCommandBuffer[2] = static_cast<uint8_t>(address >> 8U);
    spiCommandBuffer[3] = static_cast<uint8_t>(address);

    selectFlash();
    dmaSend(sizeof(spiCommandBuffer), spiCommandBuffer);
    unselectFlash();

    return waitUntilReady(10000U) ? LFS_ERR_OK : LFS_ERR_IO;
}

static int flashSync(const lfs_config *) {
    return waitUntilReady(5000U) ? LFS_ERR_OK : LFS_ERR_IO;
}

static int flashLock(const lfs_config *) {
    spiAcquireBus(&SPID1);
    return LFS_ERR_OK;
}

static int flashUnlock(const lfs_config *) {
    spiReleaseBus(&SPID1);
    return LFS_ERR_OK;
}

static bool checkJedecId() {
    spiCommandBuffer[0] = CMD_READ_ID;
    spiResponseBuffer[0] = 0U;
    spiResponseBuffer[1] = 0U;
    spiResponseBuffer[2] = 0U;

    spiAcquireBus(&SPID1);

    selectFlash();
    dmaSend(1U, spiCommandBuffer);
    dmaReceive(sizeof(spiResponseBuffer), spiResponseBuffer);
    unselectFlash();

    spiReleaseBus(&SPID1);

    // WeAct boards are populated with more than one compatible 64-Mbit part.
    // Known IDs are Winbond W25Q64 (EF 40 17) and the part on this board
    // (85 20 17). Keep this explicit so a different-capacity flash cannot be
    // mounted using the hard-coded 8 MiB geometry above.
    const bool winbondW25Q64 =
        spiResponseBuffer[0] == 0xEFU &&
        spiResponseBuffer[1] == 0x40U &&
        spiResponseBuffer[2] == 0x17U;
    const bool boardFlash64Mbit =
        spiResponseBuffer[0] == 0x85U &&
        spiResponseBuffer[1] == 0x20U &&
        spiResponseBuffer[2] == 0x17U;

    return winbondW25Q64 || boardFlash64Mbit;
}

} // namespace

bool spiFlashLittleFsInit() {
    // WeAct STM32H723 onboard SPI flash:
    // PB3 = SPI1_SCK
    // PB4 = SPI1_MISO
    // PD7 = SPI1_MOSI
    // PD6 = chip select
    palSetLineMode(
        PAL_LINE(GPIOB, 3U),
        PAL_MODE_ALTERNATE(5U) | PAL_STM32_OSPEED_HIGHEST
    );

    palSetLineMode(
        PAL_LINE(GPIOB, 4U),
        PAL_MODE_ALTERNATE(5U) | PAL_STM32_OSPEED_HIGHEST
    );

    palSetLineMode(
        PAL_LINE(GPIOD, 7U),
        PAL_MODE_ALTERNATE(5U) | PAL_STM32_OSPEED_HIGHEST
    );

    palSetLineMode(
        PAL_LINE(GPIOD, 6U),
        PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST
    );

    palSetLine(PAL_LINE(GPIOD, 6U));

    spiStart(&SPID1, &flashSpiConfig);

    if (!checkJedecId()) {
        return false;
    }

    littleFsConfig.context = nullptr;

    littleFsConfig.read = littleFsFlashRead;
    littleFsConfig.prog = littleFsFlashProgram;
    littleFsConfig.erase = flashErase;
    littleFsConfig.sync = flashSync;
    littleFsConfig.lock = flashLock;
    littleFsConfig.unlock = flashUnlock;

    littleFsConfig.read_size = 1U;
    littleFsConfig.prog_size = PAGE_SIZE;
    littleFsConfig.block_size = ERASE_BLOCK_SIZE;
    littleFsConfig.block_count = BLOCK_COUNT;
    littleFsConfig.block_cycles = 500;
    littleFsConfig.cache_size = PAGE_SIZE;
    littleFsConfig.lookahead_size = sizeof(lookaheadBuffer);

    littleFsConfig.read_buffer = readBuffer;
    littleFsConfig.prog_buffer = programBuffer;
    littleFsConfig.lookahead_buffer = lookaheadBuffer;

    int result = lfs_mount(&littleFs, &littleFsConfig);

    // Only format when LittleFS identifies an unformatted/corrupt volume.
    // Do not format for arbitrary SPI I/O errors.
    if (result == LFS_ERR_CORRUPT) {
        result = lfs_format(&littleFs, &littleFsConfig);

        if (result == LFS_ERR_OK) {
            result = lfs_mount(&littleFs, &littleFsConfig);
        }
    }

    return result == LFS_ERR_OK;
}

bool spiFlashDetail::writeFileExact(
    const char *path,
    const void *data,
    size_t size
) {
    if (path == nullptr || data == nullptr) {
        return false;
    }

    lfs_file_t file{};

    int result = lfs_file_open(
        &littleFs,
        &file,
        path,
        LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC
    );

    if (result < 0) {
        return false;
    }

    const lfs_ssize_t written =
        lfs_file_write(&littleFs, &file, data, size);

    if (written == static_cast<lfs_ssize_t>(size)) {
        result = lfs_file_sync(&littleFs, &file);
    } else {
        result = LFS_ERR_IO;
    }

    const int closeResult =
        lfs_file_close(&littleFs, &file);

    return result == LFS_ERR_OK &&
           closeResult == LFS_ERR_OK;
}

bool spiFlashDetail::readFileExact(
    const char *path,
    void *data,
    size_t size
) {
    if (path == nullptr || data == nullptr) {
        return false;
    }

    lfs_file_t file{};

    int result = lfs_file_open(
        &littleFs,
        &file,
        path,
        LFS_O_RDONLY
    );

    if (result < 0) {
        return false;
    }

    const lfs_soff_t fileSize = lfs_file_size(&littleFs, &file);
    if (fileSize < 0 || static_cast<size_t>(fileSize) != size) {
        lfs_file_close(&littleFs, &file);
        return false;
    }

    const lfs_ssize_t count =
        lfs_file_read(&littleFs, &file, data, size);

    const int closeResult =
        lfs_file_close(&littleFs, &file);

    return count == static_cast<lfs_ssize_t>(size) &&
           closeResult == LFS_ERR_OK;
}
