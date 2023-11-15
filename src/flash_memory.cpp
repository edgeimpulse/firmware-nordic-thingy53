/* Edge Impulse ingestion SDK
 * Copyright (c) 2023 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "flash_memory.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(ei_flash, LOG_LEVEL_DBG);

uint32_t EiFlashMemory::read_data(uint8_t *data, uint32_t address, uint32_t num_bytes)
{
    int ret;

    ret = flash_area_read(ext_flash_area, address, (void*)data, num_bytes);
    if(ret == 0) {
        return num_bytes;
    }

    LOG_ERR("Failed to read %u bytes from 0x%x (err: %d)", num_bytes, address, ret);
    return 0;
}

uint32_t EiFlashMemory::write_data(const uint8_t *data, uint32_t address, uint32_t num_bytes)
{
    uint16_t write_block = flash_area_align(ext_flash_area);
    uint32_t bytes_to_write = num_bytes;
    int ret;

    if(bytes_to_write % write_block != 0) {
        bytes_to_write += (4 - (bytes_to_write % write_block));
        LOG_WRN("Bytes to write (%u) not multiple of %u, extending to %u", num_bytes, write_block, bytes_to_write);
    }

    if(address % write_block) {
        LOG_ERR("Write address (0x%x) not aligned to 0x%x", address, write_block);
        return 0;
    }

    ret = flash_area_write(ext_flash_area, address, data, bytes_to_write);
    if(ret == 0) {
        return num_bytes;
    }

    LOG_ERR("Failed to write %u bytes at 0x%x (error: %d)", num_bytes, address, ret);
    return 0;
}

uint32_t EiFlashMemory::erase_data(uint32_t address, uint32_t num_bytes)
{
    int ret;
    uint32_t num_bytes_to_erase = num_bytes;
    // calculate address offset from block alignment
    uint16_t temp = address & (block_size - 1);

    LOG_DBG("(%d, %d)", address, num_bytes);
    LOG_DBG("temp = %d", temp);

    if(temp != 0) {
        LOG_WRN("Erase address not aligned to %u B (off by %u bytes), adjusting", block_size, temp);
        address -= temp;
        // we have to increase the number of bytes by the block offset bytes
        num_bytes_to_erase += temp;
    }
    LOG_DBG("address = %d", address);


    temp = num_bytes_to_erase % block_size;
    if(temp != 0) {
        LOG_WRN("Bytes to erase (%u) not multiple of block size (%u), adjusting by %u", num_bytes_to_erase, block_size, (block_size - temp));
        num_bytes_to_erase += (block_size - temp);
    }
    LOG_DBG("num_bytes_to_erase = %u", num_bytes_to_erase);


    ret = flash_area_erase(ext_flash_area, address, num_bytes_to_erase);
    if(ret == 0) {
        return num_bytes;
    }

    LOG_ERR("Failed to erase %u bytes at 0x%x (error: %d)", num_bytes_to_erase, address, ret);
    return 0;
}

EiFlashMemory::EiFlashMemory(uint32_t config_size):EiDeviceMemory(config_size, 90, 0, CONFIG_NORDIC_QSPI_NOR_FLASH_LAYOUT_PAGE_SIZE)
{
    int err;
    err = flash_area_open(FLASH_AREA_ID(external_flash), (const flash_area**)&ext_flash_area);
    if(err) {
        LOG_ERR("Failed to open flash area: external_flash");
        return;
    }
    
    memory_size = ext_flash_area->fa_size;
    memory_blocks = memory_size / block_size;

    LOG_DBG("Using flash device: %d", ext_flash_area->fa_id);
    LOG_DBG("Flash device size: %lu bytes", ext_flash_area->fa_size);
    LOG_DBG("Flash device offset: 0x%x", ext_flash_area->fa_off);
    LOG_DBG("Flash device align: 0x%x", flash_area_align(ext_flash_area));
    LOG_DBG("Flash device sector size: %d bytes", CONFIG_NORDIC_QSPI_NOR_FLASH_LAYOUT_PAGE_SIZE);
}
