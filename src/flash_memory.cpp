/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
