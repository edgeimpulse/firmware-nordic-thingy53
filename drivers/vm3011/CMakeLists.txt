# SPDX-License-Identifier: Apache-2.0

if(CONFIG_VM3011)
    zephyr_include_directories(.)
    zephyr_library()

    zephyr_library_sources(vm3011.c)
endif()