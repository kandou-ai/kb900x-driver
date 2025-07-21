/*
 * Copyright (c) Kandou-AI.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _KB_UTILS_H
#define _KB_UTILS_H

#include "kb900x_log.h"
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

//! CFFI
/**
 * \brief This struct is used to store the I2C and IPMB configuration for
 * communications with KB900x.
 */
typedef struct {
    uint8_t intf;    // Used only for BIC communication
    uint8_t slot_id; // Used only for BIC communication
    int bus_id;
    uint8_t retimer_addr;
    int handle; // Used only for direct I2C connection
} kb900x_config_t;
//! CFFI END

// Constants
// Command code as per supplemental spec Table 6-3.
// (Slave SMBus Command Code Fields)
#define CCODE_START_READ_FUNC0 (0x82)
#define CCODE_END_READ_FUNC0 (0x81)

#define CCODE_START_END_WRITE_FUNC1 (0x87)

#define CCODE_START_READ_FUNC2 (0x8A)
#define CCODE_END_READ_FUNC2 (0x89)

#define CCODE_START_END_WRITE_FUNC3 (0x8F)

// Constants
#define ABSOLUTE_ZERO (-273.15)
#define FLOAT_PRECISION (16)
#define BITS_IN_BYTE (8)
#define BYTES_IN_U32 (4)
#define KB900X_SIDE_B_RX (0)
#define KB900X_SIDE_A_RX (1)

// Custom Error numbers
#define KB900X_E_OK (0)                        /* Success */
#define KB900X_E_ERR (1000)                    /* generic error */
#define KB900X_E_PEC_NOT_SUPPORTED (1001)      /* PEC not supported */
#define KB900X_E_TX_ABORT (1002)               /* TX abort detected */
#define KB900X_E_BOOT_STATUS (1003)            /* Boot status error */
#define KB900X_E_NOT_IMPLEMENTED (1004)        /* Not implemented */
#define KB900X_E_UNKNOWN_REVID (1005)          /* Unknown chip rev id */
#define KB900X_E_FEATURE_REQ_FAILED (1006)     /* Feature request failed */
#define KB900X_E_OP_NOT_SUPPORTED_BY_FW (1007) /* Current FW does not support this operation. */
#define KB900X_E_EEPROM_FORMAT_ERROR (1008)    /* EEPROM format error */

// GCC or Clang
#ifdef __GNUC__
#define ALIGN_PACKED(alignment) __attribute__((packed, aligned(alignment)))
#define PACKED __attribute__((packed))
#else
// Default for other compilers
#define ALIGN_PACKED(alignment)
#define PACKED
#endif

// Macros for error handling
// Macro for ioctl return codes
#define CHECK_IOCTL(rc)                                                                            \
    {                                                                                              \
        if (rc < 0) {                                                                              \
            KANDOU_ERR("Error: unexpected return code: %d - %s", rc, strerror(errno));             \
            return -errno;                                                                         \
        }                                                                                          \
    }
// Macro for ioctl return codes with custom message
#define CHECK_IOCTL_MSG(rc, ...)                                                                   \
    {                                                                                              \
        if (rc < 0) {                                                                              \
            KANDOU_ERR(__VA_ARGS__);                                                               \
            return -errno;                                                                         \
        }                                                                                          \
    }

// Macro for the case when no message is provided
#define CHECK_SUCCESS(rc)                                                                          \
    {                                                                                              \
        if (rc < 0) {                                                                              \
            KANDOU_ERR("Error: unexpected return code %d - %s", rc, kb900x_strerror(rc));          \
            return rc;                                                                             \
        }                                                                                          \
    }

// Macro for the case when a message is provided
#define CHECK_SUCCESS_MSG(rc, ...)                                                                 \
    {                                                                                              \
        if ((rc) < 0) {                                                                            \
            KANDOU_ERR(__VA_ARGS__);                                                               \
            return (rc);                                                                           \
        }                                                                                          \
    }

/**
 * \brief Wait for a specified number of milliseconds.
 *
 * \param milliseconds The number of milliseconds to wait.
 */
void wait_ms(int milliseconds);

/**
 * \brief Calculate the CRC8 checksum of a given data buffer.
 *
 * \param[in] data a pointer to the data buffer
 * \param[in] len the length of the data buffer
 *
 * \return the calculated CRC8 checksum
 */
uint8_t cal_crc8(uint8_t *data, size_t len);

/**
 * \brief Read a binary file and store it in a buffer.
 *
 * \param[in] filename the name (path) of the file to read
 * \param[out] buffer the pointer to the buffer to store the file content
 * \param[out] buffer_size the size of the buffer
 *
 * \note The buffer is allocated in this function and must be freed by the caller
 *
 * \return 0 if no error, else the error code
 */
int read_file(const char *filename, uint8_t **buffer, size_t *buffer_size);

/** \brief Convert an error code to a string
 *
 * \note This function compares the absolute value of the error code.
 *
 * \param[in] rc the error code.
 */
const char *kb900x_strerror(int rc);

/**
 * \brief Decode 4 bytes into a u32, given endianness.
 *
 * \param[in] bytes a NON-NULL pointer to 4 bytes
 * \param[out] dst a pointer to a uint32_t to write the result to
 * \param[in] is_big_endian true if bytes should be decoded the big-endian way,
 *                          false if little-endian
 *
 * \return 0 if no error, else the error code
 */
int decode_u32(const uint8_t *bytes, uint32_t *dst, bool is_big_endian);

/**
 * \brief Encode a u32 value into bytes, given endianness, and write to a given pointer.
 *
 * \param[in] value the u32 to encode
 * \param[out] dst the memory address to write the result to
 * \param[in] is_big_endian true if bytes should be decoded the big-endian way,
 *                          false if little-endian
 *
 * \return 0 if no error, else the error code
 */
int encode_u32(uint32_t value, uint8_t *dst, bool is_big_endian);

#endif // _KB_UTILS_H
