/***************************************************************************************************
 *   Project:       lib
 *   Author:
 ***************************************************************************************************
 *   Distribution:
 *
 ***************************************************************************************************
 *   MCU Family:
 *   Compiler:
 ***************************************************************************************************
 *   File:          fifo.h
 *   Description:   fifo library
 *
 *   structure of FIFO-buffer must be as:
 *
 *   typedef struct
 *   {
 *       volatile counter_t begin_idx;
 *       volatile counter_t end_idx;
 *       data_t data[SIZE];
 *   } queue_t;
 *
 *   SIZE - must be binary exponent
 *   counter_t - unsigned fixed point type
 *   data_t - user data type
 *
 ***************************************************************************************************
 *   History:       15.03.2011 - [] - file created
 *
 **************************************************************************************************/

#pragma once

/***************************************************************************************************
 *                                         INCLUDED FILES
 **************************************************************************************************/

/***************************************************************************************************
 *                                           DEFINITIONS                                           *
 **************************************************************************************************/

#ifndef countof
    #define countof(a) (sizeof(a)/sizeof((a)[0]))
#endif

#define FIFO_COUNTOF(queue)                 (sizeof((queue).data)/sizeof(*((queue).data)))

#define FIFO_LEN_GET(queue)                 \
    ((uint32_t)((queue).end_idx - (queue).begin_idx))
#define FIFO_ADD(queue, value)              \
    ((queue).data[(queue).end_idx++ & (countof((queue).data) - 1)] = (value))
#define FIFO_WRITE_ITEM(queue, item, value) \
    ((queue).data[(((queue).begin_idx) + (item)) & (countof((queue).data) - 1)] = (value))
#define FIFO_EXTRACT(queue)                 \
    ((queue).data[(queue).begin_idx++ & (countof((queue).data) - 1)])
#define FIFO_READ_HEAD(queue)               \
    ((queue).data[(queue).begin_idx & (countof((queue).data) - 1)])
#define FIFO_READ_ITEM(queue, item)         \
    ((queue).data[(((queue).begin_idx) + (item)) & (countof((queue).data) - 1)])
#define FIFO_CLEAR(queue)                   \
    ((queue).end_idx = (queue).begin_idx)
#define FIFO_IS_EMPTY(queue)                \
    ((queue).end_idx == (queue).begin_idx)
#define FIFO_IS_FULL(queue)                 \
    (FIFO_LEN_GET(queue) >= countof((queue).data))

/***************************************************************************************************
 *                                          PUBLIC TYPES                                           *
 **************************************************************************************************/

typedef enum
{
    FIFO_SIZE_2     = (1U <<  1),
    FIFO_SIZE_4     = (1U <<  2),
    FIFO_SIZE_8     = (1U <<  3),
    FIFO_SIZE_16    = (1U <<  4),
    FIFO_SIZE_32    = (1U <<  5),
    FIFO_SIZE_64    = (1U <<  6),
    FIFO_SIZE_128   = (1U <<  7),
    FIFO_SIZE_256   = (1U <<  8),
    FIFO_SIZE_512   = (1U <<  9),
    FIFO_SIZE_1024  = (1U << 10),
    FIFO_SIZE_2048  = (1U << 11),
    FIFO_SIZE_4096  = (1U << 12),
    FIFO_SIZE_8192  = (1U << 13),
    FIFO_SIZE_16364 = (1U << 14),
    FIFO_SIZE_32768 = (1U << 15),
} fifo_size_t;

// Rounding to binary exponent
#define FIFO_SIZE_CEIL(SIZE)                                                                     \
    ((SIZE -   1 | SIZE >>  1 | SIZE >>  2 | SIZE >>  3 | SIZE >>  4 | SIZE >>  5 | SIZE >>  6 | \
      SIZE >>  7 | SIZE >>  8 | SIZE >>  9 | SIZE >> 10 | SIZE >> 11 | SIZE >> 12 | SIZE >> 13 | \
      SIZE >> 14 | SIZE >> 15 | SIZE >> 16 | SIZE >> 17 | SIZE >> 18 | SIZE >> 19 | SIZE >> 20 | \
      SIZE >> 21 | SIZE >> 22 | SIZE >> 23 | SIZE >> 24 | SIZE >> 25 | SIZE >> 26 | SIZE >> 27 | \
      SIZE >> 28 | SIZE >> 29 | SIZE >> 30 | SIZE >> 31 ) + 1)

#define FIFO_TYPEDEF(data_t, count_t, SIZE)\
    struct                                 \
    {                                      \
        volatile count_t                   \
        begin_idx, end_idx;                \
        data_t data[FIFO_SIZE_CEIL(SIZE)]; \
    }

/***************************************************************************************************
 *                                         GLOBAL VARIABLES                                        *
 **************************************************************************************************/

/***************************************************************************************************
 *                                    PUBLIC FUNCTION PROTOTYPES                                   *
 **************************************************************************************************/
