#ifndef __UTILS_H__
#define __UTILS_H__


uint32_t utils_time_diff(uint32_t time1, uint32_t time0);

/**
 * Get time in milliseconds.
 *
 * Get time in milliseconds since boot or some arbitrary time in the past.
 * The starting point does not change when clock time changes.
 *
 * Note that this will roll over every 49.7 days, so the caller must
 * expect that and account for it.
 *
 * \returns 32-bit unsigned monotonic milliseconds.
 */
uint32_t utils_time_ms_get(void);

/**
 * Get time in seconds.
 *
 * Get time in seconds since boot or some arbitrary time in the past.
 * The starting point does not change when clock time changes.
 *
 * \returns 32-bit unsigned monotonic seconds.
 */
uint32_t utils_time_sec_get(void);



#endif /*  __UTILS_H__ */
