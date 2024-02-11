#ifndef _NFC_WAKELOCK_H
#define _NFC_WAKELOCK_H

#include <linux/ktime.h>
#include <linux/device.h>

enum {
    WAKE_LOCK_SUSPEND, /* Prevent suspend */
    WAKE_LOCK_TYPE_COUNT
};

struct wake_lock {
/* SHARP EXTENDED Mod-Start CUST-ID:0252 */
    // struct wakeup_source ws;
    struct wakeup_source *ws;
/* SHARP EXTENDED Mod-End CUST-ID:0252 */
};

static inline void wake_lock_init(struct wake_lock *lock, int type,
                  const char *name)
{
/* SHARP EXTENDED Mod-Start CUST-ID:0252 */
    // wakeup_source_init(&lock->ws, name);
    lock->ws = wakeup_source_register(NULL, name);
/* SHARP EXTENDED Mod-End CUST-ID:0252 */
}

static inline void wake_lock_destroy(struct wake_lock *lock)
{
/* SHARP EXTENDED Mod-Start CUST-ID:0252 */
    // wakeup_source_trash(&lock->ws);
    wakeup_source_unregister(lock->ws);
/* SHARP EXTENDED Mod-End CUST-ID:0252 */
}

static inline void wake_lock(struct wake_lock *lock)
{
/* SHARP EXTENDED Mod-Start CUST-ID:0252 */
    // __pm_stay_awake(&lock->ws);
    __pm_stay_awake(lock->ws);
/* SHARP EXTENDED Mod-End CUST-ID:0252 */
}

static inline void wake_lock_timeout(struct wake_lock *lock, long timeout)
{
/* SHARP EXTENDED Mod-Start CUST-ID:0252 */
    // __pm_wakeup_event(&lock->ws, jiffies_to_msecs(timeout));
    __pm_wakeup_event(lock->ws, jiffies_to_msecs(timeout));
/* SHARP EXTENDED Mod-End CUST-ID:0252 */
}

static inline void wake_unlock(struct wake_lock *lock)
{
/* SHARP EXTENDED Mod-Start CUST-ID:0252 */
    // __pm_relax(&lock->ws);
    __pm_relax(lock->ws);
/* SHARP EXTENDED Mod-End CUST-ID:0252 */
}

static inline int wake_lock_active(struct wake_lock *lock)
{
/* SHARP EXTENDED Mod-Start CUST-ID:0252 */
    // return lock->ws.active;
    return lock->ws->active;
/* SHARP EXTENDED Mod-End CUST-ID:0252 */
}

#endif
