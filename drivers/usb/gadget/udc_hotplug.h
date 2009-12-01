/* 
 * Ingenic USB Device Contoller Hotplug External Interfaces 
 */

#ifndef __UDC_HOTPLUG_H__
#define __UDC_HOTPLUG_H__

#include <linux/notifier.h>

typedef enum {
	BROADCAST_TYPE_STATE = 0,
	BROADCAST_TYPE_EVENT,
}udc_hotplug_broadcast_type_t;

typedef enum { 
	EVENT_STATE_OFFLINE = 0,
	EVENT_STATE_ONLINE,
}udc_hotplug_event_state_t;

typedef enum {
	EVENT_TYPE_USB = 0,
	EVENT_TYPE_CABLE,
}udc_hotplug_event_type_t;

enum {
	EVENT_FLAG_UDC_PHY_TOUCHED = 0,
};

typedef struct {
	udc_hotplug_event_type_t type;
	udc_hotplug_event_state_t state;
	unsigned long flags;
}udc_hotplug_event_t;

/* Register notifier */
int udc_hotplug_register_notifier(struct notifier_block *n, int request_state);

/* Unregister notifier */
int udc_hotplug_unregister_notifier(struct notifier_block *n);

/* Start keep alive */
int udc_hotplug_start_keep_alive(unsigned long timer_interval_in_jiffies, unsigned long counter_limit);

/* Do keep alive */
void udc_hotplug_do_keep_alive(void);

/* Stop keep alive */
void udc_hotplug_stop_keep_alive(void);

#endif /* Define __UDC_HOTPLUG_H__ */
