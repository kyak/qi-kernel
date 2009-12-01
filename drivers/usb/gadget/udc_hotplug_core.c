/*
 * Ingenic USB Device Controller Hotplug Core Function
 * Detection mechanism and code are based on the old version of udc_hotplug.c  
 */

#include <linux/sched.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/timer.h>

#include <asm/jzsoc.h>

#include "udc_hotplug.h"

#define PFX "jz_hotplug_udc"

#define D(msg, fmt...)  \
//	printk(KERN_ERR PFX": %s(): "msg, __func__, ##fmt);
	
/* HAVE_DETECT_SYNC
   Provide a lock like seqlock keep the synchronization between the start and the end of a detection, 
   If the lock seems not synchronous(new interrupt comes, when doing our detection) in the end of a detection, 
   the result of the detection is discarded. No event will be broadcast, and the detection will be restarted. 
   
   Use to filter out more significant events when the interrupt is too noisy.
*/

//#define HAVE_DETECT_SYNC 1

#if defined (HAVE_DETECT_SYNC)
#define NR_RESTART_TIMES                      3
#define NR_JIFFIES_SLEEP_BEFORE_RESTART       7
#endif

#define NR_GPIO_STABLE_TIMES                  50
#define NR_JIFFIES_USB_DETECT_WAIT            11

#define DEFAULT_KEEP_ALIVE_TIMER_INTERVAL     (2 * HZ)
#define DEFAULT_KEEP_ALIVE_COUNTER_LIMIT      2

#define UDC_HOTPLUG_PIN   GPIO_UDC_HOTPLUG
#define UDC_HOTPLUG_IRQ   (IRQ_GPIO_0 + UDC_HOTPLUG_PIN)

/* UDC State bits */
enum {
	/* Online state. */
	BIT_CABLE_ONLINE = 0,      
	BIT_USB_ONLINE,
        
	/* State changed ?*/
	BIT_CABLE_CHANGE,      
	BIT_USB_CHANGE,
        
	/* What detection will be done ? */
	BIT_DO_CABLE_DETECT,
	BIT_DO_USB_DETECT,
	
	/* What detection is requested ? */
	BIT_REQUEST_CABLE_DETECT, 
	BIT_REQUEST_USB_DETECT,
	
	/* Indicate whether a detection is finisned. */
	BIT_USB_DETECT_DONE,   
	BIT_CABLE_DETECT_DONE,
	
	BIT_UDC_PHY_TOUCHED,

	/* Keep alive */
	BIT_KEEP_ALIVE,
	BIT_KEEP_ALIVE_TIMEOUT,
};

struct uh_data {
	/* Notifier */
	struct blocking_notifier_head notifier_head;
	
	/* Thread */
	struct task_struct *kthread;
	
	/* Wait queue */
	wait_queue_head_t kthread_wq; /* Kernel thread sleep here. */
	wait_queue_head_t wq;	      /* Others sleep here.  */

	/* UDC State */
	unsigned long state;

	/* Current Event */
	udc_hotplug_event_t cur_uh_event;

#if defined (HAVE_DETECT_SYNC)	
	/* Sync seq */
	unsigned long irq_sync_seq;
	unsigned long our_sync_seq;
#endif

	/* Keep alive */
	struct timer_list keep_alive_timer;

	unsigned long keep_alive_counter_limit;
	unsigned long keep_alive_timer_interval;
	unsigned long keep_alive_counter;
};

static struct uh_data *g_puh_data = NULL;

#if defined (HAVE_DETECT_SYNC)
/* Seq sync function */

static inline int is_seq_sync(struct uh_data *uh)
{
	return (uh->our_sync_seq == uh->irq_sync_seq);
}

static inline void reset_seq(struct uh_data *uh)
{
	uh->our_sync_seq = uh->irq_sync_seq = 0;

	return;
}

static inline void sync_seq(struct uh_data *uh)
{
	uh->our_sync_seq = uh->irq_sync_seq;

	return;
}
#endif 

/* Call kernel thread to detect. */
static inline void start_detect(struct uh_data *uh)
{
	D("called.\n");

#if defined (HAVE_DETECT_SYNC)
	uh->irq_sync_seq ++;
#endif
	
	wake_up_process(uh->kthread);
	
	return;
}

static void wait_gpio_pin_stable(struct uh_data *uh)
{
	unsigned long pin = 0;
	int i = 1;

	pin = __gpio_get_pin(UDC_HOTPLUG_PIN);
	
	while (i < NR_GPIO_STABLE_TIMES) {
		if (__gpio_get_pin(UDC_HOTPLUG_PIN) != pin) {
			pin = __gpio_get_pin(UDC_HOTPLUG_PIN);
			i = 1; 
		}else
			i++;
		
		sleep_on_timeout(&uh->wq, 1);
	}
	
	return;
}

/* Do cable detection */
static void cable_detect(struct uh_data *uh)
{	
	D("Wait pin stable.\n");

	/* Wait GPIO pin stable first. */
	wait_gpio_pin_stable(uh);
	
	if (__gpio_get_pin(UDC_HOTPLUG_PIN)) {
		D("Cable online.\n");
		
		if (!test_and_set_bit(BIT_CABLE_ONLINE, &uh->state)) {
			D("Cable state change to online.\n");

			set_bit(BIT_CABLE_CHANGE, &uh->state);
		}
	}else {
		D("Cable offline.\n");
		
		/* Clear keep alive bit. */
		clear_bit(BIT_KEEP_ALIVE, &uh->state);

		if (test_and_clear_bit(BIT_CABLE_ONLINE, &uh->state)) {
			D("Cable state change to offline.\n");
			
			set_bit(BIT_CABLE_CHANGE, &uh->state);
		}
	}

	set_bit(BIT_CABLE_DETECT_DONE, &uh->state);
	
	return;
}

/* Really do USB detection */
static int do_usb_detect(struct uh_data *uh)
{
	u32 intr_usb;
	int rv;
	
	D("called.\n");

	__intc_mask_irq(IRQ_UDC);

	/* Now enable PHY to start detect */
#ifdef CONFIG_SOC_JZ4740
	REG_CPM_SCR |= CPM_SCR_USBPHY_ENABLE;
#elif defined(CONFIG_SOC_JZ4750) || defined(CONFIG_SOC_JZ4750D)
	REG_CPM_OPCR |= CPM_OPCR_UDCPHY_ENABLE;
#endif
	/* Clear IRQs */
	REG16(USB_REG_INTRINE) = 0;
	REG16(USB_REG_INTROUTE) = 0;
	REG8(USB_REG_INTRUSBE) = 0;

	/* disable UDC IRQs first */
	REG16(USB_REG_INTRINE) = 0;
	REG16(USB_REG_INTROUTE) = 0;
	REG8(USB_REG_INTRUSBE) = 0;

	/* Disable DMA */
	REG32(USB_REG_CNTL1) = 0;
	REG32(USB_REG_CNTL2) = 0;

	/* Enable HS Mode */
	REG8(USB_REG_POWER) |= USB_POWER_HSENAB;
	/* Enable soft connect */
	REG8(USB_REG_POWER) |= USB_POWER_SOFTCONN;

	D("enable phy! %x %x %x %x %x\n",
	       REG8(USB_REG_POWER),
	       REG_CPM_OPCR,
	       REG16(USB_REG_INTRINE),
	       REG16(USB_REG_INTROUTE),
	       REG8(USB_REG_INTRUSBE));
	
	/* Wait a moment. */
	sleep_on_timeout(&uh->wq, NR_JIFFIES_USB_DETECT_WAIT);

	intr_usb = REG8(USB_REG_INTRUSB);
	if ((intr_usb & USB_INTR_RESET) ||
	    (intr_usb & USB_INTR_RESUME) ||
	    (intr_usb & USB_INTR_SUSPEND))
	{
		rv = 1;
	}
	else
	{
		rv = 0;
	}

	/* Detect finish ,clean every thing */
	/* Disconnect from usb */
	REG8(USB_REG_POWER) &= ~USB_POWER_SOFTCONN;
	/* Disable the USB PHY */
#ifdef CONFIG_SOC_JZ4740
	REG_CPM_SCR &= ~CPM_SCR_USBPHY_ENABLE;
#elif defined(CONFIG_SOC_JZ4750) || defined(CONFIG_SOC_JZ4750D)
	REG_CPM_OPCR &= ~CPM_OPCR_UDCPHY_ENABLE;
#endif
	/* Clear IRQs */
	REG16(USB_REG_INTRINE) = 0;
	REG16(USB_REG_INTROUTE) = 0;
	REG8(USB_REG_INTRUSBE) = 0;
	__intc_ack_irq(IRQ_UDC);
	__intc_unmask_irq(IRQ_UDC);
	
	mdelay(1);

	return rv;
}
	
/* Do USB bus protocol detection */
static void usb_detect(struct uh_data *uh)
{
	int rv = 0;
	
	D("Called.\n");

	/* If the cable has already been offline, we just pass the real USB detection. */
	if (test_bit(BIT_CABLE_ONLINE, &uh->state)) {

		D("Do real detection.\n");

		rv = do_usb_detect(uh);
		set_bit(BIT_UDC_PHY_TOUCHED, &uh->state);
	}else{
		clear_bit(BIT_UDC_PHY_TOUCHED, &uh->state);
		D("No need to do real detection.\n");
	}

	if (rv) {
		if (!test_and_set_bit(BIT_USB_ONLINE, &uh->state))
			set_bit(BIT_USB_CHANGE, &uh->state);
	}else{
		/* Clear keep alive bit. */
		clear_bit(BIT_KEEP_ALIVE, &uh->state);

		if (test_and_clear_bit(BIT_USB_ONLINE, &uh->state))
			set_bit(BIT_USB_CHANGE, &uh->state);
	}
	
	set_bit(BIT_USB_DETECT_DONE, &uh->state);
	return;
}

/* USB is active ? */
static int usb_is_active(void)
{
	unsigned long tmp;
	
	tmp = REG16(USB_REG_FRAME);
	
	mdelay(2);		/* USB 1.1 Frame length is 1ms, USB 2.0 HS Frame length is 125us */
	
	rmb();
	
	return tmp == REG16(USB_REG_FRAME) ? 0 : 1;
}

/* Broadcast event to notifier */
static void do_broadcast_event(struct uh_data *uh)
{
	udc_hotplug_event_t *e = &uh->cur_uh_event;
	
	/* Collect Information */
	if (test_and_clear_bit(BIT_CABLE_CHANGE, &uh->state)) {
		e->type = EVENT_TYPE_CABLE;
		e->state = (test_bit(BIT_CABLE_ONLINE, &uh->state)) ? EVENT_STATE_ONLINE: EVENT_STATE_OFFLINE;
		e->flags = 0;

		D("Broadcast cable event -> State: %s.\n", (e->state == EVENT_STATE_ONLINE ? "Online" : "Offline"));

		/* Kick chain. */
		blocking_notifier_call_chain(&uh->notifier_head, BROADCAST_TYPE_EVENT, e);
	}

	if (test_and_clear_bit(BIT_USB_CHANGE, &uh->state)) {
		e->type = EVENT_TYPE_USB;
		e->state = (test_bit(BIT_USB_ONLINE, &uh->state)) ? EVENT_STATE_ONLINE : EVENT_STATE_OFFLINE;
		e->flags = 0;

		if (test_bit(BIT_UDC_PHY_TOUCHED, &uh->state)) {
			set_bit(EVENT_FLAG_UDC_PHY_TOUCHED, &e->flags);
		}
		
		D("Broadcast USB event -> State: %s.\n", (e->state == EVENT_STATE_ONLINE ? "Online" : "Offline"));

               /* Kick chain. */
		blocking_notifier_call_chain(&uh->notifier_head, BROADCAST_TYPE_EVENT, e);
	}

	return;
}

/* Handle pending request */
static inline void handle_request(struct uh_data *uh)
{
	if (test_and_clear_bit(BIT_REQUEST_CABLE_DETECT, &uh->state))
		set_bit(BIT_DO_CABLE_DETECT, &uh->state);
	
	if (test_and_clear_bit(BIT_REQUEST_USB_DETECT, &uh->state))
		set_bit(BIT_DO_USB_DETECT, &uh->state);

	return;
}

/* Have pending request ? */
static inline int pending_request(struct uh_data *uh)
{
	if (test_bit(BIT_REQUEST_CABLE_DETECT, &uh->state) || test_bit(BIT_REQUEST_USB_DETECT, &uh->state))
		return 1;
	else
		return 0;
}

#if defined (HAVE_DETECT_SYNC)
static void prepare_restart(struct uh_data *uh, wait_queue_head_t *wq)
{
	
	D("Called.\n");

	if (test_bit(BIT_CABLE_DETECT_DONE, &uh->state))
		set_bit(BIT_DO_CABLE_DETECT, &uh->state);
			
	if (test_bit(BIT_USB_DETECT_DONE, &uh->state))
		set_bit(BIT_DO_USB_DETECT, &uh->state);

	sleep_on_timeout(wq, NR_JIFFIES_SLEEP_BEFORE_RESTART);

	sync_seq(uh);
	
	return;
}

/* Called from kernel thread */
static void udc_pnp_detect(struct uh_data *uh)
{
	int nr_restart = 0;

	D("Do UDC detection.\n");

	while (nr_restart != NR_RESTART_TIMES) {
		/* Do cable detection ? */
		if (test_bit(BIT_DO_CABLE_DETECT, &uh->state)) {
			D("Do cable detection.\n");

			cable_detect(uh);
		}
		
		/* Need restart ? */
		if (!is_seq_sync(uh)) {
			nr_restart ++;

			prepare_restart(uh, &uh->wq);
			continue;
		}
		
		/* Do USB detection ? */
		if (test_bit(BIT_DO_USB_DETECT, &uh->state)) {
			D("Do USB detection.\n");

			usb_detect(uh);
		}
		
		/* Need restart ? */
		if (!is_seq_sync(uh)) {
			nr_restart ++;

			prepare_restart(uh, &uh->wq);
			continue;
		} 
		
		/* Done */
		D("Done.\n");
		
		clear_bit(BIT_DO_CABLE_DETECT, &uh->state);
		clear_bit(BIT_DO_USB_DETECT, &uh->state);
		
		break;
	}
	
	return;
}

static inline void broadcast_event(struct uh_data *uh)
{
        /* Sync ? */
        if (is_seq_sync(uh)) {
		D("Sync -> Broadcast event.\n");

		do_broadcast_event(uh);
	}else{		
		D("Not sync -> Prepare restarting.\n");

		prepare_restart(uh, &uh->kthread_wq);
	}
}

static inline void udc_pnp_thread_sleep(struct uh_data *uh)
{
	/* Sync ? -> Sleep. */
	if ( !pending_request(uh) || is_seq_sync(uh)) {
		D("Sleep.\n");

		sleep_on(&uh->kthread_wq);
	}

	return;
}
 
#else /* !HAVE_DETECT_SYNC */

/* Called from kernel thread */
static void udc_pnp_detect(struct uh_data *uh)
{
	D("Do UDC detection.\n");

	/* Do cable detection ? */
	if (test_bit(BIT_DO_CABLE_DETECT, &uh->state)) {
		D("Do cable detection.\n");

		cable_detect(uh);
	}
		
	/* Do USB detection ? */
	if (test_bit(BIT_DO_USB_DETECT, &uh->state)) {
		D("Do USB detection.\n");

		usb_detect(uh);
	}
		
	/* Done */
	D("Done.\n");
		
	clear_bit(BIT_DO_CABLE_DETECT, &uh->state);
	clear_bit(BIT_DO_USB_DETECT, &uh->state);
		
	return;
}

static inline void broadcast_event(struct uh_data *uh)
{
	D("Broadcast event.\n");

	do_broadcast_event(uh);
	
	return;
}

static inline void udc_pnp_thread_sleep(struct uh_data *uh)
{
	if (!pending_request(uh)) {
		D("Sleep.\n");

		sleep_on(&uh->kthread_wq);
	}

	return;
}
#endif /* HAVE_DETECT_SYNC */

/* Kernel thread */
static int udc_pnp_thread(void *data)
{
	struct uh_data *uh = (struct uh_data *)data;
	
	while (!kthread_should_stop()) {
		/* Sleep. */
		udc_pnp_thread_sleep(uh);

		D("Running.\n");

		if (kthread_should_stop())
			break;

#if defined (HAVE_DETECT_SYNC)
		/* Sync */
		sync_seq(uh);
#endif		

		D("Will do UDC detection.\n");

		handle_request(uh);

		/* Do detect */
		udc_pnp_detect(uh);
		
		D("Done.\n");
		
		/* Broadcast event. */
		broadcast_event(uh);
	}
	
	D("Exit.\n");

	return 0;
}

static irqreturn_t udc_pnp_irq(int irq, void *dev_id)
{	
	struct uh_data *uh = (struct uh_data *)dev_id;
       
	D("called.\n");

        /* clear interrupt pending status */
        __gpio_ack_irq(UDC_HOTPLUG_PIN); 

        set_bit(BIT_REQUEST_CABLE_DETECT, &uh->state);
	set_bit(BIT_REQUEST_USB_DETECT, &uh->state);

	start_detect(uh);

	return IRQ_HANDLED;
}

static void __init init_gpio(struct uh_data *uh)
{
        /* get current pin level */
	__gpio_disable_pull(UDC_HOTPLUG_PIN);
        __gpio_as_input(UDC_HOTPLUG_PIN);
	udelay(1);
	
	cable_detect(uh);
	
	/* Because of every plug IN/OUT action will casue more than one interrupt, 
	   So whether rising trigger or falling trigger method can both start the detection.
         */

	__gpio_as_irq_rise_edge(UDC_HOTPLUG_PIN);

        if (test_bit(BIT_CABLE_ONLINE, &uh->state)) {
		D("Cable Online -> Do start detection.\n");

		set_bit(BIT_REQUEST_CABLE_DETECT, &uh->state);
		set_bit(BIT_REQUEST_USB_DETECT, &uh->state);
		
		start_detect(uh);
        }else{
		D("Cable Offline.\n");
        }
	
	return;
}

/* ---------------------------------------------------------------------------------- */
/* Export routines */
static void udc_hotplug_keep_alive_timer_func(unsigned long data)
{
	struct uh_data *uh = (struct uh_data *)data;
	
	D("Timer running.\n");
	
	/* Decrease the counter. */
	if (test_bit(BIT_KEEP_ALIVE, &uh->state) && !(--uh->keep_alive_counter)) {
		
		if (!usb_is_active()) {
			D("Timeout.\n");
		
			set_bit(BIT_KEEP_ALIVE_TIMEOUT, &uh->state);

			clear_bit(BIT_USB_ONLINE, &uh->state);		
			set_bit(BIT_USB_CHANGE, &uh->state);
		
			/* No detection needed. We just want to broadcast our event. */
			start_detect(uh);
		}
	}
	
	/* Set next active time. */
	if (test_bit(BIT_KEEP_ALIVE, &uh->state) && !test_bit(BIT_KEEP_ALIVE_TIMEOUT, &uh->state))
		mod_timer(&uh->keep_alive_timer, uh->keep_alive_timer_interval + jiffies);
	else
		D("Timer will stop.\n");

	return;
}

int udc_hotplug_register_notifier(struct notifier_block *n, int request_state)
{
	struct uh_data *uh = g_puh_data;
	
	udc_hotplug_event_t e;

	D("Register notifier: 0x%p.\n", (void *)n);
	
	/* Notifer will be registered is requesting current state. */
	if (request_state) {

		BUG_ON(!n->notifier_call);

		/* Cable State */
		e.type = EVENT_TYPE_CABLE;
		e.state = (test_bit(BIT_CABLE_ONLINE, &uh->state)) ? EVENT_STATE_ONLINE: EVENT_STATE_OFFLINE;
		
		n->notifier_call(n, BROADCAST_TYPE_STATE, &e);
		
		/* USB State */
		e.type = EVENT_TYPE_USB;
		e.state = (test_bit(BIT_CABLE_ONLINE, &uh->state)) ? EVENT_STATE_ONLINE: EVENT_STATE_OFFLINE;
		
		n->notifier_call(n, BROADCAST_TYPE_STATE, &e);
	}

	return blocking_notifier_chain_register(&uh->notifier_head, n);

}EXPORT_SYMBOL(udc_hotplug_register_notifier);

int udc_hotplug_unregister_notifier(struct notifier_block *n)
{
	struct uh_data *uh = g_puh_data;
	
	D("Unregister notifier: 0x%p.\n", (void *)n);

	return blocking_notifier_chain_unregister(&uh->notifier_head, n);

}EXPORT_SYMBOL(udc_hotplug_unregister_notifier);

/* Start keep alive, 0 - Use default value */
int udc_hotplug_start_keep_alive(unsigned long timer_interval_in_jiffies, unsigned long counter_limit)
{
	struct uh_data *uh = g_puh_data;
	
	/* Already started. */
	if (test_and_set_bit(BIT_KEEP_ALIVE, &uh->state))
		return 0;
	
	if (timer_interval_in_jiffies)
		uh->keep_alive_timer_interval = timer_interval_in_jiffies;
	else
		uh->keep_alive_timer_interval = DEFAULT_KEEP_ALIVE_TIMER_INTERVAL;
	
	if (counter_limit)
		uh->keep_alive_counter_limit = counter_limit;
	else
		uh->keep_alive_counter_limit = DEFAULT_KEEP_ALIVE_COUNTER_LIMIT;

	uh->keep_alive_counter = uh->keep_alive_counter_limit;
	
	/* Active our timer. */
	return mod_timer(&uh->keep_alive_timer, 3 + jiffies);

}EXPORT_SYMBOL(udc_hotplug_start_keep_alive);

void udc_hotplug_do_keep_alive(void)
{
	struct uh_data *uh = g_puh_data;
	
	D("Keep alive.\n");

        /* Reset counter */
	uh->keep_alive_counter = uh->keep_alive_counter_limit;
	
	/* We are alive again. */
	if (test_and_clear_bit(BIT_KEEP_ALIVE_TIMEOUT, &uh->state)) {
		D("Reactive timer.\n");

		/* Active timer. */
		set_bit(BIT_KEEP_ALIVE, &uh->state);
		mod_timer(&uh->keep_alive_timer, 3 + jiffies);
	}
	
	return;
}EXPORT_SYMBOL(udc_hotplug_do_keep_alive);

void udc_hotplug_stop_keep_alive(void)
{
	struct uh_data *uh = g_puh_data;
	
	clear_bit(BIT_KEEP_ALIVE, &uh->state);

	return;

}EXPORT_SYMBOL(udc_hotplug_stop_keep_alive);

/* ----------------------------------------------------------------------------- */

/*
 * Module init and exit
 */
static int __init udc_hotplug_init(void)
{
	struct uh_data *uh;

	unsigned long status = 0;

        int rv;
	
	g_puh_data = (struct uh_data *)kzalloc(sizeof(struct uh_data), GFP_KERNEL);
	if (!g_puh_data) {
		printk(KERN_ERR PFX": Failed to allocate memory.\n");
		return -ENOMEM;
	}
	
	uh = g_puh_data;

	set_bit(1, &status);
	
	BLOCKING_INIT_NOTIFIER_HEAD(&uh->notifier_head);

	init_waitqueue_head(&uh->kthread_wq);
	init_waitqueue_head(&uh->wq);
	
	init_timer(&uh->keep_alive_timer);
	
	uh->keep_alive_timer.function = udc_hotplug_keep_alive_timer_func;
	uh->keep_alive_timer.expires = jiffies - 1; /* Add a stopped timer */
	uh->keep_alive_timer.data = (unsigned long)uh;
	
	add_timer(&uh->keep_alive_timer);

#if defined (HAVE_DETECT_SYNC)
	reset_seq(uh);
#endif
	
	/* Create pnp thread and register IRQ */
	uh->kthread = kthread_run(udc_pnp_thread, uh, "kudcd");
	if (IS_ERR(uh->kthread)) {
		printk(KERN_ERR PFX": Failed to create system monitor thread.\n");
		rv = PTR_ERR(uh->kthread);
		goto err;
	}
	
	set_bit(2, &status);

        rv = request_irq(UDC_HOTPLUG_IRQ, udc_pnp_irq, IRQF_DISABLED, "udc_pnp", uh);
        if (rv) {
                printk(KERN_ERR PFX": Could not get udc hotplug irq %d\n", UDC_HOTPLUG_IRQ);
		goto err;
        }

	init_gpio(uh);

#if defined (HAVE_DETECT_SYNC)
	printk(KERN_ERR PFX": Registered(HAVE_DETECT_SYNC).\n");
#else
	printk(KERN_ERR PFX": Registered.\n");	
#endif
	return 0;

err:
	if (test_bit(2, &status)) {
		kthread_stop(uh->kthread);
	}
	
	if (test_bit(1, &status)) {
		kfree(g_puh_data);
	}

	return rv;
}

static void __exit udc_hotplug_exit(void)
{
	free_irq(UDC_HOTPLUG_IRQ, g_puh_data);

	kthread_stop(g_puh_data->kthread);

	kfree(g_puh_data);
	
	return;
}

module_init(udc_hotplug_init);
module_exit(udc_hotplug_exit);

MODULE_AUTHOR("River Wang <zwang@ingenic.cn>");
MODULE_LICENSE("GPL");
