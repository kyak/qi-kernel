#ifndef __ASM_SUSPEND_H
#define __ASM_SUSPEND_H

static inline int arch_prepare_suspend(void) { return 0; }
#if defined(CONFIG_PM) && defined(CONFIG_JZSOC)
extern int jz_pm_init(void);
#endif

/* References to section boundaries */
extern const void __nosave_begin, __nosave_end;

#endif /* __ASM_SUSPEND_H */
