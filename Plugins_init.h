Addition to plugins_init.h

#if ODOMETER_ENABLE
    extern void odometer_init (void);
    odometer_init(); // NOTE: this *must* be last plugin to be initialized as it claims storage at the end of NVS.
#endif
