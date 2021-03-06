/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifdef USERHOOK_INIT
void userhook_init()
{
  hal.gpio->pinMode(AN5, OUTPUT);
  hal.gpio->pinMode(AN6, OUTPUT);
  hal.gpio->pinMode(AN7, OUTPUT);
  hal.gpio->pinMode(AN8, OUTPUT);
  
  prepareUartB();

  battery.set_monitoring(AP_BATT_MONITOR_VOLTAGE_AND_CURRENT);
  
  //hal.uartA->begin(9600);
}
#endif

#ifdef USERHOOK_FASTLOOP
void userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void userhook_MediumLoop()
{
    // put your 10Hz code here
    sync_uart();
    arm_LED();
}
#endif

#ifdef USERHOOK_SLOWLOOP
void userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void userhook_SuperSlowLoop()
{
    static int count = 0;
    if(count++ %2 == 0){
      sendMessageStatus();
    }
}
#endif
