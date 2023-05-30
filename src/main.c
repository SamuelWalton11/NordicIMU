#include "ICM20600.h"


#define radius 8.5E-3
#define distance_per_step 2.0*3.14*8.5E-3/200.0
#define Sampling_period 1 // in micro secs
#define pi 3.14159265359


void my_work_handler(struct k_work *work)
{
	
    if(gpio_pin_get_dt(&Reset_Pin))
        {
            printf("Successfully Reset Offset: %d",gpio_pin_get_dt(&Reset_Pin));
            theta_offset = theta;
            Phi_offset = phi;
            Psi_offset = psi;
        }
    SimpleComplementaryFilter(getRawAccelerationX()*9.81/1000,getRawAccelerationY()*9.81/1000,getRawAccelerationZ()*9.81/1000,getRawGyroscopeX()*2*pi/360,getRawGyroscopeY()*2*pi/360,getRawGyroscopeZ()*2*pi/360,Sampling_period*pow(10,-3),theta_offset,Phi_offset,Psi_offset);
}
K_WORK_DEFINE(my_work, my_work_handler);

void my_timer_handler(struct k_timer *timer_id)
{
	k_work_submit(&my_work);
}
K_TIMER_DEFINE(my_timer, my_timer_handler, NULL);

void main(void)
{
    ICM20600_startup();
    printf("Temperature: %d",getTemperature());
    k_timer_start(&my_timer, K_SECONDS(0), K_MSEC(Sampling_period));
    while(1)
    {
        
    }

}