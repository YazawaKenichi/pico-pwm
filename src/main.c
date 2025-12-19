#include "main.h"

int main_init()
{
    led_init();
    // init_time();

    ros2_init();

    gun_init();
    servo_init();
    position_init();
}

int main()
{
    main_init();
    while(true)
    {
        ros2_spin();
    }

    return 0;
}

