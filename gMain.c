# include <stdio.h>
# include "../crazyflie-firmware/src/modules/interface/app.h"
# include "../crazyflie-firmware/vendor/FreeRTOS/include/FreeRTOS.h"
# include "myHeader.h"
# include "../crazyflie-firmware/vendor/FreeRTOS/include/task.h"
#include "../crazyflie-firmware/src/config/FreeRTOSConfig.h"

void appMain() {
    vTaskDelay(M2T(5000));
    while(1) {
        vTaskDelay(M2T(5));
        changeState();
    }
}