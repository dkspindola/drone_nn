#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "drone.h"
#include "state.h"

#define DEBUG_MODULE "FLY"

void appMain()
{
  DEBUG_PRINT("Waiting for activation ...\n");

  vTaskDelay(M2T(5000));

  DEBUG_PRINT("Initialize timer..");

  // initUsecTimer();
  // uint64_t start_time, end_time;

  DEBUG_PRINT("Fly!\n");

  while(1) {
    // vTaskDelay(M2T(2));
    // start_time = usecTimestamp();
    change();
    // end_time = usecTimestamp();
    // DEBUG_PRINT("Code took %llu usec to run.\n", end_time - start_time);
  }
}
