#include "concurrency/InterruptableDelay.h"
#include "configuration.h"

namespace concurrency
{

InterruptableDelay::InterruptableDelay() {}

InterruptableDelay::~InterruptableDelay() {}

/**
 * Returns false if we were interrupted
 */
bool InterruptableDelay::delay(uint32_t msec)
{
    // sem take will return false if we timed out (i.e. were not interrupted)
    bool r = semaphore.take(msec);

    return !r;
}

void InterruptableDelay::interrupt()
{
    semaphore.give();
}

IRAM_ATTR void InterruptableDelay::interruptFromISR(BaseType_t *pxHigherPriorityTaskWoken)
{
    semaphore.giveFromISR(pxHigherPriorityTaskWoken);
}

} // namespace concurrency