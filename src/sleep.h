#pragma once

#include "Arduino.h"
#include "Observer.h"
#include "configuration.h"

void setCPUFast(bool on);

extern int bootCount;

/// Called to tell observers we are rebooting ASAP.  Must return 0
extern Observable<void *> notifyReboot;
