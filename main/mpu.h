#ifndef MPU_H_
#define MPU_H_

#include "esp_log.h"

void taskMpu(void *arg);

esp_err_t mpuInitMaster();

#endif