/*
 * File:   config.h
 * Author: max
 *
 * Created on April 27, 2016, 1:17 PM
 */

#ifndef CONFIG_H
#define	CONFIG_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <sys/types.h>
#include "stm32f1xx_hal.h"

#define DEF_GPIO_TYPE uint16_t
#define DEF_PORT_TYPE GPIO_TypeDef*

#ifdef	__cplusplus
}
#endif

#endif	/* CONFIG_H */

