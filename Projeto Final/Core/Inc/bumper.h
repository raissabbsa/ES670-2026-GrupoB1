/**
 * @file bumper.h
 * @brief Switch frontal de emergência (PD2 / EXTI2).
 *
 * Bumper_Trigger() é chamado da ISR EXTI (em main.c HAL_GPIO_EXTI_Callback
 * quando GPIO_Pin == Switch_Fr_Pin). A flag é um latch: limpa apenas via
 * Bumper_Clear() depois do operador resetar a emergência.
 *
 * Adicionado em feature/joao por João Santos.
 */
#ifndef BUMPER_H
#define BUMPER_H

#include "main.h"

void    Bumper_Init(void);
void    Bumper_Trigger(void);   /* call from ISR */
uint8_t Bumper_IsTriggered(void);
void    Bumper_Clear(void);

#endif /* BUMPER_H */
