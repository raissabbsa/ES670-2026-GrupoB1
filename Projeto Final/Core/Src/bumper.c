/**
 * @file bumper.c
 * @brief Adicionado em feature/joao por João Santos.
 */
#include "bumper.h"

static volatile uint8_t g_bumper_flag = 0U;

void Bumper_Init(void)       { g_bumper_flag = 0U; }
void Bumper_Trigger(void)    { g_bumper_flag = 1U; }
uint8_t Bumper_IsTriggered(void) { return g_bumper_flag; }
void Bumper_Clear(void)      { g_bumper_flag = 0U; }
