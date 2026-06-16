/**
 * encoders.h
 * Driver de leitura de velocidade pelos encoders ópticos das rodas.
 * Mede o PERÍODO entre dois pulsos consecutivos via Input Capture (TIM16/TIM17).
 *
 * O período é guardado em ticks BRUTOS do timer (não em µs): o clock de tick
 * é calculado em tempo de execução em ENC_Init(), a partir do PSC configurado
 * no momento (ver encoders.c). Isso evita assumir 1 µs/tick fixo — útil porque
 * o .ioc do projeto pode ter um Prescaler diferente do sugerido em
 * 02_Driver_Encoders.md (PSC=169).
 */
#ifndef ENCODERS_H
#define ENCODERS_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/* Parâmetros mecânicos do kit ES670 (CONFIRMAR com medições no SEU robô) */
#define ENC_PULSES_PER_REV   20        /* discos com 20 furos */
#define WHEEL_DIAMETER_M     0.0670f   /* 67 mm de diâmetro */
#define WHEEL_CIRC_M         (3.14159265f * WHEEL_DIAMETER_M)
#define M_PER_PULSE          (WHEEL_CIRC_M / ENC_PULSES_PER_REV)

/* Timeout: se nenhum pulso chegar por > ENC_TIMEOUT_MS, declara roda parada */
#define ENC_TIMEOUT_MS       250

typedef struct
{
    /* atualizado no IRQ */
    volatile uint32_t period_ticks; /* tempo entre os 2 últimos pulsos, em ticks do timer */
    volatile uint32_t last_tick;    /* HAL_GetTick() do último pulso */
    volatile uint32_t pulse_count;  /* contagem absoluta (p/ odometria) */
    volatile uint16_t last_capture; /* último valor de CCR1 */
    volatile uint16_t overflow_cnt; /* nº de overflows do timer desde o último pulso */

    /* atualizado em ENC_Update() (chamado da task) */
    float    rpm;
    float    speed_mps;             /* velocidade linear da roda */
} encoder_t;

extern encoder_t g_encL;   /* esquerdo  (TIM16) */
extern encoder_t g_encR;   /* direito   (TIM17) */

void  ENC_Init(void);
void  ENC_Update(void);                       /* chame ~50 Hz da task */
float ENC_GetSpeedL(void);                    /* m/s */
float ENC_GetSpeedR(void);
int32_t ENC_GetPulsesL(void);                 /* p/ odometria */
int32_t ENC_GetPulsesR(void);
void  ENC_ResetPulses(void);                  /* zera contagem (início de trajeto) */

#endif
