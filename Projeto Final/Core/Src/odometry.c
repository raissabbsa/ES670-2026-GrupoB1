/**
 * @file odometry.c
 * @brief Adicionado em feature/joao por João Santos.
 *
 * Consome diretamente os contadores expostos por encoder.c (Encoder_GetCountLeft
 * / Encoder_GetCountRight). Como os encoders ópticos são unidirecionais,
 * assumimos direção positiva (forward) — quando precisarmos de ré, basta
 * estender com o sinal vindo do controle dos motores.
 */
#include <math.h>
#include "odometry.h"
#include "encoder.h"

static int32_t s_last_left  = 0;
static int32_t s_last_right = 0;
static float   s_x = 0.0f;
static float   s_y = 0.0f;
static float   s_theta = 0.0f;             /* rad */
static float   s_distance_total_cm = 0.0f;

void Odometry_Init(void)
{
    Odometry_Reset();
}

void Odometry_Reset(void)
{
    s_last_left  = Encoder_GetCountLeft();
    s_last_right = Encoder_GetCountRight();
    s_x = 0.0f;
    s_y = 0.0f;
    s_theta = 0.0f;
    s_distance_total_cm = 0.0f;
}

void Odometry_Update(void)
{
    int32_t cur_left  = Encoder_GetCountLeft();
    int32_t cur_right = Encoder_GetCountRight();
    int32_t d_left    = cur_left  - s_last_left;
    int32_t d_right   = cur_right - s_last_right;
    s_last_left  = cur_left;
    s_last_right = cur_right;

    float dL_left  = (float)d_left  * ODOMETRY_CM_PER_PULSE;
    float dL_right = (float)d_right * ODOMETRY_CM_PER_PULSE;
    float dL       = 0.5f * (dL_left + dL_right);
    float dTheta   = (dL_right - dL_left) / ODOMETRY_WHEEL_BASE_CM;

    float mid_theta = s_theta + 0.5f * dTheta;
    s_x += dL * cosf(mid_theta);
    s_y += dL * sinf(mid_theta);
    s_theta += dTheta;

    /* Wrap em [-pi, pi] */
    while (s_theta >  3.14159265f) s_theta -= 6.28318530f;
    while (s_theta < -3.14159265f) s_theta += 6.28318530f;

    if (dL < 0.0f) dL = -dL;
    s_distance_total_cm += dL;
}

float Odometry_GetX(void)            { return s_x; }
float Odometry_GetY(void)            { return s_y; }
float Odometry_GetThetaDeg(void)     { return s_theta * (180.0f / 3.14159265f); }
float Odometry_GetDistanceCm(void)   { return s_distance_total_cm; }
