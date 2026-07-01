#include "motor.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;

/* Compensa motores diferentes: power * scale -> duty cycle PWM */
static float s_left_scale = 1.0f;
static float s_right_scale = 1.0f;

static float Motor_ClampScale(float scale)
{
    if (scale < 0.5f) {
        return 0.5f;
    }
    if (scale > 1.5f) {
        return 1.5f;
    }
    return scale;
}

void Motor_SetLeftScale(float scale)
{
    s_left_scale = Motor_ClampScale(scale);
}

void Motor_SetRightScale(float scale)
{
    s_right_scale = Motor_ClampScale(scale);
}

void Motor_GetScales(float *left, float *right)
{
    if (left != NULL) {
        *left = s_left_scale;
    }
    if (right != NULL) {
        *right = s_right_scale;
    }
}

/*
 * Mapeamento igual ao vMotorEncoderInitMotors do exemplo do professor:
 *   Motor direito: Motor_Dir_IN3=PB12, Motor_Dir_IN4=PB9, Motor_Dir_PWM=TIM1_CH2 (PC1)
 *   Motor esquerdo: Motor_Esq_IN1=PB7, Motor_Esq_IN2=PA10, Motor_Esq_PWM=TIM1_CH1 (PC0)
 */

static void Motor_SetRightDirection(uint8_t forward)
{
    if (forward) {
        HAL_GPIO_WritePin(Motor_Dir_IN3_GPIO_Port, Motor_Dir_IN3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motor_Dir_IN4_GPIO_Port, Motor_Dir_IN4_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(Motor_Dir_IN3_GPIO_Port, Motor_Dir_IN3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Motor_Dir_IN4_GPIO_Port, Motor_Dir_IN4_Pin, GPIO_PIN_RESET);
    }
}

static void Motor_SetLeftDirection(uint8_t forward)
{
    if (forward) {
        HAL_GPIO_WritePin(Motor_Esq_IN1_GPIO_Port, Motor_Esq_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Motor_Esq_IN2_GPIO_Port, Motor_Esq_IN2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(Motor_Esq_IN1_GPIO_Port, Motor_Esq_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motor_Esq_IN2_GPIO_Port, Motor_Esq_IN2_Pin, GPIO_PIN_SET);
    }
}

static void Motor_ApplyRight(float power)
{
    uint32_t compare;

    if (power <= 0.0f) {
        HAL_GPIO_WritePin(Motor_Dir_IN3_GPIO_Port, Motor_Dir_IN3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motor_Dir_IN4_GPIO_Port, Motor_Dir_IN4_Pin, GPIO_PIN_RESET);
        compare = 0U;
    } else {
        if (power > 1.0f) {
            power = 1.0f;
        }
        Motor_SetRightDirection(1U);
        compare = (uint32_t)(power * s_right_scale * (float)MOTOR_PWM_MAX);
        if (compare > MOTOR_PWM_MAX) {
            compare = MOTOR_PWM_MAX;
        }
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, compare);
}

static void Motor_ApplyLeft(float power)
{
    uint32_t compare;

    if (power <= 0.0f) {
        HAL_GPIO_WritePin(Motor_Esq_IN1_GPIO_Port, Motor_Esq_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motor_Esq_IN2_GPIO_Port, Motor_Esq_IN2_Pin, GPIO_PIN_RESET);
        compare = 0U;
    } else {
        if (power > 1.0f) {
            power = 1.0f;
        }
        Motor_SetLeftDirection(1U);
        compare = (uint32_t)(power * s_left_scale * (float)MOTOR_PWM_MAX);
        if (compare > MOTOR_PWM_MAX) {
            compare = MOTOR_PWM_MAX;
        }
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compare);
}

void Motor_Init(void)
{
    __HAL_TIM_SET_PRESCALER(&htim1, 169U);
    __HAL_TIM_SET_AUTORELOAD(&htim1, MOTOR_PWM_MAX);
    htim1.Instance->EGR = TIM_EGR_UG;

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    __HAL_TIM_MOE_ENABLE(&htim1);

    Motor_Stop();
}

static float Motor_ClampPower(float power)
{
    if (power <= 0.0f) {
        return 0.0f;
    }
    if (power < MOTOR_MIN_START_POWER) {
        return MOTOR_MIN_START_POWER;
    }
    if (power > 1.0f) {
        return 1.0f;
    }
    return power;
}

void Motor_SetPower(float left, float right)
{
    Motor_ApplyLeft(Motor_ClampPower(left));
    Motor_ApplyRight(Motor_ClampPower(right));
}

void Motor_SetPowerDiff(float left, float right)
{
    if (left <= 0.0f && right <= 0.0f) {
        Motor_Stop();
        return;
    }

    /* Giro no eixo (so uma roda > 0): clamp minimo baixo */
    if (left <= 0.0f || right <= 0.0f) {
        if (left > 0.0f && left < MOTOR_MIN_TURN_POWER) {
            left = MOTOR_MIN_TURN_POWER;
        }
        if (right > 0.0f && right < MOTOR_MIN_TURN_POWER) {
            right = MOTOR_MIN_TURN_POWER;
        }
    } else {
        /* Seguimento: ambas rodas > 0. Se uma roda e muito baixa,
           sobe as duas juntas mantendo a diferenca. */
        float lo = (left < right) ? left : right;
        if (lo < MOTOR_MIN_START_POWER) {
            float boost = MOTOR_MIN_START_POWER - lo;
            left += boost;
            right += boost;
        }
    }

    if (left > 1.0f) left = 1.0f;
    if (right > 1.0f) right = 1.0f;

    Motor_ApplyLeft(left);
    Motor_ApplyRight(right);
}

void Motor_SetSpeed(int16_t left, int16_t right)
{
    Motor_SetPower((float)left / (float)MOTOR_PWM_MAX,
                   (float)right / (float)MOTOR_PWM_MAX);
}

void Motor_Stop(void)
{
    Motor_SetPower(0.0f, 0.0f);
}

void Motor_Brake(void)
{
    HAL_GPIO_WritePin(Motor_Esq_IN1_GPIO_Port, Motor_Esq_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Motor_Esq_IN2_GPIO_Port, Motor_Esq_IN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Motor_Dir_IN3_GPIO_Port, Motor_Dir_IN3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Motor_Dir_IN4_GPIO_Port, Motor_Dir_IN4_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MOTOR_PWM_MAX);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, MOTOR_PWM_MAX);
}
