/**
 * @file task_safety.c
 * @brief Task de segurança — checa ultrassônico e bumper @ 50Hz; injeta
 *        transições STATE_STOP_OBSTACLE / STATE_EMERGENCY na FSM.
 *
 * Coexistência com a LineCtrlTask:
 *   - Se nós setarmos follower_state = STATE_STOP_OBSTACLE, a LineCtrlTask
 *     existente da Raissa não conhece esse estado e não vai escrever motor
 *     comando — ela cai no default e fica parada. Bom o suficiente como
 *     fail-safe. Forçamos motor parado enviando MotorCmd na queue.
 *   - Quando o obstáculo limpa, voltamos para STATE_FOLLOWING (ou ALIGNING
 *     se ainda não estava em RUN).
 *
 * Adicionado em feature/joao por João Santos.
 */
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "app_tasks.h"
#include "ultrasonic.h"
#include "bumper.h"
#include "odometry.h"
#include "main.h"

extern osMessageQueueId_t queueMotorCmdHandle;

#define SAFETY_PERIOD_MS              20U      /* 50 Hz */
#define SAFETY_OBSTACLE_STOP_CM       12.0f
#define SAFETY_OBSTACLE_RESUME_CM     18.0f
#define SAFETY_OBSTACLE_DEBOUNCE_N    3        /* leituras consecutivas */

static void post_motor_stop(void)
{
    MotorCmd_t cmd = { 0.0f, 0.0f };
    osMessageQueuePut(queueMotorCmdHandle, &cmd, 0, 0);
}

void App_SafetyTask(void *argument)
{
    (void)argument;
    TickType_t  next_wake = xTaskGetTickCount();
    uint8_t     obstacle_count = 0U;
    LineFollower_State pre_obstacle_state = STATE_IDLE;

    for (;;) {
        /* --- Bumper: latch absoluto --- */
        if (Bumper_IsTriggered()) {
            if (follower_state != STATE_EMERGENCY) {
                follower_state = STATE_EMERGENCY;
                post_motor_stop();
            }
        } else {
            /* --- Ultrassônico: histerese + debounce --- */
            float dist = Ultrasonic_GetDistanceCm();

            if (follower_state == STATE_FOLLOWING ||
                follower_state == STATE_IN_CROSSING) {
                if (dist < SAFETY_OBSTACLE_STOP_CM) {
                    if (++obstacle_count >= SAFETY_OBSTACLE_DEBOUNCE_N) {
                        pre_obstacle_state = follower_state;
                        follower_state = STATE_STOP_OBSTACLE;
                        post_motor_stop();
                    }
                } else {
                    obstacle_count = 0U;
                }
            } else if (follower_state == STATE_STOP_OBSTACLE) {
                if (dist > SAFETY_OBSTACLE_RESUME_CM) {
                    follower_state = (pre_obstacle_state != STATE_IDLE)
                                     ? pre_obstacle_state
                                     : STATE_FOLLOWING;
                    obstacle_count = 0U;
                } else {
                    post_motor_stop();
                }
            } else {
                obstacle_count = 0U;
            }
        }

        /* Odometria — integramos aqui mesmo (50Hz é suficiente). */
        Odometry_Update();

        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(SAFETY_PERIOD_MS));
    }
}
