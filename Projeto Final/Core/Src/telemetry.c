#include "telemetry.h"
#include "cmsis_os2.h"

static Telemetry_t s_telem = {0};
static osMutexId_t s_mutex = NULL;
static const osMutexAttr_t s_mutex_attr = { .name = "TelemMtx" };

void Telemetry_Init(void)
{
    s_mutex = osMutexNew(&s_mutex_attr);
}

void Telemetry_Get(Telemetry_t *out)
{
    osMutexAcquire(s_mutex, osWaitForever);
    *out = s_telem;
    osMutexRelease(s_mutex);
}

void Telemetry_SetDistSpeed(float dist, float spd)
{
    osMutexAcquire(s_mutex, osWaitForever);
    s_telem.distance_cm = dist;
    s_telem.speed_cms = spd;
    osMutexRelease(s_mutex);
}

void Telemetry_SetHeading(float hdg)
{
    osMutexAcquire(s_mutex, osWaitForever);
    s_telem.heading_deg = hdg;
    osMutexRelease(s_mutex);
}

void Telemetry_SetBattery(float bat)
{
    osMutexAcquire(s_mutex, osWaitForever);
    s_telem.battery_pct = bat;
    osMutexRelease(s_mutex);
}

void Telemetry_SetObstacle(uint16_t obs)
{
    osMutexAcquire(s_mutex, osWaitForever);
    s_telem.obstacle_cm = obs;
    osMutexRelease(s_mutex);
}

void Telemetry_SetState(uint8_t st)
{
    osMutexAcquire(s_mutex, osWaitForever);
    s_telem.state = st;
    osMutexRelease(s_mutex);
}
