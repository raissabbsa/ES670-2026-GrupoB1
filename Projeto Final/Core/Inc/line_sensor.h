#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdint.h>

#define LINE_SENSOR_COUNT       5
#define LINE_THRESHOLD          0.4f
#define LINE_CENTER_ACTIVE_THRESHOLD  0.55f
#define LINE_SIDE_ACTIVE_THRESHOLD    0.25f

typedef enum {
    LINE_POLARITY_LIGHT,
    LINE_POLARITY_DARK,
} LinePolarity;

typedef enum {
    LINE_ON_TRACK,
    LINE_CROSSING,
    LINE_LOST,
} LineSensor_State;

void LineSensor_Init(void);
void LineSensor_ResetCalibration(void);
void LineSensor_UpdateCalibration(const uint16_t values[LINE_SENSOR_COUNT]);
void LineSensor_FinalizeCalibration(void);
void LineSensor_ReadAll(uint16_t values[LINE_SENSOR_COUNT]);
uint16_t LineSensor_GetRawSpread(const uint16_t values[LINE_SENSOR_COUNT]);
float LineSensor_GetSensorValue(uint8_t index, uint16_t values[LINE_SENSOR_COUNT]);
void LineSensor_GetNormalizedAll(const uint16_t values[LINE_SENSOR_COUNT],
                                 float normalized[LINE_SENSOR_COUNT]);
uint8_t LineSensor_IsCalibrationValid(void);
void LineSensor_GetCalibrationBounds(uint8_t index, uint16_t *min_value, uint16_t *max_value);
float LineSensor_GetCentroidIndex(const uint16_t values[LINE_SENSOR_COUNT]);
float LineSensor_GetInterpolatedValue(uint16_t values[LINE_SENSOR_COUNT]);
void LineSensor_SetCenterTarget(float center_index);
float LineSensor_GetCenterTarget(void);
LineSensor_State LineSensor_GetState(uint16_t values[LINE_SENSOR_COUNT]);
uint8_t LineSensor_IsLineVisible(const uint16_t values[LINE_SENSOR_COUNT]);
uint8_t LineSensor_GetActiveCount(uint16_t values[LINE_SENSOR_COUNT]);

void LineSensor_SetPolarity(LinePolarity polarity);
LinePolarity LineSensor_GetPolarity(void);
LinePolarity LineSensor_DetectPolarity(void);
LinePolarity LineSensor_DetectPolarityFromLayout(const uint16_t values[LINE_SENSOR_COUNT]);

#endif /* LINE_SENSOR_H */
