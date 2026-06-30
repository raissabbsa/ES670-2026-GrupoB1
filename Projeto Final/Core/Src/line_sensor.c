#include "line_sensor.h"
#include "main.h"

extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc4;
extern ADC_HandleTypeDef hadc5;

/*
 * Sensores:
 *   IR1 = PA0 -> ADC2 IN1
 *   IR2 = PA6 -> ADC2 IN3
 *   IR3 = PB13 -> ADC3 IN5
 *   IR4 = PB15 -> ADC4 IN5
 *   IR5 = PA9 -> ADC5 IN2
 */

#define LINE_RESET_MAX  0U
#define LINE_RESET_MIN  4095U
#define LINE_MIN_RAW_SPREAD  6U
#define LINE_MIN_CALIB_RANGE 40U
#define LINE_ERROR_CLAMP     0.35f
#define LINE_OUTLIER_DELTA   120U
#define LINE_ADC_MAX_VALID   3500U

static uint16_t s_min[LINE_SENSOR_COUNT];
static uint16_t s_max[LINE_SENSOR_COUNT];
static float s_weights[LINE_SENSOR_COUNT] = {-1.0f, -0.5f, 0.0f, 0.5f, 1.0f};

static uint8_t LineSensor_CountRawActive(const uint16_t values[LINE_SENSOR_COUNT]);

static LinePolarity s_polarity = LINE_POLARITY_LIGHT;
static float s_center_target = 2.6f;
static uint32_t s_calib_sum_center = 0;
static uint32_t s_calib_count = 0;

#define LINE_ADC_SAMPLES 4U

static uint16_t LineSensor_ReadChannel(ADC_HandleTypeDef *hadc, uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    HAL_ADC_ConfigChannel(hadc, &sConfig);

    uint32_t sum = 0;
    for (uint8_t i = 0; i < LINE_ADC_SAMPLES; i++) {
        HAL_ADC_Start(hadc);
        HAL_ADC_PollForConversion(hadc, 10);
        sum += HAL_ADC_GetValue(hadc);
        HAL_ADC_Stop(hadc);
    }

    return (uint16_t)(sum / LINE_ADC_SAMPLES);
}

static float LineSensor_Normalize(uint8_t index, uint16_t raw)
{
    float range = (float)s_max[index] - (float)s_min[index];

    if (range < (float)LINE_MIN_CALIB_RANGE) {
        return 0.0f;
    }

    float value = ((float)raw - (float)s_min[index]) / range;

    if (value < 0.0f) {
        value = 0.0f;
    }
    if (value > 1.0f) {
        value = 1.0f;
    }

    if (s_polarity == LINE_POLARITY_DARK) {
        value = 1.0f - value;
    }

    return value;
}

uint16_t LineSensor_GetRawSpread(const uint16_t values[LINE_SENSOR_COUNT])
{
    uint16_t mn = values[0];
    uint16_t mx = values[0];

    for (uint8_t i = 1; i < LINE_SENSOR_COUNT; i++) {
        if (values[i] < mn) {
            mn = values[i];
        }
        if (values[i] > mx) {
            mx = values[i];
        }
    }

    return (uint16_t)(mx - mn);
}

static float LineSensor_GetDifferentialValue(const uint16_t values[LINE_SENSOR_COUNT])
{
    float mean = 0.0f;
    uint16_t spread = LineSensor_GetRawSpread(values);

    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) {
        mean += (float)values[i];
    }
    mean /= (float)LINE_SENSOR_COUNT;

    float neg_max = 0.0f;
    float pos_max = 0.0f;
    float value = 0.0f;

    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) {
        float diff = mean - (float)values[i];

        if (s_weights[i] > 0.0f) {
            pos_max += s_weights[i];
        } else {
            neg_max -= s_weights[i];
        }

        value += diff * s_weights[i];
    }

    if (spread < LINE_MIN_RAW_SPREAD) {
        return 0.0f;
    }

    float denom = (value > 0.0f) ? (pos_max * (float)spread)
                                 : (neg_max * (float)spread);
    if (denom < 1.0f) {
        return 0.0f;
    }

    return value / denom;
}

static float LineSensor_GetThickLineError(const uint16_t values[LINE_SENSOR_COUNT])
{
    uint8_t line_idx = 0U;

    for (uint8_t i = 1U; i < LINE_SENSOR_COUNT; i++) {
        if (s_polarity == LINE_POLARITY_DARK) {
            if (values[i] < values[line_idx]) {
                line_idx = i;
            }
        } else if (values[i] > values[line_idx]) {
            line_idx = i;
        }
    }

    float pos = ((float)line_idx - 2.0f) / 2.0f;
    return pos * LINE_ERROR_CLAMP;
}

static uint8_t LineSensor_IsOnThickLine(const uint16_t values[LINE_SENSOR_COUNT])
{
    uint32_t sum = 0U;

    for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++) {
        sum += values[i];
    }

    uint16_t mean = (uint16_t)(sum / LINE_SENSOR_COUNT);

    if (s_polarity == LINE_POLARITY_DARK && mean < 280U) {
        return 1U;
    }
    if (s_polarity == LINE_POLARITY_LIGHT && mean > 180U) {
        return 1U;
    }

    return 0U;
}

uint8_t LineSensor_IsLineVisible(const uint16_t values[LINE_SENSOR_COUNT])
{
    if (LineSensor_GetRawSpread(values) >= LINE_MIN_RAW_SPREAD) {
        return 1U;
    }

    if (LineSensor_CountRawActive(values) > 0U) {
        return 1U;
    }

    if (LineSensor_IsOnThickLine(values)) {
        return 1U;
    }

  /* Antes da polaridade definida: qualquer contraste visivel */
    uint16_t mn = values[0];
    uint16_t mx = values[0];

    for (uint8_t i = 1U; i < LINE_SENSOR_COUNT; i++) {
        if (values[i] < mn) {
            mn = values[i];
        }
        if (values[i] > mx) {
            mx = values[i];
        }
    }

    if ((uint16_t)(mx - mn) >= 8U) {
        return 1U;
    }

    return 0U;
}

static float LineSensor_ClampError(float value)
{
    if (value > LINE_ERROR_CLAMP) {
        return LINE_ERROR_CLAMP;
    }
    if (value < -LINE_ERROR_CLAMP) {
        return -LINE_ERROR_CLAMP;
    }
    return value;
}

static void LineSensor_FilterOutliers(uint16_t values[LINE_SENSOR_COUNT])
{
    uint32_t sum = 0U;
    uint8_t valid_count = 0U;

    for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++) {
        if (values[i] <= LINE_ADC_MAX_VALID) {
            sum += values[i];
            valid_count++;
        }
    }

    if (valid_count == 0U) {
        for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++) {
            values[i] = 300U;
        }
        return;
    }

    uint16_t mean = (uint16_t)(sum / valid_count);

    for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++) {
        uint16_t raw = values[i];

        if (raw > LINE_ADC_MAX_VALID) {
            values[i] = mean;
            raw = mean;
        }

        uint16_t delta = (raw > mean) ? (raw - mean) : (mean - raw);
        if (delta > LINE_OUTLIER_DELTA) {
            values[i] = mean;
        }
    }
}

void LineSensor_Init(void)
{
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc5, ADC_SINGLE_ENDED);

    LineSensor_ResetCalibration();
    s_polarity = LINE_POLARITY_LIGHT;
    s_center_target = 2.6f;
}

void LineSensor_ResetCalibration(void)
{
    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) {
        s_max[i] = LINE_RESET_MAX;
        s_min[i] = LINE_RESET_MIN;
    }
    s_calib_sum_center = 0;
    s_calib_count = 0;
}

void LineSensor_UpdateCalibration(const uint16_t values[LINE_SENSOR_COUNT])
{
    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) {
        if (values[i] > s_max[i]) {
            s_max[i] = values[i];
        }
        if (values[i] < s_min[i]) {
            s_min[i] = values[i];
        }
    }

    s_calib_sum_center += values[2];
    s_calib_count++;
}

void LineSensor_ReadAll(uint16_t values[LINE_SENSOR_COUNT])
{
    values[0] = LineSensor_ReadChannel(&hadc2, ADC_CHANNEL_1);
    values[1] = LineSensor_ReadChannel(&hadc2, ADC_CHANNEL_3);
    values[2] = LineSensor_ReadChannel(&hadc3, ADC_CHANNEL_5);
    values[3] = LineSensor_ReadChannel(&hadc4, ADC_CHANNEL_5);
    values[4] = LineSensor_ReadChannel(&hadc5, ADC_CHANNEL_2);

    LineSensor_FilterOutliers(values);
}

float LineSensor_GetSensorValue(uint8_t index, uint16_t values[LINE_SENSOR_COUNT])
{
    if (index >= LINE_SENSOR_COUNT) {
        return 0.0f;
    }

    return LineSensor_Normalize(index, values[index]);
}

float LineSensor_GetCentroidIndex(const uint16_t values[LINE_SENSOR_COUNT])
{
    uint16_t spread = LineSensor_GetRawSpread(values);
    LinePolarity pol = s_polarity;

    if (spread < LINE_MIN_RAW_SPREAD) {
        return 2.0f;
    }

    uint32_t sum = 0U;
    for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++) {
        sum += values[i];
    }

    float mean = (float)sum / (float)LINE_SENSOR_COUNT;
    float margin = (float)spread * 0.18f;
    if (margin < 3.0f) {
        margin = 3.0f;
    }

    float sum_w = 0.0f;
    float sum_pos = 0.0f;

    for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++) {
        float w;

        if (pol == LINE_POLARITY_DARK) {
            w = mean - (float)values[i];
        } else {
            w = (float)values[i] - mean;
        }

        if (w < margin) {
            continue;
        }

        sum_w += w;
        sum_pos += (float)i * w;
    }

    if (sum_w < 1.0f) {
        uint8_t line_idx = 0U;

        for (uint8_t i = 1U; i < LINE_SENSOR_COUNT; i++) {
            if (pol == LINE_POLARITY_DARK) {
                if (values[i] < values[line_idx]) {
                    line_idx = i;
                }
            } else if (values[i] > values[line_idx]) {
                line_idx = i;
            }
        }

        return (float)line_idx;
    }

    return sum_pos / sum_w;
}

void LineSensor_SetCenterTarget(float center_index)
{
    if (center_index < 1.0f) {
        center_index = 1.0f;
    }
    if (center_index > 3.0f) {
        center_index = 3.0f;
    }

    s_center_target = center_index;
}

float LineSensor_GetCenterTarget(void)
{
    return s_center_target;
}

float LineSensor_GetInterpolatedValue(uint16_t values[LINE_SENSOR_COUNT])
{
    uint16_t spread = LineSensor_GetRawSpread(values);

    if (spread < LINE_MIN_RAW_SPREAD) {
        return 0.0f;
    }

    float centroid = LineSensor_GetCentroidIndex(values);
    float error = (centroid - s_center_target) / 2.0f;

    /* Leituras com spread muito alto sao instaveis (largada, cruzamento) */
    if (spread > 65U) {
        error *= 65.0f / (float)spread;
    }

    /* Zona morta: no centro, anda reto sem corrigir */
    if (error > -0.05f && error < 0.05f) {
        return 0.0f;
    }

    return LineSensor_ClampError(error);
}

static uint8_t LineSensor_CountRawActive(const uint16_t values[LINE_SENSOR_COUNT])
{
    uint16_t spread = LineSensor_GetRawSpread(values);

    if (spread < LINE_MIN_RAW_SPREAD) {
        return 0U;
    }

    uint32_t sum = 0U;
    for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++) {
        sum += values[i];
    }

    float mean = (float)sum / (float)LINE_SENSOR_COUNT;
    float margin = (float)spread * 0.28f;
    if (margin < 4.0f) {
        margin = 4.0f;
    }

    uint8_t count = 0U;
    for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++) {
        float delta = mean - (float)values[i];

        if (s_polarity == LINE_POLARITY_DARK) {
            if (delta > margin) {
                count++;
            }
        } else if (-delta > margin) {
            count++;
        }
    }

    return count;
}

LineSensor_State LineSensor_GetState(uint16_t values[LINE_SENSOR_COUNT])
{
    uint16_t spread = LineSensor_GetRawSpread(values);
    uint8_t active = LineSensor_GetActiveCount(values);

    if (spread < 8U && active == 0U && !LineSensor_IsOnThickLine(values)) {
        return LINE_LOST;
    }

    if (active >= 4U && spread > 55U) {
        return LINE_CROSSING;
    }

    if (spread >= LINE_MIN_RAW_SPREAD || active > 0U || LineSensor_IsOnThickLine(values)) {
        return LINE_ON_TRACK;
    }

    return LINE_LOST;
}

uint8_t LineSensor_GetActiveCount(uint16_t values[LINE_SENSOR_COUNT])
{
    uint8_t count = LineSensor_CountRawActive(values);

    if (count > 0U) {
        return count;
    }

    if (LineSensor_GetRawSpread(values) >= LINE_MIN_RAW_SPREAD) {
        return 2U;
    }

    if (LineSensor_IsOnThickLine(values)) {
        return 3U;
    }

    return 0U;
}

void LineSensor_SetPolarity(LinePolarity polarity)
{
    s_polarity = polarity;
}

LinePolarity LineSensor_GetPolarity(void)
{
    return s_polarity;
}

LinePolarity LineSensor_DetectPolarityFromLayout(const uint16_t values[LINE_SENSOR_COUNT])
{
    uint16_t mn = values[0];
    uint16_t mx = values[0];
    uint32_t sum = 0U;

    for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++) {
        if (values[i] < mn) {
            mn = values[i];
        }
        if (values[i] > mx) {
            mx = values[i];
        }
        sum += values[i];
    }

    uint16_t spread = (uint16_t)(mx - mn);
    if (spread < LINE_MIN_RAW_SPREAD) {
        return LINE_POLARITY_LIGHT;
    }

    float mean = (float)sum / (float)LINE_SENSOR_COUNT;
    if ((float)mn < mean - ((float)spread * 0.25f)) {
        return LINE_POLARITY_DARK;
    }

    return LINE_POLARITY_LIGHT;
}

LinePolarity LineSensor_DetectPolarity(void)
{
    if (s_calib_count == 0U) {
        return LINE_POLARITY_LIGHT;
    }

    float avg_center = (float)s_calib_sum_center / (float)s_calib_count;
    float midpoint = ((float)s_min[2] + (float)s_max[2]) / 2.0f;

    uint16_t range = s_max[2] - s_min[2];
    if (range < 200U) {
        if (avg_center > midpoint) {
            return LINE_POLARITY_LIGHT;
        }
        return LINE_POLARITY_DARK;
    }

    if (avg_center > midpoint) {
        return LINE_POLARITY_LIGHT;
    }
    return LINE_POLARITY_DARK;
}
