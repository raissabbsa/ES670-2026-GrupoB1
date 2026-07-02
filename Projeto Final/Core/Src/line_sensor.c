#include "line_sensor.h"
#include "main.h"
#include <stddef.h>

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
#define LINE_ADC_MAX_VALID   2000U
#define LINE_MIN_NORM_SPREAD    0.05f
#define LINE_NORM_MARGIN_MIN    0.08f
#define LINE_OFFSET_MIN_SPREAD  38U

/* Defaults por sensor (5 valores).
 * Para pista do professor (chao preto, linha branca):
 *   Chao preto ~ 300 raw (baixa reflexão)
 *   Linha branca ~ 1000-1400 raw (alta reflexão)
 * Ajustar apos medicao real com a fita do grupo. */
static const uint16_t LINE_CALIB_DEFAULT_MIN_S[LINE_SENSOR_COUNT] = { 300, 300, 300, 300, 300 };
static const uint16_t LINE_CALIB_DEFAULT_MAX_S[LINE_SENSOR_COUNT] = { 1100, 1100, 1100, 1100, 1100 };

static uint16_t s_min[LINE_SENSOR_COUNT];
static uint16_t s_max[LINE_SENSOR_COUNT];
static uint16_t s_last_valid_min[LINE_SENSOR_COUNT];
static uint16_t s_last_valid_max[LINE_SENSOR_COUNT];

static uint8_t LineSensor_CountRawActive(const uint16_t values[LINE_SENSOR_COUNT]);
static uint8_t LineSensor_CountNormalizedActive(const uint16_t values[LINE_SENSOR_COUNT]);

static LinePolarity s_polarity = LINE_POLARITY_LIGHT;
static float s_center_target = 2.6f;
static uint32_t s_calib_sum_center = 0;
static uint32_t s_calib_count = 0;
static uint8_t s_calib_valid = 0U;
static uint8_t s_calib_collecting = 0U;
/* Media das leituras cruas durante a janela de calibracao. Usada para
 * detectar a polaridade: media ALTA -> linha branca (LIGHT),
 * media BAIXA -> linha preta (DARK). */
static uint32_t s_calib_sum_raw = 0U;
static uint32_t s_calib_count_raw = 0U;
static float s_calib_mean_raw = 0.0f;
/* -1 = deteccao automatica, 0 = DARK forçado, 1 = LIGHT forçado. */
static int8_t s_forced_polarity = -1;

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

static void LineSensor_LoadDefaultCalibration(void)
{
    for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++) {
        s_min[i] = LINE_CALIB_DEFAULT_MIN_S[i];
        s_max[i] = LINE_CALIB_DEFAULT_MAX_S[i];
        s_last_valid_min[i] = s_min[i];
        s_last_valid_max[i] = s_max[i];
    }

    s_calib_valid = 1U;
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

    LineSensor_LoadDefaultCalibration();
    s_polarity = LINE_POLARITY_LIGHT;
    s_center_target = 2.6f;
    s_calib_collecting = 0U;
}

void LineSensor_ResetCalibration(void)
{
    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) {
        s_max[i] = LINE_RESET_MAX;
        s_min[i] = LINE_RESET_MIN;
    }
    s_calib_sum_center = 0;
    s_calib_count = 0;
    s_calib_sum_raw = 0U;
    s_calib_count_raw = 0U;
    s_calib_mean_raw = 0.0f;
    s_calib_valid = 0U;
    s_calib_collecting = 1U;
}

void LineSensor_UpdateCalibration(const uint16_t values[LINE_SENSOR_COUNT])
{
    if (s_calib_collecting == 0U) {
        return;
    }

    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) {
        if (values[i] > s_max[i]) {
            s_max[i] = values[i];
        }
        if (values[i] < s_min[i]) {
            s_min[i] = values[i];
        }
        s_calib_sum_raw += values[i];
    }
    s_calib_count_raw++;

    s_calib_sum_center += values[2];
    s_calib_count++;
}

void LineSensor_FinalizeCalibration(void)
{
    uint8_t valid_count = 0U;

    for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++) {
        uint16_t range = (s_max[i] > s_min[i]) ? (uint16_t)(s_max[i] - s_min[i]) : 0U;

        if (range >= LINE_MIN_CALIB_RANGE && s_max[i] <= LINE_ADC_MAX_VALID) {
            s_last_valid_min[i] = s_min[i];
            s_last_valid_max[i] = s_max[i];
            valid_count++;
        } else {
            s_min[i] = s_last_valid_min[i];
            s_max[i] = s_last_valid_max[i];
        }
    }

    s_calib_valid = (valid_count >= 3U) ? 1U : 0U;
    if (s_calib_valid == 0U) {
        for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++) {
            s_min[i] = s_last_valid_min[i];
            s_max[i] = s_last_valid_max[i];
        }
        s_calib_valid = 1U;
    }

    /* Calcula media das leituras cruas durante a calibracao.
     * Usada para detectar a polaridade (linha branca vs linha preta). */
    if (s_calib_count_raw > 0U) {
        s_calib_mean_raw = (float)s_calib_sum_raw / (float)s_calib_count_raw;
    }

    s_calib_collecting = 0U;
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

void LineSensor_GetNormalizedAll(const uint16_t values[LINE_SENSOR_COUNT],
                                 float normalized[LINE_SENSOR_COUNT])
{
    if (normalized == NULL) {
        return;
    }

    for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++) {
        normalized[i] = LineSensor_Normalize(i, values[i]);
    }
}

uint8_t LineSensor_IsCalibrationValid(void)
{
    return s_calib_valid;
}

void LineSensor_GetCalibrationBounds(uint8_t index, uint16_t *min_value, uint16_t *max_value)
{
    if (index >= LINE_SENSOR_COUNT) {
        return;
    }

    if (min_value != NULL) {
        *min_value = s_min[index];
    }
    if (max_value != NULL) {
        *max_value = s_max[index];
    }
}

float LineSensor_GetCentroidIndex(const uint16_t values[LINE_SENSOR_COUNT])
{
    if (s_calib_valid != 0U) {
        float normalized[LINE_SENSOR_COUNT];
        float norm_min;
        float norm_max;
        float norm_mean = 0.0f;
        float sum_w = 0.0f;
        float sum_pos = 0.0f;

        LineSensor_GetNormalizedAll(values, normalized);
        norm_min = normalized[0];
        norm_max = normalized[0];
        for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++) {
            if (normalized[i] < norm_min) {
                norm_min = normalized[i];
            }
            if (normalized[i] > norm_max) {
                norm_max = normalized[i];
            }
            norm_mean += normalized[i];
        }
        norm_mean /= (float)LINE_SENSOR_COUNT;

        float norm_spread = norm_max - norm_min;

        if (norm_spread < LINE_MIN_NORM_SPREAD) {
            /* Norm spread baixo (leitura "plana"). Usa o pico + vizinhos
             * para estimar a posicao. Antes retornava 2.0 direto, o que
             * zerava o erro quando a linha estava em uma ponta mas a
             * leitura era quase uniforme (problema classico com defaults
             * de calibracao com range largo). */
            uint8_t peak = 0U;
            for (uint8_t i = 1U; i < LINE_SENSOR_COUNT; i++) {
                if (normalized[i] > normalized[peak]) {
                    peak = i;
                }
            }
            /* Se o pico for significativo, faz interpolacao com vizinhos. */
            if (normalized[peak] > 0.08f) {
                float w_peak = normalized[peak];
                float w_left = (peak > 0) ? normalized[peak - 1] : 0.0f;
                float w_right = (peak < (LINE_SENSOR_COUNT - 1)) ? normalized[peak + 1] : 0.0f;
                float sw = w_peak + w_left + w_right;
                if (sw > 0.01f) {
                    float sp = (float)peak * w_peak;
                    if (peak > 0) sp += (float)(peak - 1) * w_left;
                    if (peak < (LINE_SENSOR_COUNT - 1)) sp += (float)(peak + 1) * w_right;
                    return sp / sw;
                }
            }
            /* Leitura realmente uniforme (todos mesmos). */
            return 2.0f;
        }

        float margin = norm_spread * 0.35f;
        if (margin < LINE_NORM_MARGIN_MIN) {
            margin = LINE_NORM_MARGIN_MIN;
        }

        for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++) {
            float threshold = (i == 2U) ? LINE_CENTER_ACTIVE_THRESHOLD : LINE_SIDE_ACTIVE_THRESHOLD;
            float w = normalized[i];

            if (w < threshold || w < (norm_mean + margin)) {
                continue;
            }

            sum_w += w;
            sum_pos += (float)i * w;
        }

        if (sum_w > 0.01f) {
            return sum_pos / sum_w;
        }

        /* Se chegou aqui, ha spread mas o threshold nao foi atingido.
         * Usa pico + vizinhos para nao jogar o erro pra zero. */
        {
            uint8_t peak = 0U;
            for (uint8_t i = 1U; i < LINE_SENSOR_COUNT; i++) {
                if (normalized[i] > normalized[peak]) {
                    peak = i;
                }
            }
            float w_peak = normalized[peak];
            float w_left = (peak > 0) ? normalized[peak - 1] : 0.0f;
            float w_right = (peak < (LINE_SENSOR_COUNT - 1)) ? normalized[peak + 1] : 0.0f;
            float sw = w_peak + w_left + w_right;
            if (sw > 0.01f) {
                float sp = (float)peak * w_peak;
                if (peak > 0) sp += (float)(peak - 1) * w_left;
                if (peak < (LINE_SENSOR_COUNT - 1)) sp += (float)(peak + 1) * w_right;
                return sp / sw;
            }
        }

        return 2.0f;
    }

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

static float LineSensor_GetPairBalanceError(const uint16_t values[LINE_SENSOR_COUNT])
{
    uint16_t spread = LineSensor_GetRawSpread(values);

    if (spread < LINE_MIN_RAW_SPREAD) {
        return 0.0f;
    }

    float left = ((float)values[0] + (float)values[1]) * 0.5f;
    float right = ((float)values[3] + (float)values[4]) * 0.5f;
    float diff;

    if (s_polarity == LINE_POLARITY_DARK) {
        diff = left - right;
    } else {
        diff = right - left;
    }

    float error = diff / (float)spread;

    if (error > LINE_ERROR_CLAMP) {
        error = LINE_ERROR_CLAMP;
    }
    if (error < -LINE_ERROR_CLAMP) {
        error = -LINE_ERROR_CLAMP;
    }

    return error;
}

static uint8_t LineSensor_UseDifferentialError(const uint16_t values[LINE_SENSOR_COUNT],
                                              uint16_t spread)
{
    uint8_t active = LineSensor_CountRawActive(values);

    if (LineSensor_IsOnThickLine(values)) {
        return 1U;
    }

    if (active >= 2U) {
        return 1U;
    }

    /* Fita grossa: spread moderado com varios sensores parecidos */
    if (spread >= LINE_MIN_RAW_SPREAD && spread < 90U && active >= 1U) {
        float centroid = LineSensor_GetCentroidIndex(values);
        float offset = centroid - s_center_target;

        if (offset < 0.0f) {
            offset = -offset;
        }

        if (offset > 0.35f) {
            return 1U;
        }
    }

    return 0U;
}

float LineSensor_GetInterpolatedValue(uint16_t values[LINE_SENSOR_COUNT])
{
    if (s_calib_valid != 0U) {
        uint8_t active = LineSensor_CountNormalizedActive(values);

        if (active == 0U) {
            return 0.0f;
        }

        float centroid = LineSensor_GetCentroidIndex(values);
        float error = (centroid - s_center_target) / 2.0f;

        /* Zona morta adaptativa: aplica dead-zone maior quando ha spread
         * suficiente, menor em leitura ruidosa de reta. */
        uint16_t spread = LineSensor_GetRawSpread(values);
        float dead_zone = (spread >= LINE_OFFSET_MIN_SPREAD) ? 0.05f : 0.08f;

        if (error > -dead_zone && error < dead_zone) {
            return 0.0f;
        }

        return LineSensor_ClampError(error);
    }

    uint16_t spread = LineSensor_GetRawSpread(values);

    if (spread < LINE_MIN_RAW_SPREAD && !LineSensor_IsOnThickLine(values)) {
        return 0.0f;
    }

    float error;

    if (LineSensor_UseDifferentialError(values, spread)) {
        error = LineSensor_GetPairBalanceError(values);
    } else {
        float centroid = LineSensor_GetCentroidIndex(values);
        error = (centroid - s_center_target) / 2.0f;

        if (spread > 65U) {
            error *= 65.0f / (float)spread;
        }
    }

    float dead_zone = (spread >= LINE_OFFSET_MIN_SPREAD) ? 0.05f : 0.08f;
    if (error > -dead_zone && error < dead_zone) {
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

static uint8_t LineSensor_CountNormalizedActive(const uint16_t values[LINE_SENSOR_COUNT])
{
    float normalized[LINE_SENSOR_COUNT];
    float norm_min;
    float norm_max;
    float norm_mean = 0.0f;
    uint8_t count = 0U;

    if (s_calib_valid == 0U) {
        return 0U;
    }

    LineSensor_GetNormalizedAll(values, normalized);
    norm_min = normalized[0];
    norm_max = normalized[0];

    for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++) {
        if (normalized[i] < norm_min) {
            norm_min = normalized[i];
        }
        if (normalized[i] > norm_max) {
            norm_max = normalized[i];
        }
        norm_mean += normalized[i];
    }
    norm_mean /= (float)LINE_SENSOR_COUNT;

    float norm_spread = norm_max - norm_min;

    /* Threshold absoluto: so conta sensor ativo se o pico normalizado
     * estiver bem acima do ruido. Usa spread normalizado E raw spread
     * para ser robusto a defaults com range largo. */
    if (norm_spread >= LINE_MIN_NORM_SPREAD) {
        float margin = norm_spread * 0.35f;
        if (margin < LINE_NORM_MARGIN_MIN) {
            margin = LINE_NORM_MARGIN_MIN;
        }

        for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++) {
            float threshold = (i == 2U) ? LINE_CENTER_ACTIVE_THRESHOLD : LINE_SIDE_ACTIVE_THRESHOLD;

            if (normalized[i] >= threshold && normalized[i] >= (norm_mean + margin)) {
                count++;
            }
        }
    }

    /* Fallback: nenhum sensor cruzou o threshold absoluto, mas ha linha.
     * Aceita o pico como 1 sensor ativo quando ha spread normalizado
     * significativo OU spread raw significativo (caso o normalizado
     * esteja apertado por causa de defaults com range largo). */
    if (count == 0U) {
        uint8_t peak = 0U;
        for (uint8_t i = 1U; i < LINE_SENSOR_COUNT; i++) {
            if (normalized[i] > normalized[peak]) {
                peak = i;
            }
        }
        uint16_t raw_spread = LineSensor_GetRawSpread(values);
        if (normalized[peak] > 0.15f &&
            (norm_spread >= 0.10f || raw_spread >= LINE_OFFSET_MIN_SPREAD)) {
            count = 1U;
        }
    }

    return count;
}

LineSensor_State LineSensor_GetState(uint16_t values[LINE_SENSOR_COUNT])
{
    if (s_calib_valid != 0U) {
        uint16_t spread = LineSensor_GetRawSpread(values);
        uint8_t active = LineSensor_CountNormalizedActive(values);

        if (active == 0U) {
            if (spread < LINE_MIN_RAW_SPREAD) {
                return LINE_LOST;
            }
            return LINE_ON_TRACK_LOW_CONTRAST;
        }

        /* Distincao por largura da fita perpendicular:
         * - active == 5 E spread baixo (todos leem igual): fita LARGA
         *   cobrindo todos os sensores = linha de chegada.
         * - active >= 4 E spread alto (parte na linha, parte no chao):
         *   fita ESTREITA em T ou X = cruzamento (passa reto). */
        if (active == 5U && spread < 50U) {
            return LINE_FINISHED;
        }
        if (active >= 4U && spread > 55U) {
            return LINE_CROSSING;
        }

        return LINE_ON_TRACK;
    }

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
    if (s_calib_valid != 0U) {
        return LineSensor_CountNormalizedActive(values);
    }

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
    /* Se o usuario forcou a polaridade via $SET,POL, usa ela. */
    if (s_forced_polarity == 0) return LINE_POLARITY_DARK;
    if (s_forced_polarity == 1) return LINE_POLARITY_LIGHT;

    /* Heuristica baseada na media das leituras durante a calibracao:
     * - Media alta (>= 700): chao escuro / linha branca -> LIGHT
     * - Media baixa (< 500): chao claro / linha preta -> DARK
     * - Media intermediaria: cai na logica antiga. */
    if (s_calib_mean_raw >= 700.0f) {
        return LINE_POLARITY_LIGHT;
    }
    if (s_calib_mean_raw > 0.0f && s_calib_mean_raw < 500.0f) {
        return LINE_POLARITY_DARK;
    }

    /* Fallback: logica antiga baseada no centro. */
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

float LineSensor_GetCalibMeanRaw(void)
{
    return s_calib_mean_raw;
}

void LineSensor_SetForcedPolarity(int8_t polarity)
{
    if (polarity == 0) s_forced_polarity = 0;
    else if (polarity == 1) s_forced_polarity = 1;
    else s_forced_polarity = -1;
}
