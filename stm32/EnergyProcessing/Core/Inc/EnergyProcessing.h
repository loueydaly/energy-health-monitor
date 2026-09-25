/*
 * EnergyProcessing.h
 *
 * Professional AC power monitoring for commercial applications.
 * Implements industry-standard algorithms per IEEE 1459-2010 and IEC 62053-22.
 *
 * Target: STM32F446RE @ 180MHz with hardware FPU
 * Memory footprint: ~1.5 KB RAM, ~8 KB flash
 * Update rate: 100ms typical
 * Accuracy: IEC 62053-22 Class 1 (±1%)
 *
 * Created on: Aug 6, 2026
 * Author: louey
 */

#ifndef INC_ENERGYPROCESSING_H_
#define INC_ENERGYPROCESSING_H_

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Configuration Constants
// ============================================================================

/* Sample Buffer Configuration
 *
 * WINDOW_SIZE justification:
 * - 100 samples × 5ms = 500ms window
 * - Contains 25 cycles at 50Hz (statistical stability)
 * - Exceeds IEC 61000-4-30 minimum of 200ms
 * - Fits comfortably in RAM (800 bytes)
 */
#define SAMPLE_WINDOW_SIZE      100

/* Grid Frequency Configuration
 *
 * Nominal frequency for detection algorithms
 * European grid: 50Hz, American grid: 60Hz
 */
#define NOMINAL_FREQUENCY       50.0f       // Hz
#define FREQUENCY_TOLERANCE     0.5f        // ±0.5Hz normal range

/* Voltage Thresholds (per IEC 60038)
 * Standard voltage tolerance is ±10% of nominal
 */
#define V_NOMINAL               230.0f      // V (European nominal)
#define OVERVOLTAGE_LIMIT       253.0f      // V_NOMINAL × 1.10
#define UNDERVOLTAGE_LIMIT      207.0f      // V_NOMINAL × 0.90
#define V_MAX_LIMIT             300.0f      // Absolute safety limit
#define V_MIN_DETECT            10.0f       // Below = system off (noise filter)

/* Current Thresholds
 * Based on typical commercial circuit ratings (IEC 60947-2)
 */
#define I_MAX_CIRCUIT           16.0f       // Circuit breaker rating (A)
#define I_WARNING_LEVEL         12.0f       // 75% of max (early warning)
#define I_INRUSH_MAX            50.0f       // Peak current limit for surges
#define I_MIN_DETECT            0.05f       // Below = negligible load

/* Power Factor Thresholds (per IEEE 519)
 * Utilities penalize commercial customers with PF < 0.9
 */
#define PF_EXCELLENT            0.95f       // Target for commercial
#define PF_UTILITY_PENALTY      0.90f       // Utility charges extra below
#define PF_LOW_WARNING          0.85f       // Warning threshold
#define PF_CRITICAL             0.70f       // Critical intervention needed

/* Load Classification Thresholds */
#define PF_RESISTIVE_MIN        0.98f       // Above = pure resistive
#define PF_INDUCTIVE_MAX        0.98f       // Below with positive Q = inductive
#define P_LOAD_MIN              100.0f      // Minimum power for classification (W)

/* Anomaly Detection Thresholds */
#define SUDDEN_CHANGE_PCT       50.0f       // % change to trigger alarm
#define CREST_FACTOR_MIN        1.3f        // Normal AC: √2 ≈ 1.414
#define CREST_FACTOR_MAX        3.0f        // High = distortion or transient
#define THD_WARNING_PCT         8.0f        // IEEE 519 residential limit
#define THD_CRITICAL_PCT        20.0f       // Severe distortion

/* Cost & CO2 Tracking */
#define COST_PER_KWH            0.15f       // €/kWh (adjust per region)
#define CO2_PER_KWH             0.4f        // kg CO2/kWh (EU average)
#define PENALTY_PF_THRESHOLD    0.90f       // Utility penalty threshold
#define PENALTY_MULTIPLIER      1.15f       // 15% penalty for low PF

/* Filter Configuration */
#define MA_FILTER_SIZE          5           // Moving average window
#define EMA_ALPHA               0.1f        // Exponential moving average coefficient

// ============================================================================
// Status Flag Definitions (Bit-Packed for CAN Efficiency)
// ============================================================================
#define FLAG_DATA_VALID         (1 << 0)    // Buffer is full
#define FLAG_OVERLOAD           (1 << 1)    // Overload detected
#define FLAG_LOW_PF             (1 << 2)    // Poor power factor
#define FLAG_OVERVOLTAGE        (1 << 3)    // Voltage too high
#define FLAG_UNDERVOLTAGE       (1 << 4)    // Voltage too low
#define FLAG_SUDDEN_CHANGE      (1 << 5)    // Rapid load change
#define FLAG_FREQ_DEVIATION     (1 << 6)    // Frequency out of range
#define FLAG_HIGH_DISTORTION    (1 << 7)    // High THD detected

// ============================================================================
// Load Type Classification
// ============================================================================
typedef enum {
    LOAD_TYPE_UNKNOWN = 0,      // Insufficient data
    LOAD_TYPE_NONE,             // No load connected
    LOAD_TYPE_RESISTIVE,        // Heaters, incandescent lights
    LOAD_TYPE_INDUCTIVE,        // Motors, transformers
    LOAD_TYPE_CAPACITIVE,       // PF correction caps, some LED drivers
    LOAD_TYPE_MIXED             // Combination of load types
} LoadType;

// ============================================================================
// Comprehensive Metrics Structure
// ============================================================================
typedef struct {
    // Fundamental RMS values
    float V_rms;                 // RMS Voltage (V)
    float I_rms;                 // RMS Current (A)

    // Filtered RMS values (noise-reduced)
    float V_rms_filtered;        // Moving average filtered
    float I_rms_filtered;        // Moving average filtered

    // Power measurements
    float P_active;              // Active Power (W)
    float S_apparent;            // Apparent Power (VA)
    float Q_reactive;            // Reactive Power (VAR)
    float PF;                    // Power Factor (-1 to +1)
    float PF_displacement;       // Displacement PF (fundamental only)

    // Peak & extreme values
    float V_peak;                // Peak voltage magnitude
    float I_peak;                // Peak current magnitude
    float V_crest_factor;        // V_peak / V_rms (waveform quality)
    float I_crest_factor;        // I_peak / I_rms (inrush indicator)

    // Frequency measurements
    float frequency;             // Estimated frequency (Hz)
    float frequency_deviation;   // Deviation from nominal (Hz)

    // Quality metrics
    float THD_estimated;         // Estimated Total Harmonic Distortion (%)
    LoadType load_type;          // Classified load type

    // Energy & cost
    float energy_Wh;             // Cumulative energy (Watt-hours)
    float energy_kWh;            // Cumulative energy (kilo-Watt-hours)
    float cost_EUR;              // Estimated cost in €
    float co2_kg;                // Estimated CO2 emissions (kg)

    // Duration tracking
    uint32_t overload_duration_ms;    // Time spent in overload
    uint32_t undervoltage_duration_ms; // Time spent undervoltage
    uint32_t overvoltage_duration_ms;  // Time spent overvoltage

    // Status
    uint8_t flags;               // Status flags (bit-encoded)

} EnergyMetrics;

// ============================================================================
// Public API Functions
// ============================================================================

// Initialization & Reset
void energy_init(void);
void energy_reset(void);
void energy_reset_energy_counter(void);  // Reset only energy accumulator

// Sample Handling
void energy_add_sample(float voltage, float current);
bool energy_is_buffer_full(void);
uint32_t energy_get_sample_count(void);

// Metrics Calculation
EnergyMetrics energy_get_metrics(void);

// Individual Metric Getters (for lightweight queries)
float energy_get_V_rms(void);
float energy_get_I_rms(void);
float energy_get_active_power(void);
float energy_get_power_factor(void);
float energy_get_energy_kWh(void);
float energy_get_frequency(void);
LoadType energy_get_load_type(void);

// Anomaly Detection
bool energy_check_overload(EnergyMetrics *metrics);
bool energy_check_voltage_anomaly(EnergyMetrics *metrics);
bool energy_check_low_pf(EnergyMetrics *metrics);
bool energy_check_frequency_anomaly(EnergyMetrics *metrics);
bool energy_check_high_distortion(EnergyMetrics *metrics);
bool energy_check_all_anomalies(EnergyMetrics *metrics);

// Cost & Efficiency Analysis
float energy_estimate_daily_cost(float current_power);
float energy_estimate_monthly_cost(void);
float energy_get_pf_penalty(EnergyMetrics *metrics);

// Diagnostic Functions
const char* energy_load_type_to_string(LoadType type);
void energy_get_diagnostic_string(EnergyMetrics *metrics, char *buffer, int buf_size);

#endif /* INC_ENERGYPROCESSING_H_ */
