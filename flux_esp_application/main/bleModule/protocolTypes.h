#ifndef H_PROTOCOL_TYPES_
#define H_PROTOCOL_TYPES_

#include <stdbool.h>

#define MAX_STEPS             20
#define MAX_PROBES             4
#define MAX_VALVES             4
#define MAX_TARGETS_PER_STEP   4
#define MAX_ACTIONS_PER_STEP   4
#define MAX_RECIPE_ID_LEN     64

// ── Enums ─────────────────────────────────────────────────────────────────

typedef enum {
    DISCONNECT_CONTINUE,
    DISCONNECT_SAFE_STATE,
} disconnect_behaviour_e;

typedef enum {
    TEMP_UNIT_C,
    TEMP_UNIT_F,
} temp_unit_e;

typedef enum {
    TRIGGER_STEP_START,
    TRIGGER_TEMP_REACHED,
    TRIGGER_TIMER_ELAPSED,
} valve_trigger_e;

// ── App → ESP ──────────────────────────────────────────────────────────────

typedef struct {
    int           probeIndex;
    bool          hasMinTemp;
    float         minTemp;
    bool          hasMaxTemp;
    float         maxTemp;
    temp_unit_e   unit;
} probe_target_t;

typedef struct {
    int             valveIndex;
    int             targetPosition;   // 0–100 percent open
    valve_trigger_e triggerOn;
} valve_action_t;

typedef struct {
    int            stepIndex;
    bool           espControlled;
    bool           hasDuration;
    int            durationSeconds;
    probe_target_t targetTemps[MAX_TARGETS_PER_STEP];
    int            targetTempCount;
    // int            valveActionCount;
    bool           alertOnComplete;
} esp_step_t;

typedef struct {
    char                   recipeId[MAX_RECIPE_ID_LEN];
    int                    probeCount;
    int                    valveCount;
    disconnect_behaviour_e disconnectBehaviour;
    esp_step_t             steps[MAX_STEPS];
    int                    stepCount;
} recipe_load_t;

#endif