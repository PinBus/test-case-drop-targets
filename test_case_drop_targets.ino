/**
 * Test-Case - Drop Targets
 *
 * Version 1.0 (01-12-2021)
 **/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/** Define pins **/
#define PIN_LED_HEARTBEAT 9

#define PIN_MATRIX_IN_0 A0
#define PIN_MATRIX_IN_1 A1
#define PIN_MATRIX_IN_2 A2
#define PIN_MATRIX_IN_3 A3

#define PIN_MATRIX_OUT_0 4
#define PIN_MATRIX_OUT_1 5
#define PIN_MATRIX_OUT_2 6
#define PIN_MATRIX_OUT_3 7

#define PIN_PCA9685_OE 8

//  0 - Solenoid: Flipper Power Winding
//  1 - Solenoid: Flipper Full Winding
//  2 - Solenoid: Drop Target Trip
//  3 - Solenoid: Drop Target Reset 5
//  4 - Solenoid: Drop Target Reset 4
//  5 - Solenoid: Drop Target Reset 3
//  6 - Solenoid: Drop Target Reset 2
//  7 - Solenoid: Drop Target Reset 1
#define PIN_PCA9685_FLIPPER_POWER 0
#define PIN_PCA9685_FLIPPER_HOLD 1
#define PIN_PCA9685_DROP_TARGET_TRIP 2
#define PIN_PCA9685_DROP_TARGET_RESET_5 3
#define PIN_PCA9685_DROP_TARGET_RESET_4 4
#define PIN_PCA9685_DROP_TARGET_RESET_3 5
#define PIN_PCA9685_DROP_TARGET_RESET_2 6
#define PIN_PCA9685_DROP_TARGET_RESET_1 7

//  8 - LED: Flipper Button
//  9 - LED: Drop Target Reset Button
// 10 - LED: Drop Target Trip Button
// 11 - LED: Unused
// 12 - LED: Unused
// 13 - LED: Unused
// 14 - LED: Unused
// 15 - LED: Unused
#define PIN_PCA9685_LED_BUTTON_FLIPPER 8
#define PIN_PCA9685_LED_BUTTON_RESET 9
#define PIN_PCA9685_LED_BUTTON_TRIP 10

/** Type definitions **/
typedef void (*SwitchCallback)();

struct SwitchState {
    uint32_t first_change;
    bool is_closed;
    SwitchCallback on_switch_close;
    SwitchCallback on_switch_open;
};

enum LedAnimationState {
    Running = 0,
    Pauzed = 1,
    FadeIn = 2,
    FadeOut = 3
};

enum LedAnimationType {
    Static = 0,
    Fade = 1
};

struct LedAnimationStep {
    LedAnimationType type;
    uint32_t duration;
    uint16_t pwm_value;
};

struct LedAnimation {
    LedAnimationStep* steps;
    uint32_t last_update;
    uint8_t current_step;
    uint16_t current_pwm_value;
    uint16_t backup_pwm_value;
    uint8_t number_of_steps;
    uint8_t pin;
    LedAnimationState state;
};

struct Flipper {
    uint8_t power_winding_pin;
    uint8_t hold_winding_pin;
    uint32_t active_time;
    bool is_active;
};

struct DropTarget {
    uint8_t pin;
    uint32_t timer;
    bool is_down;
};

/** Global variables **/

// The PCA9685 driver object
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver();

// State variables
bool state_is_drop_target_reset_allowed = false;
bool state_is_drop_target_reset_in_progress = false;
bool state_is_drop_target_trip_allowed = false;

/** Configuration **/
#define MATRIX_DEBOUNCE_MS 20
#define MATRIX_HEIGHT 4
#define MATRIX_WIDTH 4

#define FLIPPER_POWER_WINDING_MAX_MS 1000
#define FLIPPER_POWER_WINDING_POWER_PWM 4096
#define FLIPPER_HOLD_WINDING_POWER_PWM 4096

#define DROP_TARGET_TRIP_MS 50
#define DROP_TARGET_RESET_MS 40
#define DROP_TARGET_RESET_BETWEEN_MS 250

#define LED_FADE_SPEED_MS 200
#define LED_MAX_VALUE 4096
#define LED_HALF_VALUE 2048
#define LED_QUARTER_VALUE 1024

/** Heartbeat LED **/
uint8_t heartbeat_led_index = 0;
uint32_t heartbeat_led_sequence[4] = { 200, 125, 200, 600 };
uint32_t heartbeat_led_update = 0;
bool heartbeat_led_state = true;

void init_heartbeat_led() {
    // Set up the heartbeat LED pin
    pinMode(PIN_LED_HEARTBEAT, OUTPUT);
    digitalWrite(PIN_LED_HEARTBEAT, heartbeat_led_state);
}

void update_heartbeat_led() {
    if (millis() - heartbeat_led_update < heartbeat_led_sequence[heartbeat_led_index]) {
        // Nothing to update yet
        return;
    }

    // Increase the index and make sure it does not go out of bounds
    heartbeat_led_index++;
    if (heartbeat_led_index == 4) {
        heartbeat_led_index = 0;
    }

    // Mark now as the last update
    heartbeat_led_update = millis();

    // Update the pin state and flip the internal state
    digitalWrite(PIN_LED_HEARTBEAT, heartbeat_led_state);
    heartbeat_led_state = !heartbeat_led_state;
}

/** Switch Matrix **/
uint8_t matrix_input_pins[MATRIX_HEIGHT] = {
    PIN_MATRIX_IN_0,
    PIN_MATRIX_IN_1,
    PIN_MATRIX_IN_2,
    PIN_MATRIX_IN_3
};

uint8_t matrix_output_pins[MATRIX_WIDTH] = {
    PIN_MATRIX_OUT_0,
    PIN_MATRIX_OUT_1,
    PIN_MATRIX_OUT_2,
    PIN_MATRIX_OUT_3
};

uint8_t matrix_col = 0;
uint8_t matrix_row = 0;

// Scratchpad pointer for loops iterating over switch state structs
SwitchState* switch_state = NULL;

SwitchState matrix[MATRIX_WIDTH][MATRIX_HEIGHT] = {
    {
        // 0x0 - NO - Front button 1: White / Flipper
        {0, false, on_flipper_button_down, on_flipper_button_up},
        // 0x1 - NO - Front button 2: Green / Drop target reset
        {0, false, on_drop_target_reset_button_down, NULL},
        // 0x2 - NO - Front button 3: Red / Drop target trip
        {0, false, on_drop_target_trip_button_down, NULL},
        // 0x3 - NO - Unused
        {0, false, NULL, NULL}
    },
    {
        // 1x0 - NO - Unused
        {0, false, NULL, NULL},
        // 1x1 - NO - Unused
        {0, false, NULL, NULL},
        // 1x2 - NO - Unused
        {0, false, NULL, NULL},
        // 1x3 - NO - Unused
        {0, false, NULL, NULL}
    },
    {
        // 2x0 - NO - Drop target 1 down
        {0, false, on_drop_target_1_down, NULL},
        // 2x1 - NO - Drop target 2 down
        {0, false, on_drop_target_2_down, NULL},
        // 2x2 - NO - Drop target 3 down
        {0, false, on_drop_target_3_down, NULL},
        // 2x3 - NO - Drop target 4 down
        {0, false, on_drop_target_4_down, NULL}
    },
    {
        // 3x0 - NO - Drop target 5 down
        {0, false, on_drop_target_5_down, NULL},
        // 3x1 - NC - Flipper EOS
        {0, false, NULL, on_flipper_eos_open},
        // 3x2 - NO - Unused
        {0, false, NULL, NULL},
        // 3x3 - NO - Unused
        {0, false, NULL, NULL}
    }
};

void init_matrix() {
    // Set pin mode for output pins
    for (matrix_col = 0; matrix_col < MATRIX_WIDTH; matrix_col++) {
        pinMode(matrix_output_pins[matrix_col], OUTPUT);
    }

    // Set pin mode for input pins
    for (matrix_row = 0; matrix_row < MATRIX_HEIGHT; matrix_row++) {
        pinMode(matrix_input_pins[matrix_row], INPUT);
    }
}

void update_matrix() {
    for (matrix_col = 0; matrix_col < MATRIX_WIDTH; matrix_col++) {
        // Set output pins
        for (matrix_row = 0; matrix_row < MATRIX_HEIGHT; matrix_row++) {
            digitalWrite(matrix_output_pins[matrix_row], matrix_col != matrix_row);
        }

        // Scan input pins
        for (matrix_row = 0; matrix_row < MATRIX_HEIGHT; matrix_row++) {
            switch_state = &matrix[matrix_row][matrix_col];

            // Check if switch state has been changed
            if (digitalRead(matrix_input_pins[matrix_row]) == (switch_state->is_closed ? HIGH : LOW)) {
                if (!switch_state->first_change) {
                    // Mark the first time this state change has been seen
                    switch_state->first_change = millis();
                }
                else {
                    // Check if switch is in its new state for long enough
                    if (millis() - switch_state->first_change > MATRIX_DEBOUNCE_MS) {
                        if (switch_state->is_closed) {
                            // Call the on switch open callback
                            if (switch_state->on_switch_open != NULL) {
                                switch_state->on_switch_open();
                            }
                        }
                        else {
                            // Call the on switch close callback
                            if (switch_state->on_switch_close != NULL) {
                                switch_state->on_switch_close();
                            }
                        }

                        // Update internal state
                        switch_state->is_closed = !switch_state->is_closed;
                        switch_state->first_change = 0;
                    }
                }
            }
            else if (millis() - switch_state->first_change > MATRIX_DEBOUNCE_MS) {
                // Reset internal state if switch went back to its previous state within the debounce time
                switch_state->first_change = 0;
            }
        }
    }
}

/** PCA9685 **/
void init_pca9685() {
    // Constructor initializes a PCA9685 device at default address 0x40
    pca9685.begin();
    // Set maximum PWM frequency in Hz
    pca9685.setPWMFreq(1600);
    // Set output to push/pull (totempole)
    pca9685.setOutputMode(true);

    // Set Output Enable (OE) pin low
    pinMode(PIN_PCA9685_OE, OUTPUT);
    digitalWrite(PIN_PCA9685_OE, LOW);
}

/** Solenoid Control **/
Flipper flipper = {
    PIN_PCA9685_FLIPPER_POWER,
    PIN_PCA9685_FLIPPER_HOLD,
    0,
    false
};

void update_flipper() {
    if (!flipper.active_time) {
        // Flipper not active, nothing to update
        return;
    }

    // Power down the power winding after 1 second regardless of the EOS switch state
    if (millis() - flipper.active_time > FLIPPER_POWER_WINDING_MAX_MS) {
        flipper_eos();
    }
}

void flipper_activate() {
    // Update flipper state
    flipper.is_active = true;
    flipper.active_time = millis();

    // Activate the power winding of the flipper coil
    pca9685.setPin(flipper.power_winding_pin, FLIPPER_POWER_WINDING_POWER_PWM, false);
}

void flipper_eos() {
    if (!flipper.is_active) {
        return;
    }

    // Update flipper state
    flipper.active_time = 0;

    // Deactivate power winding and activate hold winding
    pca9685.setPin(PIN_PCA9685_FLIPPER_POWER, 0, false);
    pca9685.setPin(PIN_PCA9685_FLIPPER_HOLD, FLIPPER_HOLD_WINDING_POWER_PWM, false);
}

void flipper_deactivate() {
    // Update flipper state
    flipper.is_active = false;
    flipper.active_time = 0;

    // Power down both flipper coil windings
    pca9685.setPin(PIN_PCA9685_FLIPPER_POWER, 0, false);
    pca9685.setPin(PIN_PCA9685_FLIPPER_HOLD, 0, false);
}

DropTarget drop_targets[5] = {
    { PIN_PCA9685_DROP_TARGET_RESET_1, 0, false },
    { PIN_PCA9685_DROP_TARGET_RESET_2, 0, false },
    { PIN_PCA9685_DROP_TARGET_RESET_3, 0, false },
    { PIN_PCA9685_DROP_TARGET_RESET_4, 0, false },
    { PIN_PCA9685_DROP_TARGET_RESET_5, 0, false }
};

uint32_t drop_target_trip_timer = 0;
uint32_t drop_target_between_resets_timer = 0;

// Scratchpad pointer for loops iterating over drop target
DropTarget* drop_target = NULL;

void reset_drop_target(uint8_t index) {
    drop_target = &drop_targets[index];

    // Update state
    drop_target->timer = millis();
    drop_target->is_down = false;

    // Fire coil
    pca9685.setPin(drop_target->pin, 4096, false);
}

void trip_drop_targets() {
    // Update state
    drop_target_trip_timer = millis();

    // Fire coil
    pca9685.setPin(PIN_PCA9685_DROP_TARGET_TRIP, 4096, false);
}

void set_drop_target_down(uint8_t index) {
    drop_target = &drop_targets[index];

    // Update state
    drop_target->is_down = true;

    // Update button state
    update_state();
}

bool reset_next_drop_target() {
    for (uint8_t i = 0; i < 5; i++) {
        drop_target = &drop_targets[i];

        if (drop_target->is_down) {
            reset_drop_target(i);
            return true;
        }
    }

    return false;
}

void start_drop_targets_reset() {
    // Set the timer and state to start the reset sequence
    drop_target_between_resets_timer = millis();
    state_is_drop_target_reset_in_progress = true;
}

void update_drop_targets() {
    if (drop_target_trip_timer) {
        if (millis() - drop_target_trip_timer > DROP_TARGET_TRIP_MS) {
            // Reset the timer and switch off MOSFET
            drop_target_trip_timer = 0;
            pca9685.setPin(PIN_PCA9685_DROP_TARGET_TRIP, 0, false);
        }
    }

    for (uint8_t i = 0; i < 5; i++) {
        drop_target = &drop_targets[i];

        if (millis() - drop_target->timer > DROP_TARGET_RESET_MS) {
            // Reset the timer and switch off MOSFET
            drop_target->timer = 0;
            pca9685.setPin(drop_target->pin, 0, false);
        }
    }

    if (drop_target_between_resets_timer) {
        if (millis() - drop_target_between_resets_timer > DROP_TARGET_RESET_BETWEEN_MS) {
            // Try to reset a drop target
            bool did_reset = reset_next_drop_target();

            if (did_reset) {
                // It did reset, set the timer for the next drop target
                drop_target_between_resets_timer = millis();
            }
            else {
                // It did not reset, reset the timer and update the state
                drop_target_between_resets_timer = 0;
                state_is_drop_target_reset_in_progress = false;
                update_state();
            }
        }
    }
}

/** LEDs **/

// Flipper button animation
LedAnimationStep led_flipper_animation_sequence[4] = {
    { LedAnimationType::Static, 500, LED_MAX_VALUE },
    { LedAnimationType::Fade, 200, 0 },
    { LedAnimationType::Static, 300, 0 },
    { LedAnimationType::Fade, 200, LED_MAX_VALUE }
};

LedAnimation led_flipper_animation = {
    led_flipper_animation_sequence,
    0,
    0,
    0,
    0,
    4,
    PIN_PCA9685_LED_BUTTON_FLIPPER,
    LedAnimationState::Running
};

// Reset button animation (slow)
LedAnimationStep led_reset_slow_animation_sequence[4] = {
    { LedAnimationType::Static, 1000, LED_MAX_VALUE },
    { LedAnimationType::Fade, 200, 0 },
    { LedAnimationType::Static, 500, 0 },
    { LedAnimationType::Fade, 200, LED_MAX_VALUE }
};

LedAnimation led_reset_slow_animation = {
    led_reset_slow_animation_sequence,
    0,
    0,
    0,
    0,
    4,
    PIN_PCA9685_LED_BUTTON_RESET,
    LedAnimationState::Pauzed
};

// Reset button animation (fast)
LedAnimationStep led_reset_fast_animation_sequence[4] = {
    { LedAnimationType::Static, 500, LED_MAX_VALUE },
    { LedAnimationType::Fade, 200, 0 },
    { LedAnimationType::Static, 250, 0 },
    { LedAnimationType::Fade, 200, LED_MAX_VALUE }
};

LedAnimation led_reset_fast_animation = {
    led_reset_fast_animation_sequence,
    0,
    0,
    0,
    0,
    4,
    PIN_PCA9685_LED_BUTTON_RESET,
    LedAnimationState::Pauzed
};

// Trip button animation
LedAnimationStep led_trip_animation_sequence[4] = {
    { LedAnimationType::Static, 1000, LED_MAX_VALUE },
    { LedAnimationType::Fade, 300, 0 },
    { LedAnimationType::Static, 500, 0 },
    { LedAnimationType::Fade, 300, LED_MAX_VALUE }
};

LedAnimation led_trip_animation = {
    led_trip_animation_sequence,
    0,
    0,
    0,
    0,
    4,
    PIN_PCA9685_LED_BUTTON_TRIP,
    LedAnimationState::Pauzed
};

#define LED_ANIMATION_NUMBER_OF 4
LedAnimation animations[LED_ANIMATION_NUMBER_OF] = {
    led_flipper_animation,
    led_reset_slow_animation,
    led_reset_fast_animation,
    led_trip_animation
};

// Scratchpad pointers for loops iterating over animations and animation steps
LedAnimation* animation = NULL;
LedAnimationStep* step = NULL;

// Warning: Hairy code ahead
void update_leds() {
    for (uint8_t i = 0; i < LED_ANIMATION_NUMBER_OF; i++) {
        animation = &animations[i];

        if (animation->state == LedAnimationState::Pauzed) {
            // Animation is pauzed, don't do anything
            continue;
        }

        if (animation->state == LedAnimationState::FadeOut) {
            // Alternate flow, fading out is not part of the normal animation flow
            if (millis() - animation->last_update > LED_FADE_SPEED_MS) {
                // The fade out is done, set the current pwm value to 0
                animation->current_pwm_value = 0;
                pca9685.setPin(animation->pin, animation->current_pwm_value, true);
                // Update animation state
                animation->state = LedAnimationState::Pauzed;
                continue;
            }

            // Compute intermediary value of the fade out
            uint32_t time_in = millis() - animation->last_update;
            animation->current_pwm_value = map(time_in, 0, LED_FADE_SPEED_MS, animation->backup_pwm_value, 0);
            pca9685.setPin(animation->pin, animation->current_pwm_value, true);
            continue;
        }

        if (animation->state == LedAnimationState::FadeIn) {
            // Alternate flow, fading in is not part of the normal animation flow
            step = &animation->steps[0];

            if (millis() - animation->last_update > LED_FADE_SPEED_MS) {
                // The fade in is done, set the current pwm value to the first step's pwm value
                animation->current_pwm_value = step->pwm_value;
                pca9685.setPin(animation->pin, animation->current_pwm_value, true);
                // Update animation state. Let the animation resetart from the first step
                animation->state = LedAnimationState::Running;
                animation->last_update = millis();
                animation->current_step = 0;
                continue;
            }

            // Compute intermediary value of the fade in
            uint32_t time_in = millis() - animation->last_update;
            animation->current_pwm_value = map(time_in, 0, LED_FADE_SPEED_MS, 0, step->pwm_value);
            pca9685.setPin(animation->pin, animation->current_pwm_value, true);
            continue;
        }

        step = &animation->steps[animation->current_step];

        if (step->type == LedAnimationType::Fade) {
            // Compute intermediary value of the fade step
            uint16_t previous_pwm_value = animation->steps[animation->current_step == 0 ? animation->number_of_steps - 1 : animation->current_step - 1].pwm_value;
            uint32_t time_in = millis() - animation->last_update;
            animation->current_pwm_value = map(time_in, 0, step->duration, previous_pwm_value, step->pwm_value);
            pca9685.setPin(animation->pin, animation->current_pwm_value, true);
        }

        if (millis() - animation->last_update > step->duration) {
            // Move on to the next animation step if the current step is done
            animation->last_update = millis();
            animation->current_step += 1;
            if (animation->current_step == animation->number_of_steps) {
                animation->current_step = 0;
            }

            step = &animation->steps[animation->current_step];
            if (step->type == LedAnimationType::Static) {
                // If the step is a static pwm value, set it
                animation->current_pwm_value = step->pwm_value;
                pca9685.setPin(animation->pin, step->pwm_value, true);
            }
        }
    }
}

void stop_animation(uint8_t animation_index) {
    animation = &animations[animation_index];

    if (animation->state == LedAnimationState::Pauzed) {
        // Don't do anything if the animation is already pauzed
        return;
    }

    // Update animation state
    animation->state = LedAnimationState::FadeOut;
    animation->last_update = millis();
    // Set the backup pwm field. This value is needed when computing the new current
    // value while fading out. The pwm value of the current step can not be used as
    // that step might not yet be completed.
    animation->backup_pwm_value = animation->current_pwm_value;
}

void resume_animation(uint8_t animation_index) {
    animation = &animations[animation_index];

    if (animation->state == LedAnimationState::Running) {
        // Don't do anything if the animation is already running
        return;
    }

    // Update animation state
    animation->state = LedAnimationState::FadeIn;
    animation->last_update = millis();
}

/** State **/
void update_state() {
    if (
        drop_targets[0].is_down &&
        drop_targets[1].is_down &&
        drop_targets[2].is_down &&
        drop_targets[3].is_down &&
        drop_targets[4].is_down
    ) {
        state_is_drop_target_reset_allowed = true;
        stop_animation(1);
        resume_animation(2);
        state_is_drop_target_trip_allowed = false;
        stop_animation(3);
    }
    else if (
        !drop_targets[0].is_down &&
        !drop_targets[1].is_down &&
        !drop_targets[2].is_down &&
        !drop_targets[3].is_down &&
        !drop_targets[4].is_down
    ) {
        state_is_drop_target_reset_allowed = false;
        stop_animation(1);
        stop_animation(2);
        state_is_drop_target_trip_allowed = true;
        resume_animation(3);
    }
    else if (!state_is_drop_target_reset_in_progress) {
        state_is_drop_target_reset_allowed = true;
        resume_animation(1);
        stop_animation(2);
        state_is_drop_target_trip_allowed = true;
        resume_animation(3);
    }
};

/** Serial **/
void init_serial() {
    // Open the serial connection
    Serial.begin(9600);
}

void update_serial() {
    // Check for available characters on the serial port.
    if (Serial.available() > 0) {
        // If the sent character is a "c", print the credits "window"
        if (Serial.read() == 'c') {
            Serial.println("------  Test-Case - Drop Targets  ------\n        Version 1.0 (01-12-2021)\n       Made in The Netherlands by\n    Cor Gravekamp & Thomas Gravekamp\n                 for the\n          Dutch Pinball Museum\n----------------------------------------");
        }
    }
}

/** Button actions **/
void on_flipper_button_down() {
    flipper_activate();
}

void on_flipper_button_up() {
    flipper_deactivate();
}

void on_drop_target_reset_button_down() {
    if (!state_is_drop_target_reset_allowed) {
        return;
    }

    start_drop_targets_reset();
}

void on_drop_target_trip_button_down() {
    if (!state_is_drop_target_trip_allowed) {
        return;
    }

    trip_drop_targets();
}

void on_drop_target_1_down() {
    set_drop_target_down(0);
}

void on_drop_target_2_down() {
    set_drop_target_down(1);
}

void on_drop_target_3_down() {
    set_drop_target_down(2);
}

void on_drop_target_4_down() {
    set_drop_target_down(3);
}

void on_drop_target_5_down() {
    set_drop_target_down(4);
}

void on_flipper_eos_open() {
    flipper_eos();
}

void setup() {
    // Initialize heartbeat LED
    init_heartbeat_led();

    // Initialize switch matrix
    init_matrix();

    // Initialize PCA9685
    init_pca9685();

    // Initialize serial
    init_serial();
}

void loop() {
    // Update heartbeat LED
    update_heartbeat_led();

    // Update switch matrix
    update_matrix();

    // Update flipper timer
    update_flipper();

    // Update drop targets
    update_drop_targets();

    // Update LEDs
    update_leds();

    // Update serial
    update_serial();
}
