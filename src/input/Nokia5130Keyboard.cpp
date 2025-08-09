#include "Nokia5130Keyboard.h"

// Nokia 5130 keyboard size
#define NOKIA5130_ROWS 5
#define NOKIA5130_COLS 5
#define NOKIA5130_NUM_KEYS 17

#define NOKIA5130_LONG_PRESS_THRESHOLD 2000
#define NOKIA5130_MULTI_TAP_THRESHOLD 750
#define NOKIA5130_BACKLIGHT_TIME 3000

using Key = TCA8418KeyboardBase::TCA8418Key;

uint8_t Nokia5130TapMod[NOKIA5130_NUM_KEYS] = {
    1, 1, 1, 1, 13, 7, 9, 2, 7, 7, 7, 2, 7, 7, 9, 2, 1}; // Num chars per key, Modulus for rotating through characters

unsigned char Nokia5130TapMap[NOKIA5130_NUM_KEYS][13] = {
    {Key::BSP},                                                         // C
    {Key::SELECT},                                                      // Navi
    {Key::UP},                                                          // Up
    {Key::DOWN},                                                        // Down
    {'1', '.', ',', '?', '!', ':', ';', '-', '_', '\\', '/', '(', ')'}, // 1
    {'4', 'g', 'h', 'i', 'G', 'H', 'I'},                                // 4
    {'7', 'p', 'q', 'r', 's', 'P', 'Q', 'R', 'S'},                      // 7
    {'*', '+'},                                                         // *
    {'2', 'a', 'b', 'c', 'A', 'B', 'C'},                                // 2
    {'5', 'j', 'k', 'l', 'J', 'K', 'L'},                                // 5
    {'8', 't', 'u', 'v', 'T', 'U', 'V'},                                // 8
    {'0', ' '},                                                         // 0
    {'3', 'd', 'e', 'f', 'D', 'E', 'F'},                                // 3
    {'6', 'm', 'n', 'o', 'M', 'N', 'O'},                                // 6
    {'9', 'w', 'x', 'y', 'z', 'W', 'X', 'Y', 'Z'},                      // 9
    {'#', '@'},                                                         // #
    {Key::REBOOT},                                                      // Power
};

unsigned char Nokia5130LongPressMap[NOKIA5130_NUM_KEYS] = {
    Key::ESC,   // C
    Key::NONE,  // Navi
    Key::NONE,  // Up
    Key::NONE,  // Down
    Key::NONE,  // 1
    Key::LEFT,  // 4
    Key::NONE,  // 7
    Key::NONE,  // *
    Key::UP,    // 2
    Key::NONE,  // 5
    Key::DOWN,  // 8
    Key::NONE,  // 0
    Key::NONE,  // 3
    Key::RIGHT, // 6
    Key::NONE,  // 9
    Key::NONE,  // #
    Key::POWER, // Power
};

Nokia5130Keyboard::Nokia5130Keyboard()
    : TCA8418KeyboardBase(NOKIA5130_ROWS, NOKIA5130_COLS), last_key(-1), next_key(-1), last_tap(0L), char_idx(0), tap_interval(0),
      should_backspace(false)
{
}

void Nokia5130Keyboard::reset()
{
    TCA8418KeyboardBase::reset();
    writeRegister(TCA8418_REG_GPI_EM_3, 0x01);
    writeRegister(TCA8418_REG_GPIO_INT_EN_3, 0x01);
    pinMode(TCA8418_COL9, OUTPUT);
    digitalWrite(TCA8418_COL9, LOW);
    backlight_on = false;
}

void Nokia5130Keyboard::setBacklight(bool on)
{
    if (on) {
        digitalWrite(TCA8418_COL9, HIGH);
    } else {
        digitalWrite(TCA8418_COL9, LOW);
    }
    backlight_on = on;
}

void Nokia5130Keyboard::pressed(uint8_t key)
{
    if (state == Init || state == Busy) {
        return;
    }
    uint8_t next_key = 0;
    if (key == 110) {        // Power
        next_key = 16;       // TCA8418_TapMap[16]
    } else if (key > 40) {   // 3, 6, 9, #
        next_key = key - 30; // TCA8418_TapMap[12...15]
    } else if (key > 30) {   // 2, 5, 8, 0
        next_key = key - 24; // TCA8418_TapMap[8...11]
    } else if (key > 20) {   // 1, 4, 7, *
        next_key = key - 18; // TCA8418_TapMap[4..7]
    } else if (key == 12) {  // Clear
        next_key = 0;        // TCA8418_TapMap[0]
    } else if (key == 13) {  // Navi
        next_key = 1;        // TCA8418_TapMap[1]
    } else if (key == 15) {  // Up
        next_key = 2;        // TCA8418_TapMap[2]
    } else if (key == 4) {   // Down
        next_key = 3;        // TCA8418_TapMap[3]
    }

    // LOG_DEBUG("TCA8418: %u %u", key, next_key);
    state = Held;
    uint32_t now = millis();
    tap_interval = now - last_tap;
    if (tap_interval < 0) {
        // long running, millis has overflowed.
        last_tap = 0;
        state = Busy;
        return;
    }
    if (next_key != last_key || tap_interval > NOKIA5130_MULTI_TAP_THRESHOLD) {
        char_idx = 0;
        should_backspace = false; // Dont backspace on new key
    } else {
        char_idx += 1;
        if (next_key > 3) {
            should_backspace = true; // Allow backspace on same key but not function keys
        }
    }
    last_key = next_key;
    last_tap = now;
}

void Nokia5130Keyboard::released()
{
    if (state != Held) {
        return;
    }

    if (last_key < 0 || last_key > NOKIA5130_NUM_KEYS) { // reset to idle if last_key out of bounds
        last_key = -1;
        state = Idle;
        return;
    }
    uint32_t now = millis();
    int32_t held_interval = now - last_tap;
    last_tap = now;
    if (tap_interval < NOKIA5130_MULTI_TAP_THRESHOLD && should_backspace) {
        queueEvent(BSP);
    }
    if (held_interval > NOKIA5130_LONG_PRESS_THRESHOLD) {
        queueEvent(Nokia5130LongPressMap[last_key]);
        // LOG_DEBUG("Long Press Key: %i Map: %i", last_key, TCA8418LongPressMap[last_key]);
    } else {
        queueEvent(Nokia5130TapMap[last_key][(char_idx % Nokia5130TapMod[last_key])]);
        // LOG_DEBUG("Key Press: %i Index:%i if %i Map: %c", last_key, char_idx, Nokia5130TapMod[last_key],
        //           Nokia5130TapMap[last_key][(char_idx % Nokia5130TapMod[last_key])]);
    }
}