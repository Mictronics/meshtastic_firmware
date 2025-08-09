#include "TCA8418KeyboardBase.h"

/**
 * @brief 5x5 keypad with 17 keys for Nokia5130 Meshtastic node.
 */
class Nokia5130Keyboard : public TCA8418KeyboardBase
{
  public:
    Nokia5130Keyboard();
    void reset(void) override;
    void setBacklight(bool on) override;

  protected:
    void pressed(uint8_t key) override;
    void released(void) override;

    int8_t last_key;
    int8_t next_key;
    uint32_t last_tap;
    uint8_t char_idx;
    int32_t tap_interval;
    bool should_backspace;
    bool backlight_on;
};