/*
 * ZeroWriter UART output module for Horizon + nice!nano
 *
 * Intercepts ZMK key events and sends them to the ZeroWriter Inkplate
 * firmware using its proprietary single-byte protocol over UART at 921600
 * baud on P1.01 (bottom pad of nice!nano, wired to Inkplate GPIO 13).
 *
 * Protocol (from zwi_kb_feb2026.ino):
 *   - Regular key press  → single byte, index 0–60 (see table below)
 *   - Regular key release → nothing sent
 *   - Modifier press     → byte 240–246 (DOWN signal)
 *   - Modifier release   → byte 241–247 (UP signal)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>

LOG_MODULE_REGISTER(zerowriter_uart, LOG_LEVEL_INF);

/* ZeroWriter modifier signal bytes */
#define ZW_MOD_SHIFT_DOWN  240
#define ZW_MOD_SHIFT_UP    241
#define ZW_MOD_CTRL_DOWN   242
#define ZW_MOD_CTRL_UP     243
#define ZW_MOD_ALT_DOWN    244
#define ZW_MOD_ALT_UP      245
#define ZW_MOD_META_DOWN   246
#define ZW_MOD_META_UP     247

/* Sentinel: keycode not mapped to any ZeroWriter index */
#define ZW_NO_KEY          255

/* USB HID keyboard/keypad usage page (0x07) */
#define HID_USAGE_PAGE_KEY 0x07

/* HID modifier keycode range */
#define HID_MOD_FIRST      0xE0
#define HID_MOD_LAST       0xE7

static const struct device *const uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));

static void zw_send(uint8_t byte) {
    uart_poll_out(uart_dev, byte);
}

/*
 * Maps a HID keyboard usage ID to a ZeroWriter key index (0–60).
 *
 * ZeroWriter key index layout (matches their 5×14 keyIndexMap):
 *   0–13  : ` 1 2 3 4 5 6 7 8 9 0 - = Backspace
 *   14–27 : Tab Q W E R T Y U I O P [ ] \
 *   28–40 : CapsLock A S D F G H J K L ; ' Enter
 *   41    : (LShift — sent as modifier signal, not index)
 *   42–52 : Z X C V B N M , . /  (52 = RShift — modifier signal)
 *   53    : (Ctrl — modifier signal)
 *   54    : Space
 *   55    : (Alt — modifier signal)
 *   56    : (Meta — modifier signal)
 *   57–60 : Left Up Down Right
 */
static uint8_t hid_to_zw(uint32_t kc) {
    switch (kc) {
        /* Number row */
        case 0x35: return 0;   /* ` ~ */
        case 0x1E: return 1;   /* 1 ! */
        case 0x1F: return 2;   /* 2 @ */
        case 0x20: return 3;   /* 3 # */
        case 0x21: return 4;   /* 4 $ */
        case 0x22: return 5;   /* 5 % */
        case 0x23: return 6;   /* 6 ^ */
        case 0x24: return 7;   /* 7 & */
        case 0x25: return 8;   /* 8 * */
        case 0x26: return 9;   /* 9 ( */
        case 0x27: return 10;  /* 0 ) */
        case 0x2D: return 11;  /* - _ */
        case 0x2E: return 12;  /* = + */
        case 0x2A: return 13;  /* Backspace */
        /* QWERTY row */
        case 0x2B: return 14;  /* Tab */
        case 0x14: return 15;  /* Q */
        case 0x1A: return 16;  /* W */
        case 0x08: return 17;  /* E */
        case 0x15: return 18;  /* R */
        case 0x17: return 19;  /* T */
        case 0x1C: return 20;  /* Y */
        case 0x18: return 21;  /* U */
        case 0x0C: return 22;  /* I */
        case 0x12: return 23;  /* O */
        case 0x13: return 24;  /* P */
        case 0x2F: return 25;  /* [ { */
        case 0x30: return 26;  /* ] } */
        case 0x31: return 27;  /* \ | */
        /* Home row */
        case 0x39: return 28;  /* Caps Lock */
        case 0x04: return 29;  /* A */
        case 0x16: return 30;  /* S */
        case 0x07: return 31;  /* D */
        case 0x09: return 32;  /* F */
        case 0x0A: return 33;  /* G */
        case 0x0B: return 34;  /* H */
        case 0x0D: return 35;  /* J */
        case 0x0E: return 36;  /* K */
        case 0x0F: return 37;  /* L */
        case 0x33: return 38;  /* ; : */
        case 0x34: return 39;  /* ' " */
        case 0x28: return 40;  /* Enter */
        /* Bottom row — index 41 (LShift) handled as modifier below */
        case 0x1D: return 42;  /* Z */
        case 0x1B: return 43;  /* X */
        case 0x06: return 44;  /* C */
        case 0x19: return 45;  /* V */
        case 0x05: return 46;  /* B */
        case 0x11: return 47;  /* N */
        case 0x10: return 48;  /* M */
        case 0x36: return 49;  /* , < */
        case 0x37: return 50;  /* . > */
        case 0x38: return 51;  /* / ? */
        /* index 52 (RShift), 53 (Ctrl), 55 (Alt), 56 (Meta) are modifiers */
        case 0x2C: return 54;  /* Space */
        /* Arrow keys */
        case 0x50: return 57;  /* Left */
        case 0x52: return 58;  /* Up */
        case 0x51: return 59;  /* Down */
        case 0x4F: return 60;  /* Right */
        /* Escape — not in ZeroWriter layout, ignore */
        default:   return ZW_NO_KEY;
    }
}

static int zw_keycode_listener(const zmk_event_t *eh) {
    const struct zmk_keycode_state_changed *ev = as_zmk_keycode_state_changed(eh);
    if (!ev) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    /* Only handle keyboard/keypad usage page */
    if (ev->usage_page != HID_USAGE_PAGE_KEY) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    uint32_t kc = ev->keycode;

    /* Modifier keys: send DOWN on press, UP on release */
    if (kc >= HID_MOD_FIRST && kc <= HID_MOD_LAST) {
        uint8_t down, up;
        switch (kc) {
            case 0xE1: /* Left Shift  */
            case 0xE5: /* Right Shift */
                down = ZW_MOD_SHIFT_DOWN; up = ZW_MOD_SHIFT_UP; break;
            case 0xE0: /* Left Ctrl   */
            case 0xE4: /* Right Ctrl  */
                down = ZW_MOD_CTRL_DOWN;  up = ZW_MOD_CTRL_UP;  break;
            case 0xE2: /* Left Alt    */
            case 0xE6: /* Right Alt   */
                down = ZW_MOD_ALT_DOWN;   up = ZW_MOD_ALT_UP;   break;
            case 0xE3: /* Left GUI    */
            case 0xE7: /* Right GUI   */
                down = ZW_MOD_META_DOWN;  up = ZW_MOD_META_UP;  break;
            default: return ZMK_EV_EVENT_BUBBLE;
        }
        zw_send(ev->pressed ? down : up);
        return ZMK_EV_EVENT_BUBBLE;
    }

    /* Regular keys: send index byte on press only (no release byte) */
    if (ev->pressed) {
        uint8_t zw_byte = hid_to_zw(kc);
        if (zw_byte != ZW_NO_KEY) {
            zw_send(zw_byte);
        }
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(zerowriter_uart, zw_keycode_listener);
ZMK_SUBSCRIPTION(zerowriter_uart, zmk_keycode_state_changed);

static int zw_init(void) {
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("ZeroWriter: uart0 not ready — check pinctrl in overlay");
        return -ENODEV;
    }
    LOG_INF("ZeroWriter: UART ready, TX on P1.01 at 921600 baud");
    return 0;
}

SYS_INIT(zw_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
