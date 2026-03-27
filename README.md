# Cardputer-Adv keyboard

[![Crates.io](https://img.shields.io/crates/v/cardputer-adv-keyboard)](https://crates.io/crates/cardputer-adv-keyboard)
[![docs.rs](https://img.shields.io/docsrs/cardputer-adv-keyboard)](https://docs.rs/cardputer-adv-keyboard)
[![CI](https://github.com/larsbollmann/cardputer-adv-keyboard/actions/workflows/release.yml/badge.svg)](https://github.com/larsbollmann/cardputer-adv-keyboard/actions)
![License](https://img.shields.io/crates/l/cardputer-adv-keyboard)

A keyboard driver for the [M5Stack Cardputer-Adv](https://docs.m5stack.com/en/core/CardputerAdv), built on top of the [tca8418](https://crates.io/crates/tca8418) driver.
Handles press/release events, shift/Fn layer resolution and modifier tracking.

## Getting started
```rust,ignore
use cardputer_adv_keyboard::Keyboard;

let mut keyboard = Keyboard::new(i2c).unwrap();
```

## Usage
If you are only interested in inputs, use `.inputs()` to get the resolved inputs of the current key presses in the buffer.

These are what the key press *produces*, not which physical key was pressed.
This means pressing `Aa` + `G` maps to an uppercase `G` whereas pressing just `G` maps to a lowercase `g`.
Pressing Fn + `;` maps to an up arrow, etc.

In short: What you see on the keyboard is what you get.

Here is a simple example on how to capture user input:

```rust,ignore
use cardputer_adv_keyboard::KeyInput;

for input in keyboard.inputs().unwrap() {
    match input {
        KeyInput::Char(c) => { /* handle character input */ }
        KeyInput::Backspace => { /* handle backspace */ }
        KeyInput::Enter => { /* handle enter */ }
        _ => {}
    }
}
```

## Advanced usage
If you are interested in the raw key press and release events, you can use `.events()` which returns full `KeyboardEvent`s
with physical key identity, press/release state, and a current modifiers:

This allows you to track when a key is released or define custom `Modifier + Key` combinations
```rust,ignore
use cardputer_adv_keyboard::PhysicalKey;

for event in keyboard.events().unwrap() {
    match (event.physical_key, event.modifiers.ctrl) {
        (PhysicalKey::C, true) => { /* handle Ctrl+C */ }
        _ => {}
    }
}
```

See the [docs](https://docs.rs/cardputer-adv-keyboard) for more details.