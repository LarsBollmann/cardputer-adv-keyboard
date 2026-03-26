//! A keyboard driver for the [M5Stack Cardputer-Adv](https://docs.m5stack.com/en/core/Cardputer-Adv), built on top of the [tca8418](https://crates.io/crates/tca8418) driver.
//! Handles key press/release events, shift/Fn layer resolution and modifier tracking.
//!
//! This crate provides a [Keyboard] type that can be created using a type that implements [I2c].
//! This is provided by the HAL you use.
//! All methods using the underlying I²C bus return a [tca8418::Error] on I²C failure.
//!
//! You can get all pending inputs using [.inputs()](Keyboard::inputs) on the created [Keyboard].
//! This drains the FIFO of the TCA8418 which can store up to 10 events.
//!
//! You can either use polling or the TCA8418's interrupt output to know when events are available.
//!
//! # Examples
//! Here is an basic example on how to capture user text input.
//! See the examples in the repository for a full example app using esp-hal.
//!
//! ```rust,no_run
//! # fn process_input(_input: &heapless::String<64>) {}
//! # fn example(i2c: impl embedded_hal::i2c::I2c) {
//! use cardputer_adv_keyboard::{KeyInput, Keyboard};
//! use heapless::String;
//!
//! let mut input_string: String<64> = String::new();
//!
//! // You need to create i2c using your HAL crate.
//! let mut keyboard = Keyboard::new(i2c);
//! keyboard.init().unwrap();
//!
//! loop {
//!     for input in keyboard.inputs().unwrap() {
//!         match input {
//!             KeyInput::Char(c) => { input_string.push(c).ok(); }
//!             KeyInput::Backspace => { input_string.pop(); }
//!             KeyInput::Enter => {
//!                 process_input(&input_string);
//!                 input_string.clear();
//!             }
//!             _ => {}
//!         }
//!     }
//! }
//! # }
//!
//! ```
//!
//! ## KeyInput
//! A [KeyInput] represents a resolved key press. This encompasses everything that is printed on the keyboard, including the Shift and Fn layers.
//!
//!
//! ### Some examples
//! | Pressed keys | Result |
//! |------|--------|
//! | `w` | `KeyInput::Char('w')` |
//! | Shift + `w` | `KeyInput::Char('W')` |
//! | `;` | `KeyInput::Char(';')` |
//! | Shift + `;` | `KeyInput::Char(':')` |
//! | `Space` | `KeyInput::Char(' ')`|
//!
//! The other character keys work the same.
//! If a key has no Shift or Fn layer, it just returns it's default character, even when modifiers are pressed.
//!
//! All special keys have their own KeyInput variant, for example:
//! | Pressed keys | Result |
//! |------|--------|
//! | Fn + `` ` `` | `KeyInput::Escape` |
//! | Fn + `;` | `KeyInput::Arrow(Arrow::Up)` |
//! | Fn + `Backspace` | `KeyInput::Delete` |
//!
//! # Advanced usage
//! If you need more information about key events (key releases, pressed modifiers) you can directly get the [KeyboardEvents](KeyboardEvent) using `.events()`.
//! Using these you can create custom chords (like Ctrl + C) or track if a key is being held.
//!
//! ## Listening for custom combinations
//! ```rust,no_run
//! # fn copy() {}
//! # fn paste() {}
//! # fn redo() {}
//! # fn undo() {}
//! # fn example(keyboard: &mut cardputer_adv_keyboard::Keyboard<impl embedded_hal::i2c::I2c>) {
//! use cardputer_adv_keyboard::{PhysicalKey, ModifierState};

//! for event in keyboard.events().unwrap() {
//!     // Only act on presses
//!     if !event.state.is_pressed() {
//!         continue;
//!     }
//!
//!     match (event.physical_key, event.modifiers) {
//!         (PhysicalKey::C, ModifierState { ctrl: true, .. }) => copy(),
//!         (PhysicalKey::V, ModifierState { ctrl: true, .. }) => paste(),
//!         (PhysicalKey::Z, ModifierState { ctrl: true, shift: true, .. }) => redo(),
//!         (PhysicalKey::Z, ModifierState { ctrl: true, .. }) => undo(),
//!         _ => {}
//!     }
//! }
//! # }
//! ```
//!
//! ## Tracking held keys
//! ```
//! # struct Player{};
//! # impl Player {
//! #     fn move_forward(&self) {}
//! #     fn jump(&self) {}
//! # }
//! # fn example(keyboard: &mut cardputer_adv_keyboard::Keyboard<impl embedded_hal::i2c::I2c>) {
//! # let player = Player {};
//! use cardputer_adv_keyboard::{PhysicalKey, KeyState};
//! let mut w_held = false;
//!
//! loop {
//!     for event in keyboard.events().unwrap() {
//!         match (event.physical_key, event.state) {
//!             (PhysicalKey::W, KeyState::Pressed) => w_held = true,
//!             (PhysicalKey::W, KeyState::Released) => w_held = false,
//!             (PhysicalKey::Space, KeyState::Pressed) => player.jump(),
//!             _ => {}
//!         }
//!     }
//!
//!     if w_held {
//!         player.move_forward();
//!     }
//! }
//! # }
//! ```

#![no_std]

use embedded_hal::i2c::I2c;
use tca8418::{InterruptFlags, PinMask, Tca8418};

/// Maximum number of events returned from a single [`Keyboard::events`] call.
const MAX_EVENTS: usize = 10;

/// This type represents a key event (press or release).
///
/// Each event carries a [`physical_key`](Self::physical_key) identifying
/// which key on the keyboard was pressed or released.
/// 
/// On presses, [`input`](Self::input) contains the resolved [`KeyInput`]
/// after applying shift/Fn modifiers.
/// On releases it is `None`.
/// 
/// If you only care about basic input then you can
/// just use `input`.
/// If you need raw information about pressed and released keys (games, custom bindings) you can use `physical_key` and the additional information in this struct.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct KeyboardEvent {
    /// The resolved semantic input, if applicable.
    ///
    /// `Some` on key press, `None` on key release. Release events should
    /// be matched by `physical`, not by resolved input, since the modifier
    /// state may have changed between press and release.
    pub input: Option<KeyInput>,
    /// Which physical key was pressed or released. This is always independent of modifier keys.
    pub physical_key: PhysicalKey,
    /// Whether the key was pressed or released.
    pub state: KeyState,
    /// The modifier state at the time of this event.
    pub modifiers: ModifierState,
}

/// Whether a key was pressed or released.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum KeyState {
    Pressed,
    Released,
}

impl KeyState {
    pub fn is_pressed(self) -> bool {
        matches!(self, KeyState::Pressed)
    }

    pub fn is_released(self) -> bool {
        matches!(self, KeyState::Released)
    }
}
/// The semantic meaning of a key press after applying modifiers.
///
/// This represents what a key *produces*, not which key was physically
/// pressed. For example, pressing `W` while holding Shift produces
/// `KeyInput::Char('W')`, and pressing `;` while holding Fn produces
/// `KeyInput::Arrow(Arrow::Up)`.
///
/// ## Some examples
/// | Pressed keys | Resulting KeyInput |
/// |------|--------|
/// | `;` | `Char(';')` |
/// | Shift + `;` | `Char(':')` |
/// | Fn + `;` | `Arrow(Arrow::Up)` |
/// | Space | `Char(' ')` |
/// | Fn + `` ` `` | `Escape` |
/// | Fn + Backspace | `Delete` |
/// 
/// If a key has no special meaning for the currently pressed modifier, it just returns it's default.
/// 
/// Fn + `w` => `Char('w')
/// 
/// If a key has both a Shift and Fn layer, the Fn layer is always prioritized when both are pressed.
/// 
/// Fn + Shift + `;` => `Arrow(Arrow::Up)`
/// 
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum KeyInput {
    /// A printable character after applying shift/Fn modifiers.
    Char(char),
    /// Enter / Return key.
    Enter,
    /// Backspace key.
    Backspace,
    /// Delete (Fn + Backspace).
    Delete,
    /// Tab key.
    Tab,
    /// Escape (Fn + Backtick).
    Escape,
    /// An arrow key (Fn + `;` `,` `.` `/`).
    Arrow(Arrow),
    /// A modifier key was pressed (Shift, Fn, Ctrl, Opt, Alt).
    Modifier(Modifier),
}

/// Arrow key directions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Arrow {
    Up,
    Down,
    Left,
    Right,
}

/// Modifier keys.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Modifier {
    Shift,
    Fn,
    Ctrl,
    Opt,
    Alt,
}


/// This represents an actual physical key on the Cardputer keyboard.
///
/// Named after the labels on the keyboard.
///
/// | | | | | | | | | | | | | | |
/// |---|---|---|---|---|---|---|---|---|---|---|---|---|---|
/// | [`Backtick`](Self::Backtick) | [`N1`](Self::N1) | [`N2`](Self::N2) | [`N3`](Self::N3) | [`N4`](Self::N4) | [`N5`](Self::N5) | [`N6`](Self::N6) | [`N7`](Self::N7) | [`N8`](Self::N8) | [`N9`](Self::N9) | [`N0`](Self::N0) | [`Minus`](Self::Minus) | [`Equal`](Self::Equal) | [`Backspace`](Self::Backspace) |
/// | [`Tab`](Self::Tab) | [`Q`](Self::Q) | [`W`](Self::W) | [`E`](Self::E) | [`R`](Self::R) | [`T`](Self::T) | [`Y`](Self::Y) | [`U`](Self::U) | [`I`](Self::I) | [`O`](Self::O) | [`P`](Self::P) | [`LeftBracket`](Self::LeftBracket) | [`RightBracket`](Self::RightBracket) | [`Backslash`](Self::Backslash) |
/// | [`Fn`](Self::Fn) | [`Shift`](Self::Shift) | [`A`](Self::A) | [`S`](Self::S) | [`D`](Self::D) | [`F`](Self::F) | [`G`](Self::G) | [`H`](Self::H) | [`J`](Self::J) | [`K`](Self::K) | [`L`](Self::L) | [`Semicolon`](Self::Semicolon) | [`Apostrophe`](Self::Apostrophe) | [`Enter`](Self::Enter) |
/// | [`Ctrl`](Self::Ctrl) | [`Opt`](Self::Opt) | [`Alt`](Self::Alt) | [`Z`](Self::Z) | [`X`](Self::X) | [`C`](Self::C) | [`V`](Self::V) | [`B`](Self::B) | [`N`](Self::N) | [`M`](Self::M) | [`Comma`](Self::Comma) | [`Period`](Self::Period) | [`Slash`](Self::Slash) | [`Space`](Self::Space) |
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[rustfmt::skip]
pub enum PhysicalKey {
    // Row 0
    Backtick, N1, N2, N3, N4, N5, N6, N7, N8, N9, N0, Minus, Equal, Backspace,
    // Row 1
    Tab, Q, W, E, R, T, Y, U, I, O, P, LeftBracket, RightBracket, Backslash,
    // Row 2
    Fn, Shift, A, S, D, F, G, H, J, K, L, Semicolon, Apostrophe, Enter,
    // Row 3
    Ctrl, Opt, Alt, Z, X, C, V, B, N, M, Comma, Period, Slash, Space,
}

/// Defines the semantic behavior of a single key.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum KeymapEntry {
    /// A character key with (normal, shifted) variants.
    Char(char, char),
    /// One of the [modifier][Modifier] keys (Shift, Fn, Ctrl, Opt, Alt)
    Mod(Modifier),
    /// The enter key
    Enter,
    Backspace,
    Tab,
}

/// Mapping from TCA8418 reported key position to physical key.
const PHYSICAL_KEYS: [[PhysicalKey; 14]; 4] = {
    use PhysicalKey as Pk;
    [
        [
            Pk::Backtick,
            Pk::N1,
            Pk::N2,
            Pk::N3,
            Pk::N4,
            Pk::N5,
            Pk::N6,
            Pk::N7,
            Pk::N8,
            Pk::N9,
            Pk::N0,
            Pk::Minus,
            Pk::Equal,
            Pk::Backspace,
        ],
        [
            Pk::Tab,
            Pk::Q,
            Pk::W,
            Pk::E,
            Pk::R,
            Pk::T,
            Pk::Y,
            Pk::U,
            Pk::I,
            Pk::O,
            Pk::P,
            Pk::LeftBracket,
            Pk::RightBracket,
            Pk::Backslash,
        ],
        [
            Pk::Fn,
            Pk::Shift,
            Pk::A,
            Pk::S,
            Pk::D,
            Pk::F,
            Pk::G,
            Pk::H,
            Pk::J,
            Pk::K,
            Pk::L,
            Pk::Semicolon,
            Pk::Apostrophe,
            Pk::Enter,
        ],
        [
            Pk::Ctrl,
            Pk::Opt,
            Pk::Alt,
            Pk::Z,
            Pk::X,
            Pk::C,
            Pk::V,
            Pk::B,
            Pk::N,
            Pk::M,
            Pk::Comma,
            Pk::Period,
            Pk::Slash,
            Pk::Space,
        ],
    ]
};

/// Maps each physical key to its behavior.
///
/// This is a mapping that maps what each key on the keyboard actually produces when pressed.
/// This also includes multiple layers if the key supports it.
fn keymap(key: PhysicalKey) -> KeymapEntry {
    use KeymapEntry::*;
    use Modifier as Mo;

    match key {
        // Row 0
        PhysicalKey::Backtick => Char('`', '~'),
        PhysicalKey::N1 => Char('1', '!'),
        PhysicalKey::N2 => Char('2', '@'),
        PhysicalKey::N3 => Char('3', '#'),
        PhysicalKey::N4 => Char('4', '$'),
        PhysicalKey::N5 => Char('5', '%'),
        PhysicalKey::N6 => Char('6', '^'),
        PhysicalKey::N7 => Char('7', '&'),
        PhysicalKey::N8 => Char('8', '*'),
        PhysicalKey::N9 => Char('9', '('),
        PhysicalKey::N0 => Char('0', ')'),
        PhysicalKey::Minus => Char('-', '_'),
        PhysicalKey::Equal => Char('=', '+'),
        PhysicalKey::Backspace => Backspace,

        // Row 1
        PhysicalKey::Tab => Tab,
        PhysicalKey::Q => Char('q', 'Q'),
        PhysicalKey::W => Char('w', 'W'),
        PhysicalKey::E => Char('e', 'E'),
        PhysicalKey::R => Char('r', 'R'),
        PhysicalKey::T => Char('t', 'T'),
        PhysicalKey::Y => Char('y', 'Y'),
        PhysicalKey::U => Char('u', 'U'),
        PhysicalKey::I => Char('i', 'I'),
        PhysicalKey::O => Char('o', 'O'),
        PhysicalKey::P => Char('p', 'P'),
        PhysicalKey::LeftBracket => Char('[', '{'),
        PhysicalKey::RightBracket => Char(']', '}'),
        PhysicalKey::Backslash => Char('\\', '|'),

        // Row 2
        PhysicalKey::Fn => Mod(Mo::Fn),
        PhysicalKey::Shift => Mod(Mo::Shift),
        PhysicalKey::A => Char('a', 'A'),
        PhysicalKey::S => Char('s', 'S'),
        PhysicalKey::D => Char('d', 'D'),
        PhysicalKey::F => Char('f', 'F'),
        PhysicalKey::G => Char('g', 'G'),
        PhysicalKey::H => Char('h', 'H'),
        PhysicalKey::J => Char('j', 'J'),
        PhysicalKey::K => Char('k', 'K'),
        PhysicalKey::L => Char('l', 'L'),
        PhysicalKey::Semicolon => Char(';', ':'),
        PhysicalKey::Apostrophe => Char('\'', '"'),
        PhysicalKey::Enter => Enter,

        // Row 3
        PhysicalKey::Ctrl => Mod(Mo::Ctrl),
        PhysicalKey::Opt => Mod(Mo::Opt),
        PhysicalKey::Alt => Mod(Mo::Alt),
        PhysicalKey::Z => Char('z', 'Z'),
        PhysicalKey::X => Char('x', 'X'),
        PhysicalKey::C => Char('c', 'C'),
        PhysicalKey::V => Char('v', 'V'),
        PhysicalKey::B => Char('b', 'B'),
        PhysicalKey::N => Char('n', 'N'),
        PhysicalKey::M => Char('m', 'M'),
        PhysicalKey::Comma => Char(',', '<'),
        PhysicalKey::Period => Char('.', '>'),
        PhysicalKey::Slash => Char('/', '?'),
        PhysicalKey::Space => Char(' ', ' '),
    }
}

/// State that tracks which modifier keys are currently held down.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ModifierState {
    pub shift: bool,
    pub fn_key: bool,
    pub ctrl: bool,
    pub opt: bool,
    pub alt: bool,
}

/// High-level keyboard driver for the Cardputer-Adv.
///
/// The idea is to make getting input from the keyboard as easy as possible
/// while also exposing the raw key presses for more advanced use cases.
///
/// # Getting started
///
/// Create a [`Keyboard`] from [`I2c`] and call
/// [`init`](Keyboard::init) to configure the hardware:
///
/// ```rust,no_run
/// # fn example(i2c: impl embedded_hal::i2c::I2c) {
/// use tca8418::Tca8418;
/// use cardputer_adv_keyboard::Keyboard;
///
/// let mut keyboard = Keyboard::new(i2c);
/// keyboard.init().unwrap();
/// # }
/// ```
///
/// # Basic usage
///
/// Get an Iterator over [`KeyInput`]s by using [.inputs()](`Keyboard::inputs`).
/// A [`KeyInput`] encompasses everything that is printed on the keyboard.
///
/// For example, pressing the `w` key results in [`KeyInput::Char('w')`](KeyInput::Char).
/// If you press `w` while holding Shift (`Aa` key) you get
/// [`KeyInput::Char('W')`](KeyInput::Char). The same goes for every other key
/// that represents a character.
///
/// The function keys (Enter, Escape, Modifier keys, etc.)
/// have their own [`KeyInput`] variant.
///
/// Using this, getting text input is as easy as:
///
/// ```rust,no_run
/// # fn example(keyboard: &mut cardputer_adv_keyboard::Keyboard<impl embedded_hal::i2c::I2c>) {
/// use cardputer_adv_keyboard::KeyInput;
///
/// for input in keyboard.inputs().unwrap() {
///     match input {
///         KeyInput::Char(c) => { /* Do something with the input character */}
///         KeyInput::Backspace => { /* Handle backspace */}
///         KeyInput::Enter => { /* Submit */ }
///         _ => {}
///     }
/// }
/// # }
/// ```
///
/// # Advanced usage
///  
/// If you need more detailed information about key events like released keys or currently held modifiers
/// (for games, custom bindings, etc.), you can use [.events()](`Keyboard::events`) get an Iterator over
/// all pending [`KeyboardEvents`](KeyboardEvent)
///
/// in addition to an optional [`input`](KeyboardEvent::input) each [`KeyboardEvent`] also exposes:
///
/// - [`physical_key`](KeyboardEvent::physical_key): a [`PhysicalKey`] identifying
///   which key on the keyboard caused this event, regardless of modifier state.
/// - [`state`](KeyboardEvent::state): whether it was a
///   [`Pressed`](KeyState::Pressed) or [`Released`](KeyState::Released) event.
/// - [`modifiers`](KeyboardEvent::modifiers): a [`ModifierState`]
///   showing which modifiers were held at the time of the event.
///
/// For example, detecting a Ctrl+C combination:
///
/// ```rust,no_run
/// # fn example(event: cardputer_adv_keyboard::KeyboardEvent) {
/// use cardputer_adv_keyboard::PhysicalKey;
///
/// match (event.physical_key, event.modifiers.ctrl) {
///     (PhysicalKey::C, true) => { /* handle Ctrl+C */ }
///     _ => {}
/// }
/// # }
/// ```
///
/// # Iterator convenience methods
/// ```rust,no_run
/// # fn example(keyboard: &mut cardputer_adv_keyboard::Keyboard<impl embedded_hal::i2c::I2c>) {
/// // Only resolved inputs (skips releases automatically)
/// // This is what most people would need from a keyboard library.
/// keyboard.inputs().unwrap();
///
/// // If you need every single event (including releases, information about pressed modifiers)
/// // you can use .events().
/// let events = keyboard.events().unwrap();
///
/// // Only press events
/// let key_presses = keyboard.events().unwrap().presses_only();
/// // Only release events
/// let key_releases = events.releases_only();
/// // When using either of them you are throwing away all of the other events.
/// # }
/// ```
pub struct Keyboard<I2C> {
    tca8418: Tca8418<I2C>,
    modifiers: ModifierState,
}
impl<I2C, E> Keyboard<I2C>
where
    I2C: I2c<Error = E>,
{
    /// Create a new keyboard from a TCA8418 driver instance.
    pub fn new(i2c: I2C) -> Self {
        Self {
            tca8418: Tca8418::new(i2c),
            modifiers: ModifierState::default(),
        }
    }

    /// Initialize the TCA8418 for the Cardputer-Adv keyboard.
    ///
    /// Configures 7 rows (R0–R6) × 8 columns (C0–C7) and drains any
    /// stale events from the FIFO.
    pub fn init(&mut self) -> Result<(), tca8418::Error<E>> {
        let pins = PinMask::rows(0x7F) | PinMask::cols(0xFF);
        self.tca8418.configure_keypad(pins)?;
        while self.tca8418.read_event()?.is_some() {}
        Ok(())
    }

    /// Process a single raw TCA8418 event into a high-level keyboard event.
    ///
    /// Updates modifier state for modifier presses/releases. Returns a
    /// [`KeyboardEvent`] with `input: Some(...)` for presses and
    /// `input: None` for releases.
    fn process_event(&mut self, event: tca8418::KeyEvent) -> Option<KeyboardEvent> {
        let raw_key = match event.key {
            tca8418::Key::KeypadMatrix(k) => k,
            _ => return None,
        };

        let physical = Self::physical_key_from(raw_key.row, raw_key.col)?;
        let entry = keymap(physical);
        let state = if event.pressed {
            KeyState::Pressed
        } else {
            KeyState::Released
        };

        if let KeymapEntry::Mod(m) = entry {
            self.update_modifier(m, event.pressed);
            return Some(KeyboardEvent {
                input: if event.pressed {
                    Some(KeyInput::Modifier(m))
                } else {
                    None
                },
                physical_key: physical,
                state,
                modifiers: self.modifiers,
            });
        }

        Some(KeyboardEvent {
            input: if event.pressed {
                Some(self.resolve_key(physical))
            } else {
                None
            },
            physical_key: physical,
            state,
            modifiers: self.modifiers,
        })
    }

    /// Read all pending events from the TCA8418 and return them as an iterator of [KeyInputs](KeyInput)
    /// 
    /// This drains all events from the TCA8418 FIFO including key releases, which will be discarded.
    /// If you also need key releases, use [.events()](Keyboard::events) instead.
    pub fn inputs(&mut self) -> Result<impl Iterator<Item = KeyInput>, tca8418::Error<E>> {
        Ok(self.events()?.inputs())
    }

    /// Read all pending events from the TCA8418 and return an Iterator over [KeyboardEvent].
    /// 
    /// This drains all events from the TCA8418 FIFO.
    pub fn events(&mut self) -> Result<KeyboardEventIter, tca8418::Error<E>> {
        let mut events = [None; MAX_EVENTS];
        let mut count = 0;

        for raw in self.tca8418.events()? {
            if let Some(event) = self.process_event(raw) {
                events[count] = Some(event);
                count += 1;
            }
        }

        Ok(KeyboardEventIter {
            events,
            index: 0,
            count,
        })
    }

    /// Get a reference to the underlying TCA8418 driver.
    pub fn tca(&self) -> &Tca8418<I2C> {
        &self.tca8418
    }

    /// Get a mutable reference to the underlying TCA8418 driver.
    pub fn tca_mut(&mut self) -> &mut Tca8418<I2C> {
        &mut self.tca8418
    }

    /// Consume this keyboard and return the underlying TCA8418 driver.
    pub fn into_inner(self) -> Tca8418<I2C> {
        self.tca8418
    }

    /// Map a TCA8418 (row, col) to the physical key at that position.
    fn physical_key_from(tca_row: u8, tca_col: u8) -> Option<PhysicalKey> {
        let phys_row = (tca_col % 4) as usize;
        let phys_col = if tca_col >= 4 {
            tca_row as usize * 2 + 1
        } else {
            tca_row as usize * 2
        };

        if phys_row < 4 && phys_col < 14 {
            Some(PHYSICAL_KEYS[phys_row][phys_col])
        } else {
            None
        }
    }

    /// Resolve a keymap entry into a semantic input using the current
    /// modifier state.
    fn resolve_key(&self, physical_key: PhysicalKey) -> KeyInput {
        let entry = keymap(physical_key);
        match entry {
            KeymapEntry::Char(normal, shifted) => {
                if self.modifiers.fn_key {
                    // Handle fn layer
                    match physical_key {
                        PhysicalKey::Backtick => KeyInput::Escape,
                        PhysicalKey::Backslash => KeyInput::Delete,
                        PhysicalKey::Semicolon => KeyInput::Arrow(Arrow::Up),
                        PhysicalKey::Comma => KeyInput::Arrow(Arrow::Left),
                        PhysicalKey::Period => KeyInput::Arrow(Arrow::Down),
                        PhysicalKey::Slash => KeyInput::Arrow(Arrow::Right),
                        _ => {
                            if self.modifiers.shift {
                                KeyInput::Char(shifted)
                            } else {
                                KeyInput::Char(normal)
                            }
                        }
                    }
                } else if self.modifiers.shift {
                    KeyInput::Char(shifted)
                } else {
                    KeyInput::Char(normal)
                }
            }
            KeymapEntry::Enter => KeyInput::Enter,
            KeymapEntry::Backspace => {
                if self.modifiers.fn_key {
                    KeyInput::Delete
                } else {
                    KeyInput::Backspace
                }
            }
            KeymapEntry::Tab => KeyInput::Tab,
            KeymapEntry::Mod(m) => KeyInput::Modifier(m),
        }
    }

    /// Update internal modifier state.
    fn update_modifier(&mut self, modifier: Modifier, pressed: bool) {
        match modifier {
            Modifier::Shift => self.modifiers.shift = pressed,
            Modifier::Fn => self.modifiers.fn_key = pressed,
            Modifier::Ctrl => self.modifiers.ctrl = pressed,
            Modifier::Opt => self.modifiers.opt = pressed,
            Modifier::Alt => self.modifiers.alt = pressed,
        }
    }

    /// Check if the TCA8418 has available key events in its FIFO
    pub fn events_available(&mut self) -> Result<bool, tca8418::Error<E>> {
        Ok(self.tca8418.event_count()? > 0)
    }

    /// See if the keyboard event interrupt status flag is set
    pub fn has_key_interrupt_pending(&mut self) -> Result<bool, tca8418::Error<E>> {
        self.tca8418.has_pending_key_event()
    }

    /// Enable keyboard event interrupts
    pub fn enable_keyboard_interrupts(&mut self) -> Result<(), tca8418::Error<E>> {
        self.tca8418.enable_key_event_interrupt(true)
    }

    /// Enable keyboard event interrupts
    pub fn disable_keyboard_interrupts(&mut self) -> Result<(), tca8418::Error<E>> {
        self.tca8418.enable_key_event_interrupt(false)
    }

    /// Clear the key event interrupt flag.
    pub fn clear_interrupts(&mut self) -> Result<(), tca8418::Error<E>> {
        self.tca8418.clear_interrupts(InterruptFlags::K_INT)
    }
}

/// Iterator over processed keyboard events from a single [`Keyboard::events`] call.
pub struct KeyboardEventIter {
    #[doc(hidden)]
    events: [Option<KeyboardEvent>; MAX_EVENTS],
    #[doc(hidden)]
    index: usize,
    #[doc(hidden)]
    count: usize,
}

impl KeyboardEventIter {
    /// Filter to only key-press events.
    pub fn presses_only(self) -> impl Iterator<Item = KeyboardEvent> {
        self.filter(|e| e.state.is_pressed())
    }

    /// Filter to only key-release events.
    pub fn releases_only(self) -> impl Iterator<Item = KeyboardEvent> {
        self.filter(|e| e.state.is_released())
    }

    /// Extract just the resolved inputs, skipping key releases.
    pub fn inputs(self) -> impl Iterator<Item = KeyInput> {
        self.filter_map(|e| e.input)
    }
}

impl Iterator for KeyboardEventIter {
    type Item = KeyboardEvent;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index >= self.count {
            return None;
        }
        let event = self.events[self.index].take();
        self.index += 1;
        event
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.count - self.index;
        (remaining, Some(remaining))
    }
}

impl ExactSizeIterator for KeyboardEventIter {}
