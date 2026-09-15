#![no_std]

extern crate alloc;

use alloc::boxed::Box;
use core::future::Future;
use core::pin::Pin;

use xpanse_api::{
    bus::allocator::BusAllocator,
    driver::{DriverError, DriverMeta, DualSlotDriver},
    gpio_bank::{BankPins, GpioBank},
    interfaces::leds::{Generic, pin_led},
    metadata::{ModuleDetectResistor, ModuleID, ModuleSlot},
    registry::Registry,
    reexports::{
        embassy_futures::select::select_array,
        embassy_rp::{
            Peri,
            gpio::{AnyPin, Input, Level, Output, Pull},
        },
        embassy_time::Timer,
    },
};

/// ROW1..ROW4, on GPIO0..GPIO3 of the left connector (J2).
const ROWS: usize = 4;
/// COL1..COL10, on GPIO0..GPIO9 of the right connector (J1).
const COLS: usize = 10;

/// Time for a column to follow its row after the row is switched.
const SETTLE_US: u64 = 10;
const SCAN_INTERVAL_MS: u64 = 1;
/// A matrix change is accepted once it reads the same for this many scans.
const DEBOUNCE_SCANS: u32 = 5;

/// A physical key on the module.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Key {
    Q,
    W,
    E,
    R,
    T,
    Y,
    U,
    I,
    O,
    P,
    A,
    S,
    D,
    F,
    G,
    H,
    J,
    K,
    L,
    Z,
    X,
    C,
    V,
    B,
    N,
    M,
    Comma,
    Period,
    Space,
    /// Bottom row, far left (SW29).
    Mod1,
    /// Bottom row, left of the space bar (SW30).
    Mod2,
    /// Bottom row, right of the space bar (SW32).
    Mod3,
    /// Bottom row, far right (SW33).
    Mod4,
}

impl Key {
    /// The character the key types without modifiers, or `None` for modifiers.
    pub const fn as_char(self) -> Option<char> {
        Some(match self {
            Key::Q => 'q',
            Key::W => 'w',
            Key::E => 'e',
            Key::R => 'r',
            Key::T => 't',
            Key::Y => 'y',
            Key::U => 'u',
            Key::I => 'i',
            Key::O => 'o',
            Key::P => 'p',
            Key::A => 'a',
            Key::S => 's',
            Key::D => 'd',
            Key::F => 'f',
            Key::G => 'g',
            Key::H => 'h',
            Key::J => 'j',
            Key::K => 'k',
            Key::L => 'l',
            Key::Z => 'z',
            Key::X => 'x',
            Key::C => 'c',
            Key::V => 'v',
            Key::B => 'b',
            Key::N => 'n',
            Key::M => 'm',
            Key::Comma => ',',
            Key::Period => '.',
            Key::Space => ' ',
            Key::Mod1 | Key::Mod2 | Key::Mod3 | Key::Mod4 => return None,
        })
    }

    /// Bit of this key in a matrix state word.
    fn bit(self) -> u64 {
        for (index, key) in KEYMAP.iter().flatten().enumerate() {
            if *key == Some(self) {
                return 1 << index;
            }
        }
        unreachable!("every key is in the keymap")
    }
}

/// Key at each `[row][column]` of the matrix, taken from the PCB netlist.
/// Positions with no switch are `None`.
const KEYMAP: [[Option<Key>; COLS]; ROWS] = {
    use Key::*;
    [
        [Some(Q), Some(W), Some(E), Some(R), Some(T), Some(Y), Some(U), Some(I), Some(O), Some(P)],
        [Some(A), Some(S), Some(D), Some(F), Some(G), Some(H), Some(J), Some(K), Some(L), None],
        [Some(Z), Some(X), Some(C), Some(V), Some(B), Some(N), Some(M), Some(Comma), Some(Period), None],
        [Some(Mod1), Some(Mod2), None, Some(Space), None, None, None, None, Some(Mod3), Some(Mod4)],
    ]
};

/// Bits of the matrix positions that have a switch.
const KEY_MASK: u64 = {
    let mut mask = 0;
    let mut index = 0;
    while index < ROWS * COLS {
        if KEYMAP[index / COLS][index % COLS].is_some() {
            mask |= 1 << index;
        }
        index += 1;
    }
    mask
};

/// A debounced key press or release.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KeyEvent {
    pub key: Key,
    pub pressed: bool,
}

/// Async interface to the keyboard.
///
/// Apps lease it from the registry as `Box<dyn Keyboard>`.
pub trait Keyboard: Send {
    /// Wait for the next key press or release.
    fn next_event<'a>(&'a mut self) -> Pin<Box<dyn Future<Output = KeyEvent> + Send + 'a>>;

    /// Returns `true` if `key` is held, as of the last event returned by
    /// [`Keyboard::next_event`].
    fn is_pressed(&self, key: Key) -> bool;
}

/// 4x10 matrix with diodes from column to row: a row is selected by driving it
/// low, and a pressed key on that row pulls its column low through the diode.
struct KeyMatrix {
    rows: [Output<'static>; ROWS],
    cols: [Input<'static>; COLS],
    /// Debounced matrix state, one bit per position.
    debounced: u64,
    /// State already delivered to the app as events.
    reported: u64,
}

impl KeyMatrix {
    fn new(rows: [Peri<'static, AnyPin>; ROWS], cols: [Peri<'static, AnyPin>; COLS]) -> Self {
        Self {
            rows: rows.map(|pin| Output::new(pin, Level::High)),
            cols: cols.map(|pin| Input::new(pin, Pull::Up)),
            debounced: 0,
            reported: 0,
        }
    }

    /// Read every row once and return the raw matrix state.
    async fn scan(&mut self) -> u64 {
        let mut state = 0;
        for (row_index, row) in self.rows.iter_mut().enumerate() {
            row.set_low();
            Timer::after_micros(SETTLE_US).await;
            for (col_index, col) in self.cols.iter().enumerate() {
                if col.is_low() {
                    state |= 1 << (row_index * COLS + col_index);
                }
            }
            row.set_high();
        }
        state & KEY_MASK
    }

    /// Sleep until any key goes down, instead of polling an idle keyboard.
    async fn wait_for_activity(&mut self) {
        for row in &mut self.rows {
            row.set_low();
        }
        select_array(self.cols.each_mut().map(|col| col.wait_for_low())).await;
        for row in &mut self.rows {
            row.set_high();
        }
    }

    /// Scan until the matrix settles on a state different from `debounced`.
    async fn update(&mut self) {
        let mut last = self.scan().await;
        let mut stable_scans = 0;
        loop {
            Timer::after_millis(SCAN_INTERVAL_MS).await;
            let state = self.scan().await;
            if state != last {
                last = state;
                stable_scans = 0;
                continue;
            }
            stable_scans += 1;
            if stable_scans >= DEBOUNCE_SCANS && state != self.debounced {
                self.debounced = state;
                return;
            }
        }
    }

    /// Take one change between the debounced and reported states.
    fn pop_event(&mut self) -> Option<KeyEvent> {
        let changed = self.debounced ^ self.reported;
        if changed == 0 {
            return None;
        }
        let index = changed.trailing_zeros() as usize;
        let bit = 1 << index;
        self.reported ^= bit;
        let key = KEYMAP[index / COLS][index % COLS].expect("KEY_MASK only keeps real keys");
        Some(KeyEvent {
            key,
            pressed: self.reported & bit != 0,
        })
    }
}

impl Keyboard for KeyMatrix {
    fn next_event<'a>(&'a mut self) -> Pin<Box<dyn Future<Output = KeyEvent> + Send + 'a>> {
        Box::pin(async move {
            loop {
                if let Some(event) = self.pop_event() {
                    return event;
                }
                if self.debounced == 0 {
                    self.wait_for_activity().await;
                }
                self.update().await;
            }
        })
    }

    fn is_pressed(&self, key: Key) -> bool {
        self.reported & key.bit() != 0
    }
}

/// Driver for the qwertyxpansion keyboard, which spans two slots.
///
/// J1 (right) carries COL1..COL10 on GPIO0..GPIO9 and the ID resistors.
/// J2 (left) carries ROW1..ROW4 on GPIO0..GPIO3 and the LED on GPIO4.
pub struct QwertyDriver;

impl DriverMeta for QwertyDriver {
    // R2 (1K) on J1-MD0, R1 (75K) on J1-MD1; J2 leaves its MD pins open
    const ID: ModuleID = ModuleID {
        md0: ModuleDetectResistor::R1K,
        md1: ModuleDetectResistor::R75K,
    };
}

impl<G1: BankPins, G2: BankPins> DualSlotDriver<G1, G2> for QwertyDriver {
    async fn create(
        // 0 is the right slot (J1), 1 is the left slot (J2)
        (right, left): (GpioBank<G1>, GpioBank<G2>),
        slots: (ModuleSlot, ModuleSlot),
        registry: &mut Registry,
        bus_allocator: &mut BusAllocator,
    ) -> Result<(), DriverError> {
        let cols = [
            right.gpio0.into(),
            right.gpio1.into(),
            right.gpio2.into(),
            right.gpio3.into(),
            right.gpio4.into(),
            right.gpio5.into(),
            right.gpio6.into(),
            right.gpio7.into(),
            right.gpio8.into(),
            right.gpio9.into(),
        ];
        let rows = [
            left.gpio0.into(),
            left.gpio1.into(),
            left.gpio2.into(),
            left.gpio3.into(),
        ];

        registry.register(
            slots.0,
            Self::ID,
            Box::new(KeyMatrix::new(rows, cols)) as Box<dyn Keyboard>,
        );

        // D34 is driven active-high from J2 GPIO4 through R3
        registry.register(
            slots.0,
            Self::ID,
            pin_led::<Generic>(left.gpio4.into(), false),
        );

        // we don't use any busses
        let _ = bus_allocator;

        Ok(())
    }
}
