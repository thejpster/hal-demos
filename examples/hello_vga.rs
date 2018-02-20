//! Emits 800 x 600 @ 60Hz VGA in mono.
//!
//! HSYNC is PB6
//! VSYNC is PC4
//! Analog Green is PB7 (you will need a buffer/level convertor to drop 3.3V to 0.7V, ideally with 75R source impedance)
//!
//! The row timing assumes 40 MHz but we're at 80 MHz so we'll need to double
//! everything given in the timing specifications. The column timing is in
//! lines.
//!
//! ```
//! Column timing on Timer0:
//! +----+
//! |SYNC|BP                            FP
//! +    +--------------------------------
//! --------||||||||||||||||||||||||||||--
//!
//! Row timing:
//! +----+
//! |SYNC|BP                            FP
//! +    +--------------------------------
//! --------||||||||||||||||||||||||||||--
//! ```
//!
//! We use PWM output from Timer0A to drive the horizontal sync pulse. We use Timer0B also in PWM mode to interrupt when
//! the SYNC + BP (back porch) period is over; i.e when it's time to start clocking out 400 pixels at 20MHz. To do that, we
//! use SSI2 (an SPI peripheral).

#![feature(used)]
#![no_std]

extern crate bresenham;
extern crate cortex_m;
extern crate cortex_m_rt;
extern crate cortex_m_semihosting;
extern crate embedded_hal;
extern crate tm4c123x_hal;

use cortex_m::asm;
use core::fmt::Write;
use embedded_hal::prelude::*;
use tm4c123x_hal::gpio::GpioExt;
use tm4c123x_hal::delay::Delay;
use tm4c123x_hal::sysctl::{self, chip_id, SysctlExt};
use tm4c123x_hal::bb;

const H_VISIBLE_AREA: u32 = 800 * 2;
const H_FRONT_PORCH: u32 = 40 * 2;
const H_SYNC_PULSE: u32 = 128 * 2;
const H_BACK_PORCH: u32 = 88 * 2;
const H_WHOLE_LINE: u32 = H_VISIBLE_AREA + H_FRONT_PORCH + H_SYNC_PULSE + H_BACK_PORCH;
const H_SYNC_END: u32 = (H_WHOLE_LINE - H_SYNC_PULSE);
const ISR_LATENCY: u32 = 44;
const H_LINE_START: u32 = H_WHOLE_LINE - (H_SYNC_PULSE + H_BACK_PORCH) + ISR_LATENCY;
const V_VISIBLE_AREA: usize = 600;
const V_FRONT_PORCH: usize = 1;
const V_SYNC_PULSE: usize = 4;
const V_BACK_PORCH: usize = 23;
const V_WHOLE_FRAME: usize = V_SYNC_PULSE + V_BACK_PORCH + V_VISIBLE_AREA + V_FRONT_PORCH;

/// Text characters are this many pixels across
pub const FONT_WIDTH: usize = 8;
/// Text characters are this many pixels high
pub const FONT_HEIGHT: usize = 16;
/// Number of lines in frame buffer
pub const VISIBLE_LINES: usize = 300;
/// Highest Y co-ord
pub const MAX_Y: usize = VISIBLE_LINES - 1;
/// Number of columns in frame buffer
pub const VISIBLE_COLS: usize = 400;
/// Highest X co-ord
pub const MAX_X: usize = VISIBLE_COLS - 1;
/// How many 16-bit words in a line
pub const HORIZONTAL_WORDS: usize = (VISIBLE_COLS + 15) / 16;
/// How many characters in a line
pub const TEXT_NUM_COLS: usize = VISIBLE_COLS / FONT_WIDTH;
/// Highest X co-ord for text
pub const TEXT_MAX_COL: usize = TEXT_NUM_COLS - 1;
/// How many lines of characters on the screen
pub const TEXT_NUM_ROWS: usize = VISIBLE_LINES / FONT_HEIGHT;
/// Highest Y co-ord for text
pub const TEXT_MAX_ROW: usize = TEXT_NUM_ROWS - 1;

// #[repr(align(1024))]
// struct DmaInfo {
//     _data: [u8; 1024],
// }

/// This is our own custom character set.
#[repr(u8)]
#[derive(Copy, Clone)]
enum Glyph {
    Unknown,
    Space,
    ExclamationMark,
    DoubleQuote,
    Hash,
    Dollar,
    Percent,
    Ampersand,
    SingleQuote,
    OpenRoundBracket,
    CloseRoundBracket,
    Asterisk,
    Plus,
    Comma,
    Minus,
    Period,
    Slash,
    Digit0,
    Digit1,
    Digit2,
    Digit3,
    Digit4,
    Digit5,
    Digit6,
    Digit7,
    Digit8,
    Digit9,
    Colon,
    SemiColon,
    LessThan,
    Equals,
    GreaterThan,
    QuestionMark,
    At,
    UppercaseA,
    UppercaseB,
    UppercaseC,
    UppercaseD,
    UppercaseE,
    UppercaseF,
    UppercaseG,
    UppercaseH,
    UppercaseI,
    UppercaseJ,
    UppercaseK,
    UppercaseL,
    UppercaseM,
    UppercaseN,
    UppercaseO,
    UppercaseP,
    UppercaseQ,
    UppercaseR,
    UppercaseS,
    UppercaseT,
    UppercaseU,
    UppercaseV,
    UppercaseW,
    UppercaseX,
    UppercaseY,
    UppercaseZ,
    OpenSquareBracket,
    Backslash,
    CloseSquareBracket,
    Caret,
    Underscore,
    Backtick,
    LowercaseA,
    LowercaseB,
    LowercaseC,
    LowercaseD,
    LowercaseE,
    LowercaseF,
    LowercaseG,
    LowercaseH,
    LowercaseI,
    LowercaseJ,
    LowercaseK,
    LowercaseL,
    LowercaseM,
    LowercaseN,
    LowercaseO,
    LowercaseP,
    LowercaseQ,
    LowercaseR,
    LowercaseS,
    LowercaseT,
    LowercaseU,
    LowercaseV,
    LowercaseW,
    LowercaseX,
    LowercaseY,
    LowercaseZ,
    OpenBrace,
    VerticalBar,
    CloseBrace,
    Tilde,
}

const FONT_DATA: [u8; 1536] = [
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,  // <unknown>
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // <space>
    0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x00,0x00,0x10,0x10,0x00,0x00,  // !
    0x24,0x24,0x24,0x24,0x24,0x24,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // "
    0x24,0x24,0x24,0x24,0x7E,0x7E,0x24,0x24,0x7E,0x7E,0x24,0x24,0x24,0x24,0x00,0x00,  // #
    0x10,0x10,0x3C,0x3C,0x50,0x50,0x38,0x38,0x14,0x14,0x78,0x78,0x10,0x10,0x00,0x00,  // $
    0x00,0x00,0x62,0x62,0x64,0x64,0x08,0x08,0x10,0x10,0x26,0x26,0x46,0x46,0x00,0x00,  // %
    0x30,0x30,0x48,0x48,0x48,0x48,0x30,0x30,0x4A,0x4A,0x44,0x44,0x3A,0x3A,0x00,0x00,  // &
    0x10,0x10,0x10,0x10,0x10,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // '
    0x10,0x10,0x20,0x20,0x40,0x40,0x40,0x40,0x40,0x40,0x20,0x20,0x10,0x10,0x00,0x00,  // (
    0x10,0x10,0x08,0x08,0x04,0x04,0x04,0x04,0x04,0x04,0x08,0x08,0x10,0x10,0x00,0x00,  // )
    0x10,0x10,0x54,0x54,0x38,0x38,0x10,0x10,0x38,0x38,0x54,0x54,0x10,0x10,0x00,0x00,  // *
    0x00,0x00,0x10,0x10,0x10,0x10,0x7C,0x7C,0x10,0x10,0x10,0x10,0x00,0x00,0x00,0x00,  // +
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x08,0x08,0x08,0x10,0x10,0x00,0x00,  // ,
    0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x7E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // -
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x10,0x00,0x00,  // .
    0x00,0x00,0x02,0x02,0x04,0x04,0x08,0x08,0x10,0x10,0x20,0x20,0x40,0x40,0x00,0x00,  // /

    0x3C,0x3C,0x42,0x42,0x46,0x46,0x5A,0x5A,0x62,0x62,0x42,0x42,0x3C,0x3C,0x00,0x00,  // 0
    0x08,0x08,0x18,0x18,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x1C,0x1C,0x00,0x00,  // 1
    0x3C,0x3C,0x42,0x42,0x02,0x02,0x1C,0x1C,0x20,0x20,0x40,0x40,0x7E,0x7E,0x00,0x00,  // 2
    0x7E,0x7E,0x02,0x02,0x04,0x04,0x1C,0x1C,0x02,0x02,0x42,0x42,0x3C,0x3C,0x00,0x00,  // 3
    0x04,0x04,0x0C,0x0C,0x14,0x14,0x24,0x24,0x7E,0x7E,0x04,0x04,0x04,0x04,0x00,0x00,  // 4
    0x7E,0x7E,0x40,0x40,0x7C,0x7C,0x02,0x02,0x02,0x02,0x42,0x42,0x3C,0x3C,0x00,0x00,  // 5
    0x1E,0x1E,0x20,0x20,0x40,0x40,0x7C,0x7C,0x42,0x42,0x42,0x42,0x3C,0x3C,0x00,0x00,  // 6
    0x7E,0x7E,0x02,0x02,0x04,0x04,0x08,0x08,0x10,0x10,0x10,0x10,0x10,0x10,0x00,0x00,  // 7
    0x3C,0x3C,0x42,0x42,0x42,0x42,0x3C,0x3C,0x42,0x42,0x42,0x42,0x3C,0x3C,0x00,0x00,  // 8
    0x3C,0x3C,0x42,0x42,0x42,0x42,0x3E,0x3E,0x02,0x02,0x04,0x04,0x78,0x78,0x00,0x00,  // 9
    0x00,0x00,0x00,0x00,0x10,0x10,0x00,0x00,0x10,0x10,0x00,0x00,0x00,0x00,0x00,0x00,  // :
    0x00,0x00,0x00,0x00,0x08,0x08,0x00,0x00,0x08,0x08,0x08,0x08,0x10,0x00,0x00,0x00,  // ;
    0x04,0x04,0x08,0x08,0x10,0x10,0x20,0x20,0x10,0x10,0x08,0x08,0x04,0x04,0x00,0x00,  // <
    0x00,0x00,0x00,0x00,0x7E,0x7E,0x00,0x00,0x7E,0x7E,0x00,0x00,0x00,0x00,0x00,0x00,  // =
    0x20,0x20,0x10,0x10,0x08,0x08,0x04,0x04,0x08,0x08,0x10,0x10,0x20,0x20,0x00,0x00,  // >
    0x20,0x20,0x40,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // ?

    0x3C,0x3C,0x42,0x42,0x4A,0x4A,0x56,0x56,0x4C,0x4C,0x40,0x40,0x3E,0x3E,0x00,0x00,  // @
    0x18,0x18,0x24,0x24,0x42,0x42,0x42,0x42,0x7E,0x7E,0x42,0x42,0x42,0x42,0x00,0x00,  // A
    0x7C,0x7C,0x42,0x42,0x42,0x42,0x7C,0x7C,0x42,0x42,0x42,0x42,0x7C,0x7C,0x00,0x00,  // B
    0x3C,0x3C,0x42,0x42,0x40,0x40,0x40,0x40,0x40,0x40,0x42,0x42,0x3C,0x3C,0x00,0x00,  // C
    0x7C,0x7C,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x7C,0x7C,0x00,0x00,  // D
    0x7E,0x7E,0x40,0x40,0x40,0x40,0x7C,0x7C,0x40,0x40,0x40,0x40,0x7E,0x7E,0x00,0x00,  // E
    0x7E,0x7E,0x40,0x40,0x40,0x40,0x7C,0x7C,0x40,0x40,0x40,0x40,0x40,0x40,0x00,0x00,  // F
    0x3E,0x3E,0x40,0x40,0x40,0x40,0x40,0x4E,0x4E,0x42,0x42,0x42,0x3E,0x3E,0x00,0x00,  // G
    0x42,0x42,0x42,0x42,0x42,0x42,0x7E,0x7E,0x42,0x42,0x42,0x42,0x42,0x42,0x00,0x00,  // H
    0x38,0x38,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x38,0x38,0x00,0x00,  // I
    0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x42,0x42,0x3C,0x3C,0x00,0x00,  // J
    0x42,0x42,0x44,0x44,0x48,0x48,0x70,0x70,0x48,0x48,0x44,0x44,0x42,0x42,0x00,0x00,  // K
    0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x7E,0x7E,0x00,0x00,  // L
    0x42,0x42,0x66,0x66,0x5A,0x5A,0x5A,0x5A,0x42,0x42,0x42,0x42,0x42,0x42,0x00,0x00,  // M
    0x42,0x42,0x62,0x62,0x52,0x52,0x5A,0x5A,0x4A,0x4A,0x46,0x46,0x42,0x42,0x00,0x00,  // N
    0x3C,0x3C,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x3C,0x3C,0x00,0x00,  // O

    0x7C,0x7C,0x42,0x42,0x42,0x42,0x7C,0x7C,0x40,0x40,0x40,0x40,0x40,0x40,0x00,0x00,  // P
    0x3C,0x3C,0x42,0x42,0x42,0x42,0x42,0x42,0x4A,0x4A,0x44,0x44,0x3A,0x3A,0x00,0x00,  // Q
    0x7C,0x7C,0x42,0x42,0x42,0x42,0x7C,0x7C,0x48,0x48,0x44,0x44,0x42,0x42,0x00,0x00,  // R
    0x3C,0x3C,0x42,0x42,0x40,0x40,0x3C,0x3C,0x02,0x02,0x42,0x42,0x3C,0x3C,0x00,0x00,  // S
    0x7C,0x7C,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x00,0x00,  // T
    0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x3C,0x3C,0x00,0x00,  // U
    0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x24,0x24,0x18,0x18,0x00,0x00,  // V
    0x42,0x42,0x42,0x42,0x42,0x42,0x5A,0x5A,0x5A,0x5A,0x66,0x66,0x42,0x42,0x00,0x00,  // W
    0x42,0x42,0x42,0x42,0x24,0x24,0x18,0x18,0x24,0x24,0x42,0x42,0x42,0x42,0x00,0x00,  // X
    0x44,0x44,0x44,0x44,0x28,0x28,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x00,0x00,  // Y
    0x7E,0x7E,0x02,0x02,0x04,0x04,0x18,0x18,0x20,0x20,0x40,0x40,0x7E,0x7E,0x00,0x00,  // Z
    0x7E,0x7E,0x60,0x60,0x60,0x60,0x60,0x60,0x60,0x60,0x60,0x60,0x7E,0x7E,0x00,0x00,  // [
    0x00,0x00,0x40,0x40,0x20,0x20,0x10,0x10,0x08,0x08,0x04,0x04,0x02,0x02,0x00,0x00,  // <backslash>
    0x7E,0x7E,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x7E,0x7E,0x00,0x00,  // ]
    0x00,0x00,0x00,0x00,0x10,0x10,0x28,0x28,0x44,0x44,0x00,0x00,0x00,0x00,0x00,0x00,  // ^
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x7E,0x00,0x00,  // _

    0x20,0x20,0x10,0x10,0x08,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // `
    0x00,0x00,0x00,0x00,0x3C,0x3C,0x02,0x02,0x3E,0x3E,0x42,0x42,0x3E,0x3E,0x00,0x00,  // a
    0x40,0x40,0x40,0x40,0x7C,0x7C,0x42,0x42,0x42,0x42,0x42,0x42,0x7C,0x7C,0x00,0x00,  // b
    0x00,0x00,0x00,0x00,0x3E,0x3E,0x40,0x40,0x40,0x40,0x40,0x40,0x3E,0x3E,0x00,0x00,  // c
    0x02,0x02,0x02,0x02,0x3E,0x3E,0x42,0x42,0x42,0x42,0x42,0x42,0x3E,0x3E,0x00,0x00,  // d
    0x00,0x00,0x00,0x00,0x3C,0x3C,0x42,0x42,0x7E,0x7E,0x40,0x40,0x3E,0x3E,0x00,0x00,  // e
    0x1C,0x1C,0x22,0x22,0x20,0x20,0x7C,0x7C,0x20,0x20,0x20,0x20,0x20,0x20,0x00,0x00,  // f
    0x00,0x00,0x00,0x00,0x3C,0x3C,0x42,0x42,0x42,0x42,0x3E,0x3E,0x02,0x02,0x3C,0x3C,  // g
    0x40,0x40,0x40,0x40,0x7C,0x7C,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x00,0x00,  // h
    0x10,0x10,0x00,0x00,0x30,0x30,0x10,0x10,0x10,0x10,0x10,0x10,0x38,0x38,0x00,0x00,  // i
    0x04,0x04,0x00,0x00,0x3C,0x3C,0x04,0x04,0x04,0x04,0x04,0x04,0x44,0x44,0x38,0x38,  // j
    0x40,0x40,0x40,0x40,0x42,0x42,0x44,0x44,0x78,0x78,0x44,0x44,0x42,0x42,0x00,0x00,  // k
    0x30,0x30,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x38,0x38,0x00,0x00,  // l
    0x00,0x00,0x00,0x00,0x66,0x66,0x5A,0x5A,0x5A,0x5A,0x5A,0x5A,0x42,0x42,0x00,0x00,  // m
    0x00,0x00,0x00,0x00,0x7C,0x7C,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x00,0x00,  // n
    0x00,0x00,0x00,0x00,0x3C,0x3C,0x42,0x42,0x42,0x42,0x42,0x42,0x3C,0x3C,0x00,0x00,  // o

    0x00,0x00,0x00,0x00,0x7C,0x7C,0x42,0x42,0x42,0x42,0x7C,0x7C,0x40,0x40,0x40,0x40,  // p
    0x00,0x00,0x00,0x00,0x3E,0x3E,0x42,0x42,0x42,0x42,0x3E,0x3E,0x02,0x02,0x02,0x02,  // q
    0x00,0x00,0x00,0x00,0x5E,0x5E,0x60,0x60,0x40,0x40,0x40,0x40,0x40,0x40,0x00,0x00,  // r
    0x00,0x00,0x00,0x00,0x3E,0x3E,0x40,0x40,0x3C,0x3C,0x02,0x02,0x7C,0x7C,0x00,0x00,  // s
    0x10,0x10,0x10,0x10,0x7C,0x7C,0x10,0x10,0x10,0x10,0x12,0x12,0x0C,0x0C,0x00,0x00,  // t
    0x00,0x00,0x00,0x00,0x42,0x42,0x42,0x42,0x42,0x42,0x46,0x46,0x3A,0x3A,0x00,0x00,  // u
    0x00,0x00,0x00,0x00,0x42,0x42,0x42,0x42,0x42,0x42,0x24,0x24,0x18,0x18,0x00,0x00,  // v
    0x00,0x00,0x00,0x00,0x42,0x42,0x42,0x42,0x5A,0x5A,0x5A,0x5A,0x66,0x66,0x00,0x00,  // w
    0x00,0x00,0x00,0x00,0x42,0x42,0x24,0x24,0x18,0x18,0x24,0x24,0x42,0x42,0x00,0x00,  // x
    0x00,0x00,0x00,0x00,0x42,0x42,0x42,0x42,0x42,0x42,0x3E,0x3E,0x02,0x02,0x3C,0x3C,  // y
    0x00,0x00,0x00,0x00,0x7E,0x7E,0x04,0x04,0x18,0x18,0x20,0x20,0x7E,0x7E,0x00,0x00,  // z
    0x0E,0x0E,0x18,0x18,0x18,0x18,0x70,0x70,0x18,0x18,0x18,0x18,0x0E,0x0E,0x00,0x00,  // {
    0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x00,0x00,  // |
    0x70,0x70,0x18,0x18,0x18,0x18,0x0E,0x0E,0x18,0x18,0x18,0x18,0x70,0x70,0x00,0x00,  // }
    0x00,0x00,0x00,0x00,0x24,0x24,0x54,0x54,0x48,0x48,0x00,0x00,0x00,0x00,0x00,0x00,  // ~
];

struct FrameBuffer {
    line_no: usize,
    fb_line: Option<usize>,
    frame: usize,
    buffer: [[u16; HORIZONTAL_WORDS]; VISIBLE_LINES],
}

struct Cursor<'a> {
    col: usize,
    row: usize,
    fb: &'a mut FrameBuffer,
}

/// Required by the DMA engine
// #[used]
// static mut DMA_CONTROL_TABLE: DmaInfo = DmaInfo { _data: [0u8; 1024] };

/// Mono framebuffer, arranged in lines.
static mut FRAMEBUFFER: FrameBuffer = FrameBuffer {
    line_no: 0,
    fb_line: None,
    frame: 0,
    buffer: [[0xFFFFu16; HORIZONTAL_WORDS]; VISIBLE_LINES],
};

fn enable(p: sysctl::Domain, sc: &mut tm4c123x_hal::sysctl::PowerControl) {
    sysctl::control_power(sc, p, sysctl::RunMode::Run, sysctl::PowerState::On);
    sysctl::control_power(sc, p, sysctl::RunMode::Sleep, sysctl::PowerState::On);
    sysctl::reset(sc, p);
}

impl FrameBuffer {
    /// Write a character to the text buffer.
    ///
    /// `col` and `row` are in character cell co-ordinates. `col` is
    /// `[0..TEXT_MAX_COL]` and `row` is `[0..TEXT_MAX_ROWS]`.
    ///
    /// `ch` is a unicode code-point. It will be mapped to a supported glyph,
    /// or to the special `Glyph::Unknown` glyph.
    ///
    /// If `flip` is `true`, text will be rendered black on green, otherwise
    /// the normal green on black.
    fn write_char(&mut self, ch: char, col: usize, row: usize, flip: bool) {
        if (col < TEXT_NUM_COLS) && (row < TEXT_NUM_ROWS) {
            let pixel_x = col * FONT_WIDTH;
            let pixel_y = row * FONT_HEIGHT;
            let word_x = pixel_x / 16;
            let glyph = Self::map_char(ch);
            for row in 0..FONT_HEIGHT {
                let mut font_byte = Self::font_lookup(glyph, row);
                if flip {
                    font_byte = !font_byte;
                }
                let mut bits = self.buffer[pixel_y + row][word_x];
                if (col & 1) == 1 {
                    // second u8 in u16
                    bits &= 0xFF00;
                    bits |= font_byte as u16;
                } else {
                    // first u8 in u16
                    bits &= 0x00FF;
                    bits |= (font_byte as u16) << 8;
                }
                self.buffer[pixel_y + row][word_x] = bits;
            }
        }
    }

    /// Write a string to the text buffer. Wraps off the end of the screen
    /// automatically.
    ///
    /// `x` and `y` are in character cell co-ordinates. `x` is
    /// `[0..TEXT_MAX_COL]` and `y` is `[0..TEXT_MAX_ROWS]`.
    ///
    /// If `flip` is `true`, text will be rendered black on green, otherwise
    /// the normal green on black.
    fn write_string(&mut self, message: &str, mut col: usize, mut row: usize, flip: bool) {
        for ch in message.chars() {
            self.write_char(ch, col, row, flip);
            col += 1;
            if col >= TEXT_NUM_COLS {
                row += 1;
                col = 0;
            }
            if row >= TEXT_NUM_ROWS {
                row = 0;
            }
        }
    }

    /// Convert a Unicode code-point to a glyph.
    fn map_char(ch: char) -> Glyph {
        match ch {
            ' ' => Glyph::Space,
            '!' => Glyph::ExclamationMark,
            '"' => Glyph::DoubleQuote,
            '#' => Glyph::Hash,
            '$' => Glyph::Dollar,
            '%' => Glyph::Percent,
            '&' => Glyph::Ampersand,
            '\'' => Glyph::SingleQuote,
            '(' => Glyph::OpenRoundBracket,
            ')' => Glyph::CloseRoundBracket,
            '*' => Glyph::Asterisk,
            '+' => Glyph::Plus,
            ',' => Glyph::Comma,
            '-' => Glyph::Minus,
            '.' => Glyph::Period,
            '/' => Glyph::Slash,
            '0' => Glyph::Digit0,
            '1' => Glyph::Digit1,
            '2' => Glyph::Digit2,
            '3' => Glyph::Digit3,
            '4' => Glyph::Digit4,
            '5' => Glyph::Digit5,
            '6' => Glyph::Digit6,
            '7' => Glyph::Digit7,
            '8' => Glyph::Digit8,
            '9' => Glyph::Digit9,
            ':' => Glyph::Colon,
            ';' => Glyph::SemiColon,
            '<' => Glyph::LessThan,
            '=' => Glyph::Equals,
            '>' => Glyph::GreaterThan,
            '?' => Glyph::QuestionMark,
            '@' => Glyph::At,
            'A' => Glyph::UppercaseA,
            'B' => Glyph::UppercaseB,
            'C' => Glyph::UppercaseC,
            'D' => Glyph::UppercaseD,
            'E' => Glyph::UppercaseE,
            'F' => Glyph::UppercaseF,
            'G' => Glyph::UppercaseG,
            'H' => Glyph::UppercaseH,
            'I' => Glyph::UppercaseI,
            'J' => Glyph::UppercaseJ,
            'K' => Glyph::UppercaseK,
            'L' => Glyph::UppercaseL,
            'M' => Glyph::UppercaseM,
            'N' => Glyph::UppercaseN,
            'O' => Glyph::UppercaseO,
            'P' => Glyph::UppercaseP,
            'Q' => Glyph::UppercaseQ,
            'R' => Glyph::UppercaseR,
            'S' => Glyph::UppercaseS,
            'T' => Glyph::UppercaseT,
            'U' => Glyph::UppercaseU,
            'V' => Glyph::UppercaseV,
            'W' => Glyph::UppercaseW,
            'X' => Glyph::UppercaseX,
            'Y' => Glyph::UppercaseY,
            'Z' => Glyph::UppercaseZ,
            '[' => Glyph::OpenSquareBracket,
            '\\' => Glyph::Backslash,
            ']' => Glyph::CloseSquareBracket,
            '^' => Glyph::Caret,
            '_' => Glyph::Underscore,
            '`' => Glyph::Backtick,
            'a' => Glyph::LowercaseA,
            'b' => Glyph::LowercaseB,
            'c' => Glyph::LowercaseC,
            'd' => Glyph::LowercaseD,
            'e' => Glyph::LowercaseE,
            'f' => Glyph::LowercaseF,
            'g' => Glyph::LowercaseG,
            'h' => Glyph::LowercaseH,
            'i' => Glyph::LowercaseI,
            'j' => Glyph::LowercaseJ,
            'k' => Glyph::LowercaseK,
            'l' => Glyph::LowercaseL,
            'm' => Glyph::LowercaseM,
            'n' => Glyph::LowercaseN,
            'o' => Glyph::LowercaseO,
            'p' => Glyph::LowercaseP,
            'q' => Glyph::LowercaseQ,
            'r' => Glyph::LowercaseR,
            's' => Glyph::LowercaseS,
            't' => Glyph::LowercaseT,
            'u' => Glyph::LowercaseU,
            'v' => Glyph::LowercaseV,
            'w' => Glyph::LowercaseW,
            'x' => Glyph::LowercaseX,
            'y' => Glyph::LowercaseY,
            'z' => Glyph::LowercaseZ,
            '{' => Glyph::OpenBrace,
            '|' => Glyph::VerticalBar,
            '}' => Glyph::CloseBrace,
            '~' => Glyph::Tilde,
            _ => Glyph::Unknown,
        }
    }

    /// Convert the glyph enum to the bits needed for a given line of that glyph.
    fn font_lookup(glyph: Glyph, row: usize) -> u8 {
        let index = ((glyph as usize) * FONT_HEIGHT) + row;
        FONT_DATA[index]
    }

    /// Plot a point on screen.
    fn draw_point(&mut self, pos: Point, set: bool) {
        if pos.0 < VISIBLE_COLS && pos.1 < VISIBLE_LINES {
            unsafe {
                self.point(pos.0, pos.1, set);
            }
        }
    }

    unsafe fn point(&mut self, x: usize, y: usize, set: bool) {
        let word_x = x / 16;
        let word_x_offset = 15 - (x % 16);
        if set {
            self.buffer[y][word_x] |= 1 << word_x_offset;
        } else {
            self.buffer[y][word_x] &= !(1 << word_x_offset);
        }
    }

    /// Draw a box using lines. The box is hollow, not filled.
    fn hollow_rectangle(&mut self, top_left: Point, bottom_right: Point, set: bool) {
        let top_right = Point(bottom_right.0, top_left.1);
        let bottom_left = Point(top_left.0, bottom_right.1);
        self.line(top_left, top_right, set);
        self.line(top_right, bottom_right, set);
        self.line(bottom_right, bottom_left, set);
        self.line(bottom_left, top_left, set);
    }

    /// Draw a line.
    fn line(&mut self, mut start: Point, mut end: Point, set: bool) {
        start.0 = start.0.min(MAX_X);
        start.1 = start.1.min(MAX_Y);
        end.0 = end.0.min(MAX_X);
        end.1 = end.1.min(MAX_Y);
        for (x, y) in bresenham::Bresenham::new(
            (start.0 as isize, start.1 as isize),
            (end.0 as isize, end.1 as isize),
        ) {
            unsafe { self.point(x as usize, y as usize, set) }
        }
    }

    fn clear(&mut self) {
        unsafe {
            core::ptr::write_bytes(self.buffer.as_mut_ptr(), 0x00, VISIBLE_LINES);
        }
    }
}

#[derive(Copy, Clone)]
struct Point(usize, usize);

fn main() {
    let p = tm4c123x_hal::Peripherals::take().unwrap();
    let cp = tm4c123x_hal::CorePeripherals::take().unwrap();

    let mut sc = p.SYSCTL.constrain();
    sc.clock_setup.oscillator = sysctl::Oscillator::Main(
        sysctl::CrystalFrequency::_16mhz,
        sysctl::SystemClock::UsePll(sysctl::PllOutputFrequency::_80_00mhz),
    );
    let clocks = sc.clock_setup.freeze();

    let mut nvic = cp.NVIC;
    nvic.enable(tm4c123x_hal::Interrupt::TIMER0A);
    nvic.enable(tm4c123x_hal::Interrupt::TIMER0B);

    enable(sysctl::Domain::Timer0, &mut sc.power_control);
    // enable(sysctl::Domain::MicroDma, &mut sc.power_control);
    enable(sysctl::Domain::Ssi2, &mut sc.power_control);

    let mut portb = p.GPIO_PORTB.split(&sc.power_control);
    let portc = p.GPIO_PORTC.split(&sc.power_control);
    // T0CCP0
    let _h_sync = portb.pb6.into_af7(&mut portb.control);
    // GPIO controlled V-Sync
    let _v_sync = portc.pc4.into_push_pull_output();
    // Ssi2Tx
    let _green_data = portb.pb7.into_af2(&mut portb.control);

    // Need to configure SSI2 at 20 MHz
    p.SSI2.cr1.modify(|_, w| w.sse().clear_bit());
    // SSIClk = SysClk / (CPSDVSR * (1 + SCR))
    // 20 MHz = 80 MHz / (4 * (1 + 0))
    // SCR = 0
    // CPSDVSR = 4
    p.SSI2.cpsr.write(|w| unsafe { w.cpsdvsr().bits(4) });
    // Send 16 bits at a time in Freescale format
    p.SSI2.cr0.write(|w| {
        w.dss()._16();
        w.frf().moto();
        w.spo().clear_bit();
        w.sph().set_bit();
        w
    });
    // Enable TX DMA
    // p.SSI2.dmactl.write(|w| w.txdmae().set_bit());
    // Set clock source to sysclk
    p.SSI2.cc.modify(|_, w| w.cs().syspll());
    // Enable SSI2
    p.SSI2.cr1.modify(|_, w| w.sse().set_bit());

    // Need to configure MicroDMA to feed SSI2 with data
    // That's Encoder 2, Channel 13
    // let dma = p.UDMA;
    // dma.cfg.write(|w| w.masten().set_bit());
    // dma.ctlbase
    //     .write(|w| unsafe { w.addr().bits(&mut DMA_CONTROL_TABLE as *mut DmaInfo as u32) });

    let h_timer = p.TIMER0;
    // Configure Timer0A for h-sync and Timer0B for line trigger
    h_timer.ctl.modify(|_, w| {
        w.taen().clear_bit();
        w.tben().clear_bit();
        w
    });
    h_timer.cfg.modify(|_, w| w.cfg()._16_bit());
    h_timer.tamr.modify(|_, w| {
        w.taams().set_bit();
        w.tacmr().clear_bit();
        w.tapwmie().set_bit();
        w.tamr().period();
        w
    });
    h_timer.tbmr.modify(|_, w| {
        w.tbams().set_bit();
        w.tbcmr().clear_bit();
        w.tbmr().period();
        w.tbpwmie().set_bit();
        w
    });
    h_timer.ctl.modify(|_, w| w.tapwml().clear_bit());
    h_timer.ctl.modify(|_, w| {
        // Trigger Timer A capture on rising edge (i.e. line start)
        w.tapwml().clear_bit();
        // Trigger Timer B capture on falling edge (i.e. data start)
        w.tbpwml().set_bit();
        w
    });
    // We're counting down in PWM mode, so start at the end
    h_timer
        .tailr
        .modify(|_, w| unsafe { w.bits(H_WHOLE_LINE - 1) });
    h_timer
        .tbilr
        .modify(|_, w| unsafe { w.bits(H_WHOLE_LINE - 1) });
    h_timer
        .tamatchr
        .modify(|_, w| unsafe { w.bits(H_SYNC_END - 1) });
    h_timer
        .tbmatchr
        .modify(|_, w| unsafe { w.bits(H_LINE_START - 1) });
    h_timer.imr.modify(|_, w| {
        w.caeim().set_bit(); // Timer0A fires at start of line
        w.cbeim().set_bit(); // Timer0B fires at start of data
        w
    });

    // Clear interrupts
    h_timer.icr.write(|w| {
        w.tbmcint().set_bit();
        w.tbtocint().set_bit();
        w
    });

    h_timer.ctl.modify(|_, w| {
        w.taen().set_bit();
        w.tben().set_bit();
        w
    });

    let mut d = Delay::new(cp.SYST, &clocks);
    let mut i = 0;

    // Give the monitor time to auto-sync with a full green screen
    d.delay_ms(4000u32);
    let mut c = Cursor::new(unsafe { &mut FRAMEBUFFER });

    // Clear the framebuffer
    unsafe {
        FRAMEBUFFER.clear();
        for row in 0..TEXT_NUM_ROWS {
            FRAMEBUFFER.write_char('$', 0, row, true);
            FRAMEBUFFER.write_char('$', TEXT_MAX_COL, row, true);
        }
        FRAMEBUFFER.line(Point(0, 0), Point(MAX_X, MAX_Y), true);
        FRAMEBUFFER.hollow_rectangle(Point(50, 50), Point(350, 250), true);
    }

    writeln!(c, "Hello, Twitters.\nNow we have a font renderer too!").unwrap();
    writeln!(c, "\n-- @therealjpster").unwrap();
    writeln!(c, "Chip ID: {:?}", chip_id::get()).unwrap();
    writeln!(c, "X\tY").unwrap();
    writeln!(c, "ABC\t123").unwrap();
    writeln!(c, "DEF\t£4.0").unwrap();
    let mut old = 0;
    loop {
        asm::wfi();
        let new = unsafe { FRAMEBUFFER.frame };
        if new != old {
            old = new;
            write!(c, "\rLoop {}", i).unwrap();
            i = i + 1;
        }
    }
}

impl<'a> Cursor<'a> {
    fn new(fb: &'a mut FrameBuffer) -> Cursor<'a> {
        Cursor { col: 0, row: 0, fb }
    }
}

impl<'a> core::fmt::Write for Cursor<'a> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for ch in s.chars() {
            match ch {
                '\n' => {
                    self.col = 0;
                    self.row += 1;
                }
                '\r' => {
                    self.col = 0;
                }
                '\t' => {
                    let tabs = self.col / 9;
                    self.col = (tabs + 1) * 9;
                }
                ch => {
                    self.fb.write_char(ch, self.col, self.row, false);
                    self.col += 1;
                }
            }
            if self.col == TEXT_NUM_COLS {
                self.col = 0;
                self.row += 1;
            }
            if self.row == TEXT_NUM_ROWS {
                // Should really scroll screen here...
                self.row = TEXT_NUM_ROWS - 1;
                for line in 0..VISIBLE_LINES-FONT_HEIGHT {
                    self.fb.buffer[line] = self.fb.buffer[line + FONT_HEIGHT];
                }
                for line in VISIBLE_LINES-FONT_HEIGHT..VISIBLE_LINES {
                    self.fb.buffer[line] = [0u16; HORIZONTAL_WORDS];
                }
            }
        }
        Ok(())
    }
}

fn start_of_line(fb_info: &mut FrameBuffer) {
    let gpio = unsafe { &*tm4c123x_hal::tm4c123x::GPIO_PORTC::ptr() };

    fb_info.line_no += 1;

    if fb_info.line_no == V_WHOLE_FRAME {
        fb_info.line_no = 0;
        unsafe { bb::change_bit(&gpio.data, 4, true) };
    }

    if fb_info.line_no == V_SYNC_PULSE {
        unsafe { bb::change_bit(&gpio.data, 4, false) };
    }

    if (fb_info.line_no >= V_SYNC_PULSE + V_BACK_PORCH)
        && (fb_info.line_no < V_SYNC_PULSE + V_BACK_PORCH + V_VISIBLE_AREA)
    {
        // Visible lines
        // 600 visible lines, 300 output lines each shown twice
        fb_info.fb_line = Some((fb_info.line_no - (V_SYNC_PULSE + V_BACK_PORCH)) >> 1);
    } else if fb_info.line_no == V_SYNC_PULSE + V_BACK_PORCH + V_VISIBLE_AREA {
        fb_info.frame = fb_info.frame.wrapping_add(1);
    } else {
        // Front porch
        fb_info.fb_line = None;
    }
}

fn start_of_data(fb_info: &FrameBuffer) {
    let ssi = unsafe { &*tm4c123x_hal::tm4c123x::SSI2::ptr() };
    if let Some(line) = fb_info.fb_line {
        for word in fb_info.buffer[line].iter() {
            ssi.dr.write(|w| unsafe { w.data().bits(*word) });
            while ssi.sr.read().tnf().bit_is_clear() {
                asm::nop();
            }
        }
    }
}

extern "C" fn timer0a_isr() {
    let timer = unsafe { &*tm4c123x_hal::tm4c123x::TIMER0::ptr() };
    // let cs = unsafe { CriticalSection::new() };
    let mut fb_info = unsafe { &mut FRAMEBUFFER };
    start_of_line(&mut fb_info);
    timer.icr.write(|w| w.caecint().set_bit());
}

extern "C" fn timer0b_isr() {
    let timer = unsafe { &*tm4c123x_hal::tm4c123x::TIMER0::ptr() };
    // let cs = unsafe { CriticalSection::new() };
    let fb_info = unsafe { &mut FRAMEBUFFER };
    start_of_data(&fb_info);
    timer.icr.write(|w| w.cbecint().set_bit());
}

extern "C" fn default_handler() {
    asm::bkpt();
}

#[link_section = ".vector_table.interrupts"]
#[used]
static INTERRUPTS: [Option<extern "C" fn()>; 139] = [
    // GPIO Port A                      16
    Some(default_handler),
    // GPIO Port B                      17
    Some(default_handler),
    // GPIO Port C                      18
    Some(default_handler),
    // GPIO Port D                      19
    Some(default_handler),
    // GPIO Port E                      20
    Some(default_handler),
    // UART 0                           21
    Some(default_handler),
    // UART 1                           22
    Some(default_handler),
    // SSI 0                            23
    Some(default_handler),
    // I2C 0                            24
    Some(default_handler),
    // Reserved                         25
    None,
    // Reserved                         26
    None,
    // Reserved                         27
    None,
    // Reserved                         28
    None,
    // Reserved                         29
    None,
    // ADC 0 Seq 0                      30
    Some(default_handler),
    // ADC 0 Seq 1                      31
    Some(default_handler),
    // ADC 0 Seq 2                      32
    Some(default_handler),
    // ADC 0 Seq 3                      33
    Some(default_handler),
    // WDT 0 and 1                      34
    Some(default_handler),
    // 16/32 bit timer 0 A              35
    Some(timer0a_isr),
    // 16/32 bit timer 0 B              36
    Some(timer0b_isr),
    // 16/32 bit timer 1 A              37
    Some(default_handler),
    // 16/32 bit timer 1 B              38
    Some(default_handler),
    // 16/32 bit timer 2 A              39
    Some(default_handler),
    // 16/32 bit timer 2 B              40
    Some(default_handler),
    // Analog comparator 0              41
    Some(default_handler),
    // Analog comparator 1              42
    Some(default_handler),
    // Reserved                         43
    None,
    // System control                   44
    Some(default_handler),
    // Flash + EEPROM control           45
    Some(default_handler),
    // GPIO Port F                      46
    Some(default_handler),
    // Reserved                         47
    None,
    // Reserved                         48
    None,
    // UART 2                           49
    Some(default_handler),
    // SSI 1                            50
    Some(default_handler),
    // 16/32 bit timer 3 A              51
    Some(default_handler),
    // 16/32 bit timer 3 B              52
    Some(default_handler),
    // I2C 1                            53
    Some(default_handler),
    // Reserved                         54
    None,
    // CAN 0                            55
    Some(default_handler),
    // Reserved                         56
    None,
    // Reserved                         57
    None,
    // Reserved                         58
    None,
    // Hibernation module               59
    Some(default_handler),
    // USB                              60
    Some(default_handler),
    // Reserved                         61
    None,
    // UDMA SW                          62
    Some(default_handler),
    // UDMA Error                       63
    Some(default_handler),
    // ADC 1 Seq 0                      64
    Some(default_handler),
    // ADC 1 Seq 1                      65
    Some(default_handler),
    // ADC 1 Seq 2                      66
    Some(default_handler),
    // ADC 1 Seq 3                      67
    Some(default_handler),
    // Reserved                         68
    None,
    // Reserved                         69
    None,
    // Reserved                         70
    None,
    // Reserved                         71
    None,
    // Reserved                         72
    None,
    // SSI 2                            73
    Some(default_handler),
    // SSI 2                            74
    Some(default_handler),
    // UART 3                           75
    Some(default_handler),
    // UART 4                           76
    Some(default_handler),
    // UART 5                           77
    Some(default_handler),
    // UART 6                           78
    Some(default_handler),
    // UART 7                           79
    Some(default_handler),
    // Reserved                         80
    None,
    // Reserved                         81
    None,
    // Reserved                         82
    None,
    // Reserved                         83
    None,
    // I2C 2                            84
    Some(default_handler),
    // I2C 4                            85
    Some(default_handler),
    // 16/32 bit timer 4 A              86
    Some(default_handler),
    // 16/32 bit timer 4 B              87
    Some(default_handler),
    // Reserved                         88
    None,
    // Reserved                         89
    None,
    // Reserved                         90
    None,
    // Reserved                         91
    None,
    // Reserved                         92
    None,
    // Reserved                         93
    None,
    // Reserved                         94
    None,
    // Reserved                         95
    None,
    // Reserved                         96
    None,
    // Reserved                         97
    None,
    // Reserved                         98
    None,
    // Reserved                         99
    None,
    // Reserved                         100
    None,
    // Reserved                         101
    None,
    // Reserved                         102
    None,
    // Reserved                         103
    None,
    // Reserved                         104
    None,
    // Reserved                         105
    None,
    // Reserved                         106
    None,
    // Reserved                         107
    None,
    // 16/32 bit timer 5 A              108
    Some(default_handler),
    // 16/32 bit timer 5 B              109
    Some(default_handler),
    // 32/64 bit timer 0 A              110
    Some(default_handler),
    // 32/64 bit timer 0 B              111
    Some(default_handler),
    // 32/64 bit timer 1 A              112
    Some(default_handler),
    // 32/64 bit timer 1 B              113
    Some(default_handler),
    // 32/64 bit timer 2 A              114
    Some(default_handler),
    // 32/64 bit timer 2 B              115
    Some(default_handler),
    // 32/64 bit timer 3 A              116
    Some(default_handler),
    // 32/64 bit timer 3 B              117
    Some(default_handler),
    // 32/64 bit timer 4 A              118
    Some(default_handler),
    // 32/64 bit timer 4 B              119
    Some(default_handler),
    // 32/64 bit timer 5 A              120
    Some(default_handler),
    // 32/64 bit timer 5 B              121
    Some(default_handler),
    // System Exception                 122
    Some(default_handler),
    // Reserved                         123
    None,
    // Reserved                         124
    None,
    // Reserved                         125
    None,
    // Reserved                         126
    None,
    // Reserved                         127
    None,
    // Reserved                         128
    None,
    // Reserved                         129
    None,
    // Reserved                         130
    None,
    // Reserved                         131
    None,
    // Reserved                         132
    None,
    // Reserved                         133
    None,
    // Reserved                         134
    None,
    // Reserved                         135
    None,
    // Reserved                         136
    None,
    // Reserved                         137
    None,
    // Reserved                         138
    None,
    // Reserved                         139
    None,
    // Reserved                         140
    None,
    // Reserved                         141
    None,
    // Reserved                         142
    None,
    // Reserved                         143
    None,
    // Reserved                         144
    None,
    // Reserved                         145
    None,
    // Reserved                         146
    None,
    // Reserved                         147
    None,
    // Reserved                         148
    None,
    // Reserved                         149
    None,
    // Reserved                         150
    None,
    // Reserved                         151
    None,
    // Reserved                         152
    None,
    // Reserved                         153
    None,
    // Reserved                         154
    None,
];
