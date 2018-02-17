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

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate cortex_m_semihosting;
extern crate embedded_hal;
extern crate tm4c123x_hal;

use cortex_m::asm;
use core::cell::RefCell;
use cortex_m::interrupt::{CriticalSection, Mutex};
use tm4c123x_hal::gpio::GpioExt;
use tm4c123x_hal::sysctl::{self, SysctlExt};
use tm4c123x_hal::bb;

const H_VISIBLE_AREA: u32 = 800 * 2;
const H_FRONT_PORCH: u32 = 40 * 2;
const H_SYNC_PULSE: u32 = 128 * 2;
const H_BACK_PORCH: u32 = 88 * 2;
const H_WHOLE_LINE: u32 = H_VISIBLE_AREA + H_FRONT_PORCH + H_SYNC_PULSE + H_BACK_PORCH;
const H_SYNC_END: u32 = H_WHOLE_LINE - H_SYNC_PULSE;
const H_LINE_START: u32 = H_WHOLE_LINE - (H_SYNC_PULSE + H_BACK_PORCH);
const V_VISIBLE_AREA: usize = 600;
const V_FRONT_PORCH: usize = 1;
const V_SYNC_PULSE: usize = 4;
const V_BACK_PORCH: usize = 23;
const V_WHOLE_FRAME: usize = V_SYNC_PULSE + V_BACK_PORCH + V_VISIBLE_AREA + V_FRONT_PORCH;

/// Text characters are this many pixels across
const FONT_WIDTH: usize = 8;
/// Text characters are this many pixels high
const FONT_HEIGHT: usize = 16;
/// Number of lines in frame buffer
const VISIBLE_LINES: usize = 300;
/// Number of lines the text is rendered to.
const VISIBLE_LINES_WITH_TEXT: usize = TEXT_MAX_ROWS * FONT_HEIGHT;
/// Number of columns in frame buffer
const VISIBLE_COLS: usize = 400;
/// How many 16-bit words in a line
const HORIZONTAL_WORDS: usize = (VISIBLE_COLS + 15) / 16;
/// How many characters in a line
const TEXT_MAX_COLS: usize = VISIBLE_COLS / FONT_WIDTH;
/// Highest X co-ord for text
const MAX_X: usize = TEXT_MAX_COLS - 1;
/// How many lines of characters on the screen
const TEXT_MAX_ROWS: usize = VISIBLE_LINES / FONT_HEIGHT;
/// Highest Y co-ord for text
const MAX_Y: usize = TEXT_MAX_ROWS - 1;

#[repr(align(1024))]
struct DmaInfo {
    _data: [u8; 1024],
}

/// This is our own custom character set.
#[repr(u8)]
#[derive(Copy, Clone)]
#[allow(dead_code)]
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

struct FrameBuffer {
    line_no: usize,
    fb_line: Option<usize>,
    /// The first index is the row (y), then the column (x) (`text[y][x]`)
    text: [[Glyph; TEXT_MAX_COLS]; TEXT_MAX_ROWS],
    render_line: [u16; HORIZONTAL_WORDS],
}

/// Required by the DMA engine
#[used]
static mut DMA_CONTROL_TABLE: DmaInfo = DmaInfo { _data: [0u8; 1024] };
/// Mono framebuffer, arranged in lines.

static FB_INFO: Mutex<RefCell<FrameBuffer>> = Mutex::new(RefCell::new(FrameBuffer {
    line_no: 0,
    fb_line: None,
    text: [[Glyph::Space; TEXT_MAX_COLS]; TEXT_MAX_ROWS],
    render_line: [0u16; HORIZONTAL_WORDS],
}));

fn enable(p: sysctl::Domain, sc: &mut tm4c123x_hal::sysctl::PowerControl) {
    sysctl::control_power(sc, p, sysctl::RunMode::Run, sysctl::PowerState::On);
    sysctl::control_power(sc, p, sysctl::RunMode::Sleep, sysctl::PowerState::On);
    sysctl::reset(sc, p);
}

impl FrameBuffer {
    /// Write a character to the text buffer.
    fn write_char(&mut self, ch: char, x: usize, y: usize) {
        if (x < TEXT_MAX_COLS) && (y < TEXT_MAX_ROWS) {
            self.text[y][x] = FrameBuffer::map_char(ch);
        }
    }

    /// Write a string to the text buffer. Wraps off the end of the screen
    /// automatically.
    fn write_string(&mut self, message: &str, mut x: usize, mut y: usize) {
        for ch in message.chars() {
            self.write_char(ch, x, y);
            x += 1;
            if x >= TEXT_MAX_COLS {
                y += 1;
                x = 0;
            }
            if y >= TEXT_MAX_ROWS {
                y = 0;
            }
        }
    }

    /// Convert a Unicode code-point to a glyph.
    fn map_char(ch: char) -> Glyph {
        match ch {
            'A' => Glyph::UppercaseA,
            'B' => Glyph::UppercaseB,
            'C' => Glyph::UppercaseC,
            'D' => Glyph::UppercaseD,
            'R' => Glyph::UppercaseR,
            ' ' => Glyph::Space,
            _ => Glyph::Unknown,
        }
    }

    /// Calculate a line of pixels.
    fn render(&mut self) {
        if let Some(line) = self.fb_line {
            if line < VISIBLE_LINES_WITH_TEXT {
                let text_row = line / FONT_HEIGHT;
                let font_row = line % FONT_HEIGHT;
                for (word_col, word_ref) in self.render_line.iter_mut().enumerate() {
                    let text_col = word_col * 2;
                    let left = Self::font_lookup(self.text[text_row][text_col], font_row) as u16;
                    let right = Self::font_lookup(self.text[text_row][text_col + 1], font_row) as u16;
                    *word_ref = (left << 8) | right;
                }
            } else {
                for word_ref in self.render_line.iter_mut() {
                    *word_ref = 0;
                }
            }
        }
    }

    // /// Convert the glyph enum to the bits needed for a given line of that glyph.
    // fn font_lookup(glyph: Glyph, row: usize) -> u8 {
    //     match glyph {
    //         Glyph::ExclamationMark => match row {
    //             0 => 0x10,
    //             1 => 0x10,
    //             2 => 0x10,
    //             3 => 0x10,
    //             4 => 0x10,
    //             5 => 0x10,
    //             6 => 0x10,
    //             7 => 0x10,
    //             8 => 0x10,
    //             9 => 0x10,
    //             12 => 0x10,
    //             13 => 0x10,
    //             _ => 0x00,
    //         },
    //         Glyph::DoubleQuote => match row {
    //             0 => 0x24,
    //             1 => 0x24,
    //             2 => 0x24,
    //             3 => 0x24,
    //             4 => 0x24,
    //             5 => 0x24,
    //             _ => 0x00,
    //         },
    //         Glyph::Hash => match row {
    //             0 => 0x24,
    //             1 => 0x24,
    //             2 => 0x24,
    //             3 => 0x24,
    //             4 => 0x7E,
    //             5 => 0x7E,
    //             6 => 0x24,
    //             7 => 0x24,
    //             8 => 0x7E,
    //             9 => 0x7E,
    //             10 => 0x24,
    //             11 => 0x24,
    //             12 => 0x24,
    //             13 => 0x24,
    //             _ => 0x00,
    //         },
    //         Glyph::Dollar => match row {
    //             0 => 0x10,
    //             1 => 0x10,
    //             2 => 0x3C,
    //             3 => 0x3C,
    //             4 => 0x50,
    //             5 => 0x50,
    //             6 => 0x38,
    //             7 => 0x38,
    //             8 => 0x14,
    //             9 => 0x14,
    //             10 => 0x78,
    //             11 => 0x78,
    //             12 => 0x10,
    //             13 => 0x10,
    //             _ => 0x00,
    //         },
    //         Glyph::Percent => match row {
    //             2 => 0x62,
    //             3 => 0x62,
    //             4 => 0x64,
    //             5 => 0x64,
    //             6 => 0x08,
    //             7 => 0x08,
    //             8 => 0x10,
    //             9 => 0x10,
    //             10 => 0x26,
    //             11 => 0x26,
    //             12 => 0x46,
    //             13 => 0x46,
    //             _ => 0x00,
    //         },
    //         Glyph::Ampersand => match row {
    //             0 => 0x30,
    //             1 => 0x30,
    //             2 => 0x48,
    //             3 => 0x48,
    //             4 => 0x48,
    //             5 => 0x48,
    //             6 => 0x30,
    //             7 => 0x30,
    //             8 => 0x4A,
    //             9 => 0x4A,
    //             10 => 0x44,
    //             11 => 0x44,
    //             12 => 0x3A,
    //             13 => 0x3A,
    //             _ => 0x00,
    //         },
    //         Glyph::SingleQuote => match row {
    //             0 => 0x10,
    //             1 => 0x10,
    //             2 => 0x10,
    //             3 => 0x10,
    //             4 => 0x10,
    //             5 => 0x10,
    //             _ => 0x00,
    //         },
    //         Glyph::OpenRoundBracket => match row {
    //             0 => 0x10,
    //             1 => 0x10,
    //             2 => 0x20,
    //             3 => 0x20,
    //             4 => 0x40,
    //             5 => 0x40,
    //             6 => 0x40,
    //             7 => 0x40,
    //             8 => 0x40,
    //             9 => 0x40,
    //             10 => 0x20,
    //             11 => 0x20,
    //             12 => 0x10,
    //             13 => 0x10,
    //             _ => 0x00,
    //         },
    //         Glyph::CloseRoundBracket => match row {
    //             0 => 0x10,
    //             1 => 0x10,
    //             2 => 0x08,
    //             3 => 0x08,
    //             4 => 0x04,
    //             5 => 0x04,
    //             6 => 0x04,
    //             7 => 0x04,
    //             8 => 0x04,
    //             9 => 0x04,
    //             10 => 0x08,
    //             11 => 0x08,
    //             12 => 0x10,
    //             13 => 0x10,
    //             _ => 0x00,
    //         },
    //         Glyph::Asterisk => match row {
    //             0 => 0x10,
    //             1 => 0x10,
    //             2 => 0x54,
    //             3 => 0x54,
    //             4 => 0x38,
    //             5 => 0x38,
    //             6 => 0x10,
    //             7 => 0x10,
    //             8 => 0x38,
    //             9 => 0x38,
    //             10 => 0x54,
    //             11 => 0x54,
    //             12 => 0x10,
    //             13 => 0x10,
    //             _ => 0x00,
    //         },
    //         Glyph::Plus => match row {
    //             2 => 0x10,
    //             3 => 0x10,
    //             4 => 0x10,
    //             5 => 0x10,
    //             6 => 0x7C,
    //             7 => 0x7C,
    //             8 => 0x10,
    //             9 => 0x10,
    //             10 => 0x10,
    //             11 => 0x10,
    //             _ => 0x00,
    //         },
    //         Glyph::Comma => match row {
    //             8 => 0x08,
    //             9 => 0x08,
    //             10 => 0x08,
    //             11 => 0x08,
    //             12 => 0x10,
    //             13 => 0x10,
    //             _ => 0x00,
    //         },
    //         Glyph::Minus => match row {
    //             6 => 0x7E,
    //             7 => 0x7E,
    //             _ => 0x00,
    //         },
    //         Glyph::Period => match row {
    //             12 => 0x10,
    //             13 => 0x10,
    //             _ => 0x00,
    //         },
    //         Glyph::Slash => match row {
    //             2 => 0x02,
    //             3 => 0x02,
    //             4 => 0x04,
    //             5 => 0x04,
    //             6 => 0x08,
    //             7 => 0x08,
    //             8 => 0x10,
    //             9 => 0x10,
    //             10 => 0x20,
    //             11 => 0x20,
    //             12 => 0x40,
    //             13 => 0x40,
    //             _ => 0x00,
    //         },
    //         Glyph::Digit0 => match row {
    //             0 => 0x3C,
    //             1 => 0x3C,
    //             2 => 0x42,
    //             3 => 0x42,
    //             4 => 0x46,
    //             5 => 0x46,
    //             6 => 0x5A,
    //             7 => 0x5A,
    //             8 => 0x62,
    //             9 => 0x62,
    //             10 => 0x42,
    //             11 => 0x42,
    //             12 => 0x3C,
    //             13 => 0x3C,
    //             _ => 0x00,
    //         },
    //         Glyph::Digit1 => match row {
    //             0 => 0x08,
    //             1 => 0x08,
    //             2 => 0x18,
    //             3 => 0x18,
    //             4 => 0x08,
    //             5 => 0x08,
    //             6 => 0x08,
    //             7 => 0x08,
    //             8 => 0x08,
    //             9 => 0x08,
    //             10 => 0x08,
    //             11 => 0x08,
    //             12 => 0x1C,
    //             13 => 0x1C,
    //             _ => 0x00,
    //         },
    //         Glyph::Digit2 => match row {
    //             0 => 0x3C,
    //             1 => 0x3C,
    //             2 => 0x42,
    //             3 => 0x42,
    //             4 => 0x02,
    //             5 => 0x02,
    //             6 => 0x1C,
    //             7 => 0x1C,
    //             8 => 0x20,
    //             9 => 0x20,
    //             10 => 0x40,
    //             11 => 0x40,
    //             12 => 0x7E,
    //             13 => 0x7E,
    //             _ => 0x00,
    //         },
    //         Glyph::Digit3 => match row {
    //             0 => 0x7E,
    //             1 => 0x7E,
    //             2 => 0x02,
    //             3 => 0x02,
    //             4 => 0x04,
    //             5 => 0x04,
    //             6 => 0x1C,
    //             7 => 0x1C,
    //             8 => 0x02,
    //             9 => 0x02,
    //             10 => 0x42,
    //             11 => 0x42,
    //             12 => 0x3C,
    //             13 => 0x3C,
    //             _ => 0x00,
    //         },
    //         Glyph::Digit4 => match row {
    //             0 => 0x04,
    //             1 => 0x04,
    //             2 => 0x0C,
    //             3 => 0x0C,
    //             4 => 0x14,
    //             5 => 0x14,
    //             6 => 0x24,
    //             7 => 0x24,
    //             8 => 0x7E,
    //             9 => 0x7E,
    //             10 => 0x04,
    //             11 => 0x04,
    //             12 => 0x04,
    //             13 => 0x04,
    //             _ => 0x00,
    //         },
    //         Glyph::Digit5 => match row {
    //             0 => 0x7E,
    //             1 => 0x7E,
    //             2 => 0x40,
    //             3 => 0x40,
    //             4 => 0x7C,
    //             5 => 0x7C,
    //             6 => 0x02,
    //             7 => 0x02,
    //             8 => 0x02,
    //             9 => 0x02,
    //             10 => 0x42,
    //             11 => 0x42,
    //             12 => 0x3C,
    //             13 => 0x3C,
    //             _ => 0x00,
    //         },
    //         Glyph::Digit6 => match row {
    //             0 => 0x1E,
    //             1 => 0x1E,
    //             2 => 0x20,
    //             3 => 0x20,
    //             4 => 0x40,
    //             5 => 0x40,
    //             6 => 0x7C,
    //             7 => 0x7C,
    //             8 => 0x42,
    //             9 => 0x42,
    //             10 => 0x42,
    //             11 => 0x42,
    //             12 => 0x3C,
    //             13 => 0x3C,
    //             _ => 0x00,
    //         },
    //         Glyph::Digit7 => match row {
    //             0 => 0x7E,
    //             1 => 0x7E,
    //             2 => 0x02,
    //             3 => 0x02,
    //             4 => 0x04,
    //             5 => 0x04,
    //             6 => 0x08,
    //             7 => 0x08,
    //             8 => 0x10,
    //             9 => 0x10,
    //             10 => 0x10,
    //             11 => 0x10,
    //             12 => 0x10,
    //             13 => 0x10,
    //             _ => 0x00,
    //         },
    //         Glyph::Digit8 => match row {
    //             0 => 0x3C,
    //             1 => 0x3C,
    //             2 => 0x42,
    //             3 => 0x42,
    //             4 => 0x42,
    //             5 => 0x42,
    //             6 => 0x3C,
    //             7 => 0x3C,
    //             8 => 0x42,
    //             9 => 0x42,
    //             10 => 0x42,
    //             11 => 0x42,
    //             12 => 0x3C,
    //             13 => 0x3C,
    //             _ => 0x00,
    //         },
    //         Glyph::Digit9 => match row {
    //             0 => 0x3C,
    //             1 => 0x3C,
    //             2 => 0x42,
    //             3 => 0x42,
    //             4 => 0x42,
    //             5 => 0x42,
    //             6 => 0x3E,
    //             7 => 0x3E,
    //             8 => 0x02,
    //             9 => 0x02,
    //             10 => 0x04,
    //             11 => 0x04,
    //             12 => 0x78,
    //             13 => 0x78,
    //             _ => 0x00,
    //         },
    //         Glyph::Colon => match row {
    //             4 => 0x10,
    //             5 => 0x10,
    //             8 => 0x10,
    //             9 => 0x10,
    //             _ => 0x00,
    //         },
    //         Glyph::SemiColon => match row {
    //             4 => 0x08,
    //             5 => 0x08,
    //             8 => 0x08,
    //             9 => 0x08,
    //             10 => 0x08,
    //             11 => 0x08,
    //             12 => 0x10,
    //             _ => 0x00,
    //         },
    //         Glyph::LessThan => match row {
    //             0 => 0x04,
    //             1 => 0x04,
    //             2 => 0x08,
    //             3 => 0x08,
    //             4 => 0x10,
    //             5 => 0x10,
    //             6 => 0x20,
    //             7 => 0x20,
    //             8 => 0x10,
    //             9 => 0x10,
    //             10 => 0x08,
    //             11 => 0x08,
    //             12 => 0x04,
    //             13 => 0x04,
    //             _ => 0x00,
    //         },
    //         Glyph::Equals => match row {
    //             4 => 0x7E,
    //             5 => 0x7E,
    //             8 => 0x7E,
    //             9 => 0x7E,
    //             _ => 0x00,
    //         },
    //         Glyph::GreaterThan => match row {
    //             0 => 0x20,
    //             1 => 0x20,
    //             2 => 0x10,
    //             3 => 0x10,
    //             4 => 0x08,
    //             5 => 0x08,
    //             6 => 0x04,
    //             7 => 0x04,
    //             8 => 0x08,
    //             9 => 0x08,
    //             10 => 0x10,
    //             11 => 0x10,
    //             12 => 0x20,
    //             13 => 0x20,
    //             _ => 0x00,
    //         },
    //         Glyph::QuestionMark => match row {
    //             0 => 0x20,
    //             1 => 0x20,
    //             2 => 0x40,
    //             3 => 0x40,
    //             _ => 0x00,
    //         },
    //         Glyph::At => match row {
    //             0 => 0x3C,
    //             1 => 0x3C,
    //             2 => 0x42,
    //             3 => 0x42,
    //             4 => 0x4A,
    //             5 => 0x4A,
    //             6 => 0x56,
    //             7 => 0x56,
    //             8 => 0x4C,
    //             9 => 0x4C,
    //             10 => 0x40,
    //             11 => 0x40,
    //             12 => 0x3E,
    //             13 => 0x3E,
    //             _ => 0x00,
    //         },
    //         Glyph::UppercaseA => match row {
    //             0 => 0x18,
    //             1 => 0x18,
    //             2 => 0x24,
    //             3 => 0x24,
    //             4 => 0x42,
    //             5 => 0x42,
    //             6 => 0x42,
    //             7 => 0x42,
    //             8 => 0x7E,
    //             9 => 0x7E,
    //             10 => 0x42,
    //             11 => 0x42,
    //             12 => 0x42,
    //             13 => 0x42,
    //             _ => 0x00,
    //         },
    //         Glyph::UppercaseB => match row {
    //             0 => 0x7C,
    //             1 => 0x7C,
    //             2 => 0x42,
    //             3 => 0x42,
    //             4 => 0x42,
    //             5 => 0x42,
    //             6 => 0x7C,
    //             7 => 0x7C,
    //             8 => 0x42,
    //             9 => 0x42,
    //             10 => 0x42,
    //             11 => 0x42,
    //             12 => 0x7C,
    //             13 => 0x7C,
    //             _ => 0x00,
    //         },
    //         Glyph::UppercaseC => match row {
    //             0 => 0x3C,
    //             1 => 0x3C,
    //             2 => 0x42,
    //             3 => 0x42,
    //             4 => 0x40,
    //             5 => 0x40,
    //             6 => 0x40,
    //             7 => 0x40,
    //             8 => 0x40,
    //             9 => 0x40,
    //             10 => 0x42,
    //             11 => 0x42,
    //             12 => 0x3C,
    //             13 => 0x3C,
    //             _ => 0x00,
    //         },
    //         Glyph::UppercaseD => match row {
    //             0 => 0x7C,
    //             1 => 0x7C,
    //             2 => 0x42,
    //             3 => 0x42,
    //             4 => 0x42,
    //             5 => 0x42,
    //             6 => 0x42,
    //             7 => 0x42,
    //             8 => 0x42,
    //             9 => 0x42,
    //             10 => 0x42,
    //             11 => 0x42,
    //             12 => 0x7C,
    //             13 => 0x7C,
    //             _ => 0x00,
    //         },
    //         Glyph::UppercaseE => match row {
    //             0 => 0x7E,
    //             1 => 0x7E,
    //             2 => 0x40,
    //             3 => 0x40,
    //             4 => 0x40,
    //             5 => 0x40,
    //             6 => 0x7C,
    //             7 => 0x7C,
    //             8 => 0x40,
    //             9 => 0x40,
    //             10 => 0x40,
    //             11 => 0x40,
    //             12 => 0x7E,
    //             13 => 0x7E,
    //             _ => 0x00,
    //         },
    //         Glyph::UppercaseF => match row {
    //             0 => 0x7E,
    //             1 => 0x7E,
    //             2 => 0x40,
    //             3 => 0x40,
    //             4 => 0x40,
    //             5 => 0x40,
    //             6 => 0x7C,
    //             7 => 0x7C,
    //             8 => 0x40,
    //             9 => 0x40,
    //             10 => 0x40,
    //             11 => 0x40,
    //             12 => 0x40,
    //             13 => 0x40,
    //             _ => 0x00,
    //         },
    //         Glyph::UppercaseG => match row {
    //             0 => 0x3E,
    //             1 => 0x3E,
    //             2 => 0x40,
    //             3 => 0x40,
    //             4 => 0x40,
    //             5 => 0x40,
    //             6 => 0x40,
    //             7 => 0x4E,
    //             8 => 0x4E,
    //             9 => 0x42,
    //             10 => 0x42,
    //             11 => 0x42,
    //             12 => 0x3E,
    //             13 => 0x3E,
    //             _ => 0x00,
    //         },
    //         Glyph::UppercaseH => match row {
    //             0 => 0x42,
    //             1 => 0x42,
    //             2 => 0x42,
    //             3 => 0x42,
    //             4 => 0x42,
    //             5 => 0x42,
    //             6 => 0x7E,
    //             7 => 0x7E,
    //             8 => 0x42,
    //             9 => 0x42,
    //             10 => 0x42,
    //             11 => 0x42,
    //             12 => 0x42,
    //             13 => 0x42,
    //             _ => 0x00,
    //         },
    //         Glyph::UppercaseI => match row {
    //             0 => 0x38,
    //             1 => 0x38,
    //             2 => 0x10,
    //             3 => 0x10,
    //             4 => 0x10,
    //             5 => 0x10,
    //             6 => 0x10,
    //             7 => 0x10,
    //             8 => 0x10,
    //             9 => 0x10,
    //             10 => 0x10,
    //             11 => 0x10,
    //             12 => 0x38,
    //             13 => 0x38,
    //             _ => 0x00,
    //         },
    //         Glyph::UppercaseJ => match row {
    //             0 => 0x02,
    //             1 => 0x02,
    //             2 => 0x02,
    //             3 => 0x02,
    //             4 => 0x02,
    //             5 => 0x02,
    //             6 => 0x02,
    //             7 => 0x02,
    //             8 => 0x02,
    //             9 => 0x02,
    //             10 => 0x42,
    //             11 => 0x42,
    //             12 => 0x3C,
    //             13 => 0x3C,
    //             _ => 0x00,
    //         },
    //         Glyph::UppercaseK => match row {
    //             0 => 0x42,
    //             1 => 0x42,
    //             2 => 0x44,
    //             3 => 0x44,
    //             4 => 0x48,
    //             5 => 0x48,
    //             6 => 0x70,
    //             7 => 0x70,
    //             8 => 0x48,
    //             9 => 0x48,
    //             10 => 0x44,
    //             11 => 0x44,
    //             12 => 0x42,
    //             13 => 0x42,
    //             _ => 0x00,
    //         },
    //         Glyph::UppercaseL => match row {
    //             0 => 0x40,
    //             1 => 0x40,
    //             2 => 0x40,
    //             3 => 0x40,
    //             4 => 0x40,
    //             5 => 0x40,
    //             6 => 0x40,
    //             7 => 0x40,
    //             8 => 0x40,
    //             9 => 0x40,
    //             10 => 0x40,
    //             11 => 0x40,
    //             12 => 0x7E,
    //             13 => 0x7E,
    //             _ => 0x00,
    //         },
    //         Glyph::UppercaseM => match row {
    //             0 => 0x42,
    //             1 => 0x42,
    //             2 => 0x66,
    //             3 => 0x66,
    //             4 => 0x5A,
    //             5 => 0x5A,
    //             6 => 0x5A,
    //             7 => 0x5A,
    //             8 => 0x42,
    //             9 => 0x42,
    //             10 => 0x42,
    //             11 => 0x42,
    //             12 => 0x42,
    //             13 => 0x42,
    //             _ => 0x00,
    //         },
    //         Glyph::UppercaseN => match row {
    //             0 => 0x42,
    //             1 => 0x42,
    //             2 => 0x62,
    //             3 => 0x62,
    //             4 => 0x52,
    //             5 => 0x52,
    //             6 => 0x5A,
    //             7 => 0x5A,
    //             8 => 0x4A,
    //             9 => 0x4A,
    //             10 => 0x46,
    //             11 => 0x46,
    //             12 => 0x42,
    //             13 => 0x42,
    //             _ => 0x00,
    //         },
    //         Glyph::UppercaseO => match row {
    //             0 => 0x3C,
    //             1 => 0x3C,
    //             2 => 0x42,
    //             3 => 0x42,
    //             4 => 0x42,
    //             5 => 0x42,
    //             6 => 0x42,
    //             7 => 0x42,
    //             8 => 0x42,
    //             9 => 0x42,
    //             10 => 0x42,
    //             11 => 0x42,
    //             12 => 0x3C,
    //             13 => 0x3C,
    //             _ => 0x00,
    //         },
    //         Glyph::UppercaseP => match row {
    //             0 => 0x7C,
    //             1 => 0x7C,
    //             2 => 0x42,
    //             3 => 0x42,
    //             4 => 0x42,
    //             5 => 0x42,
    //             6 => 0x7C,
    //             7 => 0x7C,
    //             8 => 0x40,
    //             9 => 0x40,
    //             10 => 0x40,
    //             11 => 0x40,
    //             12 => 0x40,
    //             13 => 0x40,
    //             _ => 0x00,
    //         },
    //         Glyph::UppercaseQ => match row {
    //             0 => 0x3C,
    //             1 => 0x3C,
    //             2 => 0x42,
    //             3 => 0x42,
    //             4 => 0x42,
    //             5 => 0x42,
    //             6 => 0x42,
    //             7 => 0x42,
    //             8 => 0x4A,
    //             9 => 0x4A,
    //             10 => 0x44,
    //             11 => 0x44,
    //             12 => 0x3A,
    //             13 => 0x3A,
    //             _ => 0x00,
    //         },
    //         Glyph::UppercaseR => match row {
    //             0 => 0x7C,
    //             1 => 0x7C,
    //             2 => 0x42,
    //             3 => 0x42,
    //             4 => 0x42,
    //             5 => 0x42,
    //             6 => 0x7C,
    //             7 => 0x7C,
    //             8 => 0x48,
    //             9 => 0x48,
    //             10 => 0x44,
    //             11 => 0x44,
    //             12 => 0x42,
    //             13 => 0x42,
    //             _ => 0x00,
    //         },
    //         Glyph::UppercaseS => match row {
    //             0 => 0x3C,
    //             1 => 0x3C,
    //             2 => 0x42,
    //             3 => 0x42,
    //             4 => 0x40,
    //             5 => 0x40,
    //             6 => 0x3C,
    //             7 => 0x3C,
    //             8 => 0x02,
    //             9 => 0x02,
    //             10 => 0x42,
    //             11 => 0x42,
    //             12 => 0x3C,
    //             13 => 0x3C,
    //             _ => 0x00,
    //         },
    //         Glyph::UppercaseT => match row {
    //             0 => 0x7C,
    //             1 => 0x7C,
    //             2 => 0x10,
    //             3 => 0x10,
    //             4 => 0x10,
    //             5 => 0x10,
    //             6 => 0x10,
    //             7 => 0x10,
    //             8 => 0x10,
    //             9 => 0x10,
    //             10 => 0x10,
    //             11 => 0x10,
    //             12 => 0x10,
    //             13 => 0x10,
    //             _ => 0x00,
    //         },
    //         Glyph::UppercaseU => match row {
    //             0 => 0x42,
    //             1 => 0x42,
    //             2 => 0x42,
    //             3 => 0x42,
    //             4 => 0x42,
    //             5 => 0x42,
    //             6 => 0x42,
    //             7 => 0x42,
    //             8 => 0x42,
    //             9 => 0x42,
    //             10 => 0x42,
    //             11 => 0x42,
    //             12 => 0x3C,
    //             13 => 0x3C,
    //             _ => 0x00,
    //         },
    //         Glyph::UppercaseV => match row {
    //             0 => 0x42,
    //             1 => 0x42,
    //             2 => 0x42,
    //             3 => 0x42,
    //             4 => 0x42,
    //             5 => 0x42,
    //             6 => 0x42,
    //             7 => 0x42,
    //             8 => 0x42,
    //             9 => 0x42,
    //             10 => 0x24,
    //             11 => 0x24,
    //             12 => 0x18,
    //             13 => 0x18,
    //             _ => 0x00,
    //         },
    //         Glyph::UppercaseW => match row {
    //             0 => 0x42,
    //             1 => 0x42,
    //             2 => 0x42,
    //             3 => 0x42,
    //             4 => 0x42,
    //             5 => 0x42,
    //             6 => 0x5A,
    //             7 => 0x5A,
    //             8 => 0x5A,
    //             9 => 0x5A,
    //             10 => 0x66,
    //             11 => 0x66,
    //             12 => 0x42,
    //             13 => 0x42,
    //             _ => 0x00,
    //         },
    //         Glyph::UppercaseX => match row {
    //             0 => 0x42,
    //             1 => 0x42,
    //             2 => 0x42,
    //             3 => 0x42,
    //             4 => 0x24,
    //             5 => 0x24,
    //             6 => 0x18,
    //             7 => 0x18,
    //             8 => 0x24,
    //             9 => 0x24,
    //             10 => 0x42,
    //             11 => 0x42,
    //             12 => 0x42,
    //             13 => 0x42,
    //             _ => 0x00,
    //         },
    //         Glyph::UppercaseY => match row {
    //             0 => 0x44,
    //             1 => 0x44,
    //             2 => 0x44,
    //             3 => 0x44,
    //             4 => 0x28,
    //             5 => 0x28,
    //             6 => 0x10,
    //             7 => 0x10,
    //             8 => 0x10,
    //             9 => 0x10,
    //             10 => 0x10,
    //             11 => 0x10,
    //             12 => 0x10,
    //             13 => 0x10,
    //             _ => 0x00,
    //         },
    //         Glyph::UppercaseZ => match row {
    //             0 => 0x7E,
    //             1 => 0x7E,
    //             2 => 0x02,
    //             3 => 0x02,
    //             4 => 0x04,
    //             5 => 0x04,
    //             6 => 0x18,
    //             7 => 0x18,
    //             8 => 0x20,
    //             9 => 0x20,
    //             10 => 0x40,
    //             11 => 0x40,
    //             12 => 0x7E,
    //             13 => 0x7E,
    //             _ => 0x00,
    //         },
    //         Glyph::OpenSquareBracket => match row {
    //             0 => 0x7E,
    //             1 => 0x7E,
    //             2 => 0x60,
    //             3 => 0x60,
    //             4 => 0x60,
    //             5 => 0x60,
    //             6 => 0x60,
    //             7 => 0x60,
    //             8 => 0x60,
    //             9 => 0x60,
    //             10 => 0x60,
    //             11 => 0x60,
    //             12 => 0x7E,
    //             13 => 0x7E,
    //             _ => 0x00,
    //         },
    //         Glyph::Backslash => match row {
    //             2 => 0x40,
    //             3 => 0x40,
    //             4 => 0x20,
    //             5 => 0x20,
    //             6 => 0x10,
    //             7 => 0x10,
    //             8 => 0x08,
    //             9 => 0x08,
    //             10 => 0x04,
    //             11 => 0x04,
    //             12 => 0x02,
    //             13 => 0x02,
    //             _ => 0x00,
    //         },
    //         Glyph::CloseSquareBracket => match row {
    //             0 => 0x7E,
    //             1 => 0x7E,
    //             2 => 0x06,
    //             3 => 0x06,
    //             4 => 0x06,
    //             5 => 0x06,
    //             6 => 0x06,
    //             7 => 0x06,
    //             8 => 0x06,
    //             9 => 0x06,
    //             10 => 0x06,
    //             11 => 0x06,
    //             12 => 0x7E,
    //             13 => 0x7E,
    //             _ => 0x00,
    //         },
    //         Glyph::Caret => match row {
    //             4 => 0x10,
    //             5 => 0x10,
    //             6 => 0x28,
    //             7 => 0x28,
    //             8 => 0x44,
    //             9 => 0x44,
    //             _ => 0x00,
    //         },
    //         Glyph::Underscore => match row {
    //             12 => 0x7E,
    //             13 => 0x7E,
    //             _ => 0x00,
    //         },
    //         Glyph::Backtick => match row {
    //             0 => 0x20,
    //             1 => 0x20,
    //             2 => 0x10,
    //             3 => 0x10,
    //             4 => 0x08,
    //             5 => 0x08,
    //             _ => 0x00,
    //         },
    //         Glyph::LowercaseA => match row {
    //             4 => 0x3C,
    //             5 => 0x3C,
    //             6 => 0x02,
    //             7 => 0x02,
    //             8 => 0x3E,
    //             9 => 0x3E,
    //             10 => 0x42,
    //             11 => 0x42,
    //             12 => 0x3E,
    //             13 => 0x3E,
    //             _ => 0x00,
    //         },
    //         Glyph::LowercaseB => match row {
    //             0 => 0x40,
    //             1 => 0x40,
    //             2 => 0x40,
    //             3 => 0x40,
    //             4 => 0x7C,
    //             5 => 0x7C,
    //             6 => 0x42,
    //             7 => 0x42,
    //             8 => 0x42,
    //             9 => 0x42,
    //             10 => 0x42,
    //             11 => 0x42,
    //             12 => 0x7C,
    //             13 => 0x7C,
    //             _ => 0x00,
    //         },
    //         Glyph::LowercaseC => match row {
    //             4 => 0x3E,
    //             5 => 0x3E,
    //             6 => 0x40,
    //             7 => 0x40,
    //             8 => 0x40,
    //             9 => 0x40,
    //             10 => 0x40,
    //             11 => 0x40,
    //             12 => 0x3E,
    //             13 => 0x3E,
    //             _ => 0x00,
    //         },
    //         Glyph::LowercaseD => match row {
    //             0 => 0x02,
    //             1 => 0x02,
    //             2 => 0x02,
    //             3 => 0x02,
    //             4 => 0x3E,
    //             5 => 0x3E,
    //             6 => 0x42,
    //             7 => 0x42,
    //             8 => 0x42,
    //             9 => 0x42,
    //             10 => 0x42,
    //             11 => 0x42,
    //             12 => 0x3E,
    //             13 => 0x3E,
    //             _ => 0x00,
    //         },
    //         Glyph::LowercaseE => match row {
    //             4 => 0x3C,
    //             5 => 0x3C,
    //             6 => 0x42,
    //             7 => 0x42,
    //             8 => 0x7E,
    //             9 => 0x7E,
    //             10 => 0x40,
    //             11 => 0x40,
    //             12 => 0x3E,
    //             13 => 0x3E,
    //             _ => 0x00,
    //         },
    //         Glyph::LowercaseF => match row {
    //             0 => 0x1C,
    //             1 => 0x1C,
    //             2 => 0x22,
    //             3 => 0x22,
    //             4 => 0x20,
    //             5 => 0x20,
    //             6 => 0x7C,
    //             7 => 0x7C,
    //             8 => 0x20,
    //             9 => 0x20,
    //             10 => 0x20,
    //             11 => 0x20,
    //             12 => 0x20,
    //             13 => 0x20,
    //             _ => 0x00,
    //         },
    //         Glyph::LowercaseG => match row {
    //             4 => 0x3C,
    //             5 => 0x3C,
    //             6 => 0x42,
    //             7 => 0x42,
    //             8 => 0x42,
    //             9 => 0x42,
    //             10 => 0x3E,
    //             11 => 0x3E,
    //             12 => 0x02,
    //             13 => 0x02,
    //             14 => 0x3C,
    //             15 => 0x3C, // g
    //             _ => 0x00,
    //         },
    //         Glyph::LowercaseH => match row {
    //             0 => 0x40,
    //             1 => 0x40,
    //             2 => 0x40,
    //             3 => 0x40,
    //             4 => 0x7C,
    //             5 => 0x7C,
    //             6 => 0x42,
    //             7 => 0x42,
    //             8 => 0x42,
    //             9 => 0x42,
    //             10 => 0x42,
    //             11 => 0x42,
    //             12 => 0x42,
    //             13 => 0x42,
    //             _ => 0x00,
    //         },
    //         Glyph::LowercaseI => match row {
    //             0 => 0x10,
    //             1 => 0x10,
    //             4 => 0x30,
    //             5 => 0x30,
    //             6 => 0x10,
    //             7 => 0x10,
    //             8 => 0x10,
    //             9 => 0x10,
    //             10 => 0x10,
    //             11 => 0x10,
    //             12 => 0x38,
    //             13 => 0x38,
    //             _ => 0x00,
    //         },
    //         Glyph::LowercaseJ => match row {
    //             0 => 0x04,
    //             1 => 0x04,
    //             4 => 0x3C,
    //             5 => 0x3C,
    //             6 => 0x04,
    //             7 => 0x04,
    //             8 => 0x04,
    //             9 => 0x04,
    //             10 => 0x04,
    //             11 => 0x04,
    //             12 => 0x44,
    //             13 => 0x44,
    //             14 => 0x38,
    //             15 => 0x38, // j
    //             _ => 0x00,
    //         },
    //         Glyph::LowercaseK => match row {
    //             0 => 0x40,
    //             1 => 0x40,
    //             2 => 0x40,
    //             3 => 0x40,
    //             4 => 0x42,
    //             5 => 0x42,
    //             6 => 0x44,
    //             7 => 0x44,
    //             8 => 0x78,
    //             9 => 0x78,
    //             10 => 0x44,
    //             11 => 0x44,
    //             12 => 0x42,
    //             13 => 0x42,
    //             _ => 0x00,
    //         },
    //         Glyph::LowercaseL => match row {
    //             0 => 0x30,
    //             1 => 0x30,
    //             2 => 0x10,
    //             3 => 0x10,
    //             4 => 0x10,
    //             5 => 0x10,
    //             6 => 0x10,
    //             7 => 0x10,
    //             8 => 0x10,
    //             9 => 0x10,
    //             10 => 0x10,
    //             11 => 0x10,
    //             12 => 0x38,
    //             13 => 0x38,
    //             _ => 0x00,
    //         },
    //         Glyph::LowercaseM => match row {
    //             4 => 0x66,
    //             5 => 0x66,
    //             6 => 0x5A,
    //             7 => 0x5A,
    //             8 => 0x5A,
    //             9 => 0x5A,
    //             10 => 0x5A,
    //             11 => 0x5A,
    //             12 => 0x42,
    //             13 => 0x42,
    //             _ => 0x00,
    //         },
    //         Glyph::LowercaseN => match row {
    //             4 => 0x7C,
    //             5 => 0x7C,
    //             6 => 0x42,
    //             7 => 0x42,
    //             8 => 0x42,
    //             9 => 0x42,
    //             10 => 0x42,
    //             11 => 0x42,
    //             12 => 0x42,
    //             13 => 0x42,
    //             _ => 0x00,
    //         },
    //         Glyph::LowercaseO => match row {
    //             4 => 0x3C,
    //             5 => 0x3C,
    //             6 => 0x42,
    //             7 => 0x42,
    //             8 => 0x42,
    //             9 => 0x42,
    //             10 => 0x42,
    //             11 => 0x42,
    //             12 => 0x3C,
    //             13 => 0x3C,
    //             _ => 0x00,
    //         },
    //         Glyph::LowercaseP => match row {
    //             4 => 0x7C,
    //             5 => 0x7C,
    //             6 => 0x42,
    //             7 => 0x42,
    //             8 => 0x42,
    //             9 => 0x42,
    //             10 => 0x7C,
    //             11 => 0x7C,
    //             12 => 0x40,
    //             13 => 0x40,
    //             14 => 0x40,
    //             15 => 0x40, // p
    //             _ => 0x00,
    //         },
    //         Glyph::LowercaseQ => match row {
    //             4 => 0x3E,
    //             5 => 0x3E,
    //             6 => 0x42,
    //             7 => 0x42,
    //             8 => 0x42,
    //             9 => 0x42,
    //             10 => 0x3E,
    //             11 => 0x3E,
    //             12 => 0x02,
    //             13 => 0x02,
    //             14 => 0x02,
    //             15 => 0x02, // q
    //             _ => 0x00,
    //         },
    //         Glyph::LowercaseR => match row {
    //             4 => 0x5E,
    //             5 => 0x5E,
    //             6 => 0x60,
    //             7 => 0x60,
    //             8 => 0x40,
    //             9 => 0x40,
    //             10 => 0x40,
    //             11 => 0x40,
    //             12 => 0x40,
    //             13 => 0x40,
    //             _ => 0x00,
    //         },
    //         Glyph::LowercaseS => match row {
    //             4 => 0x3E,
    //             5 => 0x3E,
    //             6 => 0x40,
    //             7 => 0x40,
    //             8 => 0x3C,
    //             9 => 0x3C,
    //             10 => 0x02,
    //             11 => 0x02,
    //             12 => 0x7C,
    //             13 => 0x7C,
    //             _ => 0x00,
    //         },
    //         Glyph::LowercaseT => match row {
    //             0 => 0x10,
    //             1 => 0x10,
    //             2 => 0x10,
    //             3 => 0x10,
    //             4 => 0x7C,
    //             5 => 0x7C,
    //             6 => 0x10,
    //             7 => 0x10,
    //             8 => 0x10,
    //             9 => 0x10,
    //             10 => 0x12,
    //             11 => 0x12,
    //             12 => 0x0C,
    //             13 => 0x0C,
    //             _ => 0x00,
    //         },
    //         Glyph::LowercaseU => match row {
    //             4 => 0x42,
    //             5 => 0x42,
    //             6 => 0x42,
    //             7 => 0x42,
    //             8 => 0x42,
    //             9 => 0x42,
    //             10 => 0x46,
    //             11 => 0x46,
    //             12 => 0x3A,
    //             13 => 0x3A,
    //             _ => 0x00,
    //         },
    //         Glyph::LowercaseV => match row {
    //             4 => 0x42,
    //             5 => 0x42,
    //             6 => 0x42,
    //             7 => 0x42,
    //             8 => 0x42,
    //             9 => 0x42,
    //             10 => 0x24,
    //             11 => 0x24,
    //             12 => 0x18,
    //             13 => 0x18,
    //             _ => 0x00,
    //         },
    //         Glyph::LowercaseW => match row {
    //             4 => 0x42,
    //             5 => 0x42,
    //             6 => 0x42,
    //             7 => 0x42,
    //             8 => 0x5A,
    //             9 => 0x5A,
    //             10 => 0x5A,
    //             11 => 0x5A,
    //             12 => 0x66,
    //             13 => 0x66,
    //             _ => 0x00,
    //         },
    //         Glyph::LowercaseX => match row {
    //             4 => 0x42,
    //             5 => 0x42,
    //             6 => 0x24,
    //             7 => 0x24,
    //             8 => 0x18,
    //             9 => 0x18,
    //             10 => 0x24,
    //             11 => 0x24,
    //             12 => 0x42,
    //             13 => 0x42,
    //             _ => 0x00,
    //         },
    //         Glyph::LowercaseY => match row {
    //             4 => 0x42,
    //             5 => 0x42,
    //             6 => 0x42,
    //             7 => 0x42,
    //             8 => 0x42,
    //             9 => 0x42,
    //             10 => 0x3E,
    //             11 => 0x3E,
    //             12 => 0x02,
    //             13 => 0x02,
    //             14 => 0x3C,
    //             15 => 0x3C, // y
    //             _ => 0x00,
    //         },
    //         Glyph::LowercaseZ => match row {
    //             4 => 0x7E,
    //             5 => 0x7E,
    //             6 => 0x04,
    //             7 => 0x04,
    //             8 => 0x18,
    //             9 => 0x18,
    //             10 => 0x20,
    //             11 => 0x20,
    //             12 => 0x7E,
    //             13 => 0x7E,
    //             _ => 0x00,
    //         },
    //         Glyph::OpenBrace => match row {
    //             0 => 0x0E,
    //             1 => 0x0E,
    //             2 => 0x18,
    //             3 => 0x18,
    //             4 => 0x18,
    //             5 => 0x18,
    //             6 => 0x70,
    //             7 => 0x70,
    //             8 => 0x18,
    //             9 => 0x18,
    //             10 => 0x18,
    //             11 => 0x18,
    //             12 => 0x0E,
    //             13 => 0x0E,
    //             _ => 0x00,
    //         },
    //         Glyph::VerticalBar => match row {
    //             0 => 0x10,
    //             1 => 0x10,
    //             2 => 0x10,
    //             3 => 0x10,
    //             4 => 0x10,
    //             5 => 0x10,
    //             6 => 0x10,
    //             7 => 0x10,
    //             8 => 0x10,
    //             9 => 0x10,
    //             10 => 0x10,
    //             11 => 0x10,
    //             12 => 0x10,
    //             13 => 0x10,
    //             _ => 0x00,
    //         },
    //         Glyph::CloseBrace => match row {
    //             0 => 0x70,
    //             1 => 0x70,
    //             2 => 0x18,
    //             3 => 0x18,
    //             4 => 0x18,
    //             5 => 0x18,
    //             6 => 0x0E,
    //             7 => 0x0E,
    //             8 => 0x18,
    //             9 => 0x18,
    //             10 => 0x18,
    //             11 => 0x18,
    //             12 => 0x70,
    //             13 => 0x70,
    //             _ => 0x00,
    //         },
    //         Glyph::Tilde => match row {
    //             4 => 0x24,
    //             5 => 0x24,
    //             6 => 0x54,
    //             7 => 0x54,
    //             8 => 0x48,
    //             9 => 0x48,
    //             _ => 0x00,
    //         },
    //         Glyph::Space => match row {
    //             _ => 0b00000000,
    //         },
    //         _ => match row {
    //             _ => 0b11111111,
    //         },
    //     }
    // }
    fn font_lookup(glyph: Glyph, row: usize) -> u8 {
        let index = ((glyph as usize) * FONT_HEIGHT) + row;
        FONT_DATA[index]
    }
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

fn main() {
    let p = tm4c123x_hal::Peripherals::take().unwrap();
    let cp = tm4c123x_hal::CorePeripherals::take().unwrap();

    let mut sc = p.SYSCTL.constrain();
    sc.clock_setup.oscillator = sysctl::Oscillator::Main(
        sysctl::CrystalFrequency::_16mhz,
        sysctl::SystemClock::UsePll(sysctl::PllOutputFrequency::_80_00mhz),
    );
    let _clocks = sc.clock_setup.freeze();

    let mut nvic = cp.NVIC;
    nvic.enable(tm4c123x_hal::Interrupt::TIMER0A);
    nvic.enable(tm4c123x_hal::Interrupt::TIMER0B);

    enable(sysctl::Domain::Timer0, &mut sc.power_control);
    enable(sysctl::Domain::MicroDma, &mut sc.power_control);
    enable(sysctl::Domain::Ssi2, &mut sc.power_control);

    cortex_m::interrupt::free(|cs| {
        let mut fb = FB_INFO.borrow(&cs).borrow_mut();
        fb.write_string("ABRACADABRA!", 0, 0);
        fb.write_char('A', 0, MAX_Y);
        fb.write_char('A', MAX_X, 0);
        fb.write_char('A', MAX_X, MAX_Y);
    });

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
    let dma = p.UDMA;
    dma.cfg.write(|w| w.masten().set_bit());
    dma.ctlbase
        .write(|w| unsafe { w.addr().bits(&mut DMA_CONTROL_TABLE as *mut DmaInfo as u32) });

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
}

fn start_of_line(fb_info: &mut FrameBuffer) {
    let gpio = unsafe { &*tm4c123x_hal::tm4c123x::GPIO_PORTC::ptr() };

    fb_info.line_no += 1;

    if fb_info.line_no == V_WHOLE_FRAME {
        fb_info.line_no = 0;
        // unsafe { bb::change_bit(&gpio.data, 4, true) };
    }

    if fb_info.line_no == V_SYNC_PULSE {
        // unsafe { bb::change_bit(&gpio.data, 4, false) };
    }

    if (fb_info.line_no >= V_SYNC_PULSE + V_BACK_PORCH)
        && (fb_info.line_no < V_SYNC_PULSE + V_BACK_PORCH + V_VISIBLE_AREA)
    {
        // Visible lines
        // 600 visible lines, 300 output lines each shown twice
        fb_info.fb_line = Some((fb_info.line_no - (V_SYNC_PULSE + V_BACK_PORCH)) >> 1);
        unsafe { bb::change_bit(&gpio.data, 4, true) };
        fb_info.render();
        unsafe { bb::change_bit(&gpio.data, 4, false) };
    } else {
        // Front porch
        fb_info.fb_line = None;
    }
}

fn start_of_data(fb_info: &FrameBuffer) {
    let ssi = unsafe { &*tm4c123x_hal::tm4c123x::SSI2::ptr() };
    if fb_info.fb_line.is_some() {
        for word in fb_info.render_line.iter() {
            ssi.dr.write(|w| unsafe { w.data().bits(*word) });
            while ssi.sr.read().tnf().bit_is_clear() {
                asm::nop();
            }
        }
    }
}

extern "C" fn timer0a_isr() {
    let timer = unsafe { &*tm4c123x_hal::tm4c123x::TIMER0::ptr() };
    let cs = unsafe { CriticalSection::new() };
    let mut fb_info = FB_INFO.borrow(&cs).borrow_mut();
    start_of_line(&mut fb_info);
    timer.icr.write(|w| w.caecint().set_bit());
}

extern "C" fn timer0b_isr() {
    let timer = unsafe { &*tm4c123x_hal::tm4c123x::TIMER0::ptr() };
    let cs = unsafe { CriticalSection::new() };
    let fb_info = FB_INFO.borrow(&cs).borrow();
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
