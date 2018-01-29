//! Prints "Hello, world!" on the OpenOCD console using semihosting
//!
//! ---

#![feature(used)]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate cortex_m_semihosting;
extern crate tm4c123x_hal;

use core::fmt::Write;

use cortex_m::asm;
use cortex_m_semihosting::hio;

use tm4c123x_hal::sysctl::{self, chip_id, SysctlExt};

fn main() {
    let mut stdout = hio::hstdout().unwrap();
    writeln!(stdout, "Hello, world!").unwrap();

    let p = tm4c123x_hal::Peripherals::take().unwrap();
    let mut sysctl = p.SYSCTL.constrain();
    // sysctl.clock_setup.oscillator = sysctl::Oscillator::Main(
    //     sysctl::CrystalFrequency::_16mhz,
    //     sysctl::SystemClock::UseOscillator(sysctl::Divider::_2),
    // );
    sysctl.clock_setup.oscillator = sysctl::Oscillator::Main(
        sysctl::CrystalFrequency::_16mhz,
        sysctl::SystemClock::UsePll(sysctl::PllOutputFrequency::_66_67mhz),
    );
    writeln!(stdout, "Freezing clocks...").unwrap();
    let clocks = sysctl.clock_setup.freeze();
    writeln!(stdout, "Sysclk: {}", clocks.sysclk.0).unwrap();

    writeln!(stdout, "Chip: {:?}", chip_id::get()).unwrap();
}

// As we are not using interrupts, we just register a dummy catch all handler
#[link_section = ".vector_table.interrupts"]
#[used]
static INTERRUPTS: [extern "C" fn(); 240] = [default_handler; 240];

extern "C" fn default_handler() {
    asm::bkpt();
}
