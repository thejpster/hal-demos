//! Prints "Hello, world!" on the OpenOCD console using semihosting
//!
//! ---

#![feature(used)]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate cortex_m_semihosting;
extern crate tm4c123x_hal;
extern crate embedded_hal;

use core::fmt::Write;

use cortex_m::asm;
use cortex_m_semihosting::hio;

use tm4c123x_hal::sysctl::{self, chip_id, SysctlExt};
use tm4c123x_hal::gpio::GpioExt;
use embedded_hal::digital::OutputPin;

fn main() {
    let mut stdout = hio::hstdout().unwrap();
    writeln!(stdout, "Hello, world!").unwrap();

    let p = tm4c123x_hal::Peripherals::take().unwrap();
    let mut sc = p.SYSCTL.constrain();
    // sc.clock_setup.oscillator = sysctl::Oscillator::Main(
    //     sysctl::CrystalFrequency::_16mhz,
    //     sysctl::SystemClock::UseOscillator(sysctl::Divider::_2),
    // );
    sc.clock_setup.oscillator = sysctl::Oscillator::Main(
        sysctl::CrystalFrequency::_16mhz,
        sysctl::SystemClock::UsePll(sysctl::PllOutputFrequency::_66_67mhz),
    );
    writeln!(stdout, "Freezing clocks...").unwrap();
    let clocks = sc.clock_setup.freeze();
    writeln!(stdout, "Sysclk: {} Hz", clocks.sysclk.0).unwrap();
    sysctl::control_power(
        &sc.power_control,
        sysctl::PeripheralPowerDomain::GpioA,
        sysctl::RunMode::Run,
        sysctl::PowerState::On,
    );

    // turn on LED here
    let port = p.GPIO_PORTF.split(&sc.power_control);
    let mut pf1 = port.pf1.into_push_pull_output();
    pf1.set_high();

    writeln!(stdout, "Chip: {:?}", chip_id::get()).unwrap();
}

// As we are not using interrupts, we just register a dummy catch all handler
#[link_section = ".vector_table.interrupts"]
#[used]
static INTERRUPTS: [extern "C" fn(); 240] = [default_handler; 240];

extern "C" fn default_handler() {
    asm::bkpt();
}
