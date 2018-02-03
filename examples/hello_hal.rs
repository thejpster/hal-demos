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
use embedded_hal::prelude::*;
use tm4c123x_hal::gpio::GpioExt;
use tm4c123x_hal::serial::{Serial, NewlineMode};
use tm4c123x_hal::sysctl::{self, chip_id, SysctlExt};
use tm4c123x_hal::time::U32Ext;

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

    // Print chip info
    writeln!(stdout, "Chip: {:?}", chip_id::get()).unwrap();

    // Activate UART
    let mut porta = p.GPIO_PORTA.split(&sc.power_control);
    let rx_pin = porta.pa0.into_af1(&mut porta.control);
    let tx_pin = porta.pa1.into_af1(&mut porta.control);
    let mut uart = Serial::uart0(
        p.UART0,
        tx_pin,
        rx_pin,
        115200_u32.bps(),
        NewlineMode::SwapLFtoCRLF,
        &clocks,
        &sc.power_control
    );
    writeln!(uart, "Hello from Serial()").unwrap();
    let (mut tx, _rx) = uart.split();

    // turn on LED here
    let portf = p.GPIO_PORTF.split(&sc.power_control);
    let mut led_red = portf.pf1.into_push_pull_output();
    let mut led_blue = portf.pf2.into_push_pull_output();
    let mut led_green = portf.pf3.into_push_pull_output();
    led_red.set_high();
    led_blue.set_low();
    led_green.set_low();
    // let button_one = portf.pf0.into_pull_up_input();
    let button_two = portf.pf4.into_pull_up_input();

    let mut count = 0u16;
    loop {
        writeln!(tx, "Hello from TX: {}\n", count).unwrap();
        count = count.wrapping_add(1);
        if button_two.is_low() {
            led_blue.set_high();
            led_green.set_low();
        } else {
            led_blue.set_low();
            led_green.set_high();
        }
    }
}

// As we are not using interrupts, we just register a dummy catch all handler
#[link_section = ".vector_table.interrupts"]
#[used]
static INTERRUPTS: [extern "C" fn(); 240] = [default_handler; 240];

extern "C" fn default_handler() {
    asm::bkpt();
}
