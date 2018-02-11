//! Prints "Hello, world!" on the OpenOCD console using semihosting
//!
//! ---

#![feature(used)]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate cortex_m_semihosting;
extern crate embedded_hal;
extern crate tm4c123x_hal;

use core::fmt::Write;
use cortex_m::asm;
use cortex_m_semihosting::hio;
use embedded_hal::prelude::*;
use tm4c123x_hal::gpio::GpioExt;
use tm4c123x_hal::serial::{NewlineMode, Serial};
use tm4c123x_hal::sysctl::{self, chip_id, SysctlExt};
use tm4c123x_hal::time::U32Ext;
use tm4c123x_hal::delay::Delay;

fn main() {
    let mut itm = hio::hstdout().unwrap();
    writeln!(itm, "Hello, world!").unwrap();

    let p = tm4c123x_hal::Peripherals::take().unwrap();
    let core_p = tm4c123x_hal::CorePeripherals::take().unwrap();
    let mut sc = p.SYSCTL.constrain();

    // This will run you at 8MHz using the internal oscillator
    // sc.clock_setup.oscillator = sysctl::Oscillator::Main(
    //     sysctl::CrystalFrequency::_16mhz,
    //     sysctl::SystemClock::UseOscillator(sysctl::Divider::_2),
    // );

    // This runs you at 66.67MHz using the 16MHz crystal and the PLL
    sc.clock_setup.oscillator = sysctl::Oscillator::Main(
        sysctl::CrystalFrequency::_16mhz,
        sysctl::SystemClock::UsePll(sysctl::PllOutputFrequency::_80_00mhz),
    );

    let clocks = sc.clock_setup.freeze();
    let mut porta = p.GPIO_PORTA.split(&sc.power_control);
    let mut portf = p.GPIO_PORTF.split(&sc.power_control);

    // Activate UART
    let uart = Serial::uart0(
        p.UART0,
        porta.pa1.into_af1(&mut porta.control),
        porta.pa0.into_af1(&mut porta.control),
        (),
        (),
        115200_u32.bps(),
        NewlineMode::SwapLFtoCRLF,
        &clocks,
        &sc.power_control,
    );
    // Print chip info
    let (mut tx, mut rx) = uart.split();
    writeln!(tx, "Chip: {:?}", chip_id::get()).unwrap();

    // This will activate UART1 with H/W flow control
    // TODO: Test with FTDI TTL USB cable.
    let mut portb = p.GPIO_PORTB.split(&sc.power_control);
    let mut portc = p.GPIO_PORTC.split(&sc.power_control);
    let _uart1 = Serial::uart1(
        p.UART1,
        portb.pb1.into_af1(&mut portb.control),
        portb.pb0.into_af1(&mut portb.control),
        portc.pc4.into_af8(&mut portc.control),
        portc.pc5.into_af8(&mut portc.control),
        115200_u32.bps(),
        NewlineMode::SwapLFtoCRLF,
        &clocks,
        &sc.power_control,
    );

    // turn on LED here
    let mut led_red = portf.pf1.into_push_pull_output();
    let mut led_blue = portf.pf2.into_push_pull_output();
    let mut led_green = portf.pf3.into_push_pull_output();
    led_red.set_low();
    led_blue.set_low();
    led_green.set_low();
    let _sw1 = portf.pf4.into_pull_up_input();
    let sw2 = portf.pf0.unlock(&mut portf.control).into_pull_up_input();

    let mut d = Delay::new(core_p.SYST, &clocks);

    loop {
        if sw2.is_low() {
            led_blue.set_high();
            led_green.set_low();
        } else {
            led_blue.set_low();
            led_green.set_high();
        }
        if let Ok(ch) = rx.read() {
            writeln!(tx, "Read 0x{:02x} from the UART", ch).unwrap();
        }

        if led_red.is_high() {
            led_red.set_low();
        } else {
            led_red.set_high();
        }
        d.delay_ms(1000u32);
    }
}

// As we are not using interrupts, we just register a dummy catch all handler
#[link_section = ".vector_table.interrupts"]
#[used]
static INTERRUPTS: [extern "C" fn(); 240] = [default_handler; 240];

extern "C" fn default_handler() {
    asm::bkpt();
}
