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
use embedded_hal::prelude::*;
use tm4c123x_hal::gpio::GpioExt;
use tm4c123x_hal::sysctl::{self, SysctlExt};

fn main() {
    let p = tm4c123x_hal::Peripherals::take().unwrap();
    let mut sc = p.SYSCTL.constrain();

    sc.clock_setup.oscillator = sysctl::Oscillator::Main(
        sysctl::CrystalFrequency::_16mhz,
        sysctl::SystemClock::UsePll(sysctl::PllOutputFrequency::_80_00mhz),
    );
    let clocks = sc.clock_setup.freeze();

    sysctl::control_power(
        &mut sc.power_control, sysctl::PeripheralPowerDomain::Timer0,
        sysctl::RunMode::Run, sysctl::PowerState::On);
    sysctl::reset(&mut sc.power_control, sysctl::PeripheralPowerDomain::Timer0);

    sysctl::control_power(
        &mut sc.power_control, sysctl::PeripheralPowerDomain::Ssi2,
        sysctl::RunMode::Run, sysctl::PowerState::On);
    sysctl::reset(&mut sc.power_control, sysctl::PeripheralPowerDomain::Ssi2);

    let mut portb = p.GPIO_PORTB.split(&sc.power_control);
    let mut h_sync = portb.pb6.into_af7(&mut portb.control);
    // let mut h_timer = portb.pb7.into_af7(&mut portb.control);
    // let mut red_data = portb.pb2.into_push_pull_output();
    // let mut blue_data = portb.pb2.into_push_pull_output();
    let mut green_data = portb.pb7.into_af2(&mut portb.control); // Ssi2Tx

    // Need to configure SSI2 at 20 MHz

    // Need to configure MicroDMA to feed SSI2 with data, triggered on Timer0B interrupt

    let t0 = p.TIMER0;
    // Configure T0A for h-sync and T0B for line trigger
    t0.ctl.modify(|_, w| w.taen().clear_bit().tben().clear_bit());
    t0.cfg.modify(|_, w| w.cfg()._16_bit());
    t0.tamr.modify(|_, w| w.taams().set_bit().tacmr().clear_bit().tamr().period());
    t0.tbmr.modify(|_, w| w.tbams().set_bit().tbcmr().clear_bit().tbmr().period());
    t0.ctl.modify(|_, w| w.tapwml().clear_bit());
    t0.ctl.modify(|_, w| w.tbpwml().set_bit());
    t0.tapr.modify(|_, w| unsafe { w.bits(0) });
    t0.tbpr.modify(|_, w| unsafe { w.bits(0) });
    // TnILR = 528 (total horizontal pixels at 20 MHz)
    t0.tailr.modify(|_, w| unsafe { w.bits(528 * 4) });
    t0.tbilr.modify(|_, w| unsafe { w.bits(528 * 4) });
    // TAMATCHR = 528 - 64 = 464 (sync end) at 20 MHz
    t0.tamatchr.modify(|_, w| unsafe { w.bits(464 * 4) });
    // TBMATCHR = 528 - 108 = 420 (line start) at 20 MHz
    t0.tbmatchr.modify(|_, w| unsafe { w.bits(420 * 4) });
    // TODO IMR.TBMIM = 1 (for line start)
    // TODO IMR.TBTOIM = 1 (for row count)
    t0.ctl.modify(|_, w| w.taen().set_bit().tben().set_bit());
}

// As we are not using interrupts, we just register a dummy catch all handler
#[link_section = ".vector_table.interrupts"]
#[used]
static INTERRUPTS: [extern "C" fn(); 240] = [default_handler; 240];

extern "C" fn default_handler() {
    asm::bkpt();
}
