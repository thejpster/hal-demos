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
        &mut sc.power_control, sysctl::PeripheralPowerDomain::Timer1,
        sysctl::RunMode::Run, sysctl::PowerState::On);
    sysctl::reset(&mut sc.power_control, sysctl::PeripheralPowerDomain::Timer1);

    sysctl::control_power(
        &mut sc.power_control, sysctl::PeripheralPowerDomain::Ssi2,
        sysctl::RunMode::Run, sysctl::PowerState::On);
    sysctl::reset(&mut sc.power_control, sysctl::PeripheralPowerDomain::Ssi2);

    let mut portb = p.GPIO_PORTB.split(&sc.power_control);
    // T0CCP0
    let mut h_sync = portb.pb6.into_af7(&mut portb.control);
    // T0CCP1 - we use this for checking line pixel start (i.e. sync + back porch)
    // let mut h_timer = portb.pb7.into_af7(&mut portb.control);
    // T1CCP0
    let mut v_sync = portb.pb4.into_af7(&mut portb.control);
    // Ssi2Tx
    let mut green_data = portb.pb7.into_af2(&mut portb.control);

    // Need to configure SSI2 at 20 MHz
    // let mut ssi2 = p.SSI2;
    // SSIClk = SysClk / (CPSDVSR * (1 + SCR))
    // 20 MHz = 80 MHz / (4 * (1 + 0))
    // SCR = 0
    // CPSDVSR = 4
    // FIFO is 16-bits wide, 8 words deep

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
    // TnILR = 528 total horizontal pixels at 20 MHz

    // This timing assumes 40 MHz but we're at 80 MHz
    const H_VISIBLE_AREA: u32 = 800;
    const H_FRONT_PORCH: u32 = 40;
    const H_SYNC_PULSE: u32 = 128;
    const H_BACK_PORCH: u32 = 88;
    const H_WHOLE_LINE: u32 = H_VISIBLE_AREA + H_FRONT_PORCH + H_SYNC_PULSE + H_BACK_PORCH;
    const H_SYNC_END: u32 = H_WHOLE_LINE - H_SYNC_PULSE;
    const H_LINE_START: u32 = H_WHOLE_LINE - (H_SYNC_PULSE + H_BACK_PORCH);

    // We're counting down in PWM mode
    t0.tailr.modify(|_, w| unsafe { w.bits(H_WHOLE_LINE * 2) });
    t0.tbilr.modify(|_, w| unsafe { w.bits(H_WHOLE_LINE * 2) });
    // TAMATCHR = end of sync pulse
    t0.tamatchr.modify(|_, w| unsafe { w.bits(H_SYNC_END * 2) });
    // TBMATCHR = 528 - 108 = 420 (line start) at 20 MHz
    t0.tbmatchr.modify(|_, w| unsafe { w.bits(H_LINE_START * 2) });
    // TODO IMR.TBMIM = 1 (for line start)
    // TODO IMR.TBTOIM = 1 (for row count)

    const V_VISIBLE_AREA: u32 =  600;
    const V_FRONT_PORCH: u32 =  1;
    const V_SYNC_PULSE: u32 =  4;
    const V_BACK_PORCH: u32 =  23;
    const V_WHOLE_FRAME: u32 =  V_VISIBLE_AREA + V_FRONT_PORCH + V_SYNC_PULSE + V_BACK_PORCH;
    const V_SYNC_END: u32 = V_WHOLE_FRAME - V_SYNC_PULSE;
    const V_FIRST_LINE: u32 = V_WHOLE_FRAME - (V_SYNC_PULSE + V_BACK_PORCH);

    // Configure T1A for v-sync, triggered off T0A overflow
    let t1 = p.TIMER1;
    t1.ctl.modify(|_, w| w.taen().clear_bit().tben().clear_bit());
    t1.cfg.modify(|_, w| w.cfg()._16_bit());
    // PWM but enable TAWOT to trigger of previous timer (Timer 1 is chained off Timer 0)
    t1.tamr.modify(|_, w| w.taams().set_bit().tacmr().clear_bit().tamr().period().tawot().set_bit());
    t1.ctl.modify(|_, w| w.tapwml().clear_bit());
    t1.tapr.modify(|_, w| unsafe { w.bits(0) });
    t1.tailr.modify(|_, w| unsafe { w.bits(V_WHOLE_FRAME) });
    t1.tamatchr.modify(|_, w| unsafe { w.bits(V_SYNC_END) });

    t0.ctl.modify(|_, w| w.taen().set_bit().tben().set_bit());
    t1.ctl.modify(|_, w| w.taen().set_bit().tben().set_bit());
}

// As we are not using interrupts, we just register a dummy catch all handler
#[link_section = ".vector_table.interrupts"]
#[used]
static INTERRUPTS: [extern "C" fn(); 240] = [default_handler; 240];

extern "C" fn default_handler() {
    asm::bkpt();
}
