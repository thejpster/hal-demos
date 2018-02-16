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
use tm4c123x_hal::delay::Delay;
use tm4c123x_hal::gpio::GpioExt;
use tm4c123x_hal::sysctl::{self, SysctlExt};

// This timing assumes 40 MHz but we're at 80 MHz so we'll need to double everything
const H_VISIBLE_AREA: u32 = 80 * 20;
const H_FRONT_PORCH: u32 = 40 * 2;
const H_SYNC_PULSE: u32 = 12 * 28;
const H_BACK_PORCH: u32 = 88 * 2;
const H_WHOLE_LINE: u32 = H_VISIBLE_AREA + H_FRONT_PORCH + H_SYNC_PULSE + H_BACK_PORCH;
const H_SYNC_END: u32 = H_WHOLE_LINE - H_SYNC_PULSE;
const H_LINE_START: u32 = H_WHOLE_LINE - (H_SYNC_PULSE + H_BACK_PORCH);
// This timing is in lines, so it's fine as-is.
const V_VISIBLE_AREA: u32 = 600;
const V_FRONT_PORCH: u32 = 1;
const V_SYNC_PULSE: u32 = 4;
const V_BACK_PORCH: u32 = 23;
const V_WHOLE_FRAME: u32 = V_VISIBLE_AREA + V_FRONT_PORCH + V_SYNC_PULSE + V_BACK_PORCH;
const V_SYNC_END: u32 = V_WHOLE_FRAME - V_SYNC_PULSE;
// const V_FIRST_LINE: u32 = V_WHOLE_FRAME - (V_SYNC_PULSE + V_BACK_PORCH);

#[repr(align(1024))]
struct DmaInfo {
    _data: [u8; 1024],
}

#[used]
/// Required by the DMA engine
static mut DMA_CONTROL_TABLE: DmaInfo = DmaInfo { _data: [0u8; 1024] };

fn enable(p: sysctl::Domain, sc: &mut tm4c123x_hal::sysctl::PowerControl) {
    sysctl::control_power(sc, p, sysctl::RunMode::Run, sysctl::PowerState::On);
    sysctl::control_power(sc, p, sysctl::RunMode::Sleep, sysctl::PowerState::On);
    sysctl::reset(sc, p);
}

fn main() {
    let p = tm4c123x_hal::Peripherals::take().unwrap();
    let cp = tm4c123x_hal::CorePeripherals::take().unwrap();
    let mut sc = p.SYSCTL.constrain();

    unsafe {
        cortex_m::interrupt::enable();
    }

    // 400 bits of framebuffer
    let mut framebuffer: [u16; 25] = [0u16; 25];
    // light up the left edge
    framebuffer[0] = 0x8000;
    // light up the right edge
    framebuffer[24] = 0x0001;
    // fat vertical stripe
    framebuffer[10] = 0xFFFF;
    framebuffer[11] = 0x00FF;
    framebuffer[12] = 0x000F;

    sc.clock_setup.oscillator = sysctl::Oscillator::Main(
        sysctl::CrystalFrequency::_16mhz,
        sysctl::SystemClock::UsePll(sysctl::PllOutputFrequency::_80_00mhz),
    );
    let clocks = sc.clock_setup.freeze();

    let mut nvic = cp.NVIC;
    nvic.enable(tm4c123x_hal::Interrupt::TIMER0B);

    enable(sysctl::Domain::Timer0, &mut sc.power_control);
    enable(sysctl::Domain::WideTimer0, &mut sc.power_control);
    enable(sysctl::Domain::MicroDma, &mut sc.power_control);
    enable(sysctl::Domain::Ssi2, &mut sc.power_control);

    let mut portb = p.GPIO_PORTB.split(&sc.power_control);
    let mut portc = p.GPIO_PORTC.split(&sc.power_control);
    // T0CCP0
    let _h_sync = portb.pb6.into_af7(&mut portb.control);
    // T0CCP1 - we use this for checking line pixel start (i.e. sync + back porch)
    // let _h_timer = portb.pb7.into_af7(&mut portb.control);
    // WT0CCP0
    let _v_sync = portc.pc4.into_af7(&mut portc.control);
    // Ssi2Tx
    let _green_data = portb.pb7.into_af2(&mut portb.control);

    // Need to configure SSI2 at 20 MHz
    let ssi2 = p.SSI2;
    ssi2.cr1.modify(|_, w| w.sse().clear_bit());
    // SSIClk = SysClk / (CPSDVSR * (1 + SCR))
    // 20 MHz = 80 MHz / (4 * (1 + 0))
    // SCR = 0
    // CPSDVSR = 4
    ssi2.cpsr.write(|w| unsafe { w.cpsdvsr().bits(4) });
    // Send 16 bits at a time in Freescale format
    ssi2.cr0.write(|w| {
        w.dss()._16();
        w.frf().moto();
        w.spo().clear_bit();
        w.sph().clear_bit();
        w
    });
    // Enable TX DMA
    // ssi2.dmactl.write(|w| w.txdmae().set_bit());
    // Set clock source to sysclk
    ssi2.cc.modify(|_, w| w.cs().syspll());
    // Enable SSI2
    ssi2.cr1.modify(|_, w| w.sse().set_bit());

    // Need to configure MicroDMA to feed SSI2 with data
    // That's Encoder 2, Channel 13
    let dma = p.UDMA;
    dma.cfg.write(|w| w.masten().set_bit());
    dma.ctlbase
        .write(|w| unsafe { w.addr().bits(&mut DMA_CONTROL_TABLE as *mut DmaInfo as u32) });

    let h_timer = p.TIMER0;
    // Configure h_timerA for h-sync and h_timerB for line trigger
    h_timer
        .ctl
        .modify(|_, w| w.taen().clear_bit().tben().clear_bit());
    h_timer.cfg.modify(|_, w| w.cfg()._16_bit());
    h_timer.tamr.modify(|_, w| {
        w.taams().set_bit();
        w.tacmr().clear_bit();
        w.tamr().period();
        w
    });
    h_timer.tbmr.modify(|_, w| {
        w.tbams().set_bit();
        w.tbcmr().clear_bit();
        w.tbmr().period();
        w.tbmie().set_bit();
        w.tbpwmie().set_bit();
        w
    });
    h_timer.ctl.modify(|_, w| w.tapwml().clear_bit());
    // Trigger Timer B capture on falling edge (i.e. line start)
    h_timer.ctl.modify(|_, w| w.tbpwml().set_bit());
    h_timer.tapr.modify(|_, w| unsafe { w.bits(0) });
    h_timer.tbpr.modify(|_, w| unsafe { w.bits(0) });
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
    // TODO IMR.TBMIM = 1 (for line start)
    // TODO IMR.TBTOIM = 1 (for row count)
    h_timer.imr.modify(|_, w| {
        w.cbeim().set_bit(); // PWM triggers the capture event bit
        w
    });

    // Configure WT0 for v-sync in 32-bit mode
    let v_timer = p.WTIMER0;
    v_timer
        .ctl
        .modify(|_, w| w.taen().clear_bit().tben().clear_bit());
    v_timer.cfg.modify(|_, w| w.cfg()._16_bit());
    v_timer.tamr.modify(|_, w| {
        w.taams().set_bit();
        w.tacmr().clear_bit();
        w.tamr().period();
        w.tawot().set_bit();
        w
    });
    v_timer.ctl.modify(|_, w| w.tapwml().clear_bit());
    v_timer.tapr.modify(|_, w| unsafe { w.bits(0) });
    // We're counting down in PWM mode, so start at the end
    v_timer
        .tailr
        .modify(|_, w| unsafe { w.bits(V_WHOLE_FRAME * H_WHOLE_LINE - 1) });
    v_timer
        .tamatchr
        .modify(|_, w| unsafe { w.bits(V_SYNC_END * H_WHOLE_LINE - 1) });

    // Clear interrupts
    h_timer
        .icr
        .write(|w| w.tbmcint().set_bit().tbtocint().set_bit());

    v_timer.ctl.modify(|_, w| w.taen().set_bit());
    h_timer
        .ctl
        .modify(|_, w| w.taen().set_bit().tben().set_bit());

    // let mut itm = hio::hstdout().unwrap();
    // let mut d = Delay::new(cp.SYST, &clocks);
    // let mut last = 0;
    // loop {
    //     let a = unsafe { ISR_COUNT };
    //     writeln!(itm, "{}", a - last).unwrap();
    //     last = a;
    //     d.delay_ms(1000u32);
    // }
}

static mut ISR_COUNT: u32 = 0;

extern "C" fn timer0_isr() {
    let p = unsafe { tm4c123x_hal::Peripherals::steal() };
    p.SSI2.dr.write(|w| unsafe { w.data().bits(0x01) });
    p.SSI2.dr.write(|w| unsafe { w.data().bits(0x03) });
    p.SSI2.dr.write(|w| unsafe { w.data().bits(0x07) });
    p.SSI2.dr.write(|w| unsafe { w.data().bits(0x0F) });
    p.SSI2.dr.write(|w| unsafe { w.data().bits(0x1F) });
    p.SSI2.dr.write(|w| unsafe { w.data().bits(0x3F) });
    p.SSI2.dr.write(|w| unsafe { w.data().bits(0x7F) });
    p.SSI2.dr.write(|w| unsafe { w.data().bits(0xFF) });
    // p.GPIO_PORTB.data.write(|w| unsafe { w.bits(1 << 7) });
    // asm::nop();
    // asm::nop();
    // asm::nop();
    // asm::nop();
    // asm::nop();
    // p.GPIO_PORTB.data.write(|w| unsafe { w.bits(0) });
    unsafe {
        ISR_COUNT += 1;
    }
    p.TIMER0.icr.write(|w| w.cbecint().set_bit());
}

extern "C" fn default_handler() {
    let cp = unsafe { tm4c123x_hal::CorePeripherals::steal() };
    let mut interrupts: [u32; 16] = [0u32; 16];
    for (i, val) in interrupts.iter_mut().enumerate() {
        *val = cp.NVIC.iabr[i].read();
    }
    asm::bkpt();
}

// As we are not using interrupts, we just register a dummy catch all handler
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
    Some(default_handler),
    // 16/32 bit timer 0 B              36
    Some(timer0_isr),
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
