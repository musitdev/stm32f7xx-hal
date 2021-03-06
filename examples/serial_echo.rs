//! Echo characters sent back to the serial port.
//!
//! Note: This example is for the STM32F745/STM32F746

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_halt;

use nb::block;

use stm32f7xx_hal::{
    prelude::*,
    device,
    serial::{
        self,
        Serial,
    },
};
use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    let p = device::Peripherals::take().unwrap();

    let rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(216.mhz()).freeze();

    let gpioa = p.GPIOA.split();
    let gpiob = p.GPIOB.split();

    let tx = gpioa.pa9.into_alternate_af7();
    let rx = gpiob.pb7.into_alternate_af7();

    let serial = Serial::usart1(
        p.USART1,
        (tx, rx),
        115_200.bps(),
        clocks,
        false,
    );

    let (mut tx, mut rx) = serial.split();

    loop {
        let received = block!(rx.read()).unwrap_or('E' as u8);
        block!(tx.write(received)).ok();
    }
}

