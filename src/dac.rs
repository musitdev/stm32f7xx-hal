use crate::device::RCC;
use stm32f7::stm32f7x6::DAC;

//DAC Errorx

#[derive(PartialEq, PartialOrd, Clone, Eq, Ord)]
pub enum DacWord {
    B8_ALIGN_R(u8),
    B12_ALIGN_R(u16),
    B12_ALIGN_L(u16),
}

/// Write DAC interface
///
/// Some DAC interfaces support different data sizes (8 bits, 12 bits, etc.) with different alignement.
/// data use DacWord type to define the world size and alignement. 
pub trait DacWriter {
    /// write a single word from the DAC interface using specified data
    fn write(&mut self, data: DacWord);
}

pub struct Dac {
    dac: DAC,
}

impl Dac {
    pub fn new(dac: DAC) -> Dac {
        //Init dac
        // NOTE(unsafe) This executes only during initialisation
        let rcc = unsafe { &(*RCC::ptr()) };
        rcc.apb1enr.modify(|_, w| w.dacen().enabled()); //enable dac

        Dac {
            dac: dac,
        }
    }

    pub fn create_dac_channel1(&self, _pin: crate::gpio::gpioa::PA4<crate::gpio::Output<crate::gpio::Analog>>) -> Dac1 {
        self.dac1()
    }

    pub fn dac1(&self) -> Dac1 {
        Dac1{dac:&self.dac}
    }

    pub fn create_dac_channel2(&self, _pin: crate::gpio::gpioa::PA5<crate::gpio::Output<crate::gpio::Analog>>) -> Dac2 {
        self.dac2()
    }

    pub fn dac2(&self) -> Dac2 {
        Dac2{dac:&self.dac}
    }
}

pub struct Dac1<'a> {
    dac: &'a DAC,
}

impl<'a> Dac1<'a> {

    pub fn enable(&mut self) {
        self.dac.cr.modify(|_, w| w.en1().enabled());
    }

    pub fn disable(&mut self) {
        self.dac.cr.modify(|_, w| w.en1().disabled());
    }

    pub fn enable_output_buffer(&mut self) {
        self.dac.cr.modify(|_, w| w. boff1().enabled());
    }

    pub fn disable_output_buffer(&mut self) {
        self.dac.cr.modify(|_, w| w. boff1().disabled());
    }

    pub fn enable_trigger(&mut self) {
        self.dac.cr.modify(|_, w| w.ten1().enabled());
    }

    pub fn disable_trigger(&mut self) {
        self.dac.cr.modify(|_, w| w.ten1().disabled());
    }

}

pub struct Dac2<'a> {
    dac: &'a DAC,
}

impl<'a> Dac2<'a> {

    pub fn enable(&mut self) {
        self.dac.cr.modify(|_, w| w.en2().enabled());
    }

    pub fn disable(&mut self) {
        self.dac.cr.modify(|_, w| w.en2().disabled());
    }

    pub fn enable_output_buffer(&mut self) {
        self.dac.cr.modify(|_, w| w. boff2().enabled());
    }

    pub fn disable_output_buffer(&mut self) {
        self.dac.cr.modify(|_, w| w. boff2().disabled());
    }

    pub fn enable_trigger(&mut self) {
        self.dac.cr.modify(|_, w| w.ten2().enabled());
    }

    pub fn disable_trigger(&mut self) {
        self.dac.cr.modify(|_, w| w.ten2().disabled());
    }

}

impl<'a> DacWriter for Dac1<'a> {

    fn write(&mut self, data: DacWord) {
        match data {
            DacWord::B8_ALIGN_R(d) => self.dac.dhr8r1.modify(|_, w| w.dacc1dhr().bits(d)),
            DacWord::B12_ALIGN_R(d) => self.dac.dhr12r1.modify(|_, w| unsafe {w.dacc1dhr().bits(d)}),
            DacWord::B12_ALIGN_L(d) => self.dac.dhr12l1.modify(|_, w| unsafe {w.dacc1dhr().bits(d)}),
        }
    }
}

impl<'a> DacWriter for Dac2<'a> {

    fn write(&mut self, data: DacWord) {
        match data {
            DacWord::B8_ALIGN_R(d) => self.dac.dhr8r2.modify(|_, w| w.dacc2dhr().bits(d)),
            DacWord::B12_ALIGN_R(d) => self.dac.dhr12r2.modify(|_, w| unsafe {w.dacc2dhr().bits(d)}),
            DacWord::B12_ALIGN_L(d) => self.dac.dhr12l2.modify(|_, w| unsafe {w.dacc2dhr().bits(d)}),
        }
    }
}

