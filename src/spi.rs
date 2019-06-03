use core::ptr;

use embedded_hal::spi;
pub use embedded_hal::spi::{Mode, Phase, Polarity};
use nb;

#[cfg(any(feature = "stm32f745", feature = "stm32f746",))]
use crate::device::{RCC, SPI1, SPI2};

#[cfg(any(feature = "stm32f745", feature = "stm32f746",))]
use crate::device::SPI3;

#[cfg(any(feature = "stm32f745", feature = "stm32f746",))]
use crate::device::SPI4;

#[cfg(any(feature = "stm32f745", feature = "stm32f746",))]
use crate::device::SPI5;

#[cfg(any(feature = "stm32f745", feature = "stm32f746",))]
use crate::device::SPI6;

#[cfg(any(feature = "stm32f745", feature = "stm32f746",))]
use crate::gpio::gpioa::PA9;
#[cfg(any(feature = "stm32f745", feature = "stm32f746",))]
use crate::gpio::gpioa::{PA5, PA6, PA7};

#[cfg(any(feature = "stm32f745", feature = "stm32f746",))]
use crate::gpio::gpiob::PB2;
#[cfg(any(feature = "stm32f745", feature = "stm32f746",))]
use crate::gpio::gpiob::{PB10, PB13, PB14, PB15, PB3, PB4, PB5};

#[cfg(any(feature = "stm32f745", feature = "stm32f746",))]
use crate::gpio::gpioc::PC1;
#[cfg(any(feature = "stm32f745", feature = "stm32f746",))]
use crate::gpio::gpioc::{PC10, PC11, PC12};
#[cfg(any(feature = "stm32f745", feature = "stm32f746",))]
use crate::gpio::gpioc::{PC2, PC3};

#[cfg(any(feature = "stm32f745", feature = "stm32f746",))]
use crate::gpio::gpiod::{PD3, PD6};

#[cfg(any(feature = "stm32f745", feature = "stm32f746",))]
use crate::gpio::gpioe::{PE12, PE13, PE14, PE2, PE5, PE6};

#[cfg(any(feature = "stm32f745", feature = "stm32f746",))]
use crate::gpio::gpiof::{PF11, PF7, PF8, PF9};

#[cfg(any(feature = "stm32f745", feature = "stm32f746",))]
use crate::gpio::gpiog::PG14;
#[cfg(any(feature = "stm32f745", feature = "stm32f746",))]
use crate::gpio::gpiog::{PG12, PG13};

#[cfg(any(feature = "stm32f745", feature = "stm32f746",))]
use crate::gpio::gpioh::{PH6, PH7};

#[cfg(any(feature = "stm32f745", feature = "stm32f746",))]
use crate::gpio::gpioi::{PI1, PI2, PI3};

#[cfg(any(feature = "stm32f745", feature = "stm32f746",))]
use crate::gpio::AF7;
#[cfg(any(feature = "stm32f745", feature = "stm32f746",))]
use crate::gpio::{Alternate, AF5, AF6};

use crate::rcc::Clocks;
use crate::time::Hertz;

/// SPI error
#[derive(Debug)]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Mode fault occurred
    ModeFault,
    /// CRC error
    Crc,
    EndTranscationReadError,
    #[doc(hidden)]
    _Extensible,
}

pub trait Pins<SPI> {}
pub trait PinSck<SPI> {}
pub trait PinMiso<SPI> {}
pub trait PinMosi<SPI> {}

impl<SPI, SCK, MISO, MOSI> Pins<SPI> for (SCK, MISO, MOSI)
where
    SCK: PinSck<SPI>,
    MISO: PinMiso<SPI>,
    MOSI: PinMosi<SPI>,
{
}

/// A filler type for when the SCK pin is unnecessary
pub struct NoSck;
/// A filler type for when the Miso pin is unnecessary
pub struct NoMiso;
/// A filler type for when the Mosi pin is unnecessary
pub struct NoMosi;

macro_rules! pins {
    ($($SPIX:ty: SCK: [$($SCK:ty),*] MISO: [$($MISO:ty),*] MOSI: [$($MOSI:ty),*])+) => {
        $(
            $(
                impl PinSck<$SPIX> for $SCK {}
            )*
            $(
                impl PinMiso<$SPIX> for $MISO {}
            )*
            $(
                impl PinMosi<$SPIX> for $MOSI {}
            )*
        )+
    }
}

#[cfg(any(feature = "stm32f745", feature = "stm32f746",))]
pins! {
    //SPI1 *AF5
 // NSN  PA4* PA15* PB4/AF7
 // MOSI PA7* PB5*
 // MISO PA6* PB4*
 // SCK  PA5* PB3*

    SPI1:
        SCK: [
            NoSck,
            PA5<Alternate<AF5>>,
            PB3<Alternate<AF5>>
        ]
        MISO: [
            NoMiso,
            PA6<Alternate<AF5>>,
            PB4<Alternate<AF5>>
        ]
        MOSI: [
            NoMosi,
            PA7<Alternate<AF5>>,
            PB5<Alternate<AF5>>
        ]
    //SPI2
 // NSN  PB9*  PB12* PI0*
 // MOSI PB15* PC1*  PC3* PI3*
 // MISO PB14* PC2*  PI2*
 // SCK  PA9*  PB10* PB13* PD3* PI1*

    // PI3     ------> SPI2_MOSI
  //PD3     ------> SPI2_SCK

    SPI2:
        SCK: [
            NoSck,
            PA9<Alternate<AF5>>,
            PB10<Alternate<AF5>>,
            PB13<Alternate<AF5>>,
            PD3<Alternate<AF5>>,
            PI1<Alternate<AF5>>
        ]
        MISO: [
            NoMiso,
            PB14<Alternate<AF5>>,
            PC2<Alternate<AF5>>,
            PI2<Alternate<AF5>>
        ]
        MOSI: [
            NoMosi,
            PB15<Alternate<AF5>>,
            PC1<Alternate<AF5>>,
            PC3<Alternate<AF5>>,
            PI3<Alternate<AF5>>
        ]
 // NSN PA4/AF6 PA15/AF6
 // MOSI PB2/AF7  PB5/AF6 PC12/AF6 PD6*
 // MISO PB4/AF6 PC11/AF6
 //SCK PB3/AF6 PC10/AF6
    SPI3:
        SCK: [
            NoSck,
            PB3<Alternate<AF6>>,
            PC10<Alternate<AF6>>
        ]
        MISO: [
            NoMiso,
            PB4<Alternate<AF6>>,
            PC11<Alternate<AF6>>
        ]
        MOSI: [
            NoMosi,
            PB2<Alternate<AF7>>,
            PB5<Alternate<AF6>>,
            PC12<Alternate<AF6>>,
            PD6<Alternate<AF5>>
        ]

 // NSN  PE4* PE11*
 // MOSI PE6* PE14*
 // MISO PE5* PE13*
 //SCK PE2* PE12*
    SPI4:
        SCK: [
            NoSck,
            PE2<Alternate<AF5>>,
            PE12<Alternate<AF5>>
        ]
        MISO: [
            NoMiso,
            PE5<Alternate<AF5>>,
            PE13<Alternate<AF5>>
        ]
        MOSI: [
            NoMosi,
            PE6<Alternate<AF5>>,
            PE14<Alternate<AF5>>
        ]

 // NSN  PF6* PH5*
 // MOSI PF9* PF11*
 // MISO PF8* PH7*
 //SCK PF7* PH6*
    SPI5:
        SCK: [
            NoSck,
            PF7<Alternate<AF5>>,
            PH6<Alternate<AF5>>
        ]
        MISO: [
            NoMiso,
            PF8<Alternate<AF5>>,
            PH7<Alternate<AF5>>
        ]
        MOSI: [
            NoMosi,
            PF9<Alternate<AF5>>,
            PF11<Alternate<AF5>>
        ]

 // NSN  PG8*
 // MOSI PG14*
 // MISO PG12*
 //SCK PG13*
    SPI6:
        SCK: [
            NoSck,
            PG13<Alternate<AF5>>
        ]
        MISO: [
            NoMiso,
            PG12<Alternate<AF5>>
        ]
        MOSI: [
            NoMosi,
            PG14<Alternate<AF5>>
        ]
}

/// Interrupt events
pub enum Event {
    /// New data has been received
    Rxne,
    /// Data can be sent
    Txe,
    /// An error occurred
    Error,
}

#[derive(Debug)]
pub struct Spi<SPI, PINS> {
    spi: SPI,
    pins: PINS,
}

macro_rules! hal {
    ($($SPIX:ident: ($spiX:ident, $apbXenr:ident, $spiXen:ident, $pclkX:ident),)+) => {
        $(
            impl<PINS> Spi<$SPIX, PINS> {
                pub fn $spiX(
                    spi: $SPIX,
                    pins: PINS,
                    mode: Mode,
                    freq: Hertz,
                    clocks: Clocks
                ) -> Self
                where PINS: Pins<$SPIX> {
                    // NOTE(unsafe) This executes only during initialisation
                    let rcc = unsafe { &(*RCC::ptr()) };

                    // Enable clock for SPI
                    rcc.$apbXenr.modify(|_, w| w.$spiXen().set_bit());

                    // disable SS output
                    spi.cr2.write(|w| w.ssoe().clear_bit());

                    let _br = match clocks.$pclkX().0 / freq.0 {
                        0 => unreachable!(),
                        1...2 => 0b000,
                        3...5 => 0b001,
                        6...11 => 0b010,
                        12...23 => 0b011,
                        24...47 => 0b100,
                        48...95 => 0b101,
                        96...191 => 0b110,
                        _ => 0b111,
                    };

                    // mstr: master configuration
                    // lsbfirst: MSB first
                    // SPI_CRCCALCULATION_DISABLE crsen().disabled()
                    //CRCLength cdcl().sixteen_bit()
                    // ssm: enable software slave management (NSS pin free for other uses)
                    // ssi: set nss high = master mode
                    // dff: 8 bit frames
                    // bidimode: 2-line unidirectional
                    // spe: enable the SPI bus
                    spi.cr1.write(|w| {
                        w.cpha()
                            .bit(mode.phase == Phase::CaptureOnSecondTransition)
                            .cpol()
                            .bit(mode.polarity == Polarity::IdleHigh)
                            .mstr()
                            .set_bit()
                            .crcen()
                            .disabled()
                            //.cdcl()
                            //.sixteen_bit()
                            .br()
                            .div256()// .bits(br)
                            .lsbfirst()
                            .msbfirst() // clear_bit()
                            .ssm()
                            .set_bit()
                            .ssi()
                            .set_bit()
                            .rxonly()
                            .clear_bit() //.clear_bit()
                            .bidimode()
                            .clear_bit()
                            .spe()
                            .set_bit()
                    });

                    //NSS pulse management disable
                    //  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
                    spi.cr2.write(|w| {
                        w.nssp()
                            .no_pulse()
                            .frf()
                            .motorola()
                            .ds()
                            .sixteen_bit()
                    });

                    Spi { spi, pins }
                }

                /// Enable interrupts for the given `event`:
                ///  - Received data ready to be read (RXNE)
                ///  - Transmit data register empty (TXE)
                ///  - Transfer error
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::Rxne  => self.spi.cr2.modify(|_, w| { w.rxneie().set_bit() }),
                        Event::Txe   => self.spi.cr2.modify(|_, w| { w.txeie().set_bit() }),
                        Event::Error => self.spi.cr2.modify(|_, w| { w.errie().set_bit() }),
                    }
                }

                /// Disable interrupts for the given `event`:
                ///  - Received data ready to be read (RXNE)
                ///  - Transmit data register empty (TXE)
                ///  - Transfer error
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::Rxne  => self.spi.cr2.modify(|_, w| { w.rxneie().clear_bit() }),
                        Event::Txe   => self.spi.cr2.modify(|_, w| { w.txeie().clear_bit() }),
                        Event::Error => self.spi.cr2.modify(|_, w| { w.errie().clear_bit() }),
                    }
                }

                /// Return `true` if the TXE flag is set, i.e. new data to transmit
                /// can be written to the SPI.
                pub fn is_txe(&self) -> bool {
                    self.spi.sr.read().txe().bit_is_set()
                }

                /// Return `true` if the RXNE flag is set, i.e. new data has been received
                /// and can be read from the SPI.
                pub fn is_rxne(&self) -> bool {
                    self.spi.sr.read().rxne().bit_is_set()
                }

                /// Return `true` if the MODF flag is set, i.e. the SPI has experienced a
                /// Master Mode Fault. (see chapter 28.3.10 of the STM32F4 Reference Manual)
                pub fn is_modf(&self) -> bool {
                    self.spi.sr.read().modf().bit_is_set()
                }

                /// Return `true` if the OVR flag is set, i.e. new data has been received
                /// while the receive data register was already filled.
                pub fn is_ovr(&self) -> bool {
                    self.spi.sr.read().ovr().bit_is_set()
                }

                pub fn free(self) -> ($SPIX, PINS) {
                    (self.spi, self.pins)
                }

               pub fn send_only_16b(&mut self, valeur_dac: u16) -> nb::Result<(), Error> {
                    let sr = self.spi.sr.read();

                    Err(if sr.ovr().bit_is_set() {
                        nb::Error::Other(Error::Overrun)
                    } else if sr.modf().bit_is_set() {
                        nb::Error::Other(Error::ModeFault)
                    } else if sr.crcerr().bit_is_set() {
                        nb::Error::Other(Error::Crc)
                    } else if sr.txe().bit_is_set() {
                        // NOTE(write_volatile) see note above
                        unsafe {
                            ptr::write_volatile(&self.spi.dr as *const _ as *mut u16, valeur_dac as u16);
                        }
                        while sr.bsy().bit_is_set() {}
                        //write to dr and bsy end before the real send of data. Add a way time to be sure all data are send.
                        //very ugly. Don't find a better way to synchronise call output with real data send end.
                        //must be changed if send speed change. tested with a div256
                        for _ in 0..6 {
                            unsafe {
                                //read one time at the end to end the transation and way the end of the send
                                //work only for the first bit.
                                ptr::read_volatile(&self.spi.dr as *const _ as *const u16);
                            }
                        }
                        return Ok(());
                    } else {
                        nb::Error::WouldBlock
                    })
                }
            }

            impl<PINS> spi::FullDuplex<u8> for Spi<$SPIX, PINS> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    let sr = self.spi.sr.read();

                    Err(if sr.ovr().bit_is_set() {
                        nb::Error::Other(Error::Overrun)
                    } else if sr.modf().bit_is_set() {
                        nb::Error::Other(Error::ModeFault)
                    } else if sr.crcerr().bit_is_set() {
                        nb::Error::Other(Error::Crc)
                    } else if sr.rxne().bit_is_set() {
                        // NOTE(read_volatile) read only 1 byte (the svd2rust API only allows
                        // reading a half-word)
                        return Ok(unsafe {
                            ptr::read_volatile(&self.spi.dr as *const _ as *const u8)
                        });
                    } else {
                        nb::Error::WouldBlock
                    })
                }

                fn send(&mut self, byte: u8) -> nb::Result<(), Error> {
                    let sr = self.spi.sr.read();

                    if sr.ovr().bit_is_set() {
                        Err(nb::Error::Other(Error::Overrun))
                    } else if sr.modf().bit_is_set() {
                        Err(nb::Error::Other(Error::ModeFault))
                    } else if sr.crcerr().bit_is_set() {
                        Err(nb::Error::Other(Error::Crc))
                    } else if sr.txe().bit_is_set() {
                        // NOTE(write_volatile) see note above
                        unsafe {
                            ptr::write_volatile(&self.spi.dr as *const _ as *mut u8, byte);
                         }
                        while sr.bsy().bit_is_set() {}
                        Ok(())
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                }
            }

            impl<PINS> embedded_hal::blocking::spi::transfer::Default<u8> for Spi<$SPIX, PINS> {}

            impl<PINS> embedded_hal::blocking::spi::write::Default<u8> for Spi<$SPIX, PINS> {}
        )+
    }
}

#[cfg(any(feature = "stm32f745", feature = "stm32f746",))]
hal! {
    SPI1: (spi1, apb2enr, spi1en, pclk2),
    SPI2: (spi2, apb1enr, spi2en, pclk1),
}
#[cfg(any(feature = "stm32f745", feature = "stm32f746",))]
hal! {
    SPI3: (spi3, apb1enr, spi3en, pclk1),
}
#[cfg(any(feature = "stm32f745", feature = "stm32f746",))]
hal! {
    SPI4: (spi4, apb2enr, spi4enr, pclk2),
}
#[cfg(any(feature = "stm32f745", feature = "stm32f746",))]
hal! {
    SPI5: (spi5, apb2enr, spi5enr, pclk2),
}
#[cfg(any(feature = "stm32f745", feature = "stm32f746",))]
hal! {
    SPI6: (spi6, apb2enr, spi6enr, pclk2),
}
