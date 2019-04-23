<<<<<<< b4e16860317f1dbe3739e0c55b8184cfb96e1e81
//! Interface to the SPI peripheral
//!
//! See chapter 32 in the STM32F746 Reference Manual.


pub use embedded_hal::spi::{
    Mode,
    Phase,
    Polarity,
};
pub use crate::device::spi1::cr1::BRW as ClockDivider;


use core::{
    fmt,
    marker::PhantomData,
    ops::DerefMut,
    pin::Pin,
    ptr,
};

use as_slice::{
    AsMutSlice,
    AsSlice as _,
};
use embedded_hal::{
    blocking::spi::{
        transfer,
        write,
        write_iter,
    },
    spi::FullDuplex,
};

use crate::{
    device::{
        spi1::cr2,
        SPI1,
        SPI2,
        SPI3,
        SPI4,
        SPI5,
        SPI6,
    },
    dma,
    gpio::{
        Alternate,
        AF5,
        AF6,
        AF7,
        gpioa::{
            PA5,
            PA6,
            PA7,
            PA9,
        },
        gpiob::{
            PB2,
            PB3,
            PB4,
            PB5,
            PB10,
            PB13,
            PB14,
            PB15,
        },
        gpioc::{
            PC1,
            PC2,
            PC3,
            PC10,
            PC11,
            PC12,
        },
        gpiod::{
            PD3,
            PD6,
        },
        gpioe::{
            PE2,
            PE5,
            PE6,
            PE12,
            PE13,
            PE14,
        },
        gpiof::{
            PF7,
            PF8,
            PF9,
            PF11,
        },
        gpiog::{
            PG12,
            PG13,
            PG14,
        },
        gpioh::{
            PH6,
            PH7,
        },
        gpioi::{
            PI1,
            PI2,
            PI3,
        },
    },
    rcc::Rcc,
    state,
};


/// Entry point to the SPI API
pub struct Spi<I, P, State> {
    spi:    I,
    pins:   P,
    _state: State,
}

impl<I, P> Spi<I, P, state::Disabled>
    where
        I: Instance,
        P: Pins<I>,
{
    /// Create a new instance of the SPI API
    pub fn new(instance:  I, pins: P) -> Self {
        Self {
            spi:    instance,
            pins,
            _state: state::Disabled,
        }
    }

    /// Initialize the SPI peripheral
    pub fn enable<Word>(self,
        rcc:           &mut Rcc,
        clock_divider: ClockDivider,
        mode:          Mode,
    )
        -> Spi<I, P, Enabled<Word>>
        where Word: SupportedWordSize
    {
        let cpol = mode.polarity == Polarity::IdleHigh;
        let cpha = mode.phase == Phase::CaptureOnSecondTransition;

        self.spi.enable_clock(rcc);
        self.spi.configure::<Word>(clock_divider._bits(), cpol, cpha);

        Spi {
            spi:    self.spi,
            pins:   self.pins,
            _state: Enabled(PhantomData),
        }
    }
}

impl<I, P, Word> Spi<I, P, Enabled<Word>>
    where
        I:    Instance,
        P:    Pins<I>,
        Word: SupportedWordSize,
{
    /// Start an SPI transfer using DMA
    ///
    /// Sends the data in `buffer` and writes the received data into buffer
    /// right after. Returns a [`Transfer`], to represent the ongoing SPI
    /// transfer.
    ///
    /// Please note that the word "transfer" is used with two different meanings
    /// here:
    /// - An SPI transfer, as in an SPI transaction that involves both sending
    ///   and receiving data. The method name refers to this kind of transfer.
    /// - A DMA transfer, as in an ongoing DMA operation. The name of the return
    ///   type refers to this kind of transfer.
    ///
    /// This method, as well as all other DMA-related methods in this module,
    /// requires references to two DMA handles, one each for the RX and TX
    /// streams. This will actually always be the same handle, as each SPI
    /// instance uses the same DMA instance for both sending and receiving. It
    /// would be nice to simplify that, but I believe that requires an equality
    /// constraint in the where clause, which is not supported yet by the
    /// compiler.
    pub fn transfer_all<B>(self,
        buffer: Pin<B>,
        dma_rx: &dma::Handle<<Rx<I> as dma::Target>::Instance, state::Enabled>,
        dma_tx: &dma::Handle<<Tx<I> as dma::Target>::Instance, state::Enabled>,
        rx:     <Rx<I> as dma::Target>::Stream,
        tx:     <Tx<I> as dma::Target>::Stream,
    )
        -> Transfer<Word, I, P, B, Rx<I>, Tx<I>, dma::Ready>
        where
            Rx<I>:     dma::Target,
            Tx<I>:     dma::Target,
            B:         DerefMut + 'static,
            B::Target: AsMutSlice<Element=Word>,
    {
        // Create the RX/TX tokens for the transfer. Those must only exist once,
        // otherwise it would be possible to create multiple transfers trying to
        // use the same hardware resources.
        //
        // We guarantee that they only exist once by only creating them where we
        // have access to `self`, moving `self` into the `Transfer` while they
        // are in use, and dropping them when returning `self` from the
        // transfer.
        let rx_token = Rx(PhantomData);
        let tx_token = Tx(PhantomData);

        // We need to move a buffer into each of the `dma::Transfer` instances,
        // while keeping the original buffer around to return to the caller
        // later, when the transfer is finished.
        //
        // Here we create two `Buffer` from raw pointers acquired from `buffer`.
        let rx_buffer = dma::PtrBuffer {
            ptr: buffer.as_slice().as_ptr(),
            len: buffer.as_slice().len(),
        };
        let tx_buffer = dma::PtrBuffer {
            ptr: buffer.as_slice().as_ptr(),
            len: buffer.as_slice().len(),
        };

        // Create the two DMA transfers. This is safe, for the following
        // reasons:
        // 1. The trait bounds on this method guarantee that `buffer`, which we
        //    created the two buffer instances from, can be safely read from and
        //    written to.
        // 2. The semantics of the SPI peripheral guarantee that the buffer
        //    reads/writes are synchronized, preventing race conditions.
        let rx_transfer = unsafe {
            dma::Transfer::new(
                dma_rx,
                rx,
                Pin::new(rx_buffer),
                rx_token,
                self.spi.dr_address(),
                dma::Direction::PeripheralToMemory,
            )
        };
        let tx_transfer = unsafe {
            dma::Transfer::new(
                dma_tx,
                tx,
                Pin::new(tx_buffer),
                tx_token,
                self.spi.dr_address(),
                dma::Direction::MemoryToPeripheral,
            )
        };

        Transfer {
            buffer,
            target: self,

            rx: rx_transfer,
            tx: tx_transfer,

            _state: dma::Ready,
        }
    }
}

impl<I, P, Word> FullDuplex<Word> for Spi<I, P, Enabled<Word>>
    where
        I:    Instance,
        P:    Pins<I>,
        Word: SupportedWordSize,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<Word, Self::Error> {
        self.spi.read()
    }

    fn send(&mut self, word: Word) -> nb::Result<(), Self::Error> {
        self.spi.send(word)
    }
}

impl<I, P, Word> transfer::Default<Word> for Spi<I, P, Enabled<Word>>
    where
        I:    Instance,
        P:    Pins<I>,
        Word: SupportedWordSize,
{}

impl<I, P, Word> write::Default<Word> for Spi<I, P, Enabled<Word>>
    where
        I:    Instance,
        P:    Pins<I>,
        Word: SupportedWordSize,
{}

impl<I, P, Word> write_iter::Default<Word> for Spi<I, P, Enabled<Word>>
    where
        I:    Instance,
        P:    Pins<I>,
        Word: SupportedWordSize,
{}

impl<I, P, State> Spi<I, P, State>
    where
        I: Instance,
        P: Pins<I>,
{
    /// Destroy the peripheral API and return a raw SPI peripheral instance
    pub fn free(self) -> (I, P) {
        (self.spi, self.pins)
    }
}


/// Implemented for all instances of the SPI peripheral
///
/// Users of this crate should not implement this trait.
pub trait Instance {
    fn enable_clock(&self, rcc: &mut Rcc);
    fn configure<Word>(&self, br: u8, cpol: bool, cpha: bool)
        where Word: SupportedWordSize;
    fn read<Word>(&self) -> nb::Result<Word, Error>
        where Word: SupportedWordSize;
    fn send<Word>(&self, word: Word) -> nb::Result<(), Error>
        where Word: SupportedWordSize;
    fn dr_address(&self) -> u32;
}

/// Implemented for all tuples that contain a full set of valid SPI pins
pub trait Pins<I> {}

impl<I, SCK, MISO, MOSI> Pins<I> for (SCK, MISO, MOSI)
    where
        SCK:  Sck<I>,
        MISO: Miso<I>,
        MOSI: Mosi<I>,
{}

/// Implemented for all pins that can function as the SCK pin
///
/// Users of this crate should not implement this trait.
pub trait Sck<I> {}

/// Implemented for all pins that can function as the MISO pin
///
/// Users of this crate should not implement this trait.
pub trait Miso<I> {}

/// Implemented for all pins that can function as the MOSI pin
///
/// Users of this crate should not implement this trait.
pub trait Mosi<I> {}

macro_rules! impl_instance {
    (
        $(
            $name:ty {
                regs: ($bus:ident, $reset:ident, $enable:ident),
                pins: {
                    SCK: [$($sck:ty,)*],
                    MISO: [$($miso:ty,)*],
                    MOSI: [$($mosi:ty,)*],
                }
            }
        )*
    ) => {
        $(
            impl Instance for $name {
                fn enable_clock(&self, rcc: &mut Rcc) {
                    rcc.$bus.rstr().modify(|_, w| w.$reset().clear_bit());
                    rcc.$bus.enr().modify(|_, w| w.$enable().enabled());
                }

                // I don't like putting this much code into the macro, but I
                // have to: There are two different SPI variants in the PAC, and
                // while I haven't found any actual differences between them,
                // they're still using different sets of types, and I need to
                // generate different methods to interface with them, even
                // though these methods end up looking identical.
                //
                // Maybe this is a problem in the SVD file that can be fixed
                // there.

                fn configure<Word>(&self, br: u8, cpol: bool, cpha: bool)
                    where Word: SupportedWordSize
                {
                    self.cr2.write(|w| {
                        // Data size
                        //
                        // This is safe, as `Word::ds` returns an enum which can
                        // only encode valid variants for this field.
                        let w = unsafe { w.ds().bits(Word::ds()._bits()) };

                        w
                            // FIFO reception threshold.
                            .frxth().bit(Word::frxth()._bits())
                            // Disable TX buffer empty interrupt
                            .txeie().masked()
                            // Disable RX buffer not empty interrupt
                            .rxneie().masked()
                            // Disable error interrupt
                            .errie().masked()
                            // Frame format
                            .frf().motorola()
                            // NSS pulse management
                            .nssp().no_pulse()
                            // SS output
                            .ssoe().disabled()
                            // Enable DMA support
                            .txdmaen().enabled()
                            .rxdmaen().enabled()
                    });

                    self.cr1.write(|w|
                        w
                            // Use two lines for MISO/MOSI
                            .bidimode().unidirectional()
                            // Disable hardware CRC calculation
                            .crcen().disabled()
                            // Enable full-duplex mode
                            .rxonly().full_duplex()
                            // Manage slave select pin manually
                            .ssm().enabled()
                            .ssi().set_bit()
                            // Transmit most significant bit first
                            .lsbfirst().msbfirst()
                            // Set baud rate value
                            .br().bits(br)
                            // Select master mode
                            .mstr().master()
                            // Select clock polarity
                            .cpol().bit(cpol)
                            // Select clock phase
                            .cpha().bit(cpha)
                            // Enable SPI
                            .spe().enabled()
                    );
                }

                fn read<Word>(&self) -> nb::Result<Word, Error> {
                    let sr = self.sr.read();

                    // Check for errors
                    //
                    // This whole code should live in a method in `Error`, but
                    // this wouldn't be straight-forward, due to the different
                    // SPI types in the PAC, explained in more detail in
                    // another comment.
                    if sr.fre().is_error() {
                        return Err(nb::Error::Other(Error::FrameFormat));
                    }
                    if sr.ovr().is_overrun() {
                        return Err(nb::Error::Other(Error::Overrun));
                    }
                    if sr.modf().is_fault() {
                        return Err(nb::Error::Other(Error::ModeFault));
                    }

                    // Did we receive something?
                    if sr.rxne().is_not_empty() {
                        // It makes a difference whether we read this register
                        // as a `u8` or `u16`, so we can't use the standard way
                        // to access it. This is safe, as `&self.dr` is a
                        // memory-mapped register.
                        let value = unsafe {
                            ptr::read_volatile(
                                &self.dr as *const _ as *const _,
                            )
                        };

                        return Ok(value);
                    }

                    Err(nb::Error::WouldBlock)
                }

                fn send<Word>(&self, word: Word) -> nb::Result<(), Error> {
                    let sr = self.sr.read();

                    // Check for errors
                    //
                    // This whole code should live in a method in `Error`, but
                    // this wouldn't be straight-forward, due to the different
                    // SPI types in the PAC, explained in more detail in
                    // another comment.
                    if sr.fre().is_error() {
                        return Err(nb::Error::Other(Error::FrameFormat));
                    }
                    if sr.ovr().is_overrun() {
                        return Err(nb::Error::Other(Error::Overrun));
                    }
                    if sr.modf().is_fault() {
                        return Err(nb::Error::Other(Error::ModeFault));
                    }

                    // Can we write to the transmit buffer?
                    if sr.txe().is_empty() {
                        // It makes a difference whether we write a `u8` or
                        // `u16` to this register, so we can't use the standard
                        // way to access it. This is safe, as `&self.dr` is a
                        // memory-mapped register.
                        unsafe {
                            ptr::write_volatile(
                                &self.dr as *const _ as *mut _,
                                word,
                            );
                        }

                        return Ok(())
                    }

                    Err(nb::Error::WouldBlock)
                }

                fn dr_address(&self) -> u32 {
                    &self.dr as *const _ as _
                }
            }

            $(
                impl Sck<$name> for $sck {}
            )*

            $(
                impl Miso<$name> for $miso {}
            )*

            $(
                impl Mosi<$name> for $mosi {}
            )*
        )*
    }
}

impl_instance!(
    SPI1 {
        regs: (apb2, spi1rst, spi1en),
        pins: {
            SCK: [
                PA5<Alternate<AF5>>,
                PB3<Alternate<AF5>>,
            ],
            MISO: [
                PA6<Alternate<AF5>>,
                PB4<Alternate<AF5>>,
            ],
            MOSI: [
                PA7<Alternate<AF5>>,
                PB5<Alternate<AF5>>,
            ],
        }
    }
    SPI2 {
        regs: (apb1, spi2rst, spi2en),
        pins: {
            SCK: [
                PA9<Alternate<AF5>>,
                PB10<Alternate<AF5>>,
                PB13<Alternate<AF5>>,
                PD3<Alternate<AF5>>,
                PI1<Alternate<AF5>>,
            ],
            MISO: [
                PB14<Alternate<AF5>>,
                PC2<Alternate<AF5>>,
                PI2<Alternate<AF5>>,
            ],
            MOSI: [
                PB15<Alternate<AF5>>,
                PC1<Alternate<AF5>>,
                PC3<Alternate<AF5>>,
                PI3<Alternate<AF5>>,
            ],
        }
    }
    SPI3 {
        regs: (apb1, spi3rst, spi3en),
        pins: {
            SCK: [
                PB3<Alternate<AF6>>,
                PC10<Alternate<AF6>>,
            ],
            MISO: [
                PB4<Alternate<AF6>>,
                PC11<Alternate<AF6>>,
            ],
            MOSI: [
                PB2<Alternate<AF7>>,
                PB5<Alternate<AF6>>,
                PC12<Alternate<AF6>>,
                PD6<Alternate<AF5>>,
            ],
        }
    }
    SPI4 {
        regs: (apb2, spi4rst, spi4en),
        pins: {
            SCK: [
                PE2<Alternate<AF5>>,
                PE12<Alternate<AF5>>,
            ],
            MISO: [
                PE5<Alternate<AF5>>,
                PE13<Alternate<AF5>>,
            ],
            MOSI: [
                PE6<Alternate<AF5>>,
                PE14<Alternate<AF5>>,
            ],
        }
    }
    SPI5 {
        regs: (apb2, spi5rst, spi5en),
        pins: {
            SCK: [
                PF7<Alternate<AF5>>,
                PH6<Alternate<AF5>>,
            ],
            MISO: [
                PF8<Alternate<AF5>>,
                PH7<Alternate<AF5>>,
            ],
            MOSI: [
                PF9<Alternate<AF5>>,
                PF11<Alternate<AF5>>,
            ],
        }
    }
    SPI6 {
        regs: (apb2, spi6rst, spi6en),
        pins: {
            SCK: [
                PG13<Alternate<AF5>>,
            ],
            MISO: [
                PG12<Alternate<AF5>>,
            ],
            MOSI: [
                PG14<Alternate<AF5>>,
            ],
        }
    }
);


/// Placeholder for a pin when no SCK pin is required
pub struct NoSck;
impl<I> Sck<I> for NoSck {}

/// Placeholder for a pin when no MISO pin is required
pub struct NoMiso;
impl<I> Miso<I> for NoMiso {}

/// Placeholder for a pin when no MOSI pin is required
pub struct NoMosi;
impl<I> Mosi<I> for NoMosi {}


#[derive(Debug)]
pub enum Error {
    FrameFormat,
    Overrun,
    ModeFault,
}


/// RX token used for DMA transfers
pub struct Rx<I>(PhantomData<I>);

/// TX token used for DMA transfers
pub struct Tx<I>(PhantomData<I>);


/// A DMA transfer of the SPI peripheral
///
/// Since DMA can send and receive at the same time, using two DMA transfers and
/// two DMA streams, we need this type to represent this operation and wrap the
/// underlying [`dma::Transfer`] instances.
pub struct Transfer<Word: SupportedWordSize, I, P, Buffer, Rx: dma::Target, Tx: dma::Target, State> {
    buffer: Pin<Buffer>,
    target: Spi<I, P, Enabled<Word>>,
    rx:     dma::Transfer<Rx, dma::PtrBuffer<Word>, State>,
    tx:     dma::Transfer<Tx, dma::PtrBuffer<Word>, State>,
    _state: State,
}

impl<Word, I, P, Buffer, Rx, Tx>
    Transfer<Word, I, P, Buffer, Rx, Tx, dma::Ready>
    where
        Rx:   dma::Target,
        Tx:   dma::Target,
        Word: SupportedWordSize,
{
    /// Enables the given interrupts for this DMA transfer
    ///
    /// These interrupts are only enabled for this transfer. The settings
    /// doesn't affect other transfers, nor subsequent transfers using the same
    /// DMA streams.
    pub fn enable_interrupts(&mut self,
        rx_handle:  &dma::Handle<Rx::Instance, state::Enabled>,
        tx_handle:  &dma::Handle<Tx::Instance, state::Enabled>,
        interrupts: dma::Interrupts,
    ) {
        self.rx.enable_interrupts(rx_handle, interrupts);
        self.tx.enable_interrupts(tx_handle, interrupts);
    }

    /// Start the DMA transfer
    ///
    /// Consumes this instance of `Transfer` and returns another instance with
    /// its type state set to indicate the transfer has been started.
    pub fn start(self,
        rx_handle:  &dma::Handle<Rx::Instance, state::Enabled>,
        tx_handle:  &dma::Handle<Tx::Instance, state::Enabled>,
    )
        -> Transfer<Word, I, P, Buffer, Rx, Tx, dma::Started>
    {
        Transfer {
            buffer: self.buffer,
            target: self.target,
            rx:     self.rx.start(rx_handle),
            tx:     self.tx.start(tx_handle),
            _state: dma::Started,
        }
    }
}

impl<Word, I, P, Buffer, Rx, Tx>
    Transfer<Word, I, P, Buffer, Rx, Tx, dma::Started>
    where
        Rx:   dma::Target,
        Tx:   dma::Target,
        Word: SupportedWordSize,
{
    /// Checks whether the transfer is still ongoing
    pub fn is_active(&self,
        rx_handle:  &dma::Handle<Rx::Instance, state::Enabled>,
        tx_handle:  &dma::Handle<Tx::Instance, state::Enabled>,
    )
        -> bool
    {
        self.rx.is_active(rx_handle) || self.tx.is_active(tx_handle)
    }

    /// Waits for the transfer to end
    ///
    /// This method will block if the transfer is still ongoing. If you want
    /// this method to return immediately, first check whether the transfer is
    /// still ongoing by calling `is_active`.
    ///
    /// An ongoing transfer needs exlusive access to some resources, namely the
    /// data buffer, the DMA stream, and the peripheral. Those have been moved
    /// into the `Transfer` instance to prevent concurrent access to them. This
    /// method returns those resources, so they can be used again.
    pub fn wait(self,
        rx_handle:  &dma::Handle<Rx::Instance, state::Enabled>,
        tx_handle:  &dma::Handle<Tx::Instance, state::Enabled>,
    )
        -> Result<
            TransferResources<Word, I, P, Rx, Tx, Buffer>,
            (TransferResources<Word, I, P, Rx, Tx, Buffer>, dma::Error)
        >
    {
        let (rx_res, rx_err) = match self.rx.wait(rx_handle) {
            Ok(res)         => (res, None),
            Err((res, err)) => (res, Some(err)),
        };
        let (tx_res, tx_err) = match self.tx.wait(tx_handle) {
            Ok(res)         => (res, None),
            Err((res, err)) => (res, Some(err)),
        };

        let res = TransferResources {
            rx_stream: rx_res.stream,
            tx_stream: tx_res.stream,
            target:    self.target,
            buffer:    self.buffer,
        };

        if let Some(err) = rx_err {
            return Err((res, err));
        }
        if let Some(err) = tx_err {
            return Err((res, err));
        }

        Ok(res)
    }
}


/// The resources that an ongoing transfer needs exclusive access to
pub struct TransferResources<Word, I, P, Rx: dma::Target, Tx: dma::Target, Buffer> {
    pub rx_stream: Rx::Stream,
    pub tx_stream: Tx::Stream,
    pub target:    Spi<I, P, Enabled<Word>>,
    pub buffer:    Pin<Buffer>,
}

// As `TransferResources` is used in the error variant of `Result`, it needs a
// `Debug` implementation to enable stuff like `unwrap` and `expect`. This can't
// be derived without putting requirements on the type arguments.
impl<Word, I, P, Rx, Tx, Buffer> fmt::Debug
    for TransferResources<Word, I, P, Rx, Tx, Buffer>
    where
        Rx: dma::Target,
        Tx: dma::Target,
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "TransferResources {{ .. }}")
    }
}


/// Indicates that the SPI peripheral is enabled
///
/// The `Word` type parameter indicates which word size the peripheral is
/// configured for.
pub struct Enabled<Word>(PhantomData<Word>);


pub trait SupportedWordSize: dma::SupportedWordSize + private::Sealed {
    fn frxth() -> cr2::FRXTHW;
    fn ds() -> cr2::DSW;
}

impl private::Sealed for u8 {}
impl SupportedWordSize for u8 {
    fn frxth() -> cr2::FRXTHW {
        cr2::FRXTHW::QUARTER
    }

    fn ds() -> cr2::DSW {
        cr2::DSW::EIGHTBIT
    }
}

impl private::Sealed for u16 {}
impl SupportedWordSize for u16 {
    fn frxth() -> cr2::FRXTHW {
        cr2::FRXTHW::HALF
    }

    fn ds() -> cr2::DSW {
        cr2::DSW::SIXTEENBIT
    }
}


mod private {
    /// Prevents code outside of the parent module from implementing traits
    ///
    /// This trait is located in a module that is not accessible outside of the
    /// parent module. This means that any trait that requires `Sealed` cannot
    /// be implemented only in the parent module.
    pub trait Sealed {}
=======
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

                    let br = match clocks.$pclkX().0 / freq.0 {
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
                            .br()
                            .bits(br)
                            .lsbfirst()
                            .clear_bit()
                            .ssm()
                            .set_bit()
                            .ssi()
                            .set_bit()
                            .rxonly()
                            .clear_bit()
                            .bidimode()
                            .clear_bit()
                            .spe()
                            .set_bit()
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

                    Err(if sr.ovr().bit_is_set() {
                        nb::Error::Other(Error::Overrun)
                    } else if sr.modf().bit_is_set() {
                        nb::Error::Other(Error::ModeFault)
                    } else if sr.crcerr().bit_is_set() {
                        nb::Error::Other(Error::Crc)
                    } else if sr.txe().bit_is_set() {
                        // NOTE(write_volatile) see note above
                        unsafe { ptr::write_volatile(&self.spi.dr as *const _ as *mut u8, byte) }
                        return Ok(());
                    } else {
                        nb::Error::WouldBlock
                    })
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
>>>>>>> update serial and add SPI
}
