//! Serial Peripheral Interface
//!
//! You can use the `Spi` interface with these SPI instances
//!
//! # SPI1
//!
//! - NSS = PA4
//! - SCK = PA5
//! - MISO = PA6
//! - MOSI = PA7

use core::any::{Any, TypeId};
use core::ops::Deref;
use core::ptr;

use hal;
use nb;
use stm32f30x::{Gpioa, Rcc, Spi1, gpioa, spi1};

/// SPI instance that can be used with the `Spi` abstraction
pub unsafe trait SPI: Deref<Target = spi1::RegisterBlock> {
    /// GPIO block associated to this SPI instance
    type GPIO: Deref<Target = gpioa::RegisterBlock>;
}

unsafe impl SPI for Spi1 {
    type GPIO = Gpioa;
}

/// SPI result
pub type Result<T> = ::core::result::Result<T, nb::Error<Error>>;

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

/// Serial Peripheral Interface
pub struct Spi<'a, S>(pub &'a S)
where
    S: Any + SPI;

impl<'a, S> Spi<'a, S>
where
    S: Any + SPI,
{
    /// Initializes the SPI
    pub fn init(&self, gpio: &S::GPIO, rcc: &Rcc) {
        let spi = self.0;

        if spi.get_type_id() == TypeId::of::<Spi1>() {
            // enable AFIO, SPI1, GPIOA
            rcc.apb2enr.modify(|_, w| w.spi1en().enabled());
            rcc.ahbenr.modify(|_, w| w.iopaen().enabled());

            // NSS = PA4 = Alternate function push pull
            // SCK = PA5 = Alternate function push pull
            // MISO = PA6 = Floating input
            // MOSI = PA7 = Alternate function push pull
            gpio
                .afrl.modify(|_, w| unsafe {
                    w
                        .afrl4().bits(5)
                        .afrl5().bits(5)
                        .afrl6().bits(5)
                        .afrl7().bits(5)
                });
            gpio
                .moder
                .modify(|_, w| w
                        .moder4().alternate()
                        .moder5().alternate()
                        .moder6().alternate()
                        .moder7().alternate()
                );
        }

        // enable SS output
        spi.cr2.write(|w| unsafe { w.ssoe().bits(1) });

        // cpha: second clock transition is the first data capture
        // cpol: CK to 1 when idle
        // mstr: master configuration
        // br: 1 MHz frequency
        // lsbfirst: MSB first
        // ssm: disable software slave management
        // dff: 8 bit frames
        // bidimode: 2-line unidirectional
        spi.cr1.write(|w| unsafe {
            w.cpha()
                .bits(1)
                .cpol()
                .bits(1)
                .mstr()
                .bits(1)
                .br()
                .bits(0b10)
                .lsbfirst()
                .bits(0)
                .ssm()
                .bits(0)
                .rxonly()
                .bits(0)
                .dff()
                .bits(0)
                .bidimode()
                .bits(0)
        });
    }

    /// Disables the SPI bus
    ///
    /// **NOTE** This drives the NSS pin high
    pub fn disable(&self) {
        self.0.cr1.modify(|_, w| unsafe { w.spe().bits(0) })
    }

    /// Enables the SPI bus
    ///
    /// **NOTE** This drives the NSS pin low
    pub fn enable(&self) {
        self.0.cr1.modify(|_, w| unsafe { w.spe().bits(1) })
    }
}

impl<'a, S> hal::Spi<u8> for Spi<'a, S>
where
    S: Any + SPI,
{
    type Error = Error;

    fn read(&self) -> Result<u8> {
        let spi1 = self.0;
        let sr = spi1.sr.read();

        if sr.ovr().bits() == 1 {
            Err(nb::Error::Other(Error::Overrun))
        } else if sr.modf().bits() == 1 {
            Err(nb::Error::Other(Error::ModeFault))
        } else if sr.crcerr().bits() == 1 {
            Err(nb::Error::Other(Error::Crc))
        } else if sr.rxne().bits() == 1 {
            Ok(unsafe {
                ptr::read_volatile(&spi1.dr as *const _ as *const u8)
            })
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn send(&self, byte: u8) -> Result<()> {
        let spi1 = self.0;
        let sr = spi1.sr.read();

        if sr.ovr().bits() == 1 {
            Err(nb::Error::Other(Error::Overrun))
        } else if sr.modf().bits() == 1 {
            Err(nb::Error::Other(Error::ModeFault))
        } else if sr.crcerr().bits() == 1 {
            Err(nb::Error::Other(Error::Crc))
        } else if sr.txe().bits() == 1 {
            // NOTE(write_volatile) see note above
            unsafe {
                ptr::write_volatile(&spi1.dr as *const _ as *mut u8, byte)
            }
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}
