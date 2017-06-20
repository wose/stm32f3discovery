//! Serial interface
//!
//! You can use the `Serial` interface with these USART instances
//!
//! # USART1
//!
//! - TX = PA9
//! - RX = PA10
//! - Interrupt = USART1
//!
//! # USART2
//!
//! - TX = PA14
//! - RX = PA15
//! - Interrupt = USART2
//!
//! # USART3
//!
//! - TX = PC10
//! - RX = PC11
//! - Interrupt = USART3

use core::any::{Any, TypeId};
use core::marker::Unsize;
use core::ops::Deref;
use core::ptr;

use cast::u16;
use hal;
use nb;
use static_ref::Ref;
use stm32f30x::{Dma1, Gpioa, Rcc, Usart1, Usart2,
                  gpioa, usart1};

use dma::{self, Buffer, Dma1Channel4, Dma1Channel5};

/// Specialized `Result` type
pub type Result<T> = ::core::result::Result<T, nb::Error<Error>>;

/// IMPLEMENTATION DETAIL
pub unsafe trait Usart: Deref<Target = usart1::RegisterBlock> {
    /// IMPLEMENTATION DETAIL
    type GPIO: Deref<Target = gpioa::RegisterBlock>;
    /// IMPLEMENTATION DETAIL
    type Ticks: Into<u32>;
}

unsafe impl Usart for Usart1 {
    type GPIO = Gpioa;
    type Ticks = ::apb2::Ticks;
}

unsafe impl Usart for Usart2 {
    type GPIO = Gpioa;
    type Ticks = ::apb1::Ticks;
}

//unsafe impl Usart for Usart3 {
//    type GPIO = Gpiob;
//    type Ticks = ::apb1::Ticks;
//}

/// An error
#[derive(Debug)]
pub enum Error {
    /// De-synchronization, excessive noise or a break character detected
    Framing,
    /// Noise detected in the received frame
    Noise,
    /// RX buffer overrun
    Overrun,
    #[doc(hidden)]
    _Extensible,
}

/// Interrupt event
pub enum Event {
    /// RX buffer Not Empty (new data available)
    Rxne,
    /// Transmission Complete
    Tc,
    /// TX buffer Empty (more data can be send)
    Txe,
}

/// Serial interface
///
/// # Interrupts
///
/// - RXNE
pub struct Serial<'a, U>(pub &'a U)
where
    U: Any + Usart;

impl<'a, U> Clone for Serial<'a, U>
where
    U: Any + Usart,
{
    fn clone(&self) -> Self {
        *self
    }
}

impl<'a, U> Copy for Serial<'a, U>
where
    U: Any + Usart,
{
}

impl<'a, U> Serial<'a, U>
where
    U: Any + Usart,
{
    /// Initializes the serial interface with a baud rate of `baut_rate` bits
    /// per second
    ///
    /// The serial interface will be configured to use 8 bits of data, 1 stop
    /// bit, no hardware control and to omit parity checking
    pub fn init<B>(
        &self,
        baud_rate: B,
        dma1: Option<&Dma1>,
        gpio: &U::GPIO,
        rcc: &Rcc,
    ) where
        B: Into<U::Ticks>,
    {
        self._init(baud_rate.into(), dma1, gpio, rcc)
    }

    fn _init(
        &self,
        baud_rate: U::Ticks,
        dma1: Option<&Dma1>,
        gpio: &U::GPIO,
        rcc: &Rcc,
    ) {
        let usart = self.0;

        // power up peripherals
        if dma1.is_some() {
            rcc.ahbenr.modify(|_, w| w.dmaen().enabled());
        }
        if usart.get_type_id() == TypeId::of::<Usart1>() {
            rcc.apb2enr.modify(|_, w| w.usart1en().enabled());
            rcc.ahbenr.modify(|_, w| w.iopaen().enabled());
        } else if usart.get_type_id() == TypeId::of::<Usart2>() {
            rcc.apb1enr.modify(|_, w| w.usart2en().enabled());
            rcc.ahbenr.modify(|_, w| w.iopaen().enabled());
  /*      } else if usart.get_type_id() == TypeId::of::<Usart3>() {
            rcc.apb1enr.modify(|_, w| w.usart3en().enabled());
            rcc.apb2enr.modify(
                |_, w| w.afioen().enabled().iopben().enabled(),
            );*/
        }

        if usart.get_type_id() == TypeId::of::<Usart1>() {
            // PA9 = TX, PA10 = RX
            gpio
                .afrh
                .modify(|_, w| unsafe { w.afrh9().bits(7).afrh10().bits(7) });
            gpio
                .moder
                .modify(|_, w| w.moder9().alternate().moder10().alternate());
        } else if usart.get_type_id() == TypeId::of::<Usart2>() {
            // PA2 = TX, PA3 = RX
            gpio
                .afrh
                .modify(|_, w| unsafe { w.afrh14().bits(7).afrh15().bits(7) });
            gpio
                .moder
                .modify(|_, w| w.moder14().alternate().moder15().alternate());
        } /*else if usart.get_type_id() == TypeId::of::<Usart3>() {
            // PB10 = TX, PB11 = RX
            afio.mapr.modify(
                |_, w| unsafe { w.usart3_remap().bits(0b00) },
            );
            gpio.crh.modify(|_, w| {
                w.mode10()
                    .output()
                    .cnf10()
                    .alt_push()
                    .mode11()
                    .input()
                    .cnf11()
                    .bits(0b01)
            });
        } */

        if let Some(dma1) = dma1 {
            if usart.get_type_id() == TypeId::of::<Usart1>() {
                // TX DMA transfer
                // mem2mem: Memory to memory mode disabled
                // pl: Medium priority
                // msize: Memory size = 8 bits
                // psize: Peripheral size = 8 bits
                // minc: Memory increment mode enabled
                // pinc: Peripheral increment mode disabled
                // circ: Circular mode disabled
                // dir: Transfer from memory to peripheral
                // tceie: Transfer complete interrupt enabled
                // en: Disabled
                dma1.ccr4.write(|w| unsafe {
                    w.mem2mem()
                        .bits(0)
                        .pl()
                        .bits(0b01)
                        .msize()
                        .bits(0b00)
                        .psize()
                        .bits(0b00)
                        .minc()
                        .bits(1)
                        .circ()
                        .bits(0)
                        .pinc()
                        .bits(0)
                        .dir()
                        .bits(1)
                        .tcie()
                        .bits(1)
                        .en()
                        .bits(0)
                });

                // RX DMA transfer
                // mem2mem: Memory to memory mode disabled
                // pl: Medium priority
                // msize: Memory size = 8 bits
                // psize: Peripheral size = 8 bits
                // minc: Memory increment mode enabled
                // pinc: Peripheral increment mode disabled
                // circ: Circular mode disabled
                // dir: Transfer from peripheral to memory
                // tceie: Transfer complete interrupt enabled
                // en: Disabled
                dma1.ccr5.write(|w| unsafe {
                    w.mem2mem()
                        .bits(0)
                        .pl()
                        .bits(0b01)
                        .msize()
                        .bits(0b00)
                        .psize()
                        .bits(0b00)
                        .minc()
                        .bits(1)
                        .circ()
                        .bits(0)
                        .pinc()
                        .bits(0)
                        .dir()
                        .bits(0)
                        .tcie()
                        .bits(1)
                        .en()
                        .bits(0)
                });
            } else {
                // TODO enable DMA for USART{2,3}
                unimplemented!()
            }
        }

        // 8N1
        usart.cr2.write(|w| unsafe { w.stop().bits(0b00) });

        // baud rate
        let brr = baud_rate.into();

        assert!(brr >= 16, "impossible baud rate");

        usart.brr.write(|w| unsafe { w.bits(brr) });

        // disable hardware flow control
        // enable DMA TX and RX transfers
        usart.cr3.write(|w| {
            unsafe {
                w.rtse().bits(0).ctse().bits(0).dmat().bits(1).dmar().bits(1)
            }
        });

        // enable TX, RX; disable parity checking
        usart.cr1.write(|w| {
            unsafe {
                w.ue()
                    .bits(1)
                    .re()
                    .bits(1)
                    .te()
                    .bits(1)
                    .m()
                    .bits(0)
                    .pce()
                    .bits(0)
                    .rxneie()
                    .bits(0)
            }
        });
    }

    /// Starts listening for an interrupt `event`
    pub fn listen(&self, event: Event) {
        let usart = self.0;

        match event {
            Event::Rxne => usart.cr1.modify(|_, w| unsafe { w.rxneie().bits(1) }),
            Event::Tc => usart.cr1.modify(|_, w| unsafe { w.tcie().bits(1) }),
            Event::Txe => usart.cr1.modify(|_, w| unsafe { w.txeie().bits(1) }),
        }
    }

    /// Stops listening for an interrupt `event`
    pub fn unlisten(&self, event: Event) {
        let usart = self.0;

        match event {
            Event::Rxne => usart.cr1.modify(|_, w| unsafe { w.rxneie().bits(0) }),
            Event::Tc => usart.cr1.modify(|_, w| unsafe { w.tcie().bits(0) }),
            Event::Txe => usart.cr1.modify(|_, w| unsafe { w.txeie().bits(0) }),
        }
    }
}

impl<'a, U> hal::Serial for Serial<'a, U>
where
    U: Any + Usart,
{
    type Error = Error;

    fn read(&self) -> Result<u8> {
        let usart1 = self.0;
        let sr = usart1.isr.read();

        if sr.ore().bits() == 1 {
            Err(nb::Error::Other(Error::Overrun))
        } else if sr.nf().bits() == 1 {
            Err(nb::Error::Other(Error::Noise))
        } else if sr.fe().bits() == 1 {
            Err(nb::Error::Other(Error::Framing))
        } else if sr.rxne().bits() == 1 {
            // NOTE(read_volatile) the register is 9 bits big but we'll only
            // work with the first 8 bits
            Ok(unsafe {
                ptr::read_volatile(&usart1.rdr as *const _ as *const u8)
            })
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn write(&self, byte: u8) -> Result<()> {
        let usart1 = self.0;
        let sr = usart1.isr.read();

        if sr.ore().bits() == 1 {
            Err(nb::Error::Other(Error::Overrun))
        } else if sr.nf().bits() == 1 {
            Err(nb::Error::Other(Error::Noise))
        } else if sr.fe().bits() == 1 {
            Err(nb::Error::Other(Error::Framing))
        } else if sr.txe().bits() == 1 {
            // NOTE(write_volatile) see NOTE in the `read` method
            unsafe {
                ptr::write_volatile(&usart1.tdr as *const _ as *mut u8, byte)
            }
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<'a> Serial<'a, Usart1> {
    /// Starts a DMA transfer to receive serial data into a `buffer`
    ///
    /// This will mutably lock the `buffer` preventing borrowing its contents
    /// The `buffer` can be `release`d after the DMA transfer finishes
    // TODO support circular mode + half transfer interrupt as a double
    // buffering mode
    pub fn read_exact<B>(
        &self,
        dma1: &Dma1,
        buffer: Ref<Buffer<B, Dma1Channel5>>,
    ) -> ::core::result::Result<(), dma::Error>
    where
        B: Unsize<[u8]>,
    {
        let usart1 = self.0;

        if dma1.ccr5.read().en().bits() == 1 {
            return Err(dma::Error::InUse);
        }

        let buffer: &mut [u8] = buffer.lock_mut();

        dma1.cndtr5.write(|w| unsafe {
            w.ndt().bits(u16(buffer.len()).unwrap())
        });
        dma1.cpar5.write(|w| unsafe {
            w.bits(&usart1.rdr as *const _ as u32)
        });
        dma1.cmar5.write(
            |w| unsafe { w.bits(buffer.as_ptr() as u32) },
        );
        dma1.ccr5.modify(|_, w| unsafe { w.en().bits(1) });

        Ok(())
    }

    /// Starts a DMA transfer to send `buffer` through this serial port
    ///
    /// This will immutably lock the `buffer` preventing mutably borrowing its
    /// contents. The `buffer` can be `release`d after the DMA transfer finishes
    pub fn write_all<B>(
        &self,
        dma1: &Dma1,
        buffer: Ref<Buffer<B, Dma1Channel4>>,
    ) -> ::core::result::Result<(), dma::Error>
    where
        B: Unsize<[u8]>,
    {
        let usart1 = self.0;

        if dma1.ccr4.read().en().bits() == 1 {
            return Err(dma::Error::InUse);
        }

        let buffer: &[u8] = buffer.lock();

        dma1.cndtr4.write(|w| unsafe {
            w.ndt().bits(u16(buffer.len()).unwrap())
        });
        dma1.cpar4.write(|w| unsafe {
            w.bits(&usart1.tdr as *const _ as u32)
        });
        dma1.cmar4.write(
            |w| unsafe { w.bits(buffer.as_ptr() as u32) },
        );
        dma1.ccr4.modify(|_, w| unsafe { w.en().bits(1) });

        Ok(())
    }
}
