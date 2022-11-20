use crate::interrupt::{IsrCriticalSection, IsrCriticalSectionGuard};
use crate::spi::{config, to_spi_err, Lock, SpiDriver, SpiError};
use core::borrow::BorrowMut;
use core::cell::UnsafeCell;
use core::cmp::max;
use core::cmp::Ordering;
use core::future::Future;
use core::iter::Zip;
use core::marker::PhantomPinned;
use core::ops::{Deref, DerefMut};
use core::pin::Pin;
use core::slice::Chunks;
use core::slice::ChunksMut;
use core::task::{Context, Poll, Waker};
use embedded_hal::spi::ErrorType;
use embedded_hal_async::spi::{SpiBus, SpiBusFlush, SpiBusRead, SpiBusWrite};
use esp_idf_sys::*;

pub struct SpiBusDriver<T> {
    _driver: T,
    handle: spi_device_handle_t,
    _lock: Lock,
    trans_len: usize,
}

// TODO: IRAM
unsafe extern "C" fn post_transaction(transaction: *mut spi_transaction_t) {
    let transaction = unsafe { transaction.as_mut() }.unwrap();

    let waker_ptr = transaction.user as *const IsrMutex<Option<Waker>>;
    let waker_mutex = unsafe { waker_ptr.as_ref() }.unwrap();

    // Consume waker
    let mut lock = waker_mutex.lock();
    if let Some(waker) = lock.take() {
        waker.wake();
    }
}

pub struct SpiFuture<'a, T, Op: Operation> {
    driver: &'a mut SpiBusDriver<T>, // not structurally pinned
    transaction: spi_transaction_t,  // structurally pinned
    state: State<Op>,                // not structurally pinned
    _pin: PhantomPinned,
}

enum State<Op: Operation> {
    Processing(
        Op,                      // not structurally pinned
        IsrMutex<Option<Waker>>, // structurally pinned
    ),
    Done(Result<(), SpiError>), // not structurally pinned
}

impl<'a, T, Op: Operation> Drop for SpiFuture<'a, T, Op> {
    fn drop(&mut self) {
        // If the future is dropped before the transaction is complete, we need to block until it
        // is done to prevent undefined behaviour.
        todo!()
    }
}

impl<'a, T, Op: Operation> Future for SpiFuture<'a, T, Op> {
    type Output = Result<(), SpiError>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // Check if done or replace existing waker.
        match &self.state {
            State::Done(res) => return Poll::Ready(*res),
            State::Processing(_, waker) => {
                let mut waker_lock = waker.lock();
                if waker_lock.take().is_some() {
                    // If there's a waker here that means there's a transaction in progress. The
                    // waker needs to be replaced with this new one passed into `poll`.
                    *waker_lock = Some(cx.waker().clone());
                    return Poll::Pending;
                }
            }
        };

        // If there's no waker, it means either no transactions have been queued or the
        // last queued transaction has completed.

        // This also means that nothing is Self is structurally pinned, as there's no in flight
        // transaction holding a pointer to it. This is important for the use of `unsafe` below.

        let is_first_transaction = self.transaction.user.is_null();
        if !is_first_transaction {
            // If the last transaction was completed then we need to consume it to free
            // any resources associated with it.

            let mut ret_trans: *mut spi_transaction_t = core::ptr::null_mut();

            let res = esp!(unsafe {
                spi_device_get_trans_result(self.driver.handle, &mut ret_trans as *mut _, 0)
            });
            if res.is_err() {
                let res = res.map_err(to_spi_err);
                // Safe because state isn't structurally pinned.
                let state = unsafe { &mut self.get_unchecked_mut().state };
                *state = State::Done(res);
                return Poll::Ready(res);
            } else {
                // debug_assert_eq!(ret_trans, &mut self.transaction as *mut _);
            }
        }

        // As a future enhancement this code should queue multiple transactions at once
        // and keep the queue full until the operation is complete. This will allow for
        // less pause-y transfers.

        // Safe because state isn't structurally pinned.
        let this = unsafe { self.get_unchecked_mut() };

        if let State::Processing(op, waker) = &mut this.state {
            let is_filled = op.fill(&mut this.transaction);
            if is_filled {
                *waker.lock() = Some(cx.waker().clone());
                this.transaction.user = core::ptr::addr_of!(waker) as _;

                let res = esp!(unsafe {
                    spi_device_queue_trans(this.driver.handle, &mut this.transaction as *mut _, 0)
                });
                if res.is_err() {
                    let res = res.map_err(to_spi_err);
                    this.state = State::Done(res);
                    Poll::Ready(res)
                } else {
                    Poll::Pending
                }
            } else {
                // The operation is done so we can just return Ok now and forever.
                this.state = State::Done(Ok(()));
                Poll::Ready(Ok(()))
            }
        } else {
            unreachable!()
        }
    }
}

pub trait Operation {
    fn fill(&mut self, transaction: &mut spi_transaction_t) -> bool;
}

pub struct Read<'a> {
    chunks: ChunksMut<'a, u8>,
}

impl<'a> Read<'a> {
    fn new(words: &'a mut [u8], chunk_size: usize) -> Self {
        Self {
            chunks: words.chunks_mut(chunk_size),
        }
    }
}

impl<'a> Operation for Read<'a> {
    fn fill(&mut self, transaction: &mut spi_transaction_t) -> bool {
        if let Some(chunk) = self.chunks.next() {
            fill(
                transaction,
                chunk.as_mut_ptr(),
                core::ptr::null(),
                chunk.len(),
                chunk.len(),
                true,
            );
            true
        } else {
            false
        }
    }
}

pub struct Write<'a> {
    chunks: Chunks<'a, u8>,
}

impl<'a> Write<'a> {
    fn new(words: &'a [u8], chunk_size: usize) -> Self {
        Self {
            chunks: words.chunks(chunk_size),
        }
    }
}

impl<'a> Operation for Write<'a> {
    fn fill(&mut self, transaction: &mut spi_transaction_t) -> bool {
        if let Some(chunk) = self.chunks.next() {
            fill(
                transaction,
                core::ptr::null_mut(),
                chunk.as_ptr(),
                chunk.len(),
                0,
                true,
            );
            true
        } else {
            false
        }
    }
}

pub struct TransferInPlace<'a> {
    chunks: ChunksMut<'a, u8>,
}

impl<'a> TransferInPlace<'a> {
    fn new(words: &'a mut [u8], chunk_size: usize) -> Self {
        Self {
            chunks: words.chunks_mut(chunk_size),
        }
    }
}

impl<'a> Operation for TransferInPlace<'a> {
    fn fill(&mut self, transaction: &mut spi_transaction_t) -> bool {
        if let Some(chunk) = self.chunks.next() {
            let ptr = chunk.as_mut_ptr();
            let len = chunk.len();
            fill(transaction, ptr, ptr, len, len, true);
            true
        } else {
            false
        }
    }
}

pub struct Transfer<'a> {
    chunks: Zip<ChunksMut<'a, u8>, Chunks<'a, u8>>,
    trail: TransferTrail<'a>,
}

enum TransferTrail<'a> {
    None,
    Read(Read<'a>),
    Write(Write<'a>),
}

impl<'a> Transfer<'a> {
    fn new(read: &'a mut [u8], write: &'a [u8], chunk_size: usize) -> Self {
        match read.len().cmp(&write.len()) {
            Ordering::Equal => {
                let read = read.chunks_mut(chunk_size);
                let write = write.chunks(chunk_size);
                Transfer {
                    chunks: read.zip(write),
                    trail: TransferTrail::None,
                }
            }
            Ordering::Greater => {
                let (read, read_trail) = read.split_at_mut(write.len());
                let read = read.chunks_mut(chunk_size);
                let write = write.chunks(chunk_size);
                Transfer {
                    chunks: read.zip(write),
                    trail: TransferTrail::Read(Read::new(read_trail, chunk_size)),
                }
            }
            Ordering::Less => {
                let (write, write_trail) = write.split_at(read.len());
                let read = read.chunks_mut(chunk_size);
                let write = write.chunks(chunk_size);
                Transfer {
                    chunks: read.zip(write),
                    trail: TransferTrail::Write(Write::new(write_trail, chunk_size)),
                }
            }
        }
    }
}

impl<'a> Operation for Transfer<'a> {
    fn fill(&mut self, transaction: &mut spi_transaction_t) -> bool {
        if let Some((read_chunk, write_chunk)) = self.chunks.next() {
            fill(
                transaction,
                read_chunk.as_mut_ptr(),
                write_chunk.as_ptr(),
                max(read_chunk.len(), write_chunk.len()),
                read_chunk.len(),
                true,
            );
            return true;
        }
        match &mut self.trail {
            TransferTrail::Write(write) => write.fill(transaction),
            TransferTrail::Read(read) => read.fill(transaction),
            TransferTrail::None => false,
        }
    }
}

pub struct Noop;

impl Operation for Noop {
    fn fill(&mut self, _transaction: &mut spi_transaction_t) -> bool {
        false
    }
}

impl<'d, T: BorrowMut<SpiDriver<'d>>> SpiBusDriver<T> {
    fn mode_to_u8(data_mode: &embedded_hal::spi::Mode) -> u8 {
        (((data_mode.polarity == embedded_hal::spi::Polarity::IdleHigh) as u8) << 1)
            | ((data_mode.phase == embedded_hal::spi::Phase::CaptureOnSecondTransition) as u8)
    }

    pub fn new(driver: T, config: &config::Config) -> Result<Self, EspError> {
        let mut flags = 0_u32;
        if config.write_only {
            flags |= SPI_DEVICE_NO_DUMMY
        }
        // if config.cs_active_high {
        //     flags |= SPI_DEVICE_POSITIVE_CS
        // }
        flags |= config.duplex.as_flags();

        let conf = spi_device_interface_config_t {
            post_cb: Some(post_transaction),
            spics_io_num: -1,
            clock_speed_hz: config.baudrate.0 as i32,
            mode: Self::mode_to_u8(&config.data_mode),
            queue_size: 1,
            flags,
            ..Default::default()
        };

        let mut handle: spi_device_handle_t = core::ptr::null_mut();
        esp!(unsafe { spi_bus_add_device(driver.borrow().host(), &conf, &mut handle as *mut _) })?;

        let lock_result = Lock::new(handle);
        let lock = match lock_result {
            Ok(lock) => lock,
            Err(err) => {
                let _res = esp!(unsafe { spi_bus_remove_device(handle) });
                // result is explicitly ignored as the lock acquisition failure is the real error.
                return Err(err);
            }
        };

        let trans_len = driver.borrow().max_transfer_size;

        Ok(Self {
            _driver: driver,
            handle,
            _lock: lock,
            trans_len,
        })
    }

    fn future_op<Op: Operation>(&mut self, op: Op) -> SpiFuture<T, Op> {
        SpiFuture {
            driver: self,
            transaction: Default::default(),
            state: State::Processing(op, IsrMutex::new(None)),
            _pin: PhantomPinned,
        }
    }

    pub fn read<'a>(&'a mut self, words: &'a mut [u8]) -> SpiFuture<'a, T, Read<'a>> {
        self.future_op(Read::new(words, self.trans_len))
    }

    pub fn write<'a>(&'a mut self, words: &'a [u8]) -> SpiFuture<'a, T, Write<'a>> {
        self.future_op(Write::new(words, self.trans_len))
    }

    pub fn transfer<'a>(
        &'a mut self,
        read: &'a mut [u8],
        write: &'a [u8],
    ) -> SpiFuture<'a, T, Transfer<'a>> {
        self.future_op(Transfer::new(read, write, self.trans_len))
    }

    pub fn transfer_in_place<'a>(
        &'a mut self,
        words: &'a mut [u8],
    ) -> SpiFuture<'a, T, TransferInPlace<'a>> {
        self.future_op(TransferInPlace::new(words, self.trans_len))
    }

    pub fn flush(&mut self) -> SpiFuture<T, Noop> {
        // This component doesn't do any buffering so there's nothing to flush.
        self.future_op(Noop)
    }
}

impl<T> ErrorType for SpiBusDriver<T> {
    type Error = SpiError;
}

impl<'d, T: BorrowMut<SpiDriver<'d>>> SpiBusFlush for SpiBusDriver<T> {
    type FlushFuture<'a> = SpiFuture<'a, T, Noop> where T: 'a;

    fn flush(&mut self) -> Self::FlushFuture<'_> {
        SpiBusDriver::flush(self)
    }
}

impl<'d, T: BorrowMut<SpiDriver<'d>>> SpiBusRead for SpiBusDriver<T> {
    type ReadFuture<'a> = SpiFuture<'a, T, Read<'a>> where T: 'a;

    fn read<'a>(&'a mut self, words: &'a mut [u8]) -> Self::ReadFuture<'a> {
        SpiBusDriver::read(self, words)
    }
}

impl<'d, T: BorrowMut<SpiDriver<'d>>> SpiBusWrite for SpiBusDriver<T> {
    type WriteFuture<'a> = SpiFuture<'a, T, Write<'a>> where T: 'a;

    fn write<'a>(&'a mut self, words: &'a [u8]) -> Self::WriteFuture<'a> {
        SpiBusDriver::write(self, words)
    }
}

impl<'d, T: BorrowMut<SpiDriver<'d>>> SpiBus for SpiBusDriver<T> {
    type TransferFuture<'a> = SpiFuture<'a, T, Transfer<'a>> where T: 'a;

    fn transfer<'a>(&'a mut self, read: &'a mut [u8], write: &'a [u8]) -> Self::TransferFuture<'a> {
        SpiBusDriver::transfer(self, read, write)
    }

    type TransferInPlaceFuture<'a> = SpiFuture<'a, T, TransferInPlace<'a>> where T: 'a;

    fn transfer_in_place<'a>(&'a mut self, words: &'a mut [u8]) -> Self::TransferInPlaceFuture<'a> {
        SpiBusDriver::transfer_in_place(self, words)
    }
}

// These parameters assume full duplex.
fn fill(
    transaction: &mut spi_transaction_t,
    read: *mut u8,
    write: *const u8,
    transaction_length: usize,
    rx_length: usize,
    _keep_cs_active: bool,
) {
    // This unfortunately means that this implementation is incorrect for esp-idf < 4.4.
    // The CS pin should be kept active through transactions.
    #[cfg(not(esp_idf_version = "4.3"))]
    transaction.flags = if _keep_cs_active {
        SPI_TRANS_CS_KEEP_ACTIVE
    } else {
        0
    };

    transaction.__bindgen_anon_1 = spi_transaction_t__bindgen_ty_1 {
        tx_buffer: write as *const _,
    };
    transaction.__bindgen_anon_2 = spi_transaction_t__bindgen_ty_2 {
        rx_buffer: read as *mut _,
    };
    transaction.length = (transaction_length * 8) as _;
    transaction.rxlength = (rx_length * 8) as _;
}

struct IsrMutex<T: ?Sized> {
    cs: IsrCriticalSection,
    data: UnsafeCell<T>,
}

struct IsrMutexGuard<'a, T: ?Sized + 'a> {
    _guard: IsrCriticalSectionGuard<'a>,
    mutex: &'a IsrMutex<T>,
}

// TODO: Make these interrupt safe. i.e. IRAM
impl<T> IsrMutex<T> {
    fn new(data: T) -> Self {
        Self {
            cs: IsrCriticalSection::new(),
            data: UnsafeCell::new(data),
        }
    }

    fn lock(&self) -> IsrMutexGuard<'_, T> {
        IsrMutexGuard::new(self)
    }
}

impl<'a, T> IsrMutexGuard<'a, T> {
    fn new(mutex: &'a IsrMutex<T>) -> Self {
        Self {
            _guard: mutex.cs.enter(),
            mutex,
        }
    }
}

impl<'a, T> Deref for IsrMutexGuard<'a, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        unsafe { &*self.mutex.data.get() }
    }
}

impl<'a, T> DerefMut for IsrMutexGuard<'a, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { &mut *self.mutex.data.get() }
    }
}
