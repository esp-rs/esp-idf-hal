use core::fmt;
use core::marker::{PhantomData, PhantomPinned};
use core::pin::Pin;

use esp_idf_sys::EspError;

use super::Token;
use crate::rmt::blocking::TxChannelDriver;
use crate::rmt::config::TransmitConfig;
use crate::rmt::encoder::{into_raw, Encoder, EncoderWrapper};

struct Slot<S, T> {
    token: Option<Token>,
    signal: S,
    _pinned: PhantomPinned,
    _marker: PhantomData<T>,
}

impl<S, T> Slot<S, T> {
    #[must_use]
    pub fn new(token: Option<Token>, signal: S) -> Self {
        Self {
            token,
            signal,
            _pinned: PhantomPinned,
            _marker: PhantomData,
        }
    }
}

impl<T, S: AsRef<[T]>> Slot<S, T> {
    pub fn set<E>(
        self: Pin<&mut Self>,
        f: impl FnOnce(Pin<&[T]>) -> Result<Token, E>,
    ) -> Result<(), E> {
        // SAFETY: This is okay because `signal` is pinned when `self` is.
        let signal = unsafe { self.as_ref().map_unchecked(|s| s.signal.as_ref()) };
        let token = f(signal)?;
        unsafe {
            let mut this = self.map_unchecked_mut(|this| &mut this.token);
            this.set(Some(token));
        }

        Ok(())
    }
}

pub struct TxQueue<'c, 'd, E: Encoder, S, const N: usize> {
    #[allow(clippy::type_complexity)]
    slots: [(EncoderWrapper<E>, Option<Slot<S, E::Item>>); N],
    channel: &'c mut TxChannelDriver<'d>,
    _pinned: PhantomPinned,
}

impl<'c, 'd, E: Encoder, S, const N: usize> TxQueue<'c, 'd, E, S, N> {
    pub(crate) fn new(encoders: [E; N], channel: &'c mut TxChannelDriver<'d>) -> Self {
        Self {
            slots: encoders.map(|e| (into_raw(e), None)),
            channel,
            _pinned: PhantomPinned,
        }
    }

    #[must_use]
    fn find_next_slot(self: Pin<&Self>) -> usize {
        // Find an encoder that is currently unused or the oldest pending transmission (which would finish first).
        let (idx, _) = self
            .slots
            .iter()
            .enumerate()
            // None elements will be before Some elements. Tokens are ordered by their age,
            // the oldest ones will finish first.
            .min_by_key(|(_idx, (_, slot))| slot.as_ref().map(|t| t.token))
            // N is > 0 -> there is always at least one element
            .unwrap();

        idx
    }

    #[must_use]
    fn get_token(self: Pin<&Self>, index: usize) -> Option<Token> {
        self.slots
            .get(index)
            .and_then(|(_, slot)| slot.as_ref().and_then(|s| s.token))
    }

    #[must_use]
    #[allow(clippy::type_complexity)]
    fn get(
        self: Pin<&mut Self>,
        index: usize,
    ) -> (
        &mut TxChannelDriver<'d>,
        Pin<&mut EncoderWrapper<E>>,
        Pin<&mut Option<Slot<S, E::Item>>>,
    ) {
        unsafe {
            // SAFETY: The get_unchecked_mut has the safety requirement that the data is never moved out of the pin
            //         e.g. calling mem::replace.
            //         This is not happening here, because we only destructure the references:
            //         &mut (T, U) -> (&mut T, &mut U)
            //         and then wrap both of the references again (to ensure that they are still pinned).
            let this = self.get_unchecked_mut();
            // SAFETY: The encoder and slot are still pinned
            (
                &mut this.channel,
                Pin::new_unchecked(&mut this.slots[index].0),
                Pin::new_unchecked(&mut this.slots[index].1),
            )
        }
    }

    /// Returns an iterator over all tokens that are currently in use.
    ///
    /// The order of the tokens is not defined.
    fn pending_tokens(self: Pin<&Self>) -> impl Iterator<Item = Token> + '_ {
        self.get_ref()
            .slots
            .iter()
            .flat_map(|(_, slot)| slot.as_ref().and_then(|s| s.token))
    }

    /// Returns a mutable reference to the channel this queue is using.
    #[must_use]
    pub fn channel(self: Pin<&mut Self>) -> &mut TxChannelDriver<'d> {
        // SAFETY: The channel is not structural for pinning, it is okay to be moved
        unsafe { self.get_unchecked_mut().channel }
    }
}

impl<'c, 'd, E: Encoder, S: AsRef<[E::Item]> + 'c, const N: usize> TxQueue<'c, 'd, E, S, N> {
    /// Returns the oldest token which would finish before all other tokens or
    /// `None` if there are no ongoing transmissions.
    #[must_use]
    pub fn oldest_token(self: Pin<&Self>) -> Option<Token> {
        self.pending_tokens().min_by_key(|t| *t)
    }

    /// This function is the code that actually does the work of waiting for an encoder to become available
    /// and starting the transmission.
    ///
    /// To prevent duplicate code, this function is called by both `push` and the async `push` in `AsyncTxQueue`.
    ///
    /// Regarding the error type:
    /// - If `should_block` is true, the functions error will always be `Err(...)`
    /// - If `should_block` is false, the function can return either `Ok(())` or `Err(Ok(...))` or `Err(Err(...))`
    ///   The `Err(Ok(...))` case contains the original signal and the token of the encoder that should be waited for.
    pub(crate) fn push_internal(
        mut self: Pin<&mut Self>,
        signal: S,
        config: &TransmitConfig,
        should_block: bool,
    ) -> Result<(), Result<(S, Token), EspError>> {
        let idx = self.as_ref().find_next_slot();
        if let Some(token) = self.as_ref().get_token(idx) {
            // If the transmission is still in progress, and we should not block,
            // return the original signal and the token
            if !self.as_mut().channel().is_finished(token) && !should_block {
                return Err(Ok((signal, token)));
            }

            self.as_mut().channel().wait_for(token);
        }

        let (channel, encoder, mut slot) = self.get(idx);
        slot.set(Some(Slot::new(None, signal)));

        // SAFETY: The slot was set above -> it is safe to unwrap here.
        let slot = unsafe { slot.map_unchecked_mut(|s| s.as_mut().unwrap_unchecked()) };

        // SAFETY: The transmission using this encoder is finished -> should be safe to reuse the encoder
        slot.set(|signal| unsafe {
            channel.start_send(encoder.get_unchecked_mut(), signal.get_ref(), config)
        })
        .map_err(|err| Err(err))?;

        Ok(())
    }

    /// Pushes a new signal to be transmitted using the next available encoder.
    ///
    /// If all encoders are busy, this will block until one is available.
    pub fn push(self: Pin<&mut Self>, signal: S, config: &TransmitConfig) -> Result<(), EspError> {
        self.push_internal(signal, config, true)
            // SAFETY: The error is always Err(EspError) because should_block is true
            .map_err(|err| unsafe { err.unwrap_err_unchecked() })
    }
}

impl<'c, 'd, E: Encoder, S, const N: usize> Drop for TxQueue<'c, 'd, E, S, N> {
    fn drop(&mut self) {
        // SAFETY: For a pinned type, drop must behave as if it were pinned.
        //
        // The problem is that splitting borrows, mutably accessing channel while the slots are immutably borrowed,
        // seems to be impossible without the use of unsafe.
        //
        // -> Code would be a Pin::new_unchecked, followed by a get_unchecked_mut, which defeats the purpose
        //    of wrapping it in a pin in the first place

        for token in self
            .slots
            .iter()
            .flat_map(|(_, slot)| slot.as_ref().and_then(|s| s.token))
        {
            self.channel.wait_for(token);
        }
    }
}

impl<'c, 'd, E, S, const N: usize> fmt::Debug for TxQueue<'c, 'd, E, S, N>
where
    E: Encoder + fmt::Debug,
    E::Item: fmt::Debug,
    S: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("TxQueue")
            .field("slots", &self.slots.len())
            .field("channel", &self.channel)
            .finish()
    }
}
