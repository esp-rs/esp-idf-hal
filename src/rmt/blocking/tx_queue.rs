use core::marker::{PhantomData, PhantomPinned};
use core::pin::Pin;

use esp_idf_sys::EspError;

use super::Token;
use crate::rmt::blocking::TxChannelDriver;
use crate::rmt::config::TransmitConfig;
use crate::rmt::encoder::Encoder;

struct Slot<S, T> {
    token: Option<Token>,
    signal: S,
    _pinned: PhantomPinned,
    _marker: PhantomData<T>,
}

impl<T, S: AsRef<[T]>> Slot<S, T> {
    #[must_use]
    pub fn get_signal(self: Pin<&Self>) -> Pin<&[T]> {
        // This is okay because `signal` is pinned when `self` is.
        unsafe { self.map_unchecked(|s| s.signal.as_ref()) }
    }

    pub fn set<E>(
        self: Pin<&mut Self>,
        f: impl FnOnce(Pin<&[T]>) -> Result<Token, E>,
    ) -> Result<(), E> {
        let signal = self.as_ref().get_signal();
        let token = f(signal)?;
        unsafe {
            let mut this = self.map_unchecked_mut(|this| &mut this.token);
            this.set(Some(token));
        }

        Ok(())
    }
}

// TODO: use pin-project crate? Should simplify this

pub struct TxQueue<'c, 'd, E: Encoder, S, const N: usize> {
    #[allow(clippy::type_complexity)]
    slots: [(E, Option<Slot<S, E::Item>>); N],
    channel: &'c mut TxChannelDriver<'d>,
    _pinned: PhantomPinned,
}

impl<'c, 'd, E: Encoder, S, const N: usize> TxQueue<'c, 'd, E, S, N> {
    pub(crate) fn new(encoders: [E; N], channel: &'c mut TxChannelDriver<'d>) -> Self {
        Self {
            slots: encoders.map(|e| (e, None)),
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
        Pin<&mut E>,
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
    fn pending_tokens(
        self: Pin<&mut Self>,
    ) -> (&mut TxChannelDriver<'d>, impl Iterator<Item = Token> + '_) {
        unsafe {
            let this = self.get_unchecked_mut();
            (
                this.channel,
                this.slots
                    .iter()
                    .flat_map(|(_, slot)| slot.as_ref().and_then(|s| s.token)),
            )
        }
    }

    // TODO: fix this duplicate
    fn pending_tokens2(self: Pin<&Self>) -> impl Iterator<Item = Token> + '_ {
        self.get_ref()
            .slots
            .iter()
            .flat_map(|(_, slot)| slot.as_ref().and_then(|s| s.token))
    }

    fn channel(self: Pin<&mut Self>) -> &mut TxChannelDriver<'d> {
        // TODO: can this be exposed globally? Should this be exposed?
        // SAFETY: The channel is not necessary to be pinned?
        unsafe { self.get_unchecked_mut().channel }
    }
}

impl<'c, 'd, E: Encoder, S: AsRef<[E::Item]> + 'c, const N: usize> TxQueue<'c, 'd, E, S, N> {
    /// Returns the oldest token which would finish before all other tokens or
    /// `None` if there are no ongoing transmissions.
    #[must_use]
    pub fn oldest_token(self: Pin<&Self>) -> Option<Token> {
        self.pending_tokens2().min_by_key(|t| *t)
    }

    // TODO: naming of this function, should it be called send? enqueue? queue?

    /// Pushes a new signal to be transmitted using the next available encoder.
    ///
    /// If all encoders are busy, this will block until one is available.
    pub fn push(
        mut self: Pin<&mut Self>,
        signal: S,
        config: &TransmitConfig,
    ) -> Result<(), EspError> {
        let idx = self.as_ref().find_next_slot();
        if let Some(token) = self.as_ref().get_token(idx) {
            self.as_mut().channel().wait_for(token);
        }

        let (channel, encoder, mut slot) = self.get(idx);
        slot.set(Some(Slot {
            token: None,
            signal,
            _pinned: PhantomPinned,
            _marker: PhantomData,
        }));

        // SAFETY: We just set the slot to Some above -> it is safe to unwrap here.
        let slot = unsafe { slot.map_unchecked_mut(|s| s.as_mut().unwrap_unchecked()) };

        // SAFETY: The transmission using this encoder is finished -> should be safe to reuse the encoder
        slot.set(|signal| unsafe {
            channel.start_send(encoder.get_unchecked_mut(), signal.get_ref(), config)
        })?;

        Ok(())
    }
}

impl<'c, 'd, E: Encoder, S, const N: usize> Drop for TxQueue<'c, 'd, E, S, N> {
    fn drop(&mut self) {
        // SAFETY: For a pinned type, drop must behave as if it was pinned.
        let this = unsafe { Pin::new_unchecked(self) };
        // Wait for all ongoing transmissions to finish:
        let (channel, iter) = this.pending_tokens();
        for token in iter {
            channel.wait_for(token);
        }
    }
}
