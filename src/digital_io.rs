// phidget-rs/src/digital_io.rs
//
// Copyright (c) 2023, Frank Pagliughi
//
// This file is part of the 'phidget-rs' library.
//
// Licensed under the MIT license:
//   <LICENSE or http://opensource.org/licenses/MIT>
// This file may not be copied, modified, or distributed except according
// to those terms.
//

use crate::{AttachCallback, DetachCallback, GenericPhidget, Phidget, Result, ReturnCode};
use phidget_sys::{
    self as ffi, PhidgetDigitalInputHandle as DigitalInputHandle,
    PhidgetDigitalOutputHandle as DigitalOutputHandle, PhidgetHandle,
};
use std::{
    mem,
    os::raw::{c_int, c_void},
    ptr,
};

/// The function signature for the safe Rust digital input state change callback.
pub type DigitalInputCallback = dyn Fn(&DigitalInput, i32) + Send + 'static;

/////////////////////////////////////////////////////////////////////////////

/// Phidget digital input
#[derive(Debug)]
pub struct DigitalInput {
    // Handle to the digital input in the phidget22 library
    chan: DigitalInputHandle,
    // Double-boxed DigitalInputCallback, if registered
    cb: Option<*mut c_void>,
    // Double-boxed attach callback, if registered
    attach_cb: Option<*mut c_void>,
    // Double-boxed detach callback, if registered
    detach_cb: Option<*mut c_void>,
}

impl DigitalInput {
    /// Create a new digital input.
    pub fn new() -> Self {
        let mut chan: DigitalInputHandle = ptr::null_mut();
        unsafe {
            ffi::PhidgetDigitalInput_create(&mut chan);
        }
        Self::from(chan)
    }

    // Low-level, unsafe, callback for the digital input state change event.
    // The context is a double-boxed pointer to the safe Rust callback.
    unsafe extern "C" fn on_state_change(chan: DigitalInputHandle, ctx: *mut c_void, state: c_int) {
        if !ctx.is_null() {
            let cb: &mut Box<DigitalInputCallback> = &mut *(ctx as *mut _);
            let sensor = Self::from(chan);
            cb(&sensor, state as i32);
            mem::forget(sensor);
        }
    }

    /// Get a reference to the underlying sensor handle
    pub fn as_channel(&self) -> &DigitalInputHandle {
        &self.chan
    }

    /// Get the state of the digital input channel
    pub fn state(&self) -> Result<i32> {
        let mut state: c_int = 0;
        ReturnCode::result(unsafe { ffi::PhidgetDigitalInput_getState(self.chan, &mut state) })?;
        Ok(state as i32)
    }

    /// Sets a handler to receive digital input state change callbacks.
    pub fn set_on_state_change_handler<F>(&mut self, cb: F) -> Result<()>
        where
            F: Fn(&DigitalInput, i32) + Send + 'static,
    {
        // 1st box is fat ptr, 2nd is regular pointer.
        let cb: Box<Box<DigitalInputCallback>> = Box::new(Box::new(cb));
        let ctx = Box::into_raw(cb) as *mut c_void;
        self.cb = Some(ctx);

        ReturnCode::result(unsafe {
            ffi::PhidgetDigitalInput_setOnStateChangeHandler(
                self.chan,
                Some(Self::on_state_change),
                ctx,
            )
        })
    }

    /// Sets a handler to receive attach callbacks
    pub fn set_on_attach_handler<F>(&mut self, cb: F) -> Result<()>
        where
            F: Fn(&GenericPhidget) + Send + 'static,
    {
        let ctx = crate::phidget::set_on_attach_handler(self, cb)?;
        self.attach_cb = Some(ctx);
        Ok(())
    }

    /// Sets a handler to receive detach callbacks
    pub fn set_on_detach_handler<F>(&mut self, cb: F) -> Result<()>
        where
            F: Fn(&GenericPhidget) + Send + 'static,
    {
        let ctx = crate::phidget::set_on_detach_handler(self, cb)?;
        self.detach_cb = Some(ctx);
        Ok(())
    }
}

impl Phidget for DigitalInput {
    fn as_handle(&mut self) -> PhidgetHandle {
        self.chan as PhidgetHandle
    }
}

unsafe impl Send for DigitalInput {}

impl Default for DigitalInput {
    fn default() -> Self {
        Self::new()
    }
}

impl From<DigitalInputHandle> for DigitalInput {
    fn from(chan: DigitalInputHandle) -> Self {
        Self {
            chan,
            cb: None,
            attach_cb: None,
            detach_cb: None,
        }
    }
}

impl Drop for DigitalInput {
    fn drop(&mut self) {
        if let Ok(true) = self.is_open() {
            let _ = self.close();
        }
        unsafe {
            ffi::PhidgetDigitalInput_delete(&mut self.chan);
            crate::drop_cb::<DigitalInputCallback>(self.cb.take());
            crate::drop_cb::<AttachCallback>(self.attach_cb.take());
            crate::drop_cb::<DetachCallback>(self.detach_cb.take());
        }
    }
}

/////////////////////////////////////////////////////////////////////////////

/// Phidget digital output
#[derive(Debug)]
pub struct DigitalOutput {
    // Handle to the digital output in the phidget22 library
    chan: DigitalOutputHandle,
    // Double-boxed attach callback, if registered
    attach_cb: Option<*mut c_void>,
    // Double-boxed detach callback, if registered
    detach_cb: Option<*mut c_void>,
}

impl DigitalOutput {
    /// Create a new digital input.
    pub fn new() -> Self {
        let mut chan: DigitalOutputHandle = ptr::null_mut();
        unsafe {
            ffi::PhidgetDigitalOutput_create(&mut chan);
        }
        Self::from(chan)
    }

    /// Get the state of the digital output channel
    pub fn state(&self) -> Result<i32> {
        let mut state: c_int = 0;
        ReturnCode::result(unsafe { ffi::PhidgetDigitalOutput_getState(self.chan, &mut state) })?;
        Ok(state as i32)
    }

    /// Set the state of the digital output
    /// This overrides any duty cycle that was previously set.
    pub fn set_state(&self, state: i32) -> Result<()> {
        ReturnCode::result(unsafe { ffi::PhidgetDigitalOutput_setState(self.chan, state as c_int) })
    }

    /// Get the duty cycle of the digital output channel
    /// This is the fraction of the time the output is high. A value of 1.0
    /// means constantly high; 0.0 means constantly low
    pub fn duty_cycle(&self) -> Result<f64> {
        let mut dc: f64 = 0.0;
        ReturnCode::result(unsafe { ffi::PhidgetDigitalOutput_getDutyCycle(self.chan, &mut dc) })?;
        Ok(dc)
    }

    /// Set the duty cycle of the digital output
    /// This is the fraction of the time the output is high. A value of 1.0
    /// means constantly high; 0.0 means constantly low
    pub fn set_duty_cycle(&self, dc: f64) -> Result<()> {
        ReturnCode::result(unsafe { ffi::PhidgetDigitalOutput_setDutyCycle(self.chan, dc) })
    }

    /// Enables the failsafe feature for the channel, with a given failsafe time.
    ///
    /// The failsafe feature is intended for use in applications where it is important for the channel to enter a known safe state if the program controlling it locks up or crashes. If you do not enable the failsafe feature, the channel will carry out whatever instructions it was last given until it is explicitly told to stop.
    ///
    /// Enabling the failsafe feature starts a recurring failsafe timer for the channel. Once the failsafe timer is enabled, it must be reset within the specified time or the channel will enter a failsafe state. The failsafe timer may be reset by sending any valid command to the device*. Resetting the failsafe timer will reload the timer with the specified failsafe time, starting when the message to reset the timer is received by the Phidget.
    ///
    /// *(get requests do not typically send commands and won't reset the failsafe timer)
    ///
    /// For example: if the failsafe is enabled with a failsafe time of 1000ms, you will have 1000ms to reset the failsafe timer. Every time the failsafe timer is reset, you will have 1000ms from that time to reset the failsafe again.
    ///
    /// If the failsafe timer is not reset before it runs out, the channel will enter a failsafe state. For Digital Output channels, this will set the output State to FALSE. Once the channel enters the failsafe state, it will reject any further input until the channel is reopened.
    ///
    /// To prevent the channel from falsely entering the failsafe state, we recommend resetting the failsafe timer as frequently as is practical for your application. A good rule of thumb is to not let more than a third of the failsafe time pass before resetting the timer.
    ///
    /// Once the failsafe timer has been set, it cannot be disabled by any means other than closing and reopening the channel.
    pub fn enable_failsafe(&self, time: u32) -> Result<()> {
        ReturnCode::result(unsafe { ffi::PhidgetDigitalOutput_enableFailsafe(self.chan, time) })
    }

    /// Resets the failsafe timer, if one has been set. See PhidgetDigitalOutput_enableFailsafe() for details.
    ///
    ///    This function will fail if no failsafe timer has been set for the channel.
    pub fn reset_failsafe(&self) -> Result<()> {
        ReturnCode::result(unsafe { ffi::PhidgetDigitalOutput_resetFailsafe(self.chan) })
    }

    /// Sets a handler to receive attach callbacks
    pub fn set_on_attach_handler<F>(&mut self, cb: F) -> Result<()>
        where
            F: Fn(&GenericPhidget) + Send + 'static,
    {
        let ctx = crate::phidget::set_on_attach_handler(self, cb)?;
        self.attach_cb = Some(ctx);
        Ok(())
    }

    /// Sets a handler to receive detach callbacks
    pub fn set_on_detach_handler<F>(&mut self, cb: F) -> Result<()>
        where
            F: Fn(&GenericPhidget) + Send + 'static,
    {
        let ctx = crate::phidget::set_on_detach_handler(self, cb)?;
        self.detach_cb = Some(ctx);
        Ok(())
    }
}

impl Phidget for DigitalOutput {
    fn as_handle(&mut self) -> PhidgetHandle {
        self.chan as PhidgetHandle
    }
}

unsafe impl Send for DigitalOutput {}

impl Default for DigitalOutput {
    fn default() -> Self {
        Self::new()
    }
}

impl From<DigitalOutputHandle> for DigitalOutput {
    fn from(chan: DigitalOutputHandle) -> Self {
        Self {
            chan,
            attach_cb: None,
            detach_cb: None,
        }
    }
}

impl Drop for DigitalOutput {
    fn drop(&mut self) {
        if let Ok(true) = self.is_open() {
            let _ = self.close();
        }
        unsafe {
            ffi::PhidgetDigitalOutput_delete(&mut self.chan);
            crate::drop_cb::<AttachCallback>(self.attach_cb.take());
            crate::drop_cb::<DetachCallback>(self.detach_cb.take());
        }
    }
}
