// phidget-rs/src/voltage_io.rs
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
    self as ffi, PhidgetHandle, PhidgetVoltageInputHandle as VoltageInputHandle,
    PhidgetVoltageOutputHandle as VoltageOutputHandle,
};
use std::{mem, os::raw::c_void, ptr};

/// The function signature for the safe Rust voltage change callback.
pub type VoltageChangeCallback = dyn Fn(&VoltageInput, f64) + Send + 'static;

/////////////////////////////////////////////////////////////////////////////

/// Phidget voltage input
#[derive(Debug)]
pub struct VoltageInput {
    // Handle to the voltage input in the phidget22 library
    chan: VoltageInputHandle,
    // Double-boxed VoltageChangeCallback, if registered
    cb: Option<*mut c_void>,
    // Double-boxed attach callback, if registered
    attach_cb: Option<*mut c_void>,
    // Double-boxed detach callback, if registered
    detach_cb: Option<*mut c_void>,
}

impl VoltageInput {
    /// Create a new voltage input.
    pub fn new() -> Self {
        let mut chan: VoltageInputHandle = ptr::null_mut();
        unsafe {
            ffi::PhidgetVoltageInput_create(&mut chan);
        }
        Self::from(chan)
    }

    // Low-level, unsafe, callback for the voltage change event.
    // The context is a double-boxed pointer to the safe Rust callback.
    unsafe extern "C" fn on_voltage_change(
        chan: VoltageInputHandle,
        ctx: *mut c_void,
        voltage: f64,
    ) {
        if !ctx.is_null() {
            let cb: &mut Box<VoltageChangeCallback> = &mut *(ctx as *mut _);
            let sensor = Self::from(chan);
            cb(&sensor, voltage);
            mem::forget(sensor);
        }
    }

    /// Get a reference to the underlying sensor handle
    pub fn as_channel(&self) -> &VoltageInputHandle {
        &self.chan
    }

    /// Get the voltage on the input channel
    pub fn voltage(&self) -> Result<f64> {
        let mut v: f64 = 0.0;
        ReturnCode::result(unsafe { ffi::PhidgetVoltageInput_getVoltage(self.chan, &mut v) })?;
        Ok(v)
    }

    /// Sets a handler to receive voltage change callbacks.
    pub fn set_on_voltage_change_handler<F>(&mut self, cb: F) -> Result<()>
    where
        F: Fn(&VoltageInput, f64) + Send + 'static,
    {
        // 1st box is fat ptr, 2nd is regular pointer.
        let cb: Box<Box<VoltageChangeCallback>> = Box::new(Box::new(cb));
        let ctx = Box::into_raw(cb) as *mut c_void;
        self.cb = Some(ctx);

        ReturnCode::result(unsafe {
            ffi::PhidgetVoltageInput_setOnVoltageChangeHandler(
                self.chan,
                Some(Self::on_voltage_change),
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

impl Phidget for VoltageInput {
    fn as_handle(&mut self) -> PhidgetHandle {
        self.chan as PhidgetHandle
    }
}

unsafe impl Send for VoltageInput {}

impl Default for VoltageInput {
    fn default() -> Self {
        Self::new()
    }
}

impl From<VoltageInputHandle> for VoltageInput {
    fn from(chan: VoltageInputHandle) -> Self {
        Self {
            chan,
            cb: None,
            attach_cb: None,
            detach_cb: None,
        }
    }
}

impl Drop for VoltageInput {
    fn drop(&mut self) {
        if let Ok(true) = self.is_open() {
            let _ = self.close();
        }
        unsafe {
            ffi::PhidgetVoltageInput_delete(&mut self.chan);
            crate::drop_cb::<VoltageChangeCallback>(self.cb.take());
            crate::drop_cb::<AttachCallback>(self.attach_cb.take());
            crate::drop_cb::<DetachCallback>(self.detach_cb.take());
        }
    }
}

/////////////////////////////////////////////////////////////////////////////

/// Phidget voltage output
#[derive(Debug)]
pub struct VoltageOutput {
    // Handle to the voltage output in the phidget22 library
    chan: VoltageOutputHandle,
    // Double-boxed attach callback, if registered
    attach_cb: Option<*mut c_void>,
    // Double-boxed detach callback, if registered
    detach_cb: Option<*mut c_void>,
}

impl VoltageOutput {
    /// Create a new voltage input.
    pub fn new() -> Self {
        let mut chan: VoltageOutputHandle = ptr::null_mut();
        unsafe {
            ffi::PhidgetVoltageOutput_create(&mut chan);
        }
        Self::from(chan)
    }

    /// Get the voltage value that the channel will output
    pub fn voltage(&self) -> Result<f64> {
        let mut v: f64 = 0.0;
        ReturnCode::result(unsafe { ffi::PhidgetVoltageOutput_getVoltage(self.chan, &mut v) })?;
        Ok(v)
    }

    /// Set the voltage value that the channel will output.
    pub fn set_voltage(&self, v: f64) -> Result<()> {
        ReturnCode::result(unsafe { ffi::PhidgetVoltageOutput_setVoltage(self.chan, v) })
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

impl Phidget for VoltageOutput {
    fn as_handle(&mut self) -> PhidgetHandle {
        self.chan as PhidgetHandle
    }
}

unsafe impl Send for VoltageOutput {}

impl Default for VoltageOutput {
    fn default() -> Self {
        Self::new()
    }
}

impl From<VoltageOutputHandle> for VoltageOutput {
    fn from(chan: VoltageOutputHandle) -> Self {
        Self {
            chan,
            attach_cb: None,
            detach_cb: None,
        }
    }
}

impl Drop for VoltageOutput {
    fn drop(&mut self) {
        if let Ok(true) = self.is_open() {
            let _ = self.close();
        }
        unsafe {
            ffi::PhidgetVoltageOutput_delete(&mut self.chan);
            crate::drop_cb::<AttachCallback>(self.attach_cb.take());
            crate::drop_cb::<DetachCallback>(self.detach_cb.take());
        }
    }
}
