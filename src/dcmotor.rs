// phidget-rs/src/dcmotor
//
// This file is part of the 'phidget-rs' library.
// Provides a safe Rust wrapper for the DC motor control using the Phidget22 library.

use std::ffi::c_uint;
use std::mem;
use std::os::raw::c_void;
use crate::{Result, ReturnCode, Phidget, GenericPhidget, AttachCallback, DetachCallback};
use phidget_sys::*;



/// The function signature for the safe Rust digital input state change callback.
pub type VelocityCallback = dyn Fn(&DCMotor, f64) + Send + 'static;


/// Represents a DC motor in the Phidgets system.
pub struct DCMotor {
    handle: PhidgetDCMotorHandle,
    // Double-boxed VelocityCallback, if registered
    velocity_cb: Option<*mut c_void>,
    // Double-boxed attach callback, if registered
    attach_cb: Option<*mut c_void>,
    // Double-boxed detach callback, if registered
    detach_cb: Option<*mut c_void>,
}

impl DCMotor {
    /// Creates a new DC motor control.
    pub fn new() -> Self {
        let mut handle = std::ptr::null_mut();
        unsafe { PhidgetDCMotor_create(&mut handle) };
        Self::from(handle)
    }


    /// Get a reference to the underlying sensor handle
    pub fn as_channel(&self) -> &PhidgetDCMotorHandle {
        &self.handle
    }

    /// Sets the velocity of the motor.
    pub fn set_target_velocity(&self, velocity: f64) -> Result<()> {
        ReturnCode::result(unsafe { PhidgetDCMotor_setTargetVelocity(self.handle, velocity) })
    }

    /// Gets the velocity of the motor.
    pub fn get_target_velocity(&self) -> Result<f64> {
        let mut velocity = 0.0;
        ReturnCode::result(unsafe { PhidgetDCMotor_getTargetVelocity(self.handle, &mut velocity) })?;
        Ok(velocity)
    }

    /// Gets the minimum velocity of the motor.
    pub fn get_min_velocity(&self) -> Result<f64> {
        let mut min_velocity = 0.0;
        ReturnCode::result(unsafe { PhidgetDCMotor_getMinVelocity(self.handle, &mut min_velocity) })?;
        Ok(min_velocity)
    }

    /// Gets the maximum velocity of the motor.
    pub fn get_max_velocity(&self) -> Result<f64> {
        let mut max_velocity = 0.0;
        ReturnCode::result(unsafe { PhidgetDCMotor_getMaxVelocity(self.handle, &mut max_velocity) })?;
        Ok(max_velocity)
    }

    /// Gets the velocity of the motor.
    pub fn get_velocity(&self) -> Result<f64> {
        let mut velocity = 0.0;
        ReturnCode::result(unsafe { PhidgetDCMotor_getVelocity(self.handle, &mut velocity) })?;
        Ok(velocity)
    }


    /// Sets the acceleration of the motor.
    pub fn set_acceleration(&self, acceleration: f64) -> Result<()> {
        ReturnCode::result(unsafe { PhidgetDCMotor_setAcceleration(self.handle, acceleration) })
    }

    /// Gets the minimum acceleration of the motor.
    pub fn get_min_acceleration(&self) -> Result<f64> {
        let mut min_acceleration = 0.0;
        ReturnCode::result(unsafe { PhidgetDCMotor_getMinAcceleration(self.handle, &mut min_acceleration) })?;
        Ok(min_acceleration)
    }

    /// Gets the maximum acceleration of the motor.
    pub fn get_max_acceleration(&self) -> Result<f64> {
        let mut max_acceleration = 0.0;
        ReturnCode::result(unsafe { PhidgetDCMotor_getMaxAcceleration(self.handle, &mut max_acceleration) })?;
        Ok(max_acceleration)
    }

    /// Sets the braking strength of the motor.
    pub fn set_target_braking_strength(&self, strength: f64) -> Result<()> {
        ReturnCode::result(unsafe { PhidgetDCMotor_setTargetBrakingStrength(self.handle, strength) })
    }

    /// Gets the minimum breaking strength of the motor.
    pub fn get_min_braking_strength(&self) -> Result<f64> {
        let mut min_braking_strength = 0.0;
        ReturnCode::result(unsafe { PhidgetDCMotor_getMinBrakingStrength(self.handle, &mut min_braking_strength) })?;
        Ok(min_braking_strength)
    }

    /// Gets the maximum breaking strength of the motor.
    pub fn get_max_braking_strength(&self) -> Result<f64> {
        let mut max_braking_strength = 0.0;
        ReturnCode::result(unsafe { PhidgetDCMotor_getMaxBrakingStrength(self.handle, &mut max_braking_strength) })?;
        Ok(max_braking_strength)
    }

    /// Sets the current limit of the motor.
    pub fn set_current_limit(&self, limit: f64) -> Result<()> {
        ReturnCode::result(unsafe { PhidgetDCMotor_setCurrentLimit(self.handle, limit) })
    }

    /// Gets the current limit of the motor.
    pub fn get_current_limit(&self) -> Result<f64> {
        let mut current_limit = 0.0;
        ReturnCode::result(unsafe { PhidgetDCMotor_getCurrentLimit(self.handle, &mut current_limit) })?;
        Ok(current_limit)
    }

    /// Gets the minimum current limit of the motor.
    pub fn get_min_current_limit(&self) -> Result<f64> {
        let mut min_current_limit = 0.0;
        ReturnCode::result(unsafe { PhidgetDCMotor_getMinCurrentLimit(self.handle, &mut min_current_limit) })?;
        Ok(min_current_limit)
    }

    /// Gets the maximum current limit of the motor.
    pub fn get_max_current_limit(&self) -> Result<f64> {
        let mut max_current_limit = 0.0;
        ReturnCode::result(unsafe { PhidgetDCMotor_getMaxCurrentLimit(self.handle, &mut max_current_limit) })?;
        Ok(max_current_limit)
    }

    /// Sets the failsafe time of the motor.
    pub fn get_min_failsafe_time(&self) -> Result<u32> {
        let mut min_failsafe_time:c_uint = 0;
        ReturnCode::result(unsafe { PhidgetDCMotor_getMinFailsafeTime(self.handle, &mut min_failsafe_time) })?;
        Ok(min_failsafe_time)
    }

    /// Gets the maximum failsafe time of the motor.
    pub fn get_max_failsafe_time(&self) -> Result<u32> {
        let mut max_failsafe_time: c_uint = 0;
        ReturnCode::result(unsafe { PhidgetDCMotor_getMaxFailsafeTime(self.handle, &mut max_failsafe_time) })?;
        Ok(max_failsafe_time)
    }

    /// Enables fail safe mode for the motor.
    pub fn enable_fail_safe(&self, failsafe_time: i32) -> Result<()> {
        ReturnCode::result(unsafe { PhidgetDCMotor_enableFailsafe(self.handle, failsafe_time as c_uint) })
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

    /// Sets a handler to receive velocity change callbacks.
    pub fn set_on_velocity_change_handler<F>(&mut self, cb: F) -> Result<()>
        where
            F: Fn(&DCMotor, f64) + Send + 'static,
    {
        // 1st box is fat ptr, 2nd is regular pointer.
        let cb: Box<Box<VelocityCallback>> = Box::new(Box::new(cb));
        let ctx = Box::into_raw(cb) as *mut c_void;
        ReturnCode::result(unsafe {
            PhidgetDCMotor_setOnVelocityUpdateHandler(
                self.handle,
                Some(Self::on_velocity_change),
                ctx,
            )
        })
    }

    // Low-level, unsafe, callback for thevelocity change event.
    // The context is a double-boxed pointer to the safe Rust callback.
    unsafe extern "C" fn on_velocity_change(chan: PhidgetDCMotorHandle, ctx: *mut c_void, state: f64) {
        if !ctx.is_null() {
            let cb: &mut Box<VelocityCallback> = &mut *(ctx as *mut _);
            let motor = Self::from(chan);
            cb(&motor, state as f64);
            mem::forget(motor);
        }
    }
}

impl Drop for DCMotor {
    fn drop(&mut self) {
        if let Ok(true) = self.is_open() {
            let _ = self.close();
        }
        unsafe {
            PhidgetDCMotor_delete(&mut self.handle);
            crate::drop_cb::<VelocityCallback>(self.velocity_cb.take());
            crate::drop_cb::<AttachCallback>(self.attach_cb.take());
            crate::drop_cb::<DetachCallback>(self.detach_cb.take());
        }
    }
}

impl Phidget for DCMotor {
    fn as_handle(&mut self) -> PhidgetHandle {
        self.handle as PhidgetHandle
    }
}

unsafe impl Send for DCMotor {}

impl Default for DCMotor {
    fn default() -> Self {
        Self::new()
    }
}


impl From<PhidgetDCMotorHandle> for DCMotor {
    fn from(handle: PhidgetDCMotorHandle) -> Self {
        Self {
            handle,
            velocity_cb: None,
            attach_cb: None,
            detach_cb: None,
        }
    }
}
