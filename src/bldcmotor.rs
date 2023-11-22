use std::ffi::c_uint;
use std::mem;
use std::os::raw::c_void;
use crate::{Result, ReturnCode, Phidget, GenericPhidget, AttachCallback, DetachCallback};
use phidget_sys::*;


/// The function signature for the safe Rust velocity change callback.
pub type VelocityCallback = dyn Fn(&BLDCMotor, f64) + Send + 'static;

/// The function signature for the safe Rust position change callback.
pub type PositionChangeCallback = dyn Fn(&BLDCMotor, f64) + Send + 'static;


/// Represents a DC motor in the Phidgets system.
pub struct BLDCMotor {
    handle: PhidgetBLDCMotorHandle,
    // Double-boxed VelocityCallback, if registered
    velocity_cb: Option<*mut c_void>,
    // Double-boxed PositionChangeCallback, if registered
    position_cb: Option<*mut c_void>,
    // Double-boxed attach callback, if registered
    attach_cb: Option<*mut c_void>,
    // Double-boxed detach callback, if registered
    detach_cb: Option<*mut c_void>,
}

impl BLDCMotor {
    /// Creates a new DC motor control.
    pub fn new() -> Self {
        let mut handle = std::ptr::null_mut();
        unsafe { PhidgetBLDCMotor_create(&mut handle) };
        Self::from(handle)
    }


    /// Get a reference to the underlying sensor handle
    pub fn as_channel(&self) -> &PhidgetBLDCMotorHandle {
        &self.handle
    }

    /// Sets the velocity of the motor.
    pub fn set_target_velocity(&self, velocity: f64) -> Result<()> {
        ReturnCode::result(unsafe { PhidgetBLDCMotor_setTargetVelocity(self.handle, velocity) })
    }

    /// Gets the velocity of the motor.
    pub fn get_target_velocity(&self) -> Result<f64> {
        let mut velocity = 0.0;
        ReturnCode::result(unsafe { PhidgetBLDCMotor_getTargetVelocity(self.handle, &mut velocity) })?;
        Ok(velocity)
    }

    /// Gets the minimum velocity of the motor.
    pub fn get_min_velocity(&self) -> Result<f64> {
        let mut min_velocity = 0.0;
        ReturnCode::result(unsafe { PhidgetBLDCMotor_getMinVelocity(self.handle, &mut min_velocity) })?;
        Ok(min_velocity)
    }

    /// Gets the maximum velocity of the motor.
    pub fn get_max_velocity(&self) -> Result<f64> {
        let mut max_velocity = 0.0;
        ReturnCode::result(unsafe { PhidgetBLDCMotor_getMaxVelocity(self.handle, &mut max_velocity) })?;
        Ok(max_velocity)
    }

    /// Gets the velocity of the motor.
    pub fn get_velocity(&self) -> Result<f64> {
        let mut velocity = 0.0;
        ReturnCode::result(unsafe { PhidgetBLDCMotor_getVelocity(self.handle, &mut velocity) })?;
        Ok(velocity)
    }


    /// The rate at which the controller can change the motor's Velocity.
    ///
    ///     The acceleration is bounded by MinAcceleration and MaxAcceleration
    pub fn set_acceleration(&self, acceleration: f64) -> Result<()> {
        ReturnCode::result(unsafe { PhidgetBLDCMotor_setAcceleration(self.handle, acceleration) })
    }

    /// The rate at which the controller can change the motor's Velocity.
    ///
    ///     The acceleration is bounded by MinAcceleration and MaxAcceleration
    pub fn get_acceleration(&self) -> Result<f64> {
        let mut acceleration = 0.0;
        ReturnCode::result(unsafe { PhidgetBLDCMotor_getAcceleration(self.handle, &mut acceleration) })?;
        Ok(acceleration)
    }

    /// Gets the minimum acceleration of the motor.
    pub fn get_min_acceleration(&self) -> Result<f64> {
        let mut min_acceleration = 0.0;
        ReturnCode::result(unsafe { PhidgetBLDCMotor_getMinAcceleration(self.handle, &mut min_acceleration) })?;
        Ok(min_acceleration)
    }

    /// Gets the maximum acceleration of the motor.
    pub fn get_max_acceleration(&self) -> Result<f64> {
        let mut max_acceleration = 0.0;
        ReturnCode::result(unsafe { PhidgetBLDCMotor_getMaxAcceleration(self.handle, &mut max_acceleration) })?;
        Ok(max_acceleration)
    }

    /// When a motor is not being actively driven forward or reverse, you can choose if the motor will be allowed to freely turn, or will resist being turned.
    ///
    ///  * A low TargetBrakingStrength value corresponds to free wheeling, this will have the following effects:
    ///     * The motor will continue to rotate after the controller is no longer driving the motor (i.e. Velocity is 0), due to inertia.
    ///     * The motor shaft will provide little resistance to being turned when it is stopped.
    ///  * A higher TargetBrakingStrength value will resist being turned, this will have the following effects:
    ///     * The motor will more stop more quickly if it is in motion and braking has been requested. It will fight against the rotation of the shaft.
    ///  * Braking mode is enabled by setting the Velocity to MinVelocity
    pub fn set_target_braking_strength(&self, strength: f64) -> Result<()> {
        ReturnCode::result(unsafe { PhidgetBLDCMotor_setTargetBrakingStrength(self.handle, strength) })
    }


    /// Gets the braking strength of the motor.
    pub fn get_target_braking_strength(&self) -> Result<f64> {
        let mut braking_strength = 0.0;
        ReturnCode::result(unsafe { PhidgetBLDCMotor_getTargetBrakingStrength(self.handle, &mut braking_strength) })?;
        Ok(braking_strength)
    }

    /// Gets the minimum breaking strength of the motor.
    pub fn get_min_braking_strength(&self) -> Result<f64> {
        let mut min_braking_strength = 0.0;
        ReturnCode::result(unsafe { PhidgetBLDCMotor_getMinBrakingStrength(self.handle, &mut min_braking_strength) })?;
        Ok(min_braking_strength)
    }

    /// Gets the maximum breaking strength of the motor.
    pub fn get_max_braking_strength(&self) -> Result<f64> {
        let mut max_braking_strength = 0.0;
        ReturnCode::result(unsafe { PhidgetBLDCMotor_getMaxBrakingStrength(self.handle, &mut max_braking_strength) })?;
        Ok(max_braking_strength)
    }

    /// The controller will limit the current through the motor to the currentLimit value.
    pub fn set_current_limit(&self, limit: f64) -> Result<()> {
        ReturnCode::result(unsafe { PhidgetBLDCMotor_setCurrentLimit(self.handle, limit) })
    }

    /// The controller will limit the current through the motor to the currentLimit value.
    pub fn get_current_limit(&self) -> Result<f64> {
        let mut current_limit = 0.0;
        ReturnCode::result(unsafe { PhidgetBLDCMotor_getCurrentLimit(self.handle, &mut current_limit) })?;
        Ok(current_limit)
    }

    /// Gets the minimum current limit of the motor.
    pub fn get_min_current_limit(&self) -> Result<f64> {
        let mut min_current_limit = 0.0;
        ReturnCode::result(unsafe { PhidgetBLDCMotor_getMinCurrentLimit(self.handle, &mut min_current_limit) })?;
        Ok(min_current_limit)
    }

    /// Gets the maximum current limit of the motor.
    pub fn get_max_current_limit(&self) -> Result<f64> {
        let mut max_current_limit = 0.0;
        ReturnCode::result(unsafe { PhidgetBLDCMotor_getMaxCurrentLimit(self.handle, &mut max_current_limit) })?;
        Ok(max_current_limit)
    }

    /// Sets the failsafe time of the motor.
    pub fn get_min_failsafe_time(&self) -> Result<u32> {
        let mut min_failsafe_time: c_uint = 0;
        ReturnCode::result(unsafe { PhidgetBLDCMotor_getMinFailsafeTime(self.handle, &mut min_failsafe_time) })?;
        Ok(min_failsafe_time)
    }

    /// gets the maximum failsafe time of the motor.
    pub fn get_max_failsafe_time(&self) -> Result<u32> {
        let mut max_failsafe_time: c_uint = 0;
        ReturnCode::result(unsafe { PhidgetBLDCMotor_getMaxFailsafeTime(self.handle, &mut max_failsafe_time) })?;
        Ok(max_failsafe_time)
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
    /// If the failsafe timer is not reset before it runs out, the channel will enter a failsafe state. For BLDC Motor channels, this will disengage the motor. Once the channel enters the failsafe state, it will reject any further input until the channel is reopened.
    ///
    /// To prevent the channel from falsely entering the failsafe state, we recommend resetting the failsafe timer as frequently as is practical for your application. A good rule of thumb is to not let more than a third of the failsafe time pass before resetting the timer.
    ///
    /// Once the failsafe timer has been set, it cannot be disabled by any means other than closing and reopening the channel.

    pub fn enable_fail_safe(&self, failsafe_time: i32) -> Result<()> {
        ReturnCode::result(unsafe { PhidgetBLDCMotor_enableFailsafe(self.handle, failsafe_time as c_uint) })
    }

    /// Gets the position of the motor.
    pub fn get_position(&self) -> Result<f64> {
        let mut position = 0.0;
        ReturnCode::result(unsafe { PhidgetBLDCMotor_getPosition(self.handle, &mut position) })?;
        Ok(position)
    }

    /// Gets the minimum position of the motor.
    pub fn get_min_position(&self) -> Result<f64> {
        let mut min_position = 0.0;
        ReturnCode::result(unsafe { PhidgetBLDCMotor_getMinPosition(self.handle, &mut min_position) })?;
        Ok(min_position)
    }

    /// gets the maximum position of the motor.
    pub fn get_max_position(&self) -> Result<f64> {
        let mut max_position = 0.0;
        ReturnCode::result(unsafe { PhidgetBLDCMotor_getMaxPosition(self.handle, &mut max_position) })?;
        Ok(max_position)
    }

    /// Sets the position of the motor.
    pub fn set_rescale_factor(&self, rescale_factor: f64) -> Result<()> {
        ReturnCode::result(unsafe { PhidgetBLDCMotor_setRescaleFactor(self.handle, rescale_factor) })
    }

    /// Gets the rescale factor of the motor.
    pub fn get_rescale_factor(&self) -> Result<f64> {
        let mut rescale_factor = 0.0;
        ReturnCode::result(unsafe { PhidgetBLDCMotor_getRescaleFactor(self.handle, &mut rescale_factor) })?;
        Ok(rescale_factor)
    }

    /// Sets the stall velocity of the motor.
    pub fn set_stall_velocity(&self, stall_velocity: f64) -> Result<()> {
        ReturnCode::result(unsafe { PhidgetBLDCMotor_setStallVelocity(self.handle, stall_velocity) })
    }

    /// Gets the stall velocity of the motor.
    pub fn get_stall_velocity(&self) -> Result<f64> {
        let mut stall_velocity = 0.0;
        ReturnCode::result(unsafe { PhidgetBLDCMotor_getStallVelocity(self.handle, &mut stall_velocity) })?;
        Ok(stall_velocity)
    }

    /// Gets the minimum stall velocity of the motor.
    pub fn get_min_stall_velocity(&self) -> Result<f64> {
        let mut min_stall_velocity = 0.0;
        ReturnCode::result(unsafe { PhidgetBLDCMotor_getMinStallVelocity(self.handle, &mut min_stall_velocity) })?;
        Ok(min_stall_velocity)
    }

    /// Gets the maximum stall velocity of the motor.
    pub fn get_max_stall_velocity(&self) -> Result<f64> {
        let mut max_stall_velocity = 0.0;
        ReturnCode::result(unsafe { PhidgetBLDCMotor_getMaxStallVelocity(self.handle, &mut max_stall_velocity) })?;
        Ok(max_stall_velocity)
    }

    /// The DataInterval is the time that must elapse before the channel will fire another VelocityUpdate / PositionChange / BrakingStrengthChange event.
    ///
    /// * The data interval is bounded by MinDataInterval and MaxDataInterval.
    pub fn set_data_interval(&self, data_interval: u32) -> Result<()> {
        ReturnCode::result(unsafe { PhidgetBLDCMotor_setDataInterval(self.handle, data_interval) })
    }

    /// The DataInterval is the time that must elapse before the channel will fire another VelocityUpdate / PositionChange / BrakingStrengthChange event.
    ///
    /// * The data interval is bounded by MinDataInterval and MaxDataInterval.
    pub fn get_data_interval(&self) -> Result<u32> {
        let mut data_interval: c_uint = 0;
        ReturnCode::result(unsafe { PhidgetBLDCMotor_getDataInterval(self.handle, &mut data_interval) })?;
        Ok(data_interval)
    }

    /// Gets the minimunm value for the data interval.
    pub fn get_min_data_interval(&self) -> Result<u32> {
        let mut min_data_interval: c_uint = 0;
        ReturnCode::result(unsafe { PhidgetBLDCMotor_getMinDataInterval(self.handle, &mut min_data_interval) })?;
        Ok(min_data_interval)
    }

    /// Gets the maximum value for the data interval.
    pub fn get_max_data_interval(&self) -> Result<u32> {
        let mut max_data_interval: c_uint = 0;
        ReturnCode::result(unsafe { PhidgetBLDCMotor_getMaxDataInterval(self.handle, &mut max_data_interval) })?;
        Ok(max_data_interval)
    }

    /// The DataRate is the frequency of events from the device.
    ///
    ///     The data rate is bounded by MinDataRate and MaxDataRate.
    ///     Changing DataRate will change the channel's DataInterval to a corresponding value, rounded to the nearest integer number of milliseconds.
    ///     The timing between events can also affected by the change trigger.
    pub fn set_data_rate(&self, data_rate: f64) -> Result<()> {
        ReturnCode::result(unsafe { PhidgetBLDCMotor_setDataRate(self.handle, data_rate) })
    }

    pub fn get_data_rate(&self) -> Result<f64> {
        let mut data_rate = 0.0;
        ReturnCode::result(unsafe { PhidgetBLDCMotor_getDataRate(self.handle, &mut data_rate) })?;
        Ok(data_rate)
    }

    pub fn get_min_data_rate(&self) -> Result<f64> {
        let mut min_data_rate = 0.0;
        ReturnCode::result(unsafe { PhidgetBLDCMotor_getMinDataRate(self.handle, &mut min_data_rate) })?;
        Ok(min_data_rate)
    }

    pub fn get_max_data_rate(&self) -> Result<f64> {
        let mut max_data_rate = 0.0;
        ReturnCode::result(unsafe { PhidgetBLDCMotor_getMaxDataRate(self.handle, &mut max_data_rate) })?;
        Ok(max_data_rate)
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
            F: Fn(&BLDCMotor, f64) + Send + 'static,
    {
        // 1st box is fat ptr, 2nd is regular pointer.
        let cb: Box<Box<VelocityCallback>> = Box::new(Box::new(cb));
        let ctx = Box::into_raw(cb) as *mut c_void;
        self.velocity_cb = Some(ctx);
        ReturnCode::result(unsafe {
            PhidgetBLDCMotor_setOnVelocityUpdateHandler(
                self.handle,
                Some(Self::on_velocity_change),
                ctx,
            )
        })
    }

    // Low-level, unsafe, callback for the velocity change event.
    // The context is a double-boxed pointer to the safe Rust callback.
    unsafe extern "C" fn on_velocity_change(chan: PhidgetBLDCMotorHandle, ctx: *mut c_void, state: f64) {
        if !ctx.is_null() {
            let cb: &mut Box<VelocityCallback> = &mut *(ctx as *mut _);
            let motor = Self::from(chan);
            cb(&motor, state as f64);
            mem::forget(motor);
        }
    }

    /// The most recent position value will be reported in this event, which occurs when the DataInterval has elapsed.
    ///
    ///    * Regardless of the DataInterval, this event will occur only when the position value has changed from the previous value reported.
    ///    * Position values are calculated using Hall Effect sensors mounted on the motor, therefore, the resolution of position depends on the motor you are using.
    ///    * Units for Position can be set by the user through the RescaleFactor. The RescaleFactor allows you to use more intuitive units such as rotations, or degrees. For more information on how to apply the RescaleFactor to your application, see your controller's User Guide.
    pub fn set_on_position_change_handler<F>(&mut self, cb: F) -> Result<()>
        where
            F: Fn(&BLDCMotor, f64) + Send + 'static,
    {
        // 1st box is fat ptr, 2nd is regular pointer.
        let cb: Box<Box<VelocityCallback>> = Box::new(Box::new(cb));
        let ctx = Box::into_raw(cb) as *mut c_void;
        self.position_cb = Some(ctx);
        ReturnCode::result(unsafe {
            PhidgetBLDCMotor_setOnPositionChangeHandler(
                self.handle,
                Some(Self::on_position_change),
                ctx,
            )
        })
    }

    unsafe extern "C" fn on_position_change(chan: PhidgetBLDCMotorHandle, ctx: *mut c_void, state: f64) {
        if !ctx.is_null() {
            let cb: &mut Box<VelocityCallback> = &mut *(ctx as *mut _);
            let motor = Self::from(chan);
            cb(&motor, state as f64);
            mem::forget(motor);
        }
    }
}

impl Drop for BLDCMotor {
    fn drop(&mut self) {
        if let Ok(true) = self.is_open() {
            let _ = self.close();
        }
        unsafe {
            PhidgetBLDCMotor_delete(&mut self.handle);
            crate::drop_cb::<VelocityCallback>(self.position_cb.take());
            crate::drop_cb::<VelocityCallback>(self.velocity_cb.take());
            crate::drop_cb::<AttachCallback>(self.attach_cb.take());
            crate::drop_cb::<DetachCallback>(self.detach_cb.take());
        }
    }
}

impl Phidget for BLDCMotor {
    fn as_handle(&mut self) -> PhidgetHandle {
        self.handle as PhidgetHandle
    }
}

unsafe impl Send for BLDCMotor {}

impl Default for BLDCMotor {
    fn default() -> Self {
        Self::new()
    }
}


impl From<PhidgetBLDCMotorHandle> for BLDCMotor {
    fn from(handle: PhidgetBLDCMotorHandle) -> Self {
        Self {
            handle,
            position_cb: None,
            velocity_cb: None,
            attach_cb: None,
            detach_cb: None,
        }
    }
}
