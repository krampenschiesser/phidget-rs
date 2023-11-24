// phidget-rs/src/temperature_sensor.rs
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
    self as ffi, PhidgetHandle, PhidgetTemperatureSensorHandle as TemperatureSensorHandle,
};
use std::{mem, os::raw::c_void, ptr};
use std::ffi::c_uint;

/// The function type for the safe Rust temperature change callback.
pub type TemperatureCallback = dyn Fn(&TemperatureSensor, f64) + Send + 'static;

/// The RTDType must correspond to the RTD type you are using in your application.
#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum ThermoCoupleType {
    /// Configures the thermocouple input as a J-Type thermocouple.
    TypeJ = ffi::PhidgetTemperatureSensor_ThermocoupleType_THERMOCOUPLE_TYPE_J as isize,
    /// Configures the thermocouple input as a K-Type thermocouple.
    TypeK = ffi::PhidgetTemperatureSensor_ThermocoupleType_THERMOCOUPLE_TYPE_K as isize,
    /// Configures the thermocouple input as a E-Type thermocouple.
    TypeE = ffi::PhidgetTemperatureSensor_ThermocoupleType_THERMOCOUPLE_TYPE_E as isize,
    /// Configures the thermocouple input as a T-Type thermocouple.
    TypeT = ffi::PhidgetTemperatureSensor_ThermocoupleType_THERMOCOUPLE_TYPE_T as isize,
}

impl Into<c_uint> for ThermoCoupleType {
    fn into(self) -> c_uint {
        self as c_uint
    }
}

impl From<c_uint> for ThermoCoupleType {
    fn from(value: c_uint) -> Self {
        if value == ffi::PhidgetTemperatureSensor_ThermocoupleType_THERMOCOUPLE_TYPE_J as c_uint {
            ThermoCoupleType::TypeJ
        } else if value == ffi::PhidgetTemperatureSensor_ThermocoupleType_THERMOCOUPLE_TYPE_K as c_uint {
            ThermoCoupleType::TypeK
        } else if value == ffi::PhidgetTemperatureSensor_ThermocoupleType_THERMOCOUPLE_TYPE_E as c_uint {
            ThermoCoupleType::TypeE
        } else if value == ffi::PhidgetTemperatureSensor_ThermocoupleType_THERMOCOUPLE_TYPE_T as c_uint {
            ThermoCoupleType::TypeT
        } else {
            panic!("Invalid ThermoCoupleType value")
        }
    }
}

/// The RTDType must correspond to the RTD type you are using in your application.
#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum RtdType {
    /// Configures the RTD type as a PT100 with a 3850ppm curve.
    PT100_3850 = ffi::PhidgetTemperatureSensor_RTDType_RTD_TYPE_PT100_3850 as isize,
    /// Configures the RTD type as a PT100 with a 3920ppm curve.
    PT100_3920 = ffi::PhidgetTemperatureSensor_RTDType_RTD_TYPE_PT100_3920 as isize,
    /// Configures the RTD type as a PT1000 with a 3850ppm curve.
    PT1000_3850 = ffi::PhidgetTemperatureSensor_RTDType_RTD_TYPE_PT1000_3850 as isize,
    /// Configures the RTD type as a PT1000 with a 3920ppm curve.
    PT1000_3920 = ffi::PhidgetTemperatureSensor_RTDType_RTD_TYPE_PT1000_3920 as isize,
}

impl Into<c_uint> for RtdType {
    fn into(self) -> c_uint {
        self as c_uint
    }
}

impl From<c_uint> for RtdType {
    fn from(value: c_uint) -> Self {
        if value == ffi::PhidgetTemperatureSensor_RTDType_RTD_TYPE_PT100_3850 as c_uint {
            RtdType::PT100_3850
        } else if value == ffi::PhidgetTemperatureSensor_RTDType_RTD_TYPE_PT100_3920 as c_uint {
            RtdType::PT100_3920
        } else if value == ffi::PhidgetTemperatureSensor_RTDType_RTD_TYPE_PT1000_3850 as c_uint {
            RtdType::PT1000_3850
        } else if value == ffi::PhidgetTemperatureSensor_RTDType_RTD_TYPE_PT1000_3920 as c_uint {
            RtdType::PT1000_3920
        } else {
            panic!("Invalid RtdType value")
        }
    }
}

/// The RTDWireSetup must correspond to the wire configuration you are using in your application.
//
//     If you are unsure which RTDWireSetup to use, visit your device's User Guide for more information.
#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum RtdWireSetup {
    /// Configures the device to make resistance calculations based on a 2-wire RTD setup.
    Wires2 = ffi::Phidget_RTDWireSetup_RTD_WIRE_SETUP_2WIRE as isize,
    /// Configures the device to make resistance calculations based on a 3-wire RTD setup.
    Wires3 = ffi::Phidget_RTDWireSetup_RTD_WIRE_SETUP_3WIRE as isize,
    /// Configures the device to make resistance calculations based on a 4-wire RTD setup.
    Wires4 = ffi::Phidget_RTDWireSetup_RTD_WIRE_SETUP_4WIRE as isize,
}

impl Into<c_uint> for RtdWireSetup {
    fn into(self) -> c_uint {
        self as c_uint
    }
}

impl From<c_uint> for RtdWireSetup {
    fn from(value: c_uint) -> Self {
        if value == ffi::Phidget_RTDWireSetup_RTD_WIRE_SETUP_2WIRE as c_uint {
            RtdWireSetup::Wires2
        } else if value == ffi::Phidget_RTDWireSetup_RTD_WIRE_SETUP_3WIRE as c_uint {
            RtdWireSetup::Wires3
        } else if value == ffi::Phidget_RTDWireSetup_RTD_WIRE_SETUP_4WIRE as c_uint {
            RtdWireSetup::Wires4
        } else {
            panic!("Invalid RtdWireSetup value")
        }
    }
}

/// Phidget temperature sensor
#[derive(Debug)]
pub struct TemperatureSensor {
    // Handle to the sensor for the phidget22 library
    chan: TemperatureSensorHandle,
    // Double-boxed TemperatureCallback, if registered
    cb: Option<*mut c_void>,
    // Double-boxed attach callback, if registered
    attach_cb: Option<*mut c_void>,
    // Double-boxed detach callback, if registered
    detach_cb: Option<*mut c_void>,
}

impl TemperatureSensor {
    /// Create a new temperature sensor.
    pub fn new() -> Self {
        let mut chan: TemperatureSensorHandle = ptr::null_mut();
        unsafe {
            ffi::PhidgetTemperatureSensor_create(&mut chan);
        }
        Self::from(chan)
    }

    // Low-level, unsafe, callback for temperature change events.
    // The context is a double-boxed pointer the the safe Rust callback.
    unsafe extern "C" fn on_temperature_change(
        chan: TemperatureSensorHandle,
        ctx: *mut c_void,
        temperature: f64,
    ) {
        if !ctx.is_null() {
            let cb: &mut Box<TemperatureCallback> = &mut *(ctx as *mut _);
            let sensor = Self::from(chan);
            cb(&sensor, temperature);
            mem::forget(sensor);
        }
    }

    /// Get a reference to the underlying sensor handle
    pub fn as_channel(&self) -> &TemperatureSensorHandle {
        &self.chan
    }

    /// Read the current temperature
    pub fn temperature(&self) -> Result<f64> {
        let mut temperature = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetTemperatureSensor_getTemperature(self.chan, &mut temperature)
        })?;
        Ok(temperature)
    }

    /// Set a handler to receive temperature change callbacks.
    pub fn set_on_temperature_change_handler<F>(&mut self, cb: F) -> Result<()>
        where
            F: Fn(&TemperatureSensor, f64) + Send + 'static,
    {
        // 1st box is fat ptr, 2nd is regular pointer.
        let cb: Box<Box<TemperatureCallback>> = Box::new(Box::new(cb));
        let ctx = Box::into_raw(cb) as *mut c_void;
        self.cb = Some(ctx);

        ReturnCode::result(unsafe {
            ffi::PhidgetTemperatureSensor_setOnTemperatureChangeHandler(
                self.chan,
                Some(Self::on_temperature_change),
                ctx,
            )
        })
    }

    /// The channel will not issue a TemperatureChange event until the temperature value has changed by the amount specified by the TemperatureChangeTrigger.
    ///
    /// Setting the TemperatureChangeTrigger to 0 will result in the channel firing events every DataInterval. This is useful for applications that implement their own data filtering
    pub fn get_temperature_change_trigger(&self) -> Result<f64> {
        let mut trigger: f64 = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetTemperatureSensor_getTemperatureChangeTrigger(self.chan, &mut trigger)
        })?;
        Ok(trigger)
    }

    /// The channel will not issue a TemperatureChange event until the temperature value has changed by the amount specified by the TemperatureChangeTrigger.
    ///
    /// Setting the TemperatureChangeTrigger to 0 will result in the channel firing events every DataInterval. This is useful for applications that implement their own data filtering
    pub fn set_temperature_change_trigger(&mut self, trigger: f64) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetTemperatureSensor_setTemperatureChangeTrigger(self.chan, trigger)
        })
    }

    /// The minimum value that TemperatureChangeTrigger can be set to.
    pub fn get_min_temperature_change_trigger(&self) -> Result<f64> {
        let mut trigger: f64 = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetTemperatureSensor_getMinTemperatureChangeTrigger(self.chan, &mut trigger)
        })?;
        Ok(trigger)
    }

    /// The maximum value that TemperatureChangeTrigger can be set to.
    pub fn get_max_temperature_change_trigger(&self) -> Result<f64> {
        let mut trigger: f64 = 0.0;
        ReturnCode::result(unsafe {
            ffi::PhidgetTemperatureSensor_getMaxTemperatureChangeTrigger(self.chan, &mut trigger)
        })?;
        Ok(trigger)
    }

    /// The DataInterval is the time that must elapse before the channel will fire another TemperatureChange event.
    //
    //     The data interval is bounded by MinDataInterval and MaxDataInterval.
    //     The timing between TemperatureChange events can also be affected by the TemperatureChangeTrigger.
    pub fn set_data_interval(&mut self, data_interval: u32) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetTemperatureSensor_setDataInterval(self.chan, data_interval)
        })
    }

    /// The DataInterval is the time that must elapse before the channel will fire another TemperatureChange event.
    //
    //     The data interval is bounded by MinDataInterval and MaxDataInterval.
    //     The timing between TemperatureChange events can also be affected by the TemperatureChangeTrigger.
    pub fn get_data_interval(&self) -> Result<u32> {
        let mut data_interval: u32 = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetTemperatureSensor_getDataInterval(self.chan, &mut data_interval)
        })?;
        Ok(data_interval)
    }

    /// The minimum value that DataInterval can be set to.
    pub fn get_min_data_interval(&self) -> Result<u32> {
        let mut data_interval: u32 = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetTemperatureSensor_getMinDataInterval(self.chan, &mut data_interval)
        })?;
        Ok(data_interval)
    }

    /// The maximum value that DataInterval can be set to.
    pub fn get_max_data_interval(&self) -> Result<u32> {
        let mut data_interval: u32 = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetTemperatureSensor_getMaxDataInterval(self.chan, &mut data_interval)
        })?;
        Ok(data_interval)
    }

    ///The RTDType must correspond to the RTD type you are using in your application.
    //
    //     If you are unsure which RTDType to use, visit your device's User Guide for more information.
    pub fn set_rtd_type(&mut self, rtd_type: RtdType) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetTemperatureSensor_setRTDType(self.chan, rtd_type.into())
        })
    }

    ///The RTDType must correspond to the RTD type you are using in your application.
    //
    //     If you are unsure which RTDType to use, visit your device's User Guide for more information.
    pub fn get_rtd_type(&self) -> Result<RtdType> {
        let mut rtd_type: ffi::PhidgetTemperatureSensor_RTDType = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetTemperatureSensor_getRTDType(self.chan, &mut rtd_type)
        })?;
        Ok(RtdType::from(rtd_type))
    }

    ///    The RTDWireSetup must correspond to the wire configuration you are using in your application.
    ///
    /// If you are unsure which RTDWireSetup to use, visit your device's User Guide for more information.
    pub fn set_rtd_wire_setup(&mut self, rtd_wire_setup: RtdWireSetup) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetTemperatureSensor_setRTDWireSetup(self.chan, rtd_wire_setup.into())
        })
    }

    ///    The RTDWireSetup must correspond to the wire configuration you are using in your application.
    ///
    /// If you are unsure which RTDWireSetup to use, visit your device's User Guide for more information.
    pub fn get_rtd_wire_setup(&self) -> Result<RtdWireSetup> {
        let mut rtd_wire_setup: ffi::Phidget_RTDWireSetup = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetTemperatureSensor_getRTDWireSetup(self.chan, &mut rtd_wire_setup)
        })?;
        Ok(RtdWireSetup::from(rtd_wire_setup))
    }


    /// The ThermocoupleType must correspond to the thermocouple type you are using in your application.
    //
    //     If you are unsure which ThermocoupleType to use, visit the [Thermocouple Primer](https://www.phidgets.com/docs/Thermocouple_Primer) for more information.
    pub fn set_thermo_couple_type(&mut self, thermo_couple_type: ThermoCoupleType) -> Result<()> {
        ReturnCode::result(unsafe {
            ffi::PhidgetTemperatureSensor_setThermocoupleType(self.chan, thermo_couple_type.into())
        })
    }

    /// The ThermocoupleType must correspond to the thermocouple type you are using in your application.
    //
    //     If you are unsure which ThermocoupleType to use, visit the [Thermocouple Primer](https://www.phidgets.com/docs/Thermocouple_Primer) for more information.
    pub fn get_thermo_couple_type(&self) -> Result<ThermoCoupleType> {
        let mut thermo_couple_type: ffi::PhidgetTemperatureSensor_ThermocoupleType = 0;
        ReturnCode::result(unsafe {
            ffi::PhidgetTemperatureSensor_getThermocoupleType(self.chan, &mut thermo_couple_type)
        })?;
        Ok(ThermoCoupleType::from(thermo_couple_type))
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

impl Phidget for TemperatureSensor {
    fn as_handle(&mut self) -> PhidgetHandle {
        self.chan as PhidgetHandle
    }
}

unsafe impl Send for TemperatureSensor {}

impl Default for TemperatureSensor {
    fn default() -> Self {
        Self::new()
    }
}

impl From<TemperatureSensorHandle> for TemperatureSensor {
    fn from(chan: TemperatureSensorHandle) -> Self {
        Self {
            chan,
            cb: None,
            attach_cb: None,
            detach_cb: None,
        }
    }
}

impl Drop for TemperatureSensor {
    fn drop(&mut self) {
        if let Ok(true) = self.is_open() {
            let _ = self.close();
        }
        unsafe {
            ffi::PhidgetTemperatureSensor_delete(&mut self.chan);
            crate::drop_cb::<TemperatureCallback>(self.cb.take());
            crate::drop_cb::<AttachCallback>(self.attach_cb.take());
            crate::drop_cb::<DetachCallback>(self.detach_cb.take());
        }
    }
}
