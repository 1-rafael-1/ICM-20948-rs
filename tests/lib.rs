//! Test runner for ICM-20948 driver
//!
//! This module organizes all tests for the ICM-20948 driver.

#[cfg(test)]
mod common;

#[cfg(test)]
mod unit {
    mod bank_switching;
    mod calibration;
    mod config_validation;
    mod data_integrity;
    mod error_handling;
    mod gyro_calibration;
    mod magnetometer;
    mod self_test;
    mod temperature;
}

#[cfg(test)]
mod integration {
    mod basic_workflow;
}
