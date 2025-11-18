#!/usr/bin/env python3
#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0
#
# Test script for LimePSB-RPCM GPSDO via LiteX RemoteClient
#

import time
import argparse
import math
from litex import RemoteClient

# Constants ----------------------------------------------------------------------------------------
REG_CONTROL               = "ppsdo_enable"                  # bits [0]=EN, [1]=CLK_SEL (if implemented)
REG_PPS_1S_TARGET         = "ppsdo_config_one_s_target"     # 32-bit
REG_PPS_1S_ERR_TOL        = "ppsdo_config_one_s_tol"
REG_PPS_10S_TARGET        = "ppsdo_config_ten_s_target"
REG_PPS_10S_ERR_TOL       = "ppsdo_config_ten_s_tol"
REG_PPS_100S_TARGET       = "ppsdo_config_hundred_s_target"
REG_PPS_100S_ERR_TOL      = "ppsdo_config_hundred_s_tol"

REG_PPS_1S_ERR            = "ppsdo_status_one_s_error"      # signed 32-bit
REG_PPS_10S_ERR           = "ppsdo_status_ten_s_error"
REG_PPS_100S_ERR          = "ppsdo_status_hundred_s_error"
REG_DAC_TUNED_VAL         = "ppsdo_status_dac_tuned_val"
REG_STATUS_ACCURACY       = "ppsdo_status_accuracy"
REG_STATUS_PPS_ACTIVE     = "ppsdo_status_pps_active"
REG_STATUS_STATE          = "ppsdo_status_state"

# Status bit fields.
STATUS_STATE_OFFSET = 0
STATUS_STATE_SIZE   = 4
STATUS_ACCURACY_OFFSET = 4
STATUS_ACCURACY_SIZE   = 4
STATUS_TPULSE_OFFSET = 8
STATUS_TPULSE_SIZE   = 1

# Control bit fields
CONTROL_EN_OFFSET      = 0
CONTROL_EN_SIZE        = 1
CONTROL_CLK_SEL_OFFSET = 1
CONTROL_CLK_SEL_SIZE   = 1

# Helper function to get a field from a register value.
def get_field(reg_value, offset, size):
    mask = ((1 << size) - 1) << offset
    return (reg_value & mask) >> offset

# Helper function to set a field within a register value.
def set_field(reg_value, offset, size, value):
    mask = ((1 << size) - 1) << offset
    return (reg_value & ~mask) | ((value << offset) & mask)

# GPSDODriver --------------------------------------------------------------------------------------
class GPSDODriver:
    """
    Driver for LimePSB-RPCM GPSDO gpsdocfg registers via LiteX RemoteClient.
    """
    def __init__(self, host="localhost", port=1234):
        self.bus = RemoteClient(host=host, port=port)
        self.bus.open()
        self.regs = self.bus.regs

    def read_register(self, name):
        """Read a register by its LiteX CSR name."""
        return getattr(self.regs, name).read()

    def write_register(self, name, value):
        """Write a register by its LiteX CSR name."""
        getattr(self.regs, name).write(value)

    def get_signed_32bit(self, name):
        """Get signed 32-bit value from a register."""
        value = self.read_register(name)
        if value & (1 << 31):  # Sign extend if negative
            value -= (1 << 32)
        return value

    def get_1s_error(self):
        """Get 1s error as signed 32-bit."""
        return self.get_signed_32bit(REG_PPS_1S_ERR)

    def get_10s_error(self):
        """Get 10s error as signed 32-bit."""
        return self.get_signed_32bit(REG_PPS_10S_ERR)

    def get_100s_error(self):
        """Get 100s error as signed 32-bit."""
        return self.get_signed_32bit(REG_PPS_100S_ERR)

    def get_dac_value(self):
        """Get DAC tuned value."""
        return self.read_register(REG_DAC_TUNED_VAL)

    def get_status(self):
        """Get decoded status: state, accuracy, tpulse_active."""
        state    = self.read_register(REG_STATUS_STATE)
        accuracy = self.read_register(REG_STATUS_ACCURACY)
        tpulse   = self.read_register(REG_STATUS_PPS_ACTIVE)

        state_str = "Coarse Tune" if state == 0 else "Fine Tune" if state == 1 else f"Unknown ({state})"
        accuracy_str = ['Disabled/Lowest', '1s Tune', '2s Tune', '3s Tune (Highest)'][accuracy] if accuracy < 4 else f"Unknown ({accuracy})"

        return {
            "state": state_str,
            "accuracy": accuracy_str,
            "tpulse_active": bool(tpulse)
        }

    def get_enabled(self):
        """Get enabled status from control register."""
        control = self.read_register(REG_CONTROL)
        return bool(control & 0x0001)

    def set_enabled(self, enable):
        """Set enabled bit, preserving other control bits."""
        control = self.read_register(REG_CONTROL)
        control = set_field(control, CONTROL_EN_OFFSET, CONTROL_EN_SIZE, 1 if enable else 0)
        self.write_register(REG_CONTROL, control)

    def close(self):
        """Close the LiteX connection."""
        self.bus.close()

# Test Functions -----------------------------------------------------------------------------------
def run_monitoring(driver, num_dumps=0, delay=1.0, banner_interval=10):
    # Header banner
    header = "Dump | Enabled | 1s Error | 10s Error | 100s Error | DAC Value | State | Accuracy | TPulse"
    print("Monitoring GPSDO regulation loop (press Ctrl+C to stop):")
    print(header)
    dump_count = 0
    try:
        while num_dumps == 0 or dump_count < num_dumps:
            enabled = driver.get_enabled()
            error_1s = driver.get_1s_error()
            error_10s = driver.get_10s_error()
            error_100s = driver.get_100s_error()
            dac = driver.get_dac_value()
            status = driver.get_status()
            # Single-line output
            print(f"{dump_count + 1:4d} | {str(enabled):7} | {error_1s:8d} | {error_10s:9d} | {error_100s:10d} | 0x{dac:04X} | {status['state']:12} | {status['accuracy']:17} | {str(status['tpulse_active']):6}")
            dump_count += 1
            # Print banner every banner_interval dumps
            if dump_count % banner_interval == 0:
                print(header)
            if num_dumps == 0 or dump_count < num_dumps:
                time.sleep(delay)
    except KeyboardInterrupt:
        print("\nMonitoring stopped.")

def dump_registers(driver, num_dumps=1, delay=1.0):
    # Not strictly needed anymore (all info in monitoring), but kept for compatibility
    print("Note: Raw register dump not available via named CSRs — use --check for full status")

def reset_gpsdo(driver, reset_delay=2.0):
    print("Resetting GPSDO...")
    driver.set_enabled(False)
    time.sleep(reset_delay) # Wait for disable to take effect
    driver.set_enabled(True)
    print("GPSDO reset complete (re-enabled).")

def enable_gpsdo(driver, clk_freq_mhz=30.72, ppm=0.1):
    freq = clk_freq_mhz * 1e6
    # Compute targets (expected counter values for intervals).
    target_1s   = int(freq)
    target_10s  = int(10 * freq)
    target_100s = int(100 * freq)
    # Compute tolerances in Hz for constant ppm across intervals.
    tol_1s_hz   = round(freq * ppm / 1e6)
    tol_10s_hz  = tol_1s_hz * 10
    tol_100s_hz = tol_1s_hz * 100

    # Configure targets and tolerances (32-bit registers)
    print(target_1s)
    print(tol_1s_hz)
    #exit()
    driver.write_register(REG_PPS_1S_TARGET,   target_1s)
    driver.write_register(REG_PPS_1S_ERR_TOL,  tol_1s_hz)
    driver.write_register(REG_PPS_10S_TARGET,  target_10s)
    driver.write_register(REG_PPS_10S_ERR_TOL, tol_10s_hz)
    driver.write_register(REG_PPS_100S_TARGET, target_100s)
    driver.write_register(REG_PPS_100S_ERR_TOL,tol_100s_hz)

    # Set CLK_SEL (0: 30.72MHz, 1: 10MHz) and Enable
    clk_sel = 1 if math.isclose(clk_freq_mhz, 10.0) else 0
    control = set_field(0, CONTROL_CLK_SEL_OFFSET, CONTROL_CLK_SEL_SIZE, clk_sel)
    control = set_field(control, CONTROL_EN_OFFSET, CONTROL_EN_SIZE, 1)
    driver.write_register(REG_CONTROL, control)

    print(f"GPSDO enabled: CLK_SEL={clk_sel} ({clk_freq_mhz}MHz), {ppm}ppm tolerance "
          f"(1s tol={tol_1s_hz}Hz, 10s={tol_10s_hz}Hz, 100s={tol_100s_hz}Hz).")

def disable_gpsdo(driver):
    # Disable.
    driver.write_register(REG_CONTROL, 0x0000)
    print("GPSDO disabled.")

# Main ----------------------------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="GPSDO Test Script (LiteX RemoteClient)")
    parser.add_argument("--host", default="localhost", help="LiteX server host")
    parser.add_argument("--port", type=int, default=1234, help="LiteX server port")
    parser.add_argument("--check", action="store_true", help="Run monitoring mode")
    parser.add_argument("--dump", action="store_true", help="Dump registers (not supported in LiteX named CSR mode)")
    parser.add_argument("--reset", action="store_true", help="Reset GPSDO")
    parser.add_argument("--enable", action="store_true", help="Configure and enable GPSDO")
    parser.add_argument("--disable", action="store_true", help="Disable GPSDO")
    parser.add_argument("--num", default=0, type=int, help="Number of iterations (for --check: 0 for infinite; for --dump: default 1 if not specified)")
    parser.add_argument("--delay", default=1.0, type=float, help="Delay between iterations (seconds, for --check and --dump)")
    parser.add_argument("--banner", default=10, type=int, help="Banner repeat interval (for --check)")
    parser.add_argument("--reset-delay", default=2.0, type=float, help="Delay after disable before re-enable (seconds, for --reset)")
    parser.add_argument("--clk-freq", default=30.72, type=float, help="Clock frequency in MHz (10 or 30.72)")
    parser.add_argument("--ppm", default=0.1, type=float, help="Tolerance in ppm")
    args = parser.parse_args()

    driver = GPSDODriver(host=args.host, port=args.port)
    try:
        # Dump (kept for compatibility, prints note)
        if args.dump:
            dump_registers(driver, num_dumps=args.num if args.num > 0 else 1, delay=args.delay)
        # Enable.
        if args.enable:
            enable_gpsdo(driver, clk_freq_mhz=args.clk_freq, ppm=args.ppm)
        # Disable.
        if args.disable:
            disable_gpsdo(driver)
        # Reset.
        if args.reset:
            reset_gpsdo(driver, reset_delay=args.reset_delay)
        # Check.
        if args.check:
            run_monitoring(driver, num_dumps=args.num, delay=args.delay, banner_interval=args.banner)
    finally:
        driver.close()

if __name__ == "__main__":
    main()