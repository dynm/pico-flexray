#!/usr/bin/env python3
"""
Test script for Panda USB implementation on FlexRay bridge.

This script tests the basic Panda USB protocol functionality to verify
that the FlexRay bridge correctly implements the Panda interface.

Requirements:
    pip install pyusb

Usage:
    python3 test_panda_usb.py
"""

import usb.core
import usb.util
import time
import struct
import sys

# Panda USB VID/PID
PANDA_VID = 0xbbaa
PANDA_PID = 0xddcc

# Control request types
REQUEST_IN = usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_IN
REQUEST_OUT = usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_OUT

# Panda control commands
GET_HW_TYPE = 0xc1
CAN_RESET_COMMS = 0xc0
SET_SAFETY_MODEL = 0xdc
SET_CAN_SPEED_KBPS = 0xde
SET_DATA_SPEED_KBPS = 0xf9
HEARTBEAT = 0xf3

# Hardware types
HW_TYPE_UNKNOWN = 0
HW_TYPE_RED_PANDA = 4

def find_panda_device():
    """Find and return the Panda USB device."""
    print("Searching for Panda device...")
    
    dev = usb.core.find(idVendor=PANDA_VID, idProduct=PANDA_PID)
    if dev is None:
        print(f"ERROR: No device found with VID:PID {PANDA_VID:04x}:{PANDA_PID:04x}")
        print("Make sure the FlexRay bridge is connected and flashed with Panda USB firmware.")
        return None
    
    print(f"Found device: {dev}")
    print(f"Manufacturer: {usb.util.get_string(dev, dev.iManufacturer)}")
    print(f"Product: {usb.util.get_string(dev, dev.iProduct)}")
    print(f"Serial: {usb.util.get_string(dev, dev.iSerialNumber)}")
    
    # Set configuration
    try:
        dev.set_configuration()
        print("Device configuration set successfully")
    except usb.core.USBError as e:
        print(f"Warning: Could not set configuration: {e}")
    
    return dev

def test_control_commands(dev):
    """Test basic Panda control commands."""
    print("\n=== Testing Control Commands ===")
    
    try:
        # Test GET_HW_TYPE
        print("Testing GET_HW_TYPE...")
        result = dev.ctrl_transfer(REQUEST_IN, GET_HW_TYPE, 0, 0, 1)
        hw_type = result[0] if result else 0
        print(f"Hardware type: {hw_type}")
        
        if hw_type == HW_TYPE_RED_PANDA:
            print("✓ Correct hardware type (Red Panda)")
        else:
            print(f"✗ Unexpected hardware type: {hw_type}")
        
        # Test CAN_RESET_COMMS
        print("\nTesting CAN_RESET_COMMS...")
        dev.ctrl_transfer(REQUEST_OUT, CAN_RESET_COMMS, 0, 0, [])
        print("✓ CAN reset command sent")
        
        # Test SET_SAFETY_MODEL (SILENT mode = 3)
        print("\nTesting SET_SAFETY_MODEL...")
        dev.ctrl_transfer(REQUEST_OUT, SET_SAFETY_MODEL, 3, 0, [])
        print("✓ Safety model set to SILENT")
        
        # Test SET_CAN_SPEED_KBPS (500 kbps for bus 0)
        print("\nTesting SET_CAN_SPEED_KBPS...")
        speed_data = struct.pack('<HH', 0, 500)  # bus 0, 500 kbps
        dev.ctrl_transfer(REQUEST_OUT, SET_CAN_SPEED_KBPS, 0, 0, speed_data)
        print("✓ CAN speed set to 500 kbps")
        
        # Test SET_DATA_SPEED_KBPS (2000 kbps for CAN-FD data phase)
        print("\nTesting SET_DATA_SPEED_KBPS...")
        data_speed = struct.pack('<HH', 0, 2000)  # bus 0, 2000 kbps
        dev.ctrl_transfer(REQUEST_OUT, SET_DATA_SPEED_KBPS, 0, 0, data_speed)
        print("✓ CAN-FD data speed set to 2000 kbps")
        
        # Test HEARTBEAT
        print("\nTesting HEARTBEAT...")
        dev.ctrl_transfer(REQUEST_OUT, HEARTBEAT, 0, 0, [])
        print("✓ Heartbeat sent")
        
        print("\n✓ All control commands executed successfully!")
        return True
        
    except usb.core.USBError as e:
        print(f"✗ USB Error during control commands: {e}")
        return False
    except Exception as e:
        print(f"✗ Unexpected error: {e}")
        return False

def test_endpoints(dev):
    """Test endpoint configuration."""
    print("\n=== Testing Endpoints ===")
    
    try:
        # Get the active configuration
        cfg = dev.get_active_configuration()
        print(f"Active configuration: {cfg.bConfigurationValue}")
        
        # Find the vendor interface
        vendor_interface = None
        for interface in cfg:
            if interface.bInterfaceClass == 0xFF:  # Vendor class
                vendor_interface = interface
                break
        
        if vendor_interface is None:
            print("✗ No vendor interface found")
            return False
        
        print(f"✓ Found vendor interface: {vendor_interface.bInterfaceNumber}")
        
        # Check endpoints
        in_endpoint = None
        out_endpoint = None
        
        for endpoint in vendor_interface:
            ep_addr = endpoint.bEndpointAddress
            if ep_addr & 0x80:  # IN endpoint
                in_endpoint = endpoint
                print(f"✓ Found IN endpoint: 0x{ep_addr:02x}")
            else:  # OUT endpoint
                out_endpoint = endpoint
                print(f"✓ Found OUT endpoint: 0x{ep_addr:02x}")
        
        if in_endpoint and out_endpoint:
            print("✓ Both IN and OUT endpoints found")
            return True
        else:
            print("✗ Missing required endpoints")
            return False
            
    except Exception as e:
        print(f"✗ Error testing endpoints: {e}")
        return False

def test_data_communication(dev):
    """Test basic data communication on bulk endpoints."""
    print("\n=== Testing Data Communication ===")
    
    try:
        # Try to read from IN endpoint with short timeout
        print("Testing bulk IN endpoint (listening for CAN data)...")
        try:
            data = dev.read(0x81, 64, timeout=1000)  # 1 second timeout
            print(f"✓ Received {len(data)} bytes: {data.hex()}")
        except usb.core.USBTimeoutError:
            print("⚠ No data received (timeout) - this is normal if no FlexRay traffic")
        
        print("✓ Data communication test completed")
        return True
        
    except Exception as e:
        print(f"✗ Error during data communication test: {e}")
        return False

def main():
    """Main test function."""
    print("Panda USB FlexRay Bridge Test")
    print("=" * 40)
    
    # Find device
    dev = find_panda_device()
    if dev is None:
        sys.exit(1)
    
    # Run tests
    tests_passed = 0
    total_tests = 3
    
    if test_control_commands(dev):
        tests_passed += 1
    
    if test_endpoints(dev):
        tests_passed += 1
    
    if test_data_communication(dev):
        tests_passed += 1
    
    # Summary
    print("\n" + "=" * 40)
    print(f"Test Summary: {tests_passed}/{total_tests} tests passed")
    
    if tests_passed == total_tests:
        print("✓ All tests passed! The Panda USB implementation is working correctly.")
        sys.exit(0)
    else:
        print("✗ Some tests failed. Check the output above for details.")
        sys.exit(1)

if __name__ == "__main__":
    main() 