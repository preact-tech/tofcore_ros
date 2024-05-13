import time
import struct
import pytest
import pytofcrust
import threading
from typing import List

def pytest_configure(config):
    config.addinivalue_line(
        "markers", "functional: mark test to run only on named environment")
    config.addinivalue_line(
    "markers", "sdram_selftest: SDRAM Tests to run with functional sensor connected")

@pytest.fixture(scope="function")
def dut(request) -> pytofcrust.Sensor:
    args = {}

    if request.config.getoption('--sensor-uri'):
        args['uri'] = request.config.getoption('--sensor-uri')

    sensor = pytofcrust.Sensor(**args)
    yield sensor
    sensor.stop_stream()
    sensor = None


@pytest.mark.functional
def test_frame_set_limits(dut: pytofcrust.Sensor):
    
    original_limits = dut.get_frame_period_and_limits()    # Get original values
    original_value, original_min, original_max = original_limits
    
    test_min = 42      # Test values
    test_max = 42000
    
    dut.set_factory_mode(True)
    dut.set_frame_period_limits(test_min, test_max) # Set test values
    
    current_limits = dut.get_frame_period_and_limits()
    current_value, current_min, current_max = current_limits
    
    assert current_min == test_min # Verify test values
    assert current_max == test_max
    
    dut.set_frame_period_limits(original_min, original_max) # Restore original values
    
    current_limits = dut.get_frame_period_and_limits()
    current_value, current_min, current_max = current_limits
    
    assert current_min == original_min  # Verify restoration of original values
    assert current_max == original_max

    dut.set_factory_mode(False)
    
    dut.set_frame_period(original_value)
    current_value = dut.get_frame_period()
    assert original_value == current_value


@pytest.mark.functional
def test_integration_time_set_limits(dut: pytofcrust.Sensor):
    
    original_limits = dut.get_integration_time_and_limits()    # Get original values
    original_value, original_min, original_max = original_limits
    
    test_min = 42      # Test values
    test_max = 1942
    
    dut.set_factory_mode(True)
    dut.set_integration_time_limits(test_min, test_max) # Set test values
    
    current_limits = dut.get_integration_time_and_limits()
    current_value, current_min, current_max = current_limits
    
    assert current_min == test_min # Verify test values
    assert current_max == test_max
    
    dut.set_integration_time_limits(original_min, original_max) # Restore original values
    
    current_limits = dut.get_integration_time_and_limits()
    current_value, current_min, current_max = current_limits
    
    assert current_min == original_min  # Verify restoration of original values
    assert current_max == original_max

    dut.set_factory_mode(False)
    
    dut.set_integration_time(original_value)
    current_value = dut.get_integration_time()
    assert original_value == current_value


@pytest.mark.functional
def test_modulation_frequency_set_limits(dut: pytofcrust.Sensor):
    
    original_limits = dut.get_modfreq_and_limits_and_step()    # Get original values
    original_value, original_min, original_max, original_step = original_limits

    dut.set_factory_mode(True)

    # Error will occur with trying to set any one of these values to 0
    # Test 0 values are rejected
    test_min = 0      # Test values
    test_max = 24000
    test_step = 10
    limit_set_failed = 0
    try:
        dut.set_modulation_frequency_limits(test_min, test_max, test_step) # Set test values
        limit_set_failed = 0
    except:
        print("set rejected due to 0 values as expected!")
        limit_set_failed = 1
        
    assert limit_set_failed == 1
        
    test_min = 6000      # Test values
    test_max = 0
    test_step = 10
    try:
        dut.set_modulation_frequency_limits(test_min, test_max, test_step) # Set test values
        limit_set_failed = 0
    except:
        print("set rejected due to 0 values as expected!")
        limit_set_failed = 1
        
    assert limit_set_failed == 1
        
    test_min = 6000      # Test values
    test_max = 24000
    test_step = 0
    try:
        dut.set_modulation_frequency_limits(test_min, test_max, test_step) # Set test values
        limit_set_failed = 0
    except:
        print("set rejected due to 0 values as expected!")
        limit_set_failed = 1

    assert limit_set_failed == 0    # not rejected, but not used either
   
    current_limits = dut.get_modfreq_and_limits_and_step()
    current_value, current_min, current_max, current_step = current_limits
    
    # Verify no change (otherwise 0 values rejected)
    assert current_min == original_min 
    assert current_max == original_max
    assert current_step == original_step 

    # Test actual values
    test_min = 42      # Test values
    test_max = 42000
    test_step = 43
    
    dut.set_modulation_frequency_limits(test_min, test_max, test_step) # Set test values
    
    current_limits = dut.get_modfreq_and_limits_and_step()
    current_value, current_min, current_max, current_step = current_limits
    
    assert current_min == test_min # Verify test values
    assert current_max == test_max
    assert current_step == test_step
    
    dut.set_modulation_frequency_limits(original_min, original_max, original_step) # Restore original values
    
    current_limits = dut.get_modfreq_and_limits_and_step()
    current_value, current_min, current_max, current_step = current_limits
    
    assert current_min == original_min  # Verify restoration of original values
    assert current_max == original_max
    assert current_step == original_step

    dut.set_factory_mode(False)
    
    dut.modulation_frequency = original_value
    current_value = dut.modulation_frequency
    assert original_value == current_value


@pytest.mark.functional
def test_min_amplitude_set_limits(dut: pytofcrust.Sensor):
    
    original_limits = dut.get_min_amplitude_and_limits()    # Get original values
    original_value, original_min, original_max = original_limits
    
    test_min = 42      # Test values
    test_max = 542
    
    dut.set_factory_mode(True)
    dut.set_min_amplitude_limits(test_min, test_max) # Set test values
    
    current_limits = dut.get_min_amplitude_and_limits()
    current_value, current_min, current_max = current_limits
    
    assert current_min == test_min # Verify test values
    assert current_max == test_max
    
    dut.set_min_amplitude_limits(original_min, original_max) # Restore original values
    
    current_limits = dut.get_min_amplitude_and_limits()
    current_value, current_min, current_max = current_limits
    
    assert current_min == original_min  # Verify restoration of original values
    assert current_max == original_max

    dut.set_factory_mode(False)
    
    dut.set_min_amplitude(original_value)
    current_value = dut.get_min_amplitude()
    assert original_value == current_value
    

@pytest.mark.functional
def test_set_dll(dut: pytofcrust.Sensor):
    
    # Register Definitions
    DLL_CONTROL_REG = 0xAE

    DLL_FINE_STEP_REG   = 0x71
    DLL_FINEST_STEP_REG = 0x72
    DLL_COARSE_STEP_REG = 0x73

    regs = [DLL_CONTROL_REG, DLL_COARSE_STEP_REG, DLL_FINE_STEP_REG, DLL_FINEST_STEP_REG]
    
    # Test settings
    dll_enable      = 0x01
    dll_coarse_step = 0x13
    dll_fine_step   = 0x02
    dll_finest_step = 0x78

    writeVals = [dll_enable, dll_coarse_step, dll_fine_step, dll_finest_step]

    # Write DLL settings
    set_dll_successful = dut.set_dll_step(dll_enable, dll_coarse_step, dll_fine_step, dll_finest_step)

    assert set_dll_successful == True

    for regIndex, register in enumerate(regs):
        read_register = dut.read_sensor_register(register)
        if regIndex == 0:
            print("")
            continue
        assert read_register[1] == writeVals[regIndex]
        print("Reg (" + str(hex(register)) + "), write value: " + str(hex(writeVals[regIndex])) + ", read value: " + str(hex(read_register[1])))


@pytest.mark.functional
def test_read_write_registers(dut: pytofcrust.Sensor):
    DLL_FINEST_STEP_REG = 0x72
    testReg = DLL_FINEST_STEP_REG
    testVal = 0xE7
    dut.write_sensor_register(testReg, testVal)

    # read back write value
    readVal = dut.read_sensor_register(testReg)

    assert readVal[1] == testVal
    print("\nTest Write then Read")
    print("Reg (" + str(hex(testReg)) + "), write value: " + str(hex(testVal)) + ", read value: " + str(hex(readVal[1])))

@pytest.mark.functional
def test_set_vled_enables(dut: pytofcrust.Sensor):

    testVal = 0x0F

    print("Testing set VLED enables. Value: " + str(hex(testVal)))

    dut.set_vled_enables(testVal)

    # read back write value
    readVal = dut.get_vled_enables()

    assert readVal == testVal
    print("\nTest Write then Read")
    print("write value: " + str(hex(testVal)) + ", read value: " + str(hex(readVal)))


@pytest.mark.functional
def test_modulation_frequencies(dut: pytofcrust.Sensor):

    def run(testFreqs, testName):

        # Cycle through test frequencies
        for testFreq in testFreqs:

            expectedFrequency = None

            dut.modulation_frequency = testFreq

            if (testFreq < 6000):
                expectedFrequency = 6000
            elif (testFreq > 24000):
                expectedFrequency = 24000
            else:
                expectedFrequency = int(round(testFreq/10.0) * 10)

            #WART: This is a quick fix to deal with MOS-877 where you can't read back the same 
            #   mod frequency that was set until streaming has been initiated.
            dut.stream_distance()
            # It can take several hundred mS for the sensor to initially calculate
            # the correction values for modulation frequencies that are not exactly
            # the calibration frequencies (it has to interpolate)
            time.sleep(0.5)
            dut.stop_stream()
            time.sleep(0.1)

            readVal = dut.modulation_frequency
            print(f"Frequency set: {testFreq}, Read back (calculated): {readVal} (kHz)")

            assert readVal == expectedFrequency, testName + "Mod Freq read does NOT equal mod frequency set"

    # Test rounding
    run([6001, 6009], "Rounding Test")

    #Test valid frequencies 
    run([6050, 8000, 10100, 13000, 22000, 6000, 7000, 12000, 23000, 24000], "Valid Freqs Test")

    # Test out of bounds
    run([3000, 25000], "Freq outside bounds")


@pytest.mark.functional
def test_meta_data_dll(dut: pytofcrust.Sensor):

    def run(enable_dll, coarse_step, fine_step, finest_step):
        def callback(measurement: pytofcrust.Measurement, **kwargs):
            with callback.mutex:
                callback.count += 1
                #Note: for some reason it seems to take at least 3 measurement iterations 
                # for new settings to take effect this is probably a bug in the sensor
                # firmware but it's not import ATM
                if callback.count > 3:
                    if measurement.data_type == measurement.DataType.DISTANCE and not callback.measurement:
                        callback.measurement = measurement

        callback.mutex = threading.Lock()
        callback.count = 0
        callback.measurement = None

        dut.set_dll_step(enable_dll=enable_dll, coarse_step=coarse_step, fine_step=fine_step, finest_step=finest_step)
        dut.subscribe_measurement(callback)
        dut.stream_distance()
        count = 0 # we will wait for upto 1 second in 0.1 second increments
        while count != 10:
            count += 1
            time.sleep(0.1)
            with callback.mutex:
                if callback.measurement:
                    count = 10
        dut.stop_stream()
        
        assert callback.measurement, "No measurement received"

        dll_settings = callback.measurement.dll_settings
        assert dll_settings is not None, "No DLL settings data included with the measurement"
        assert dll_settings.enabled == enable_dll, "Incorrect DLL enabled value included in meta-data"
        assert dll_settings.coarse_step == coarse_step, "Incorrect DLL coarse step value included in meta-data"
        assert dll_settings.fine_step == fine_step, "Incorrect DLL fine step value included in meta-data"
        assert dll_settings.finest_step == finest_step, "Incorrect DLL finest step value included in meta-data"

    run(**{'enable_dll': True, 'coarse_step': 15, 'fine_step': 1, 'finest_step': 2})
    run(**{'enable_dll': True, 'coarse_step': 0, 'fine_step': 0, 'finest_step': 0})
    run(**{'enable_dll': False, 'coarse_step': 0, 'fine_step': 0, 'finest_step': 0})



@pytest.mark.functional
def test_meta_data_illuminator_info(dut: pytofcrust.Sensor):
    '''Functional test to verify the illuminator info metadata is flowing with measuremnt data.

    The tests setup known VLED voltage and VLED enables, streams a bit of data and then verifies
    the illuminator metadata re-acts appropriately.
    '''
    def run(vled_enables: int, vled_voltage: float, stream_method: str):
        def callback(measurement: pytofcrust.Measurement, **kwargs):
            with callback.mutex:
                callback.count += 1
                #Note: for some reason it seems to take a few measurement iterations 
                # for new settings to take effect this is probably a bug in the sensor
                # firmware but it's not import ATM
                if callback.count > 10:
                    callback.measurement = measurement

        callback.mutex = threading.Lock()
        callback.count = 0
        callback.measurement = None

        dut.set_vled_enables(vled_enables)
        dut.vled_voltage = vled_voltage
        dut.subscribe_measurement(callback)
        f = getattr(dut, stream_method)
        f()
        count = 0 # we will wait for upto 1 second in 0.1 second increments
        while count != 30:
            count += 1
            time.sleep(0.1)
            with callback.mutex:
                if callback.measurement:
                    count = 30
        dut.stop_stream()
        
        assert callback.measurement, "No measurement received"

        info = callback.measurement.illuminator_info
        assert info is not None, "No illuminator info included with the measurement"
        #Check that temperature is there and is a float, I can't verify the value, typically its ~30c but I really don't know the operating environment
        assert isinstance(info.temperature_c, float), "Invalid temperature value included with illuminator_info"
        #Check that photodiode is there and is a float, I really can't verify the value.
        assert isinstance(info.photodiode_v, float), "Invalid photodiode voltage value included with illuminator_info"
        assert info.led_segments_enabled == vled_enables, "Illuminator info metadata led_segments_enabled does not match requested value"
        #Ignore the vled voltage if the led segments are disabled. 
        if info.led_segments_enabled != 0:
            assert info.vled_v == pytest.approx(vled_voltage, 0.1), "Illuminator info vled voltage does not match requested value"

    for stream_method in ('stream_distance_amplitude', 'stream_dcs_ambient'):
        run(**{'vled_enables': 0, 'vled_voltage': 5.9, 'stream_method': stream_method})
        run(**{'vled_enables': 0xf, 'vled_voltage': 5.0, 'stream_method': stream_method})
        run(**{'vled_enables': 0xf, 'vled_voltage': 5.3, 'stream_method': stream_method})
        run(**{'vled_enables': 0xf, 'vled_voltage': 5.9, 'stream_method': stream_method})

@pytest.mark.functional
def test_thermal_limit_control():
    '''Test to verify that the thermal limits for protection can be modifyed.
    '''
    dut = pytofcrust.Sensor()
    originalLimits = dut.get_thermal_limits()
    # Unlock factory mode
    dut.set_factory_mode(True)
    # Change limits
    entryDegC = 45.1
    exitDegC = 44.8
    dut.set_thermal_limits(entryDegC, exitDegC)
    # Read them back
    limits = dut.get_thermal_limits()
    assert limits == (entryDegC,exitDegC)

    dut.set_thermal_limits(originalLimits[0], originalLimits[1])
    limits = dut.get_thermal_limits()
    assert limits == originalLimits
    # Connection will be lost. force reference to none
    dut = None

@pytest.mark.functional
def test_vled_set_limits(dut: pytofcrust.Sensor):
    def run(min_requested, max_requested):
        dut.set_vled_limits(min_requested, max_requested)
        limits = dut.get_vled_setting_and_limits()
        setting, min_actual, max_actual = limits
        assert min_requested == min_actual, "MIN VLED read does not match requested."
        assert max_requested == max_actual, "MAX VLED read does not match requested."
        assert ((setting <= max_actual) and (setting >= min_actual)), "VLED Setting outside requested limits."
    
    original_limits = dut.get_vled_setting_and_limits()    # Get original values
    original_setting, original_min, original_max = original_limits
    
    dut.set_factory_mode(True)
    test_vals = [(3800, 8770), (3800, 5800), (4000, 5000)]
    for limit_pair in test_vals:
        run(limit_pair[0], limit_pair[1])
    
    #reset the limits to original, this will also verify the settings are correct.
    run(original_min, original_max)
    dut.vled_voltage = original_setting

    dut.set_factory_mode(False)

#
# NOTE SDRAM Test should be last - it resets the sensor!
#
@pytest.mark.sdram_selftest
def test_initiate_sdram_test():
    '''Test to verify SDRAM test request goes through
        Note: sensor communication is lost when the commands below are issued. 
        Check must be done manually via serial prinouts. 
        Should see SDRAM test results from previous run. 
        Note connection to the sensor will be lost. 
    '''

    dut = pytofcrust.Sensor()

    # Unlock factory mode
    dut.set_factory_mode(True)

    # Issue sdram test command
    dut.sdram_test_request()

    # Connection will be lost. force reference to none
    dut = None
