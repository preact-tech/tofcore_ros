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
def test_stream_distance_amplitude_frames(dut: pytofcrust.Sensor):
    def callback(measurement: pytofcrust.Measurement, **kwargs):
        if not callback.measurement and (measurement.data_type == pytofcrust.Measurement.DataType.DISTANCE_AMPLITUDE):
            callback.measurement = measurement

    callback.measurement = None
    dut.subscribe_measurement(callback)
    dut.stream_distance_amplitude()
    
    time.sleep(1.0)
    assert callback.measurement is not None, "No Distance & Amplitude measurements received"


@pytest.mark.functional
def test_stream_distance(dut: pytofcrust.Sensor):

    def callback(measurement: pytofcrust.Measurement, **kwargs):
        if not callback.measurement and (measurement.data_type == pytofcrust.Measurement.DataType.DISTANCE):
            callback.measurement = measurement

    callback.measurement = None
    dut.subscribe_measurement(callback)
    dut.stream_distance()
    
    time.sleep(1.0)
    assert callback.measurement is not None, "No Distance measurements received"


@pytest.mark.functional
def test_stream_dcs(dut: pytofcrust.Sensor):

    def callback(measurement: pytofcrust.Measurement, **kwargs):
        if not callback.measurement and (measurement.data_type == pytofcrust.Measurement.DataType.DCS):
            callback.measurement = measurement

    callback.measurement = None
    dut.subscribe_measurement(callback)
    dut.stream_dcs()
    
    time.sleep(1.0)
    assert callback.measurement is not None, "No DCS measurements received"


@pytest.mark.functional
def test_stream_dcs_ambient(dut: pytofcrust.Sensor):
    
    def callback(measurement: pytofcrust.Measurement, **kwargs):
        if not callback.dcs_measurement and (measurement.data_type == pytofcrust.Measurement.DataType.DCS):
            callback.dcs_measurement = measurement
        if not callback.ambient_measurement and (measurement.data_type == pytofcrust.Measurement.DataType.AMBIENT):
            callback.ambient_measurement = measurement

    callback.dcs_measurement = None
    callback.ambient_measurement = None
    dut.subscribe_measurement(callback)
    dut.stream_dcs_ambient()
    time.sleep(1.0)
    assert callback.dcs_measurement is not None, "No DCS measurements received"
    assert callback.ambient_measurement is not None, "No Ambient measurements received"

@pytest.mark.functional
def test_get_sensor_info(dut: pytofcrust.Sensor):
    
    versionInfo = dut.get_sensor_info()

    print("\n")
    print("Device Serial number: " + versionInfo.deviceSerialNumber)
    print("CPU Board Serial number: " + versionInfo.cpuBoardSerialNumber)
    print("Illuminator Serial Number: " + versionInfo.illuminatorBoardSerialNumber)
    print("Model name: " + versionInfo.modelName)
    print("Last Reset Type: " + versionInfo.lastResetType)

    print("Software Source ID: " + versionInfo.softwareId)
    print("Sensor Description: " + versionInfo.softwareVersion)

    print("CPU Board Hardware Version: " + str(versionInfo.cpuVersion))
    print("Preact Chip ID: " + hex(versionInfo.chipId))
    print("Illuminator Version: " + versionInfo.illuminatorSwVersion + "." + versionInfo.illuminatorSwId)
    print("Backpack Module/Version: " + str(versionInfo.backpackModule))

    assert versionInfo._fields == ('deviceSerialNumber', 'cpuBoardSerialNumber', 'illuminatorBoardSerialNumber', 'modelName', 'lastResetType', 'softwareId', 'softwareVersion', 'cpuVersion', 'chipId', 'illuminatorSwVersion', 'illuminatorSwId', 'illuminatorHwCfg' ,'backpackModule')


@pytest.mark.functional
def test_get_accelerometer_data(dut: pytofcrust.Sensor):
    """Getting accelerometer data fails at the time of writing this test"""

    with pytest.raises(Exception):
        accel_info = dut.accelerometer_data
        assert accel_info._fields == ('x', 'y', 'z', 'g_range')


@pytest.mark.functional
def test_get_lens_rays(dut: pytofcrust.Sensor):

    pixel_rays = dut.pixel_rays
    
    assert pixel_rays._fields == ('x', 'y', 'z')
    assert isinstance(pixel_rays.x, list)
    assert isinstance(pixel_rays.y, list)
    assert isinstance(pixel_rays.z, list)
    assert len(pixel_rays.x) == (320*240)
    assert len(pixel_rays.y) == (320*240)
    assert len(pixel_rays.z) == (320*240)


@pytest.mark.functional
def test_meta_data_sensor_temperature(dut: pytofcrust.Sensor):
    def callback(measurement: pytofcrust.Measurement, **kwargs):
        with callback.mutex:
            if measurement.data_type == measurement.DataType.DISTANCE and not callback.measurement:
                callback.measurement = measurement

    callback.mutex = threading.Lock()
    callback.measurement = None
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

    #check for temperature data, note there is no good way to check for actual
    # values so we just check for 4 floats in a reasonable range.
    sensor_temps = callback.measurement.sensor_temperatures
    assert sensor_temps, "No sensor temperature data included with the measurement"
    assert len(sensor_temps) == 4, "Not enough sensor temperature values in the meta-data"
    for v in sensor_temps:
        assert isinstance(v, float), "Sensor temperature value is not a float"
        assert 0 < v < 60.0, "Sensor temperature value appears to be out of range"


@pytest.mark.functional
def test_meta_data_integration_times(dut: pytofcrust.Sensor):

    def run(TEST_VALUES: List[int]):
        def callback(measurement: pytofcrust.Measurement, **kwargs):
            with callback.mutex:
                callback.count += 1
                #Note: for some reason it seems to take at least 2 measurement iterations 
                # for new integration time settings to take effect
                # this is probably a bug in the sensor firmware but it's not import ATM
                if callback.count > 2:
                    if measurement.data_type == measurement.DataType.DISTANCE and not callback.measurement:
                        callback.measurement = measurement

        callback.mutex = threading.Lock()
        callback.count = 0
        callback.measurement = None
        dut.set_integration_times(*TEST_VALUES)
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

        #check for integration time data with the measurement.
        int_times = callback.measurement.integration_times
        assert int_times, "No integration time data included with the measurement"
        assert len(int_times) == 3, "Not enough integration time values in the meta-data"
        assert TEST_VALUES == int_times, "Incorrect integration time values included in meta-data"

    run([11, 22, 33])
    run([100, 0, 0])
    run([111, 222, 333])

@pytest.mark.functional
def test_modulation_frequencies(dut: pytofcrust.Sensor):

    def run(testFreqs, factoryMode, testName):

        dut.set_factory_mode(factoryMode)

        # Cycle through test frequencies
        for testFreq in testFreqs:

            expectedFrequency = None

            dut.modulation_frequency = testFreq

            # While not in factory mode, round to the nearest valid frequency
            if (not factoryMode):
                if (testFreq < 8000):
                    expectedFrequency = 6000
                elif (testFreq < 18000):
                    expectedFrequency = 12000
                else:
                    expectedFrequency = 24000

            # While in factory mode, enable all frequencies
            # but round to nearest 10 kHz
            # and cap at bounds (6000, 24000) for low / high respectively
            else:
                if (testFreq < 6000):
                    expectedFrequency = 6000
                elif (testFreq > 24000):
                    expectedFrequency = 24000
                else:
                    expectedFrequency = int(round(testFreq/10.0) * 10)

            time.sleep(0.1)

            readVal = dut.modulation_frequency
            print(f"Frequency set: {testFreq}, Read back (calculated): {readVal} (kHz)")

            assert readVal == expectedFrequency, testName + "Mod Freq read does NOT equal mod frequency set"

    # Test rounding
    run([6001, 6009], True, "Rounding Test")

    #Test valid frequencies (factory mode enabled)
    run([6050, 8000, 10100, 13000, 22000], True, "Valid Freqs Test (Factory Mode)")

    # Test out of bounds
    run([3000, 25000], True, "Freq outside bounds")

    # Test frequencies (factory mode disabled)
    run([6000, 7000, 12000, 23000, 24000], False, "Valid Freqs Test (Outside Factory Mode)")

@pytest.mark.functional
def test_meta_data_modulation_frequencies(dut: pytofcrust.Sensor):

    def run(TEST_VALUE: int):
        def callback(measurement: pytofcrust.Measurement, **kwargs):
            with callback.mutex:
                callback.count += 1
                #Note: for some reason it seems to take at least 2 measurement iterations 
                # for new integration time settings to take effect
                # this is probably a bug in the sensor firmware but it's not import ATM
                if callback.count > 1:
                    if measurement.data_type == measurement.DataType.DISTANCE and not callback.measurement:
                        callback.measurement = measurement

        callback.mutex = threading.Lock()
        callback.count = 0
        callback.measurement = None
 
        dut.modulation_frequency = TEST_VALUE
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

        mod_freqs = callback.measurement.modulation_frequencies
        assert mod_freqs, "No modulation frequency data included with the measurement"
        assert len(mod_freqs) == 1, "Not enough modulation frequency values in the meta-data"
        assert TEST_VALUE *1000 == mod_freqs[0], "Incorrect modulation frequency values included in meta-data"

    # Test meta-data in header. Note that meta data reports frequency in Hz (not kHz as is argument)
    run(6000)
    run(12000)
    run(24000)


@pytest.mark.functional
def test_meta_data_binning(dut: pytofcrust.Sensor):

    def run(TEST_VALUE: List[int]):
        def callback(measurement: pytofcrust.Measurement, **kwargs):
            with callback.mutex:
                callback.count += 1
                #Note: for some reason it seems to take at least 2 measurement iterations 
                # for new settings to take effect this is probably a bug in the sensor
                # firmware but it's not import ATM
                if callback.count > 2:
                    if measurement.data_type == measurement.DataType.DISTANCE and not callback.measurement:
                        callback.measurement = measurement

        callback.mutex = threading.Lock()
        callback.count = 0
        callback.measurement = None

        h_setting = TEST_VALUE[0] == 2
        v_setting = TEST_VALUE[1] == 2
        dut.set_binning(v_setting, h_setting)
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

        horizontal_binning = callback.measurement.horizontal_binning
        assert horizontal_binning is not None, "No horizontal binning data included with the measurement"
        assert TEST_VALUE[0] == horizontal_binning, "Incorrect horizontal binning value included in meta-data"

        vertical_binning = callback.measurement.vertical_binning
        assert vertical_binning is not None, "No vertical binning data included with the measurement"
        assert TEST_VALUE[1] == vertical_binning, "Incorrect vertical binning value included in meta-data"

    run([2, 2])
    run([0, 2])
    run([2, 0])
    run([0, 0])



@pytest.mark.functional
def test_meta_data_dll(dut: pytofcrust.Sensor):

    def run(enable_dll, coarse_step, fine_step, finest_step):
        def callback(measurement: pytofcrust.Measurement, **kwargs):
            with callback.mutex:
                callback.count += 1
                #Note: for some reason it seems to take at least 2 measurement iterations 
                # for new settings to take effect this is probably a bug in the sensor
                # firmware but it's not import ATM
                if callback.count > 2:
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
        while count != 10:
            count += 1
            time.sleep(0.1)
            with callback.mutex:
                if callback.measurement:
                    count = 10
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
