# This message contains a DCS or Ambient frame from a Preact T21 camera

#Note: Usage of header copied from ROS2 sensor_msg/Image
std_msgs/Header header # Header timestamp is the timestamp reported by the camera.
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image
                             # If the frame_id here and the frame_id of the CameraInfo
                             # message associated with the image conflict
                             # the behavior is undefined

string chip_id             # Unique sensor identifier as reported by the camera, used to locate calibration data
uint32 height                # image height, that is, number of rows
uint32 width                 # image width, that is, number of columns

string encoding              # Encoding of pixels -- channel meaning, ordering, size
                             # Possible values is TBD

uint32 step                  # Full row length in bytes
uint16 index                 # Raw data frame index, used to detect missing frames

uint8 TYPE_DCS0 = 0
uint8 TYPE_DCS1 = 1
uint8 TYPE_DCS2 = 2
uint8 TYPE_DCS3 = 3
uint8 TYPE_AMBIENT = 4
uint8 type                   # type of frame, see TYPE_XXX constants above

# Camera status information
float64 temperature          # temperature at the time of the last ambient frame (degress celcius)
float32 modulation_frequency # Modulation frequency setting in MHz
uint16 integration_time      # Integration time in microseconds
# Binning refers here to any camera setting which combines rectangular
#  neighborhoods of pixels into larger "super-pixels." It reduces the
#  resolution of the output image to
#  (width / binning_h) x (height / binning_v).
# The default values binning_h = binning_v = 0 is considered the same
#  as binning_h = binning_v = 1 (no subsampling).
uint32 binning_h
uint32 binning_v
uint8 config_index           # The configuration index in effect when the frame was captured
bool input_signal            # state of the input signal GPIO, true = high, false = low
uint16 vled_mv               # ??? 

uint8[] klv                  # Binary packed KLV data

# Frame data
uint8[] data                 # actual matrix data, size is (step * rows)
