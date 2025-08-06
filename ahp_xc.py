# This Python file uses the following encoding: utf-8
from ctypes.util import find_library

class ahp_xc_correlation(Structure): _fields_ = [
    ("num_indexes", ctypes.c_int)
    ("indexes", ctypes.c_int_p)
    ("lags", ctypes.c_double_p)
    ("lag", ctypes.c_double)
    ("real", ctypes.c_longlong)
    ("imaginary", ctypes.c_longlong)
    ("counts", ctypes.c_ulonglong)
    ("magnitude", ctypes.c_double)
    ("phase", ctypes.c_double)]

class ahp_xc_scan_request(Structure): _fields_ = [
    ("index", ctypes.c_uint)
    ("start", ctypes.c_longlong)
    ("len", ctypes.c_ulonglong)
    ("step", ctypes.c_ulonglong)
    ("cur_chan", ctypes.c_longlong)]

class ahp_xc_sample(Structure): _fields_ = [
    ("lag", ctypes.c_double)
    ("lag_size", ctypes.c_ulonglong)
    ("correlations", POINTER(ahp_xc_correlation)]

class ahp_xc_packet(Structure): _fields_ = [
    ("timestamp", ctypes.c_double)
    ("n_lines", ctypes.c_ulonglong)
    ("n_baselines", ctypes.c_ulonglong)
    ("tau", ctypes.c_ulonglong)
    ("bps", ctypes.c_ulonglong)
    ("cross_lag", ctypes.c_ulonglong)
    ("auto_lag", ctypes.c_ulonglong)
    ("counts", ctypes.c_ulonglong_p)
    ("autocorrelations", POINTER(ahp_xc_sample))
    ("crosscorrelations", POINTER(ahp_xc_sample))
    ("lock", ctypes.c_void_p)
    ("buf", ctypes.c_char_p)]

class ahp_xc:
    def __init__(self):
        libahp_xc = find_library("ahp_xc")
        set_debug_level = libahp_xc.ahp_set_debug_level
        set_debug_level.argtypes = [("value", ctypes.c_int)]
        get_debug_level = libahp_xc.ahp_get_debug_level
        get_debug_level.restype = ctypes.c_int
        set_app_name = libahp_xc.ahp_set_app_name
        set_app_name.argtypes = [("name", ctypes.c_char_p)]
        get_app_name = libahp_xc.ahp_get_app_name
        get_app_name.restype = ctypes.c_char_p
        set_stdout(FILE *f)
        set_stderr(FILE *f)
        get_2d_projection = libahp_xc.ahp_xc_get_2d_projection
        get_2d_projection.argtypes = [("alt", ctypes.c_double), ("az", ctypes.c_double), ("baseline", ctypes.c_double_p)]
        get_2d_projection.restype = ctypes.c_double_p;
        max_threads = libahp_xc.ahp_xc_max_threads
        max_threads.argtypes = [("value", ctypes.c_ulonglong)]
        max_threads.restype = ctypes.c_ulonglong
        connect = libahp_xc.ahp_xc_connect
        connect.argtypes = [("port", ctypes.c_char_p)]
        connect.restype = ctypes.c_int
        connect_fd = libahp_xc.ahp_xc_connect_fd
        connect_fd.argtypes = [("fd", ctypes.c_int)]
        connect_fd.restype = ctypes.c_int
        get_fd = libahp_xc.ahp_xc_get_fd
        get_fd.restype = ctypes.c_int
        disconnect = libahp_xc.ahp_xc_disconnect
        is_connected = libahp_xc.ahp_xc_is_connected
        is_connected.restype = ctypes.c_uint
        is_detected = libahp_xc.ahp_xc_is_detected
        is_detected.restype = ctypes.c_uint
        get_baudrate = libahp_xc.ahp_xc_get_baudrate
        get_baudrate.restype = ctypes.c_int
        set_baudrate(baud_rate rate)
        set_correlation_order = libahp_xc.ahp_xc_set_correlation_order
        set_correlation_order.argtypes = [("order", ctypes.c_uint)]
        get_correlation_order = libahp_xc.ahp_xc_get_correlation_order
        get_correlation_order.restype = ctypes.c_int
        get_properties = libahp_xc.ahp_xc_get_properties
        get_properties.restype = ctypes.c_int
        get_header = libahp_xc.ahp_xc_get_header
        get_header.restype = ctypes.c_char_p
        get_bps = libahp_xc.ahp_xc_get_bps
        get_bps.restype = ctypes.c_uint
        get_nlines = libahp_xc.ahp_xc_get_nlines
        get_nlines.restype = ctypes.c_uint
        get_nbaselines = libahp_xc.ahp_xc_get_nbaselines
        get_nbaselines.restype = ctypes.c_uint
        get_crosscorrelation_index = libahp_xc.ahp_xc_get_crosscorrelation_index
        get_crosscorrelation_index.argtypes = [("", ctypes.c_int)*lines, ("order", ctypes.c_int)]
        get_crosscorrelation_index.restype = ctypes.c_int
        get_line_index = libahp_xc.ahp_xc_get_line_index
        get_line_index.argtypes = [("idx", ctypes.c_int), ("order", ctypes.c_int)]
        get_line_index.restype = ctypes.c_int
        get_npolytopes = libahp_xc.ahp_xc_get_npolytopes
        get_npolytopes.argtypes = [("order", ctypes.c_int)]
        get_npolytopes.restype = ctypes.c_uint
        get_delaysize = libahp_xc.ahp_xc_get_delaysize
        get_delaysize.restype = ctypes.c_uint
        get_autocorrelator_lagsize = libahp_xc.ahp_xc_get_autocorrelator_lagsize
        get_autocorrelator_lagsize.restype = ctypes.c_uint
        get_crosscorrelator_lagsize = libahp_xc.ahp_xc_get_crosscorrelator_lagsize
        get_crosscorrelator_lagsize.restype = ctypes.c_uint
        get_frequency = libahp_xc.ahp_xc_get_frequency
        get_frequency.restype = ctypes.c_double
        get_sampletime = libahp_xc.ahp_xc_get_sampletime
        get_sampletime.restype = ctypes.c_double
        get_packettime = libahp_xc.ahp_xc_get_packettime
        get_packettime.restype = ctypes.c_double
        get_packetsize = libahp_xc.ahp_xc_get_packetsize
        get_packetsize.restype = ctypes.c_uint
        enable_crosscorrelator = libahp_xc.ahp_xc_enable_crosscorrelator
        enable_crosscorrelator.argtypes = [("enable", ctypes.c_int)]
        has_crosscorrelator = libahp_xc.ahp_xc_has_crosscorrelator
        has_crosscorrelator.restype = ctypes.c_int
        has_psu = libahp_xc.ahp_xc_has_psu
        has_psu.restype = ctypes.c_int
        has_leds = libahp_xc.ahp_xc_has_leds
        has_leds.restype = ctypes.c_int
        has_cumulative_only = libahp_xc.ahp_xc_has_cumulative_only
        has_cumulative_only.restype = ctypes.c_int
        packet *ahp_xc_alloc_packet = libahp_xc.ahp_xc_alloc_packet
        packet *ahp_xc_copy_packet(ahp_xc_packet *packet)
        free_packet(ahp_xc_packet *packet)
        sample *ahp_xc_alloc_samples = libahp_xc.ahp_xc_alloc_samples
        sample *ahp_xc_alloc_samples.argtypes = [("nlines", ctypes.c_ulonglong), ("size", ctypes.c_ulonglong)]
        sample *ahp_xc_copy_samples(ahp_xc_sample* src, ("nlines", ctypes.c_ulonglong), ("size", ctypes.c_ulonglong)]
        free_samples = libahp_xc.ahp_xc_free_samples
        free_samples.argtypes = [("nlines", ctypes.c_ulonglong), ahp_xc_sample *samples)
        get_packet(ahp_xc_packet *packet)
        get_packet.restype = ctypes.c_int
        start_autocorrelation_scan = libahp_xc.ahp_xc_start_autocorrelation_scan
        start_autocorrelation_scan.argtypes = [("index", ctypes.c_uint)]
        end_autocorrelation_scan = libahp_xc.ahp_xc_end_autocorrelation_scan
        end_autocorrelation_scan.argtypes = [("index", ctypes.c_uint)]
        scan_autocorrelations(ahp_xc_scan_request *lines, ("nlines", ctypes.c_uint), ahp_xc_sample **autocorrelations, ("", ctypes.c_int)*("ctypes", ctypes.c_interrupt,).c_double_p percent)
        scan_autocorrelations.restype = ctypes.c_int
        start_crosscorrelation_scan = libahp_xc.ahp_xc_start_crosscorrelation_scan
        start_crosscorrelation_scan.argtypes = [("index", ctypes.c_uint)]
        end_crosscorrelation_scan = libahp_xc.ahp_xc_end_crosscorrelation_scan
        end_crosscorrelation_scan.argtypes = [("index", ctypes.c_uint)]
        scan_crosscorrelations(ahp_xc_scan_request *lines, ("nlines", ctypes.c_uint), ahp_xc_sample **crosscorrelations, ("", ctypes.c_int)*("ctypes", ctypes.c_interrupt,).c_double_p percent)
        scan_crosscorrelations.restype = ctypes.c_int
        set_capture_flags(xc_capture_flags flags)
        set_capture_flags.restype = ctypes.c_int
        get_capture_flags = libahp_xc.ahp_xc_get_capture_flags
        get_capture_flags.restype = ctypes.c_int
        set_leds = libahp_xc.ahp_xc_set_leds
        set_leds.argtypes = [("index", ctypes.c_uint), ("leds", ctypes.c_int)]
        set_channel_cross = libahp_xc.ahp_xc_set_channel_cross
        set_channel_cross.argtypes = [("index", ctypes.c_uint), ("value", ctypes.c_longlong), ("size", ctypes.c_ulonglong), ("step", ctypes.c_ulonglong)]
        set_channel_auto = libahp_xc.ahp_xc_set_channel_auto
        set_channel_auto.argtypes = [("index", ctypes.c_uint), ("value", ctypes.c_longlong), ("size", ctypes.c_ulonglong), ("step", ctypes.c_ulonglong)]
        set_frequency_divider(unsigned char value)
        set_voltage = libahp_xc.ahp_xc_set_voltage
        set_voltage.argtypes = [("index", ctypes.c_uint), unsigned char value)
        set_test_flags = libahp_xc.ahp_xc_set_test_flags
        set_test_flags.argtypes = [("index", ctypes.c_uint), ("test", ctypes.c_int)]
        get_test_flags = libahp_xc.ahp_xc_get_test_flags
        get_test_flags.restype = ctypes.c_byte
        get_test_flags.argtypes = [("index", ctypes.c_uint)]
        get_leds = libahp_xc.ahp_xc_get_leds
        get_leds.restype = ctypes.c_byte
        get_leds.argtypes = [("index", ctypes.c_uint)]
        select_input = libahp_xc.ahp_xc_select_input
        select_input.argtypes = [("index", ctypes.c_uint)]
        current_input = libahp_xc.ahp_xc_current_input
        current_input.restype = ctypes.c_uint
        send_command(xc_cmd cmd, unsigned char value)
        send_command.restype = ctypes.c_int
        get_version = libahp_xc.ahp_xc_get_version
        get_version.restype = ctypes.c_uint
