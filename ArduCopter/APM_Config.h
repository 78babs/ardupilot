// User specific config file.  Any items listed in config.h can be overridden here.

// uncomment the lines below to disable features (flash sizes listed are for APM2 boards and will underestimate savings on Pixhawk and other boards)
//#define LOGGING_ENABLED       DISABLED            // disable logging to save 11K of flash space
//#define MOUNT                 DISABLED            // disable the camera gimbal to save 8K of flash space
//#define AUTOTUNE_ENABLED      DISABLED            // disable the auto tune functionality to save 7k of flash
//#define RANGEFINDER_ENABLED   DISABLED            // disable rangefinder to save 1k of flash
//#define AC_AVOID_ENABLED      DISABLED            // disable stop-at-fence library
//#define AC_OAPATHPLANNER_ENABLED DISABLED         // disable path planning around obstacles
//#define PARACHUTE             DISABLED            // disable parachute release to save 1k of flash
//#define NAV_GUIDED            DISABLED            // disable external navigation computer ability to control vehicle through MAV_CMD_NAV_GUIDED mission commands
//#define PRECISION_LANDING     DISABLED            // disable precision landing using companion computer or IRLock sensor
//#define BEACON_ENABLED        DISABLED            // disable beacon support
//#define STATS_ENABLED         DISABLED            // disable statistics support
//#define MODE_ACRO_ENABLED     DISABLED            // disable acrobatic mode support
//#define MODE_AUTO_ENABLED     DISABLED            // disable auto mode support
//#define MODE_BRAKE_ENABLED    DISABLED            // disable brake mode support
//#define MODE_CIRCLE_ENABLED   DISABLED            // disable circle mode support
//#define MODE_DRIFT_ENABLED    DISABLED            // disable drift mode support
//#define MODE_FLIP_ENABLED     DISABLED            // disable flip mode support
//#define MODE_FOLLOW_ENABLED   DISABLED            // disable follow mode support
//#define MODE_GUIDED_ENABLED   DISABLED            // disable guided mode support
//#define MODE_GUIDED_NOGPS_ENABLED   DISABLED      // disable guided/nogps mode support
//#define MODE_LOITER_ENABLED   DISABLED            // disable loiter mode support
//#define MODE_POSHOLD_ENABLED  DISABLED            // disable poshold mode support
//#define MODE_RTL_ENABLED      DISABLED            // disable rtl mode support
//#define MODE_SMARTRTL_ENABLED DISABLED            // disable smartrtl mode support
//#define MODE_SPORT_ENABLED    DISABLED            // disable sport mode support
//#define MODE_SYSTEMID_ENABLED DISABLED            // disable system ID mode support
//#define MODE_THROW_ENABLED    DISABLED            // disable throw mode support
//#define MODE_ZIGZAG_ENABLED   DISABLED            // disable zigzag mode support
//#define OSD_ENABLED           DISABLED            // disable on-screen-display support

// features below are disabled by default on all boards
//#define CAL_ALWAYS_REBOOT                         // flight controller will reboot after compass or accelerometer calibration completes
//#define DISALLOW_GCS_MODE_CHANGE_DURING_RC_FAILSAFE   // disable mode changes from GCS during Radio failsafes.  Avoids a race condition for vehicle like Solo in which the RC and telemetry travel along the same link
//#define ADVANCED_FAILSAFE     ENABLED             // enabled advanced failsafe which allows running a portion of the mission in failsafe events

// other settings
//#define THROTTLE_IN_DEADBAND   100                // redefine size of throttle deadband in pwm (0 ~ 1000)

// User Hooks : For User Developed code that you wish to run
// Put your variable definitions into the UserVariables.h file (or another file name and then change the #define below).
//#define USERHOOK_VARIABLES "UserVariables.h"
// Put your custom code into the UserCode.cpp with function names matching those listed below and ensure the appropriate #define below is uncommented below
//#define USERHOOK_INIT userhook_init();                      // for code to be run once at startup
//#define USERHOOK_FASTLOOP userhook_FastLoop();            // for code to be run at 100hz
//#define USERHOOK_50HZLOOP userhook_50Hz();                  // for code to be run at 50hz
//#define USERHOOK_MEDIUMLOOP userhook_MediumLoop();        // for code to be run at 10hz
//#define USERHOOK_SLOWLOOP userhook_SlowLoop();            // for code to be run at 3.3hz
//#define USERHOOK_SUPERSLOWLOOP userhook_SuperSlowLoop();  // for code to be run at 1hz
//#define USERHOOK_AUXSWITCH ENABLED                        // for code to handle user aux switches
//#define USER_PARAMS_ENABLED ENABLED                       // to enable user parameters

// Manual APM configuration for Baudoin identical to that flying on early octobrer 2023

define HAL_EXTERNAL_AHRS_ENABLED 1
define HAL_NAVEKF2_AVAILABLE 0
define HAL_NAVEKF3_AVAILABLE 1
define EK3_FEATURE_EXTERNAL_NAV 1
define EK3_FEATURE_DRAG_FUSION 0
define HAL_INS_TEMPERATURE_CAL_ENABLE 0
define HAL_VISUALODOM_ENABLED 0
define HAL_PERIPH_SUPPORT_LONG_CAN_PRINTF 0
define AP_FETTEC_ONEWIRE_ENABLED 0
define AP_DRONECAN_HIMARK_SERVO_ENABLED 0
define AP_DRONECAN_HOBBYWING_ESC_ENABLED 0
define AP_ROBOTISSERVO_ENABLED 0
define AP_VOLZ_ENABLED 0
define AP_DRONECAN_VOLZ_FEEDBACK_ENABLED 0
define AP_AIRSPEED_ASP5033_ENABLED 0
define AP_AIRSPEED_ANALOG_ENABLED 0
define AP_AIRSPEED_DLVR_ENABLED 0
define AP_AIRSPEED_MS4525_ENABLED 0
define AP_AIRSPEED_MS5525_ENABLED 0
define AP_AIRSPEED_MSP_ENABLED 0
define AP_AIRSPEED_NMEA_ENABLED 0
define AP_AIRSPEED_SDP3X_ENABLED 0
define AP_AIRSPEED_UAVCAN_ENABLED 0
define AP_BATTERY_FUELFLOW_ENABLED 0
define AP_BATTERY_FUELLEVEL_ANALOG_ENABLED 1
define AP_BATTERY_FUELLEVEL_PWM_ENABLED 0
define AP_BATTERY_INA2XX_ENABLED 0
define AP_BATTERY_SMBUS_ENABLED 0
define AP_BATTERY_SYNTHETIC_CURRENT_ENABLED 0
define AP_CAMERA_ENABLED 0
define AP_CAMERA_MAVLINK_ENABLED 0
define AP_CAMERA_MOUNT_ENABLED 0
define AP_CAMERA_RELAY_ENABLED 0
define AP_CAMERA_SERVO_ENABLED 0
define AP_CAMERA_SOLOGIMBAL_ENABLED 0
define HAL_RUNCAM_ENABLED 0
define AP_COMPASS_AK09916_ENABLED 0
define AP_COMPASS_AK8963_ENABLED 0
define AP_COMPASS_BMM150_ENABLED 0
define AP_COMPASS_EXTERNALAHRS_ENABLED 0
define AP_COMPASS_HMC5843_ENABLED 1
define AP_COMPASS_ICM20948_ENABLED 0
define AP_COMPASS_IST8308_ENABLED 0
define AP_COMPASS_LIS3MDL_ENABLED 0
define AP_COMPASS_LSM303D_ENABLED 1
define AP_COMPASS_LSM9DS1_ENABLED 0
define AP_COMPASS_MAG3110_ENABLED 0
define AP_COMPASS_MMC3416_ENABLED 0
define AP_COMPASS_MMC5XX3_ENABLED 0
define AP_COMPASS_QMC5883L_ENABLED 0
define AP_COMPASS_RM3100_ENABLED 0
define AP_COMPASS_UAVCAN_ENABLED 0
define MODE_FLIP_ENABLED 0
define MODE_FLOWHOLD_ENABLED 0
define MODE_FOLLOW_ENABLED 0
define MODE_GUIDED_NOGPS_ENABLED 0
define MODE_SPORT_ENABLED 0
define MODE_SYSTEMID_ENABLED 0
define MODE_TURTLE_ENABLED 0
define MODE_ZIGZAG_ENABLED 0
define HAL_PICCOLO_CAN_ENABLE 0
define HAL_TORQEEDO_ENABLED 0
define AP_DRONECAN_SEND_GPS 0
define AP_GPS_ERB_ENABLED 0
define AP_GPS_GSOF_ENABLED 0
define AP_GPS_MAV_ENABLED 0
define AP_GPS_NMEA_ENABLED 0
define AP_GPS_NMEA_UNICORE_ENABLED 0
define AP_GPS_NOVA_ENABLED 0
define AP_GPS_SBF_ENABLED 0
define AP_GPS_SBP_ENABLED 0
define AP_GPS_SBP2_ENABLED 0
define AP_GPS_SIRF_ENABLED 0
define AP_GPS_UBLOX_ENABLED 1
define HAL_MOUNT_ALEXMOS_ENABLED 0
define HAL_MOUNT_GREMSY_ENABLED 0
define HAL_MOUNT_ENABLED 0
define HAL_MOUNT_SERVO_ENABLED 0
define HAL_MOUNT_SIYI_ENABLED 0
define HAL_SOLO_GIMBAL_ENABLED 0
define HAL_MOUNT_STORM32MAVLINK_ENABLED 0
define HAL_MOUNT_STORM32SERIAL_ENABLED 0
define HAL_EFI_ENABLED 0
define AP_EFI_CURRAWONG_ECU_ENABLED 0
define AP_EFI_DRONECAN_ENABLED 0
define AP_EFI_SERIAL_LUTAN_ENABLED 0
define AP_EFI_SERIAL_MS_ENABLED 0
define AP_EFI_NWPWU_ENABLED 0
define HAL_GENERATOR_ENABLED 0
define AP_GENERATOR_RICHENPOWER_ENABLED 0
define AP_ICENGINE_ENABLED 0
define HAL_ADSB_ENABLED 0
define HAL_ADSB_SAGETECH_ENABLED 0
define HAL_ADSB_SAGETECH_MXS_ENABLED 0
define HAL_ADSB_UAVIONIX_MAVLINK_ENABLED 0
define HAL_ADSB_UCP_ENABLED 0
define AP_AIS_ENABLED 0
define HAL_MSP_ENABLED 0
define AP_COMPASS_MSP_ENABLED 0
define HAL_WITH_MSP_DISPLAYPORT 0
define HAL_MSP_GPS_ENABLED 0
define HAL_MSP_OPTICALFLOW_ENABLED 0
define HAL_MSP_RANGEFINDER_ENABLED 0
define HAL_MSP_SENSORS_ENABLED 0
define AP_NOTIFY_MAVLINK_LED_CONTROL_SUPPORT_ENABLED 0
define AP_NOTIFY_NCP5623_ENABLED 0
define AP_NOTIFY_NEOPIXEL_ENABLED 0
define AP_NOTIFY_PROFILED_ENABLED 0
define AP_NOTIFY_PROFILED_SPI_ENABLED 0
define AP_NOTIFY_MAVLINK_PLAY_TUNE_SUPPORT_ENABLED 0
define OSD_ENABLED 0
define OSD_PARAM_ENABLED 0
define HAL_OSD_SIDEBAR_ENABLE 0
define HAL_PLUSCODE_ENABLE 0
define AP_ADVANCEDFAILSAFE_ENABLED 1
define HAL_BARO_WIND_COMP_ENABLED 0
define HAL_DISPLAY_ENABLED 1
define HAL_GYROFFT_ENABLED 1
define HAL_NMEA_OUTPUT_ENABLED 0
define AP_SDCARD_STORAGE_ENABLED 0
define AP_GRIPPER_ENABLED 1
define AP_LANDINGGEAR_ENABLED 0
define HAL_SPRAYER_ENABLED 0
define WINCH_ENABLED 0
define HAL_LANDING_DEEPSTALL_ENABLED 0
define QAUTOTUNE_ENABLED 0
define HAL_QUADPLANE_ENABLED 0
define HAL_SOARING_ENABLED 0
define AP_RCPROTOCOL_SRXL_ENABLED 0
define AP_RANGEFINDER_ENABLED 1
define AP_RANGEFINDER_ANALOG_ENABLED 1
define AP_RANGEFINDER_BENEWAKE_CAN_ENABLED 0
define AP_RANGEFINDER_BENEWAKE_TF02_ENABLED 0
define AP_RANGEFINDER_BENEWAKE_TF03_ENABLED 0
define AP_RANGEFINDER_BENEWAKE_TFMINI_ENABLED 0
define AP_RANGEFINDER_BENEWAKE_TFMINIPLUS_ENABLED 0
define AP_RANGEFINDER_BLPING_ENABLED 0
define AP_RANGEFINDER_GYUS42V2_ENABLED 0
define AP_RANGEFINDER_HC_SR04_ENABLED 0
define AP_RANGEFINDER_LANBAO_ENABLED 0
define AP_RANGEFINDER_LEDDARONE_ENABLED 0
define AP_RANGEFINDER_LEDDARVU8_ENABLED 0
define AP_RANGEFINDER_LIGHTWARE_SERIAL_ENABLED 0
define AP_RANGEFINDER_LWI2C_ENABLED 0
define AP_RANGEFINDER_MAVLINK_ENABLED 0
define AP_RANGEFINDER_MAXBOTIX_SERIAL_ENABLED 1
define AP_RANGEFINDER_MAXSONARI2CXL_ENABLED 1
define AP_RANGEFINDER_NMEA_ENABLED 0
define AP_RANGEFINDER_PULSEDLIGHTLRF_ENABLED 0
define AP_RANGEFINDER_PWM_ENABLED 1
define AP_RANGEFINDER_TRI2C_ENABLED 0
define AP_RANGEFINDER_TERARANGER_SERIAL_ENABLED 0
define AP_RANGEFINDER_UAVCAN_ENABLED 0
define AP_RANGEFINDER_USD1_CAN_ENABLED 0
define AP_RANGEFINDER_USD1_SERIAL_ENABLED 0
define AP_RANGEFINDER_VL53L0X_ENABLED 0
define AP_RANGEFINDER_VL53L1X_ENABLED 0
define AP_RANGEFINDER_WASP_ENABLED 0
define AC_AVOID_ENABLED 1
define AC_OAPATHPLANNER_ENABLED 1
define AP_FENCE_ENABLED 1
define HAL_PARACHUTE_ENABLED 0
define HAL_PROXIMITY_ENABLED 1
define AP_AIRSPEED_ENABLED 0
define BEACON_ENABLED 0
define AP_BARO_BMP085_ENABLED 0
define AP_BARO_BMP280_ENABLED 0
define AP_BARO_BMP388_ENABLED 0
define AP_BARO_DPS280_ENABLED 0
define AP_BARO_DUMMY_ENABLED 0
define AP_BARO_EXTERNALAHRS_ENABLED 0
define AP_BARO_FBM320_ENABLED 0
define GPS_MOVING_BASELINE 0
define AP_BARO_ICM20789_ENABLED 0
define AP_BARO_ICP101XX_ENABLED 0
define AP_BARO_ICP201XX_ENABLED 0
define AP_BARO_KELLERLD_ENABLED 0
define AP_BARO_LPS2XH_ENABLED 0
define AP_BARO_MS56XX_ENABLED 1
define AP_BARO_MSP_ENABLED 0
define AP_OPTICALFLOW_ENABLED 0
define AP_OPTICALFLOW_CXOF_ENABLED 0
define AP_OPTICALFLOW_HEREFLOW_ENABLED 0
define AP_OPTICALFLOW_MAV_ENABLED 0
define AP_OPTICALFLOW_ONBOARD_ENABLED 0
define AP_OPTICALFLOW_PIXART_ENABLED 0
define AP_OPTICALFLOW_PX4FLOW_ENABLED 0
define AP_OPTICALFLOW_UPFLOW_ENABLED 0
define AP_RPM_ENABLED 0
define AP_RPM_EFI_ENABLED 0
define AP_RPM_ESC_TELEM_ENABLED 0
define AP_RPM_GENERATOR_ENABLED 0
define AP_RPM_HARMONICNOTCH_ENABLED 0
define AP_RPM_PIN_ENABLED 0
define AP_BARO_SPL06_ENABLED 0
define AP_TEMPERATURE_SENSOR_ENABLED 0
define AP_TEMPERATURE_SENSOR_MCP9600_ENABLED 0
define AP_TEMPERATURE_SENSOR_TSYS01_ENABLED 0
define AP_BARO_UAVCAN_ENABLED 0
define AP_RC_CHANNEL_AUX_FUNCTION_STRINGS_ENABLED 0
define HAL_CRSF_TELEM_ENABLED 0
define HAL_CRSF_TELEM_TEXT_SELECTION_ENABLED 0
define AP_FRSKY_TELEM_ENABLED 1
define AP_FRSKY_D_TELEM_ENABLED 0
define AP_FRSKY_SPORT_TELEM_ENABLED 1
define AP_FRSKY_SPORT_PASSTHROUGH_ENABLED 1
define HAL_HIGH_LATENCY2_ENABLED 0
define HAL_HOTT_TELEM_ENABLED 0
define AP_LTM_TELEM_ENABLED 0
define HAL_SPEKTRUM_TELEM_ENABLED 0
define AP_MOTORS_FRAME_DECA_ENABLED 0
define AP_MOTORS_FRAME_DODECAHEXA_ENABLED 0
define AP_MOTORS_FRAME_HEXA_ENABLED 1
define AP_MOTORS_FRAME_OCTA_ENABLED 0
define AP_MOTORS_FRAME_OCTAQUAD_ENABLED 0
define AP_MOTORS_FRAME_QUAD_ENABLED 0
define AP_MOTORS_FRAME_Y6_ENABLED 0
define AP_SMARTAUDIO_ENABLED 0
define AP_TRAMP_ENABLED 0
define AP_VIDEOTX_ENABLED 0