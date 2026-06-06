#ifndef CONFIG_H
#define CONFIG_H

// ------------------------------------------------------------
// WiFi:
// ------------------------------------------------------------

const char* WIFI_SSID = "NanoFloat2.0";
const char* WIFI_PASSWORD = "fernano";


// ------------------------------------------------------------
// Encoder Test:
// ------------------------------------------------------------

// Tolerance:
// (Allowed mission tolerance is +/-33cm)
#define ENCODER_TOLERANCE  0.20f 

// Calibration targets (please replace with actual counts after test is run):                                         
#define ENCODER_COUNT_0_4M  1000000     
#define ENCODER_COUNT_2_5M  6250000

// The magnitude of each encoder step
#define ENCODER_CORRECTION_STEP 100000

#endif
