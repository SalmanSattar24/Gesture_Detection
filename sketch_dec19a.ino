/**
 * Gesture Recognition System using MPU6050 and Edge Impulse
 * Detects Circle and Static gestures using accelerometer and gyroscope data
 */

#include <Wire.h>
#include <Gesture_Recognition_inferencing.h>

// System Configuration Constants
#define FREQUENCY_HZ 60           // Sampling frequency in Hz
#define INTERVAL_MS (1000 / (FREQUENCY_HZ + 1))  // Time between samples
#define MPU_ADDR 0x68            // I2C address of MPU6050
#define RED_PIN 13               // LED pin for Circle gesture
#define BLUE_PIN 12              // LED pin for Static gesture

// Gesture Detection Thresholds
#define CIRCLE_CONFIDENCE_THRESHOLD 0.9  // High threshold for circle detection accuracy
#define STATIC_CONFIDENCE_THRESHOLD 0.4  // Lower threshold for static state
#define MOTION_THRESHOLD 3000            // Threshold for determining significant motion
#define STATIC_COUNT_THRESHOLD 2         // Minimum samples needed for static state

// Global Variables
static unsigned long last_interval_ms = 0;  // Tracks last sample time
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];  // Buffer for sensor data
size_t feature_ix = 0;                              // Current position in feature buffer
String currentGesture = "none";                     // Current detected gesture
float lastMotionMagnitude = 0;                     // Previous motion magnitude
int staticCounter = 0;                             // Counter for static state detection

/**
 * Initialize hardware and MPU6050 sensor
 */
void setup() {
    Serial.begin(115200);         // Initialize serial communication
    pinMode(RED_PIN, OUTPUT);     // Configure LED pins
    pinMode(BLUE_PIN, OUTPUT);
    
    // Initialize MPU6050
    Wire.begin();
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);            // Power management register
    Wire.write(0);               // Wake up MPU6050
    Wire.endTransmission(true);
}

/**
 * Main program loop - handles sensor reading and gesture detection
 */
void loop() {
    // Check if it's time for next sample
    if (millis() > last_interval_ms + INTERVAL_MS) {
        last_interval_ms = millis();

        // Read sensor data
        int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
        readMPU6050Data(AcX, AcY, AcZ, GyX, GyY, GyZ);

        // Calculate total motion magnitude from gyroscope data
        float motionMagnitude = sqrt(GyX*GyX + GyY*GyY + GyZ*GyZ);
        
        // Store sensor readings in feature buffer
        features[feature_ix++] = AcX;
        features[feature_ix++] = AcY;
        features[feature_ix++] = AcZ;
        features[feature_ix++] = GyX;
        features[feature_ix++] = GyY;
        features[feature_ix++] = GyZ;

        // Process data when buffer is full
        if (feature_ix == EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
            // Prepare data for inference
            signal_t signal;
            ei_impulse_result_t result;

            // Run inference using Edge Impulse model
            numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
            run_classifier(&signal, &result, false);

            // Extract confidence scores for each gesture
            float circleConfidence = 0;
            float staticConfidence = 0;
            for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                if (strcmp(result.classification[ix].label, "Circle") == 0) {
                    circleConfidence = result.classification[ix].value;
                }
                else if (strcmp(result.classification[ix].label, "Static") == 0) {
                    staticConfidence = result.classification[ix].value;
                }
            }

            // Debug output
            Serial.print("Circle: ");
            Serial.print(circleConfidence, 3);
            Serial.print(" Static: ");
            Serial.print(staticConfidence, 3);
            Serial.print(" Motion: ");
            Serial.println(motionMagnitude);

            // Detect Circle gesture - requires motion and high confidence
            if (motionMagnitude > MOTION_THRESHOLD && 
                circleConfidence > CIRCLE_CONFIDENCE_THRESHOLD) {
                staticCounter = 0;
                digitalWrite(RED_PIN, HIGH);
                digitalWrite(BLUE_PIN, LOW);
                currentGesture = "Circle";
                Serial.println("DETECTED: Circle gesture!");
            } 
            // Detect Static gesture - requires low motion
            else if (motionMagnitude < MOTION_THRESHOLD) {
                staticCounter++;
                // Trigger on sustained low motion or very high confidence
                if ((staticCounter >= STATIC_COUNT_THRESHOLD && 
                     staticConfidence > STATIC_CONFIDENCE_THRESHOLD) ||
                    staticConfidence > 0.8) {
                    digitalWrite(BLUE_PIN, HIGH);
                    digitalWrite(RED_PIN, LOW);
                    currentGesture = "Static";
                    Serial.println("DETECTED: Static gesture!");
                }
            } else {
                staticCounter = 0;
            }

            // Reset state if no gesture detected
            if (circleConfidence < 0.2 && staticConfidence < 0.2) {
                digitalWrite(RED_PIN, LOW);
                digitalWrite(BLUE_PIN, LOW);
                currentGesture = "none";
                Serial.println("No gesture detected");
            }
            
            feature_ix = 0;  // Reset buffer index
        }
    }
}

/**
 * Read raw sensor data from MPU6050
 * @param AcX, AcY, AcZ - Accelerometer values for each axis
 * @param GyX, GyY, GyZ - Gyroscope values for each axis
 */
void readMPU6050Data(int16_t &AcX, int16_t &AcY, int16_t &AcZ, 
                     int16_t &GyX, int16_t &GyY, int16_t &GyZ) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);  // Starting register for accel data
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);  // Request 14 bytes of data

    // Read accelerometer data (3 axes x 2 bytes each)
    AcX = Wire.read() << 8 | Wire.read();  // X-axis
    AcY = Wire.read() << 8 | Wire.read();  // Y-axis 
    AcZ = Wire.read() << 8 | Wire.read();  // Z-axis
    Wire.read(); Wire.read();              // Skip temperature
    // Read gyroscope data (3 axes x 2 bytes each)
    GyX = Wire.read() << 8 | Wire.read();  // X-axis rotation
    GyY = Wire.read() << 8 | Wire.read();  // Y-axis rotation
    GyZ = Wire.read() << 8 | Wire.read();  // Z-axis rotation
}