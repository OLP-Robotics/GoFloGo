# AprilTags in WPILib

Example code from wpilib: https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/apriltagsvision

## 2D Alignment

1. Scan tag ID
2. Verify ID is correct
3. Rotate robot until tag is centered and aligned

Note: Homography not required

## AprilTag API

### AprilTagJNI

| Method                                                             | Description
| ------------------------------------------------------------------ | ------------------------------------------------------------------
| addFamily(long detector, String fam, int bitsCorrected)            | Pass in detector, string of family you wish to use (16h5), and ...
| createDetector()                                                   | Create the detector object
| destroyDetector(long det)                                          | Destroy the detector object, freeing memory
| detect(long det, int width, int height, int stride, long bufAddr)  | Pass in detector and image information
| setDetectorConfig(long det, AprilTagDetector.Config config)        | Pass in detector, add config parameters

## Java Pseudocode

```java
// Creates detector object
long detector = AprilTagJNI.Helper.createDetector();

// Add tag family to detector
String famString = "16h5" // Not sure if this is correct
int bitsCorrected = 0     // Not sure what this does

addFamily(detector, famString, bitsCorrected); 

// Set detector configuration
AprilTagDetector.Config config = new Config();
config.numThreads = 1;           // Number of parallel threads to use for detection
config.quadDecimate = 2.0;       // How much image is down-sampled before processing; Increases speed, lowers range
config.quadSigma = 1;            // Increasing can increase speed for noisy images
config.decodeSharpening = 0.25;  // Sharpen image?
config.debug = false;            // Slow, should not be used on space-limited systems such as RoboRIO

AprilTagJNI.Helper.setDetectorConfig(detector, config);

// Obtain these values from your image
int imageWidth;
int imageLength;
int imageStride;
int bufAddr;

// Pass in detector and image values to obtain detections
AprilTagDetection[] detections = AprilTagJNI.Helper.detect(
    detector,
    imageWidth,
    imageLength,
    imageStride,
    bufAddr
);

// For each detection... do something
for(AprilTagDetection ad in detections) {
    // Do thing 
}

// Destroy detector when done using to free up memory
destroyDetector(detector);
```
