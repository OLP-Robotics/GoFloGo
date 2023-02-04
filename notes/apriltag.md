# AprilTags in WPILib

Example code from wpilib: https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/apriltagsvision
CheifDelphi example code: https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/9 ***works

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

//Sample code from cheifdelphi
import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Thread visionThread = new Thread(() -> apriltagVisionThreadProc());
    visionThread.setDaemon(true);
    visionThread.start();
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  void apriltagVisionThreadProc() {
    AprilTagDetector detector = new AprilTagDetector();
    detector.addFamily("tag16h5", 0);
  
    // Get the UsbCamera from CameraServer
    UsbCamera camera = CameraServer.startAutomaticCapture();
    // Set the resolution
    camera.setResolution(640, 480);

    // Get a CvSink. This will capture Mats from the camera
    CvSink cvSink = CameraServer.getVideo();
    // Setup a CvSource. This will send images back to the Dashboard
    CvSource outputStream = CameraServer.putVideo("detect", 640, 480);

    // Mats are very memory expensive. Lets reuse this Mat.
    Mat mat = new Mat();
    Mat grayMat = new Mat();
    ArrayList<Integer> tags = new ArrayList<>();

    //
    Scalar outlineColor = new Scalar(0, 255, 0);
    Scalar xColor = new Scalar(0, 0, 255);

    // This cannot be 'true'. The program will never exit if it is. This
    // lets the robot stop this thread when restarting robot code or
    // deploying.
    while (!Thread.interrupted()) {
      // Tell the CvSink to grab a frame from the camera and put it
      // in the source mat.  If there is an error notify the output.
      if (cvSink.grabFrame(mat) == 0) {
        // Send the output the error.
        outputStream.notifyError(cvSink.getError());
        // skip the rest of the current iteration
        continue;
      }

      Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

      AprilTagDetection[] detections = detector.detect(grayMat);
      tags.clear();
      for (AprilTagDetection detection : detections) {
        tags.add(detection.getId());

        for (var i = 0; i <= 3; i++) {
          var j = (i + 1) % 4;
          var pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
          var pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
          Imgproc.line(mat, pt1, pt2, outlineColor, 2);
        }

        var cx = detection.getCenterX();
        var cy = detection.getCenterY();
        var ll = 10;
        Imgproc.line(mat, new Point(cx - ll, cy), new Point(cx + ll, cy), xColor, 2);
        Imgproc.line(mat, new Point(cx, cy - ll), new Point(cx, cy + ll), xColor, 2);
        Imgproc.putText(mat, Integer.toString(detection.getId()), new Point (cx + ll, cy), Imgproc.FONT_HERSHEY_SIMPLEX, 1, xColor, 3);
      }

      SmartDashboard.putString("tag", tags.toString());
      // Give the output stream a new image to display
      outputStream.putFrame(mat);
    }

    detector.close();
  }

}
