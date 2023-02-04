// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;


/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive myRobot;
  private XboxController xbox;

  private final WPI_TalonFX talMotorL1 = new WPI_TalonFX(1);
  private final WPI_TalonFX talMotorL2 = new WPI_TalonFX(2);
  private final WPI_TalonFX talMotorR3 = new WPI_TalonFX(3);
  private final WPI_TalonFX talMotorR4 = new WPI_TalonFX(4);
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(talMotorL1, talMotorL2);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(talMotorR3, talMotorR4);
  //private static final double kAngleSetpoint = 0.0;
	private static final double kP = 1; // propotional turning constant
  // The heading of the robot when starting the motion
double heading;

	// gyro calibration constant, may need to be adjusted;
	// gyro value of 360 is set to correspond to one full revolution
	//private static final double kVoltsPerDegreePerSecond = 0.0128;
  private ADXRS450_Gyro mgyro = new ADXRS450_Gyro();
  private final Timer mtimer = new Timer();

  @Override
  public void robotInit() {
    xbox = new XboxController(0);
    rightMotors.setInverted(true);
    myRobot = new DifferentialDrive(leftMotors, rightMotors);
    CameraServer.startAutomaticCapture();
    mgyro.calibrate();


  }
  @Override
  public void autonomousInit() {
      // Set setpoint to current heading at start of auto
      heading = mgyro.getAngle();
      mtimer.reset();
      mtimer.start();
  }
  
  @Override
  public void autonomousPeriodic() {
    double error = heading - mgyro.getAngle();
   if(mtimer.get() < 2.0) {
    
  
      // Drives forward continuously at half speed, using the gyro to stabilize the heading
      myRobot.tankDrive(.5 + kP * error, .5 - kP * error);
      
    }
    else{
      myRobot.tankDrive(0,0);
    }
    SmartDashboard.putNumber("error", error);
      SmartDashboard.putNumber("tankdriver", .5 + kP * error);
  }
  @Override
  public void teleopPeriodic() {
    
    //Uses the 2 joysticks on the xboxController to control the left and right side of the tank drive
    myRobot.tankDrive(-xbox.getLeftY(), -xbox.getRightY());

  }
}
