// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;


/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive myRobot;
  private XboxController xbox;
  private XboxController xboxCoDriver;
  Encoder encoder = new Encoder (1, 0, false, Encoder. EncodingType.k2X);
  Encoder encoder2 = new Encoder(3,2, false, Encoder. EncodingType.k2X); 
  private final WPI_TalonFX talMotorL1 = new WPI_TalonFX(1);
  private final WPI_TalonFX talMotorL2 = new WPI_TalonFX(2);
  private final WPI_TalonFX talMotorR3 = new WPI_TalonFX(3);
  private final WPI_TalonFX talMotorR4 = new WPI_TalonFX(4);
  private final WPI_VictorSPX vicMotorL5 = new WPI_VictorSPX (5);
  private final WPI_VictorSPX vicMotorR6 = new WPI_VictorSPX (6);
  private final WPI_TalonFX talMotorVise = new WPI_TalonFX(7);
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(talMotorL1, talMotorL2);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(talMotorR3, talMotorR4);
  //private static final double kAngleSetpoint = 0.0;

  private DigitalInput limitSwitch = new DigitalInput(0);
  
  private final DoubleSolenoid m_doubleSolenoid =
  new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
 

  private final DoubleSolenoid m_doubleSolenoid2 =
  new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 4);

  // The heading of the robot when starting the motion
double heading;
int stationID = 0;
	// gyro calibration constant, may need to be adjusted;
	// gyro value of 360 is set to correspond to one full revolution
	//private static final double kVoltsPerDegreePerSecond = 0.0128;
  private ADXRS450_Gyro mgyro = new ADXRS450_Gyro();
  private final Timer mtimer = new Timer();

  @Override
  public void robotInit() {
    xbox = new XboxController(0);
    xboxCoDriver = new XboxController(1);
    rightMotors.setInverted(true);
    myRobot = new DifferentialDrive(leftMotors, rightMotors);
    CameraServer.startAutomaticCapture();
    mgyro.calibrate();
    encoder.setDistancePerPulse(1./256.);
    encoder2.setDistancePerPulse(1./256.);


  }
  @Override
  public void autonomousInit() {
      // Set setpoint to current heading at start of auto
      heading = mgyro.getAngle();
      // This is where we will get the AprilTag
      stationID = 1;
      mtimer.reset();
      mtimer.start();
  }
  
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("right", encoder.getDistance());
   // SmartDashboard.putNumber("left", encoder2.getDistance());
   SmartDashboard.putNumber("left", mtimer.get());
   if(mtimer.get() < 1.0) {
    
  
      // Drives forward continuously at half speed, using the gyro to stabilize the heading
      myRobot.tankDrive(.5, .5 );
      
    }
    
    else if (mtimer.get() < 2) {
      if (stationID == 1 || stationID == 6){myRobot.tankDrive(0.6,0);}
    else if (stationID == 3 || stationID == 8){myRobot.tankDrive(0,0.6);}
    else if (stationID == 2 || stationID == 7){myRobot.tankDrive(-0.5,-0.5);}
    }
    else if (mtimer.get() < 3) {
      if (stationID == 2 || stationID == 7){myRobot.tankDrive(-0.5,-0.5);}
      else {myRobot.tankDrive(0.5,0.5);}
      
    }

        else{
          myRobot.tankDrive(0,0);
    }
    //SmartDashboard.putNumber("error", error);
      //SmartDashboard.putNumber("tankdriver", .5 + kP * error);
  }
  @Override
  public void teleopPeriodic() {
    
    //Uses the 2 joysticks on the xboxController to control the left and right side of the tank drive
    //myRobot.tankDrive(-xbox.getLeftY(), -xbox.getRightY());
    if(xbox.getLeftTriggerAxis()>0){
    myRobot.arcadeDrive(xbox.getLeftTriggerAxis(),xbox.getRightY()/2);
    }
    else{
      myRobot.arcadeDrive(-xbox.getRightTriggerAxis(),xbox.getRightY()/2);
    }
//when u press left trigger, it goes up depending how fast you hold it down
    if(xboxCoDriver.getLeftTriggerAxis()>0){

      }

      //when you press x, the vise grips in
    if (xboxCoDriver.getXButton()){
      if (limitSwitch.get()) {
        talMotorVise.set(0);
         //failsafe so the vise it won't keep going far away if limit switch is hit
    }else {
      talMotorVise.set(.25);
    
    }
  }
    //when you press b, the vise goes out
    if (xboxCoDriver.getBButton()){
      if (limitSwitch.get()) {
        talMotorVise.set(0);
         //failsafe so the vise it won't keep going far away if limit switch is hit
    }else {
      talMotorVise.set(-.25);
    }
    }
    if (xboxCoDriver.getYButton()) {
      m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
      m_doubleSolenoid2.set(DoubleSolenoid.Value.kForward);
    } else if (xboxCoDriver.getAButton()) {
      m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
      m_doubleSolenoid2.set(DoubleSolenoid.Value.kReverse);
    }

  }
  @Override
  public void testInit() {
      
  }
 
  @Override
  public void testPeriodic() {

  }
}
