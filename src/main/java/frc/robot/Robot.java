// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  //private final PWMSparkMax m_leftDrive = new PWMSparkMax(0);
  //private final PWMSparkMax m_rightDrive = new PWMSparkMax(1);
  //private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
  private final XboxController m_controller = new XboxController(0);
  private final Timer m_timer = new Timer();
  private final TalonSRX talonMotorL = new TalonSRX(2);
  private final TalonSRX talonMotorR = new TalonSRX(3);
  private final double kP = 0.05;
  DigitalInput toplimitSwitch = new DigitalInput(6);
  private GenericEntry m_maxSpeed;
  ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    //m_rightDrive.setInverted(true);
    CameraServer.startAutomaticCapture();
    System.out.println("Hi World!");
    Shuffleboard.getTab("Gyro").add(gyro); 
    // Add a 'max speed' widget to a tab named 'Configuration', using a number slider
    // The widget will be placed in the second column and row and will be TWO columns wide
    m_maxSpeed =
        Shuffleboard.getTab("Configuration")
            .add("Max Speed", 1)
            .withWidget("Number Slider")
            .withPosition(1, 1)
            .withSize(2, 1)
            .getEntry();

            // Add the tank drive and encoders to a 'Drivebase' tab
    ShuffleboardTab driveBaseTab = Shuffleboard.getTab("Drivebase");
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double error = 90 - gyro.getAngle();
    //talonMotorL.set(TalonSRXControlMode.PercentOutput, kP-error);
      //talonMotorR.set(TalonSRXControlMode.PercentOutput, kP*-1+error);
    
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      talonMotorL.set(TalonSRXControlMode.PercentOutput, .5);
      talonMotorR.set(TalonSRXControlMode.PercentOutput,-.5);
  
      // Drive forwards half speed, make sure to turn input squaring off
      //m_robotDrive.arcadeDrive(0.5, 0.0, false);
    } else {
      talonMotorL.set(TalonSRXControlMode.PercentOutput, 0);
      talonMotorR.set(TalonSRXControlMode.PercentOutput, 0);
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    double speed = Math.abs(m_controller.getLeftY());
    SmartDashboard.putNumber("speed before",speed);
    if(speed<0.05){
      speed = 0;
    }

    speed = Math.sqrt(speed);
    if(m_controller.getLeftY()<0){
      speed*=-1;
    }
  
    SmartDashboard.putNumber("speed",speed);
   if(toplimitSwitch.get()){
    //if limit is tripped we stop
   talonMotorL.set(TalonSRXControlMode.PercentOutput, 0);
   talonMotorR.set(TalonSRXControlMode.PercentOutput, 0);
  }else{
// if limit is not tripped so go at commanded speed
talonMotorL.set(TalonSRXControlMode.PercentOutput, speed);
talonMotorR.set(TalonSRXControlMode.PercentOutput, speed *-1);
  }

}

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
