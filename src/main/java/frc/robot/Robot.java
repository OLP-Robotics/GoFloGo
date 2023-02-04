// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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
  
  @Override
  public void robotInit() {
    xbox = new XboxController(0);
    myRobot = new DifferentialDrive(leftMotors, rightMotors);
    CameraServer.startAutomaticCapture();


  }

  @Override
  public void teleopPeriodic() {
    //Uses the 2 joysticks on the xboxController to control the left and right side of the tank drive
    myRobot.tankDrive(-xbox.getLeftY(), -xbox.getRightY());

  }
}
