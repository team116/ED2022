// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
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

  SerialPort arduino;

  PigeonIMU pigeon;
  WPI_TalonFX backLeft;
  WPI_TalonFX frontLeft;
  WPI_TalonFX frontRight;

  PS4Controller logitech;
  XboxController xbox;
  MecanumDrive driveTrain;
  // both work, figure out difference next
  WPI_TalonFX backRight;
  double[] direction = {0.0, 0.0, 0.0};

  WPI_Pigeon2 pigeon2;

  public Robot() {
    super();
  }

  public Robot(double period) {
    super(period);
  }

  @Override
  public void robotInit() {
    // Ports based on CAN id, found through phoenix tuner and running the diagnostic server

    backLeft = new WPI_TalonFX(2);
    frontLeft = new WPI_TalonFX(1);
    frontRight = new WPI_TalonFX(3);
    backRight = new WPI_TalonFX(4);

    backLeft.setInverted(true);
    frontLeft.setInverted(true);
    pigeon = new PigeonIMU(12);
    pigeon.configFactoryDefault();
    pigeon.enterCalibrationMode(PigeonIMU.CalibrationMode.BootTareGyroAccel);
    driveTrain = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);

    pigeon.getYawPitchRoll(direction);
    pigeon2 = new WPI_Pigeon2(12);
    try {
      arduino = new SerialPort(115200, SerialPort.Port.kUSB);
    } catch (Exception exception){
      System.out.println("I SCREWED UP 1");
      try {
        arduino = new SerialPort(115200, SerialPort.Port.kUSB2);
      } catch (Exception exception1){
        System.out.println("I SCREWED UP 2");
      }
    }
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {
    frontRight.set(ControlMode.PercentOutput, -0.2);
    backLeft.set(ControlMode.PercentOutput,0.2);
    frontLeft.set(ControlMode.PercentOutput,-0.2);
    backRight.set(ControlMode.PercentOutput, 0.2);
    pigeon.getYawPitchRoll(direction);
    // prints to the console of the driver station (under the settings cog wheel on the right)
    System.out.println("yaw " + direction[0]);
    System.out.println("pitch " + direction[1]);
    System.out.println("roll " + direction[2]);
    // this works
    SmartDashboard.putNumber("yaw", direction[0]);
    SmartDashboard.putNumber("pitch", direction[1]);
    SmartDashboard.putNumber("roll", direction[2]);
  }

  @Override
  public void teleopInit() {
    logitech = new PS4Controller(0);
    xbox = new XboxController(0);
    pigeon.enterCalibrationMode(PigeonIMU.CalibrationMode.BootTareGyroAccel);
    pigeon.configFactoryDefault();
    pigeon2.reset();
  }

  @Override
  public void teleopPeriodic() {
    pigeon.getYawPitchRoll(direction);


    System.out.println("yaw " + pigeon2.getAngle());
    System.out.println(xbox.getRightX());


    driveTrain.driveCartesian(-xbox.getLeftY(), xbox.getLeftX(), -xbox.getRightX(), pigeon2.getAngle());
//    if (logitech.getR1Button()) {
//      driveTrain.driveCartesian(0, 0.4, logitech.getRightY(), direction[0]);
//    } else if (logitech.getL1Button()) {
//      driveTrain.driveCartesian(0, -0.4, logitech.getRightY(), direction[0]);
//
//    } else {
//      driveTrain.driveCartesian(0, 0.0, logitech.getRightY(), direction[0]);
//
//    }
//    frontLeft.set(logitech.getLeftX());
//    frontRight.set(logitech.getLeftY());
//    backLeft.set(logitech.getRightX());
//    backRight.set(logitech.getRightY());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {
    System.out.println(arduino.readString());
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
