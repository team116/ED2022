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

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

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
  PigeonIMU pigeon;
  WPI_TalonFX backLeft;
  WPI_TalonFX frontLeft;
  WPI_TalonFX rightFront;

  // both work, figure out difference next
  TalonFX backRight;
  double[] direction = {0.0, 0.0, 0.0};

  @Override
  public void robotInit() {
    // Ports based on CAN id, found through phoenix tuner and running the diagnostic server

    backLeft = new WPI_TalonFX(2);
    frontLeft = new WPI_TalonFX(1);
    rightFront = new WPI_TalonFX(3);
    backRight = new TalonFX(4);
    pigeon = new PigeonIMU(12);
    }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {
    rightFront.set(ControlMode.PercentOutput, 0.2);
    backLeft.set(ControlMode.PercentOutput,0.2);
    frontLeft.set(ControlMode.PercentOutput,0.2);
    backRight.set(ControlMode.PercentOutput, 0.2);
    pigeon.getYawPitchRoll(direction);
    // doesn't seem to print anywhere. Should investigate
    System.out.println("yaw " + direction[0]);
    System.out.println("pitch " + direction[1]);
    System.out.println("roll " + direction[2]);
  }

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
}
