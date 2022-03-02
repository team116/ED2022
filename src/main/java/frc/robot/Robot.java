// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.naming.ldap.Control;


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
  WPI_TalonSRX backLeft;
  WPI_TalonSRX frontLeft;
  WPI_TalonSRX frontRight;
  WPI_TalonSRX backRight;

  WPI_TalonSRX intake;
  DoubleSolenoid intakeRelease;

  WPI_TalonFX shooter;

  DoubleSolenoid shooterHammer;

  WPI_TalonFX leftClimber;
  WPI_TalonFX rightClimber;

  WPI_TalonFX testFalcon;

  PIDController pidController;

  PS4Controller logitech;
  XboxController xbox;
  MecanumDrive driveTrain;

  NetworkTable limeLight;

  double[] direction = {0.0, 0.0, 0.0};

  WPI_Pigeon2 pigeon2;

  private final int RADIUS_OF_WHEEL = 3;
  private final double GEAR_RATIO = 10.71;
  private final int TICKS_PER_REVOLUTION = 2048;

  private final double TICKS_PER_INCH = (TICKS_PER_REVOLUTION * GEAR_RATIO) / (RADIUS_OF_WHEEL * 2 * Math.PI);

  // THIS WORKS FOR VELOCITY (FOR SOME REASON)
  private final double kP = .01; //0.1
  private final double kI = 0.001; //0.3
  private final double kD = 1.0; // 7.0

  public Robot() {
    super();
  }

  public Robot(double period) {
    super(period);
  }

  @Override
  public void robotInit() {
    // Ports based on CAN id, found through phoenix tuner and running the diagnostic server

    backLeft = new WPI_TalonSRX(2);
    frontLeft = new WPI_TalonSRX(1);
    frontRight = new WPI_TalonSRX(3);
    backRight = new WPI_TalonSRX(4);

    backLeft.setNeutralMode(NeutralMode.Brake);
    frontLeft.setNeutralMode(NeutralMode.Brake);
    backRight.setNeutralMode(NeutralMode.Brake);
    backRight.setNeutralMode(NeutralMode.Brake);

    testFalcon = new WPI_TalonFX(16);
    testFalcon.setNeutralMode(NeutralMode.Brake);

    frontLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    backLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    frontRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    backRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    frontLeft.setSensorPhase(true);
    backLeft.setSensorPhase(true);
    frontRight.setSensorPhase(true);
    backRight.setSensorPhase(true);

    backLeft.setInverted(true);
    frontLeft.setInverted(true);


    /***
    CHANGE LOCATIONS
     */
    shooter = new WPI_TalonFX(5);

    pigeon = new PigeonIMU(12);
    pigeon.configFactoryDefault();
    pigeon.enterCalibrationMode(PigeonIMU.CalibrationMode.BootTareGyroAccel);
    //driveTrain = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);

    pigeon.getYawPitchRoll(direction);
    pigeon2 = new WPI_Pigeon2(12);
    try {
      arduino = new SerialPort(115200, SerialPort.Port.kUSB);
    } catch (Exception exception){
      System.out.println("I SCREWED UP 1");
      try {
        arduino = new SerialPort(115200, SerialPort.Port.kUSB2);
      } catch (Exception exception1) {
        System.out.println("I SCREWED UP 2");
      }
    }

    //shooterHammer = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1 );

    shooter.set(ControlMode.Velocity, 0.0);

    limeLight = NetworkTableInstance.getDefault().getTable("limelight");
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {

    frontLeft.setSensorPhase(true);
    backLeft.setSensorPhase(true);
    frontRight.setSensorPhase(true);
    backRight.setSensorPhase(true);
    frontLeft.setSelectedSensorPosition(0.0);
    backLeft.setSelectedSensorPosition(0.0);
    frontRight.setSelectedSensorPosition(0.0);
    backRight.setSelectedSensorPosition(0.0);
//    frontRight.set(ControlMode.Position, 1000);
//    backLeft.set(ControlMode.Position, 1000);
//    frontLeft.set(ControlMode.Position, 1000);
//    backRight.set(ControlMode.Position, 1000);
//    frontLeft.set(ControlMode.PercentOutput, 0.0);
//    backLeft.set(ControlMode.PercentOutput, 0.0);
//    frontRight.set(ControlMode.PercentOutput, 0.0);
//    backRight.set(ControlMode.PercentOutput, 0.0);
    backLeft.selectProfileSlot(0,0);
    frontLeft.selectProfileSlot(0,0);
    backRight.selectProfileSlot(0,0);
    frontRight.selectProfileSlot(0,0);

    backLeft.config_kP(0, kP);
    frontLeft.config_kP(0, kP);
    backRight.config_kP(0, kP);
    frontRight.config_kP(0, kP);

    testFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    testFalcon.setSelectedSensorPosition(0.0);

    testFalcon.selectProfileSlot(0, 0);
    testFalcon.config_kP(0, kP);
    testFalcon.config_kI(0, kI);
    testFalcon.config_kD(0, kD);
    testFalcon.configClosedLoopPeakOutput(0,1.0);
    testFalcon.configPeakOutputForward(0.6);
    //testFalcon.set(ControlMode.Position, TICKS_PER_REVOLUTION);
    testFalcon.set(ControlMode.Velocity, 4000);
  }

  @Override
  public void autonomousPeriodic() {
//    frontRight.set(ControlMode.MotionMagic, 1000, DemandType.AuxPID, 0.2);
//    backLeft.set(ControlMode.MotionMagic, 1000, DemandType.AuxPID, 0.2);
//    frontLeft.set(ControlMode.MotionMagic, 1000, DemandType.AuxPID, 0.2);
//    backRight.set(ControlMode.MotionMagic, 1000, DemandType.AuxPID, 0.2);

//    frontRight.set(ControlMode.MotionMagic, 1000);
//    backLeft.set(ControlMode.MotionMagic, 1000);
//    frontLeft.set(ControlMode.MotionMagic, 1000);
//    backRight.set(ControlMode.MotionMagic, 1000);
//    driveTrain.driveCartesian(0.2, 0.0, 0.0);
    System.out.println(testFalcon.getSelectedSensorVelocity());

  }

  @Override
  public void teleopInit() {
    logitech = new PS4Controller(0);
    xbox = new XboxController(0);
    pigeon.enterCalibrationMode(PigeonIMU.CalibrationMode.BootTareGyroAccel);
    pigeon.configFactoryDefault();
    pigeon2.reset();
    frontLeft.setSelectedSensorPosition(0.0);
  }

  @Override
  public void teleopPeriodic() {
    pigeon.getYawPitchRoll(direction);


    System.out.println("yaw " + pigeon2.getAngle());
    System.out.printf("Front Left: %f, Front Right: %f, Back Left: %f, Back Right: %f %n",
            frontLeft.getSelectedSensorPosition(), frontRight.getSelectedSensorPosition(),backLeft.getSelectedSensorPosition(),backRight.getSelectedSensorPosition());

    if (xbox.getAButtonPressed()) {
      frontLeft.setSelectedSensorPosition(0.0);
      backLeft.setSelectedSensorPosition(0.0);
      frontRight.setSelectedSensorPosition(0.0);
      backRight.setSelectedSensorPosition(0.0);
    }
    driveTrain.driveCartesian(-xbox.getLeftY(), xbox.getLeftX(), -xbox.getRightX(), pigeon2.getAngle());

    if (xbox.getBButton()) {
      shooter.set(ControlMode.Velocity, 5000);
    }
    if (xbox.getBButton()) {
      shooterHammer.set(DoubleSolenoid.Value.kForward);
    } else {
      shooterHammer.set(DoubleSolenoid.Value.kReverse);
    }

    if (xbox.getBButton()) {
      if (limeLight.getEntry("tx").getDouble(0) < 0) {
        driveTrain.driveCartesian(0, 0, -0.2);
      } else {
        driveTrain.driveCartesian(0, 0, 0.1);
      }
    }

    if (xbox.getXButton()) {
      killAllSolenoids();
    }




//    if (logitech.getR1Button()) {
//      driveTrain.driveCartesian(0, 0.4, logitech.getRightY(), direction[0]);
//    } else if (logitech.getL1Button()) {
//      driveTrain.driveCartesian(0, -0.4, logitech.getRightY(), direction[0]);
//
//    } else {
//      driveTrain.driveCartesian(0, 0.0, logitech.getRightY(), direction[0]);
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

  public void killAllSolenoids() {
    shooterHammer.set(DoubleSolenoid.Value.kOff);
  }
}
