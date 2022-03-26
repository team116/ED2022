// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final boolean IS_REAL_ROBOT = true;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  private static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE =
        IS_REAL_ROBOT ? PneumaticsModuleType.REVPH : PneumaticsModuleType.CTREPCM;

  private static final int SHOOTER_HAMMER_RELEASE_FORWARD_CHANNEL = 2;
  private static final int SHOOTER_HAMMER_RELEASE_REVERSE_CHANNEL = 10;
  private static final int INTAKE_RELEASE_FORWARD_CHANNEL = 1;
  private static final int INTAKE_RELEASE_REVERSE_CHANNEL = IS_REAL_ROBOT ? 9 : 7;
  private static final int FIRST_SPARE_CHANNEL = 0;
  private static final int SECOND_SPARE_CHANNEL = 8;
  private static final int LEFT_CLIMBER_RELEASE_FORWARD_CHANNEL = 4;
  private static final int LEFT_CLIMBER_RELEASE_REVERSE_CHANNEL = 12;
  private static final int RIGHT_CLIMBER_RELEASE_FORWARD_CHANNEL = 3;
  private static final int RIGHT_CLIMBER_RELEASE_REVERSE_CHANNEL = 11;

  SerialPort arduino;

  PneumaticHub pH;

  PowerDistribution PDP;

  Compressor compressor;

  PigeonIMU pigeon;
  WPI_TalonSRX backLeft;
  WPI_TalonSRX frontLeft;
  WPI_TalonSRX frontRight;
  WPI_TalonSRX backRight;

  WPI_TalonSRX intake;
  RobotSolenoid intakeRelease;

  WPI_TalonFX shooter;

  RobotSolenoid shooterHammer;

  WPI_TalonFX leftClimber;
  WPI_TalonFX rightClimber;

  RobotSolenoid leftClimberRelease;
  RobotSolenoid rightClimberRelease;

  WPI_TalonFX testFalcon;

  WPI_TalonFX shooterHoodAdjustment;

  RobotSolenoid[] diffSolenoids = new RobotSolenoid[4];

  PS4Controller logitech;
  XboxController xbox;
  Joystick joystick;

  POVButton joystickPOV_0;
  POVButton joystickPOV_45;
  POVButton joystickPOV_90;
  POVButton joystickPOV_135;
  POVButton joystickPOV_180;
  POVButton joystickPOV_225;
  POVButton joystickPOV_270;
  POVButton joystickPOV_315;
  JoystickButton joystickButton3;
  JoystickButton joystickButton4;
  JoystickButton joystickButton5;
  JoystickButton joystickButton6;
  JoystickButton joystickButton7;
  JoystickButton climberSafety1;
  JoystickButton climberSafety2;
  JoystickButton joystickButton9;
  JoystickButton joystickButton10;
  JoystickButton climberRelease;
  JoystickButton joystickButton11;
  JoystickButton joystickButton12;

  boolean climberIsReleased;
  boolean climberReleasePreviouslyPressed;

  boolean intakeIsReleased;
  boolean intakeReleasePreviouslyPressed;

  MecanumDrive driveTrain;

  NetworkTable limeLight;

  private int step = 0;

  double[] direction = {0.0, 0.0, 0.0};

  WPI_Pigeon2 pigeon2;

  AnalogPotentiometer analogPotentiometer;

  private final int RADIUS_OF_WHEEL = 3;
  private final double GEAR_RATIO = 10.71;
  private final int TICKS_PER_REVOLUTION = 2048;

  private final double TICKS_PER_INCH = (TICKS_PER_REVOLUTION * GEAR_RATIO) / (RADIUS_OF_WHEEL * 2 * Math.PI);

  // THIS WORKS FOR VELOCITY (FOR SOME REASON)
  private final double SHOOTER_kP = .1;
  private final double SHOOTER_kI = 0.001;
  private final double SHOOTER_kD = 0.0;

  // for Falcons
  private final double DRIVETRAIN_kP  = 1.0;
  private final double DRIVETRAIN_kI = 0.001;
  private final double DRIVETRAIN_kD = 1.0;

  // CAN bus IDs
  private final int LEFT_FRONT_ID = 1;
  private final int LEFT_REAR_ID = 2;
  private final int RIGHT_FRONT_ID = 3;
  private final int RIGHT_REAR_ID = 4;
  private final int INTAKE_ID = 5;
  private final int SHOOTER_ID = 6;
  private final int WINCH_RIGHT_ID = 7;
  private final int WINCH_FOLLOWER_ID = 8;
  private final int SHOOTER_HOOD_ID = 9;
  private final int PIGEON_ID = 12;
  private final int PDP_ID = 14;
  private final int PCM_ID = 15;


  Timer timer;

  enum Play {
    DRIVE_BACK_GET_TWO_BALLS,
    DO_NOTHING,
    DRIVE_BACK,
    SHOOT_ONE_BALL_AND_DRIVE_BACK,
    TEST
  }

  SendableChooser<Play> play = new SendableChooser<>();

  Play chosenPlay;

  public Robot() {
    super();
  }

  public Robot(double period) {
    super(period);
  }

  @Override
  public void robotInit() {
    // Ports based on CAN id, found through phoenix tuner and running the diagnostic server
    if (IS_REAL_ROBOT) {
      pH = new PneumaticHub(PCM_ID);
//
      compressor = new Compressor(PCM_ID, PneumaticsModuleType.REVPH);

      System.out.println("Has Enabled Analog Compressor");
      System.out.println(pH.getModuleNumber());
    }

    PDP = new PowerDistribution(PDP_ID, PowerDistribution.ModuleType.kRev );

    backLeft = new WPI_TalonSRX(LEFT_REAR_ID);
    frontLeft = new WPI_TalonSRX(LEFT_FRONT_ID);
    frontRight = new WPI_TalonSRX(RIGHT_FRONT_ID);
    backRight = new WPI_TalonSRX(RIGHT_REAR_ID);

    if (! IS_REAL_ROBOT) {
      backLeft.setExpiration(0.3);
      frontLeft.setExpiration(0.3);
      frontRight.setExpiration(0.3);
      backRight.setExpiration(0.3);
    }

    backLeft.setNeutralMode(NeutralMode.Brake);
    frontLeft.setNeutralMode(NeutralMode.Brake);
    backRight.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);

//    testFalcon = new WPI_TalonFX(16);
//    testFalcon.setNeutralMode(NeutralMode.Brake);

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

    shooter = new WPI_TalonFX(SHOOTER_ID);
    shooterHammer = new RobotSolenoid(SHOOTER_HAMMER_RELEASE_FORWARD_CHANNEL, SHOOTER_HAMMER_RELEASE_REVERSE_CHANNEL, pH, 0.3);
//    shooterHammer = pH.makeDoubleSolenoid(SHOOTER_HAMMER_RELEASE_FORWARD_CHANNEL, SHOOTER_HAMMER_RELEASE_REVERSE_CHANNEL);
//    shooterHammer = createDoubleSolenoid(SHOOTER_HAMMER_RELEASE_FORWARD_CHANNEL, SHOOTER_HAMMER_RELEASE_REVERSE_CHANNEL);

    intake = new WPI_TalonSRX(INTAKE_ID);
    intakeRelease = new RobotSolenoid(INTAKE_RELEASE_FORWARD_CHANNEL, INTAKE_RELEASE_REVERSE_CHANNEL, pH, 0.3);
//    intakeRelease = pH.makeDoubleSolenoid(INTAKE_RELEASE_FORWARD_CHANNEL, INTAKE_RELEASE_REVERSE_CHANNEL);
//    leftIntakeRelease = createDoubleSolenoid(LEFT_INTAKE_RELEASE_FORWARD_CHANNEL, LEFT_INTAKE_RELEASE_REVERSE_CHANNEL);
//    rightIntakeRelease = createDoubleSolenoid(RIGHT_INTAKE_RELEASE_FORWARD_CHANNEL, RIGHT_INTAKE_RELEASE_REVERSE_CHANNEL);

    leftClimber = new WPI_TalonFX(WINCH_FOLLOWER_ID);
    rightClimber = new WPI_TalonFX(WINCH_RIGHT_ID);

    leftClimber.setNeutralMode(NeutralMode.Brake);
    rightClimber.setNeutralMode(NeutralMode.Brake);

    if (IS_REAL_ROBOT) {
      leftClimberRelease = new RobotSolenoid(LEFT_CLIMBER_RELEASE_FORWARD_CHANNEL, LEFT_CLIMBER_RELEASE_REVERSE_CHANNEL, pH, 0.3);
      rightClimberRelease = new RobotSolenoid(RIGHT_CLIMBER_RELEASE_FORWARD_CHANNEL, RIGHT_CLIMBER_RELEASE_REVERSE_CHANNEL, pH, 0.3);
//      leftClimberRelease = pH.makeDoubleSolenoid(LEFT_CLIMBER_RELEASE_FORWARD_CHANNEL, LEFT_CLIMBER_RELEASE_REVERSE_CHANNEL);
//      rightClimberRelease = pH.makeDoubleSolenoid(RIGHT_CLIMBER_RELEASE_FORWARD_CHANNEL, RIGHT_CLIMBER_RELEASE_REVERSE_CHANNEL);
    }

    shooterHoodAdjustment = new WPI_TalonFX(SHOOTER_HOOD_ID);

    /*
          testFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      testFalcon.setSelectedSensorPosition(0.0);

      testFalcon.selectProfileSlot(0, 0);
      testFalcon.config_kP(0, SHOOTER_kP);
      testFalcon.config_kI(0, SHOOTER_kI);
      testFalcon.config_kD(0, SHOOTER_kD);
      testFalcon.configClosedLoopPeakOutput(0, 1.0);
      testFalcon.configPeakOutputForward(1.0);
     */
    shooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    shooter.setSelectedSensorPosition(0.0);
    shooter.setNeutralMode(NeutralMode.Coast);

    shooter.selectProfileSlot(0, 0);
    shooter.config_kP(0, SHOOTER_kP);
    shooter.config_kI(0, SHOOTER_kI);
    shooter.config_kD(0, SHOOTER_kD);
    shooter.configClosedLoopPeakOutput(0,1.0);

    shooter.set(TalonFXControlMode.Velocity, 0.0);

//    pigeon = new PigeonIMU(PIGEON_ID);
//    pigeon.configFactoryDefault();
//    pigeon.enterCalibrationMode(PigeonIMU.CalibrationMode.BootTareGyroAccel);

//    pigeon.getYawPitchRoll(direction);
    pigeon2 = new WPI_Pigeon2(PIGEON_ID);
    analogPotentiometer = new AnalogPotentiometer(0);
//    try {
//      arduino = new SerialPort(115200, SerialPort.Port.kUSB);
//    } catch (Exception exception){
//      System.out.println("I SCREWED UP 1");
//      try {
//        arduino = new SerialPort(115200, SerialPort.Port.kUSB2);
//      } catch (Exception exception1) {
//        System.out.println("I SCREWED UP 2");
//      }
//    }
//
    limeLight = NetworkTableInstance.getDefault().getTable("limelight-ed");

    play.setDefaultOption("Do Nothing", Play.DRIVE_BACK);
    play.addOption("Do Nothing", Play.DO_NOTHING);
    play.addOption("Drive Back", Play.DRIVE_BACK);
    play.addOption("Drive Back and Shoot 2 Balls", Play.DRIVE_BACK_GET_TWO_BALLS);
    play.addOption("Test", Play.TEST);
    SmartDashboard.putData(play);
    diffSolenoids[0] = rightClimberRelease;
    diffSolenoids[1] = leftClimberRelease;
    diffSolenoids[2] = shooterHammer;
    diffSolenoids[3] = intakeRelease;
  }

  @Override
  public void robotPeriodic() {


  }

  @Override
  public void autonomousInit() {

    frontLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    backLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    frontRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    backRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

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

    backLeft.config_kP(0, DRIVETRAIN_kP);
    frontLeft.config_kP(0, DRIVETRAIN_kP);
    backRight.config_kP(0, DRIVETRAIN_kP);
    frontRight.config_kP(0, DRIVETRAIN_kP);

    backLeft.config_kI(0, DRIVETRAIN_kI);
    frontLeft.config_kI(0, DRIVETRAIN_kI);
    backRight.config_kI(0, DRIVETRAIN_kI);
    frontRight.config_kI(0, DRIVETRAIN_kI);

    backLeft.config_kD(0, DRIVETRAIN_kD);
    frontLeft.config_kD(0, DRIVETRAIN_kD);
    backRight.config_kD(0, DRIVETRAIN_kD);
    frontRight.config_kD(0, DRIVETRAIN_kD);

    backLeft.configPeakOutputForward(0.2);
    frontLeft.configPeakOutputForward(0.2);
    backRight.configPeakOutputForward(0.2);
    frontRight.configPeakOutputForward(0.2);

    backLeft.set(ControlMode.Position, 0.0);
    frontLeft.set(ControlMode.Position, 0.0);
    backRight.set(ControlMode.Position, 0.0);
    frontRight.set(ControlMode.Position, 0.0);

    rightClimberRelease.set(DoubleSolenoid.Value.kReverse);
    leftClimberRelease.set(DoubleSolenoid.Value.kReverse);

    if (!IS_REAL_ROBOT) {
      testFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      testFalcon.setSelectedSensorPosition(0.0);

      testFalcon.selectProfileSlot(0, 0);
      testFalcon.config_kP(0, SHOOTER_kP);
      testFalcon.config_kI(0, SHOOTER_kI);
      testFalcon.config_kD(0, SHOOTER_kD);
      testFalcon.configClosedLoopPeakOutput(0, 1.0);
      testFalcon.configPeakOutputForward(1.0);
//    testFalcon.set(ControlMode.Position, TICKS_PER_REVOLUTION);
    }
    step = 0;
    pigeon2.reset();

    timer = new Timer();
    timer.start();
    chosenPlay = play.getSelected();
    pH.clearStickyFaults();
    PDP.clearStickyFaults();
  }

  @Override
  public void autonomousPeriodic() {
    switch (chosenPlay) {
      case DO_NOTHING:
        break;

      case DRIVE_BACK:

        switch (step) {
          case 0:

            step++;
            timer.reset();
            break;

          case 1:
//            if (timer.get() < 2.00) {
//              backRight.set(TalonSRXControlMode.PercentOutput, 0.2);
//              frontRight.set(TalonSRXControlMode.PercentOutput, 0.2);
//              backLeft.set(TalonSRXControlMode.PercentOutput, 0.2);
//              frontLeft.set(TalonSRXControlMode.PercentOutput, 0.2);
//            } else {
//              step++;
//            }

            runToInchesForTime(36, 5);
            break;
          case 2:
            backRight.set(TalonSRXControlMode.PercentOutput, 0.0);
            frontRight.set(TalonSRXControlMode.PercentOutput, 0.0);
            backLeft.set(TalonSRXControlMode.PercentOutput, 0.0);
            frontLeft.set(TalonSRXControlMode.PercentOutput, 0.0);
        }
        System.out.println(step);
        break;

      case DRIVE_BACK_GET_TWO_BALLS:

        switch (step) {
          case 0:

            intakeRelease.set(DoubleSolenoid.Value.kForward);
            step++;
            timer.reset();

            break;

          case 1:
            runToInchesForTime(36, 5.0);
            break;

          case 2:
            turnToDegreesAtSpeed(180, 0.2);
            break;

          case 3:
            shooter.set(TalonFXControlMode.Velocity, 5000);
            if (timer.get() > 5.0) {
              step++;
              timer.reset();
            }
            break;

          case 4:
            shooterHammer.set(DoubleSolenoid.Value.kForward);
            if (timer.get() > 1.0) {
              step++;
              shooterHammer.set(DoubleSolenoid.Value.kReverse);
              timer.reset();
            }
            break;

          case 5:
            shooterHammer.set(DoubleSolenoid.Value.kForward);
            if (timer.get() > 1.00) {
              step++;
              shooterHammer.set(DoubleSolenoid.Value.kReverse);
              timer.reset();
            }
            break;

          case 6:
            break;
          default:
            System.out.println("STOOOOOPID");
            break;
      }
      break;

      case SHOOT_ONE_BALL_AND_DRIVE_BACK:
        switch (step) {
          case 0:
            shooter.set(TalonFXControlMode.Velocity, 4000*2048/600);
            if (timer.get() > 6.0) {
              timer.reset();
              step++;
            }
            break;
          case 1:
            shooterHammer.set(DoubleSolenoid.Value.kForward);
            if (timer.get() > 1.0) {
             step++;
            }
            break;
          case 2:
            chosenPlay = Play.DRIVE_BACK;
            break;
        }
        break;

      case TEST:
        switch (step) {
          case 0:
//            testFalcon.set(TalonFXControlMode.Velocity, 6500 * 2048/600);
            System.out.println(findDistanceToHub(limeLight.getEntry("ty" ).getDouble(0)));
            //step++;
            break;
          case 1:
            break;
        }
        break;
    }
//    System.out.println(chosenPlay);
//    System.out.printf("Front Left: %f, Front Right: %f, Back Left: %f, Back Right: %f %n",
//            frontLeft.getSelectedSensorPosition(), frontRight.getSelectedSensorPosition(),backLeft.getSelectedSensorPosition(),backRight.getSelectedSensorPosition());
//    System.out.println("Pigeon: " + pigeon2.getAngle());
    System.out.println(testFalcon.getSelectedSensorVelocity()*600/2048);
//    System.out.println(testFalcon.getSelectedSensorPosition());
  }

  @Override
  public void teleopInit() {
    logitech = new PS4Controller(0);
    xbox = new XboxController(0);
    joystick = new Joystick(1);
    joystickPOV_0 = new POVButton(joystick, 0);
    joystickPOV_45 = new POVButton(joystick, 45);
    joystickPOV_90 = new POVButton(joystick, 90);
    joystickPOV_135 = new POVButton(joystick, 135);
    joystickPOV_180 = new POVButton(joystick, 180);
    joystickPOV_225 = new POVButton(joystick, 225);
    joystickPOV_270 = new POVButton(joystick, 270);
    joystickPOV_315 = new POVButton(joystick, 315);

    joystickButton3 = new JoystickButton(joystick, 3);
    joystickButton4 = new JoystickButton(joystick, 4);
    joystickButton5 = new JoystickButton(joystick, 5);
    joystickButton6 = new JoystickButton(joystick, 6);
    joystickButton7 = new JoystickButton(joystick, 7);
    joystickButton9 = new JoystickButton(joystick, 9);
    joystickButton10 = new JoystickButton(joystick, 10);
    joystickButton11 = new JoystickButton(joystick, 11);
    joystickButton12 = new JoystickButton(joystick, 12);

    climberSafety1 = new JoystickButton(joystick, 7);
    climberSafety2 = new JoystickButton(joystick, 8);
    climberRelease = new JoystickButton(joystick, 5);
    //leftClimber.follow(rightClimber);
    leftClimber.setInverted(true);
//    pigeon.enterCalibrationMode(PigeonIMU.CalibrationMode.BootTareGyroAccel);
//    pigeon.configFactoryDefault();
    pigeon2.reset();
    frontLeft.setSelectedSensorPosition(0.0);
    frontLeft.set(TalonSRXControlMode.PercentOutput, 0.0);
    backLeft.set(TalonSRXControlMode.PercentOutput, 0.0);
    frontRight.set(TalonSRXControlMode.PercentOutput, 0.0);
    backRight.set(TalonSRXControlMode.PercentOutput, 0.0);
    driveTrain = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
    driveTrain.setExpiration(0.3);
    limeLight.getEntry("pipeline").setNumber(0);
//    testFalcon.set(TalonFXControlMode.Velocity, 0);
//    rightClimberRelease.set(DoubleSolenoid.Value.kForward);
//    leftClimberRelease.set(DoubleSolenoid.Value.kForward);

    frontLeft.configFactoryDefault();
    backLeft.configFactoryDefault();
    frontRight.configFactoryDefault();
    backRight.configFactoryDefault();

    if (! IS_REAL_ROBOT) {
      backLeft.setExpiration(0.3);
      frontLeft.setExpiration(0.3);
      frontRight.setExpiration(0.3);
      backRight.setExpiration(0.3);
    }

    backLeft.setNeutralMode(NeutralMode.Brake);
    frontLeft.setNeutralMode(NeutralMode.Brake);
    backRight.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);

//    testFalcon = new WPI_TalonFX(16);
//    testFalcon.setNeutralMode(NeutralMode.Brake);

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
    pH.clearStickyFaults();
    PDP.clearStickyFaults();
  }

  @Override
  public void teleopPeriodic() {
    compressor.enableAnalog(100, 120);

    if (joystickButton11.get() /*&& rightIntakeRelease.get() == DoubleSolenoid.Value.kForward*/) {
//      shooter.set(ControlMode.Velocity, 6380*(2048/600)*(-(joystick.getThrottle()-1)/2));
      /***
       * CHECK TO SEE IF THE MATH FOR THE JOYSTICK THROTTLE INPUT IS CORRECT, I MAY BE WRONG HERE
       */
      shooter.set(ControlMode.PercentOutput, -(joystick.getThrottle()-1)/2);
    } else if (joystick.getTop() && joystick.getTriggerPressed()){
//      shooter.set(ControlMode.Velocity, findShooterVelocity(findDistanceToHub(limeLight.getEntry("ty").getDouble(0))));
    } else {
      shooter.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    if (joystick.getTriggerPressed()) {
      shooterHammer.set(DoubleSolenoid.Value.kReverse);

    } else {
      shooterHammer.set(DoubleSolenoid.Value.kForward);
    }

//    if (joystick.getTop()) {
//      driveTrain.driveCartesian(0, 0, -limeLight.getEntry("tx").getDouble(0)/60);
//    } else {
      driveTrain.driveCartesian(applyDeadBandAndShape(xbox.getLeftY()), applyDeadBandAndShape(-xbox.getLeftX()), applyDeadBandAndShape( -xbox.getRightX())/*, pigeon2.getAngle()*/);
//      driveTrain.driveCartesian(xbox.getLeftY(), (xbox.getLeftX()), ( -xbox.getRightX())/*, pigeon2.getAngle()*/);

   // }


    if (xbox.getBButton() || xbox.getYButton()) {
      intake.set(ControlMode.PercentOutput, 1.0);
    } else {
      intake.set(TalonSRXControlMode.PercentOutput, 0.0);
    }

    boolean intakeReleasedIsBeingPressed = xbox.getAButton() || xbox.getXButton();

    if (intakeReleasedIsBeingPressed && !intakeReleasePreviouslyPressed) {
      intakeIsReleased = !intakeIsReleased;
    }

    intakeReleasePreviouslyPressed = intakeReleasedIsBeingPressed;

    if (intakeIsReleased) {
//      leftIntakeRelease.set(DoubleSolenoid.Value.kOff);
//      rightIntakeRelease.set(DoubleSolenoid.Value.kOff);
      if (intakeRelease.get() != DoubleSolenoid.Value.kForward) {
        intakeRelease.set(DoubleSolenoid.Value.kForward);
      }
    } else {
//      leftIntakeRelease.set(DoubleSolenoid.Value.kOff);
//      rightIntakeRelease.set(DoubleSolenoid.Value.kOff);
      if (intakeRelease.get() != DoubleSolenoid.Value.kReverse) {
        intakeRelease.set(DoubleSolenoid.Value.kReverse);
      }
    }

    if (joystickPOV_0.get()) {
      rightClimber.set(TalonFXControlMode.PercentOutput, -0.5);
      leftClimber.set(TalonFXControlMode.PercentOutput, -0.5);
    } else if (joystickPOV_180.get() || joystickPOV_225.get() || joystickPOV_135.get()) {
      rightClimber.set(TalonFXControlMode.PercentOutput, 0.5);
      leftClimber.set(TalonFXControlMode.PercentOutput, 0.5);
    } else if (joystickButton5.get()) {
      leftClimber.set(TalonFXControlMode.PercentOutput, -0.5);
    } else if (joystickButton6.get()) {
      rightClimber.set(TalonFXControlMode.PercentOutput, -0.5);
    } else if (joystickButton3.get()) {
      leftClimber.set(TalonFXControlMode.PercentOutput, 0.5);
    } else if (joystickButton4.get()) {
      rightClimber.set(TalonFXControlMode.PercentOutput, 0.5);
    } else {
      rightClimber.set(TalonFXControlMode.PercentOutput, 0.0);
      leftClimber.set(TalonFXControlMode.PercentOutput, 0.0);

    }

    boolean climberReleasedIsBeingPressed = joystickButton7.get();

    if (climberReleasedIsBeingPressed && !climberReleasePreviouslyPressed) {
      climberIsReleased = !climberIsReleased;
    }

    climberReleasePreviouslyPressed = climberReleasedIsBeingPressed;

    if (IS_REAL_ROBOT) {
      if (climberIsReleased) {
//        rightClimberRelease.set(DoubleSolenoid.Value.kOff);
//        leftClimberRelease.set(DoubleSolenoid.Value.kOff);
        if (rightClimberRelease.get() != DoubleSolenoid.Value.kForward) {
          rightClimberRelease.set(DoubleSolenoid.Value.kForward);
          leftClimberRelease.set(DoubleSolenoid.Value.kForward);
        }
      } else {
//        rightClimberRelease.set(DoubleSolenoid.Value.kOff);
//        leftClimberRelease.set(DoubleSolenoid.Value.kOff);
        if (rightClimberRelease.get() != DoubleSolenoid.Value.kReverse) {
          rightClimberRelease.set(DoubleSolenoid.Value.kReverse);
          leftClimberRelease.set(DoubleSolenoid.Value.kReverse);
        }
      }
    }

    if (joystickButton9.get()) {
      shooterHoodAdjustment.set(TalonFXControlMode.PercentOutput, 0.25);
    } else if (joystickButton10.get()) {
      shooterHoodAdjustment.set(TalonFXControlMode.PercentOutput, -0.2);
    } else {
      shooterHoodAdjustment.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    if (xbox.getLeftBumper()) {
      killAllSolenoids();
    }
    for (RobotSolenoid solenoid: diffSolenoids){
      solenoid.checkTurnOff();
    }

//    System.out.println(shooter.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Shooter Speed", shooter.getSelectedSensorVelocity()*(600.0/2048.0));
    SmartDashboard.putNumber("Compressor Pressure", compressor.getPressure());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
//    compressor.disable();

    logitech = new PS4Controller(0);
    xbox = new XboxController(0);
    joystick = new Joystick(1);
    joystickPOV_0 = new POVButton(joystick, 0);
    joystickPOV_45 = new POVButton(joystick, 45);
    joystickPOV_90 = new POVButton(joystick, 90);
    joystickPOV_135 = new POVButton(joystick, 135);
    joystickPOV_180 = new POVButton(joystick, 180);
    joystickPOV_225 = new POVButton(joystick, 225);
    joystickPOV_270 = new POVButton(joystick, 270);
    joystickPOV_315 = new POVButton(joystick, 315);
  }

  @Override
  public void testPeriodic() {

//    if (joystickPOV_315.get() || joystickPOV_0.get() || joystickPOV_45.get()) {
//      System.out.printf("45: %b\n 315: %b\n0: %b\n", joystickPOV_45.get(), joystickPOV_315.get(), joystickPOV_0.get());
////      rightClimber.set(TalonFXControlMode.PercentOutput, -0.5);
//    } else if (joystickPOV_180.get() || joystickPOV_225.get() || joystickPOV_135.get()) {
//      System.out.printf("90: %b\n 225: %b\n270: %b\n", joystickPOV_90.get(), joystickPOV_225.get(), joystickPOV_270.get());
//
////      leftClimber.set(TalonFXControlMode.PercentOutput, 0.5);
//    } else {
//
//      rightClimber.set(TalonFXControlMode.PercentOutput, 0.0);
//      leftClimber.set(TalonFXControlMode.PercentOutput, 0.0);
//    }
    //System.out.println(arduino.readString());
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  private void killAllSolenoids() {
    shooterHammer.set(DoubleSolenoid.Value.kOff);
    intakeRelease.set(DoubleSolenoid.Value.kOff);

    if (IS_REAL_ROBOT) {
      rightClimberRelease.set(DoubleSolenoid.Value.kOff);
      leftClimberRelease.set(DoubleSolenoid.Value.kOff);
    }
  }

  private void runToInchesForTime(double inches, double time) {
    backLeft.set(ControlMode.Position, TICKS_PER_INCH * inches);
    frontLeft.set(ControlMode.Position, TICKS_PER_INCH * inches);
    backRight.set(ControlMode.Position, TICKS_PER_INCH * inches);
    frontRight.set(ControlMode.Position, TICKS_PER_INCH * inches);

    if (timer.get() >= time) {
      backLeft.set(ControlMode.PercentOutput, 0.0);
      frontLeft.set(ControlMode.PercentOutput, 0.0);
      backRight.set(ControlMode.PercentOutput, 0.0);
      frontRight.set(ControlMode.PercentOutput, 0.0);
      step++;
    }
  }

  private void turnToDegreesAtSpeed(double degrees, double speed) {
    if (Math.abs(pigeon2.getAngle() % 360) < degrees) {
      backLeft.set(ControlMode.PercentOutput, speed);
      frontLeft.set(ControlMode.PercentOutput, speed);
      backRight.set(ControlMode.PercentOutput, -speed);
      frontRight.set(ControlMode.PercentOutput, -speed);
    } else {
      backLeft.set(ControlMode.PercentOutput, 0.0);
      frontLeft.set(ControlMode.PercentOutput, 0.0);
      backRight.set(ControlMode.PercentOutput, 0.0);
      frontRight.set(ControlMode.PercentOutput, 0.0);
      step++;
    }
  }

  private static DoubleSolenoid createDoubleSolenoid(int forwardChannel, int reverseChannel) {
    if (IS_REAL_ROBOT) {
      return new DoubleSolenoid(PneumaticsModuleType.REVPH, forwardChannel, reverseChannel);
    }
    return new DoubleSolenoid(15, PneumaticsModuleType.CTREPCM, forwardChannel, reverseChannel);
  }

  private double findDistanceToHub(double limelightAngle) {
    return 67.5/Math.tan((40+limelightAngle)*(Math.PI/180));
  }

  private double findShooterVelocity(double distanceToHub) {
    // numberz, math, pain
    return Math.sqrt(((9.81*distanceToHub)/41.056)*((9.81*distanceToHub)/41.056) + 1685)/(4*Math.PI);
  }

  private double findShooterAngle(double distanceToHub) {
    return 60*Math.atan(1685/(9.8*distanceToHub));
  }

  private double applyDeadBandAndShape(double value) {

    if (value < 0) {
      return -(value * value);
    } else {
      return value * value;
    }

  }
}
