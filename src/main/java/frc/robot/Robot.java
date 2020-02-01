/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final WPI_TalonSRX motorZero = new WPI_TalonSRX(0);
  private final WPI_TalonSRX motorOne = new WPI_TalonSRX(1);
  private final WPI_TalonSRX motorTwo = new WPI_TalonSRX(2);
  private final WPI_TalonSRX motorThree = new WPI_TalonSRX(3);
  private final XboxController controller = new XboxController(0);

  private final DoubleSolenoid solenoid = new DoubleSolenoid(0, 1);
  private final Compressor compressor = new Compressor(0);
  private final Value pneumaticOut = Value.kForward;
  private final Value pneumaticIn = Value.kReverse;

  private final DigitalInput irSensor = new DigitalInput(0);

  private final SimpleMotorFeedforward motorFeedForward = new SimpleMotorFeedforward(.644, .0901, .0926);

  private int targetSpeed = 60000;
  private int targetSpeedRPM = (targetSpeed * 600) / 8192;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    System.out.println("Target RPM: " + targetSpeedRPM);
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
    talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
    talonConfig.neutralDeadband = 0.001;
    talonConfig.slot0.kF = 0.0;
    talonConfig.slot0.kP = .2;
    talonConfig.slot0.kI = 0.0;
    talonConfig.slot0.kD = 14.1;
    talonConfig.slot0.integralZone = 400;
    talonConfig.slot0.closedLoopPeakOutput = 1.0;
    talonConfig.closedloopRamp = 0.1;
    talonConfig.openloopRamp = 0;
    
    motorZero.configAllSettings(talonConfig);
    motorZero.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    motorZero.setSensorPhase(true);

    motorTwo.configOpenloopRamp(0);

    motorOne.setInverted(InvertType.OpposeMaster);
    motorOne.follow(motorZero);

    solenoid.set(pneumaticIn);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putBoolean("IR Sensor", !irSensor.get());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    int rpm = 940;
    int speed = (rpm * 8192) / (60 * 10);
    motorZero.set(ControlMode.Velocity, speed);
    SmartDashboard.putNumber("Ticks Per Decisec", motorZero.getSelectedSensorVelocity());
    SmartDashboard.putNumber("RPM", (motorZero.getSelectedSensorVelocity() * 60 * 10) / (4096));
  }

  @Override
  public void teleopInit() {
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    if (controller.getAButton()) {
      motorZero.set(ControlMode.Velocity, targetSpeed, DemandType.ArbitraryFeedForward, motorFeedForward.calculate(targetSpeedRPM / 60) / 12);
    } else {
      motorZero.set(0);
    }
    // motorZero.set(-controller.getY(Hand.kLeft));
    if (controller.getTriggerAxis(Hand.kRight) > controller.getTriggerAxis(Hand.kLeft)) {
      motorTwo.set(controller.getTriggerAxis(Hand.kRight ) * .75);
      controller.setRumble(RumbleType.kLeftRumble, controller.getTriggerAxis(Hand.kRight));
    } else if (controller.getTriggerAxis(Hand.kLeft) > controller.getTriggerAxis(Hand.kRight)) {
      motorTwo.set(-controller.getTriggerAxis(Hand.kLeft) * .75);
      controller.setRumble(RumbleType.kLeftRumble, controller.getTriggerAxis(Hand.kLeft));
    } else {
      motorTwo.set(0);
      controller.setRumble(RumbleType.kLeftRumble, 0);
    }
    SmartDashboard.putNumber("Ticks Per Decisec", motorZero.getSelectedSensorVelocity());
    SmartDashboard.putNumber("RPM", (motorZero.getSelectedSensorVelocity() * 60l  * 10l) / (8192l));
    if (controller.getBumper(Hand.kLeft)) {
      solenoid.set(pneumaticIn);
    } else if (controller.getBumper(Hand.kRight)) {
      solenoid.set(pneumaticOut);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
