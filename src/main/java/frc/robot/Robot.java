/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
    TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
    talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    talonConfig.neutralDeadband = 0.001;
    talonConfig.slot0.kF = 1023.0 / 23500.0;
    talonConfig.slot0.kP = 1.0;
    talonConfig.slot0.kI = 0.0;
    talonConfig.slot0.kD = 0.0;
    talonConfig.slot0.integralZone = 400;
    talonConfig.slot0.closedLoopPeakOutput = 1.0;
    talonConfig.closedloopRamp = .2;
    talonConfig.openloopRamp = 2;
    
    motorZero.configAllSettings(talonConfig);
    motorZero.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    motorZero.setSensorPhase(true);

    motorTwo.configOpenloopRamp(0);

    motorOne.setInverted(InvertType.OpposeMaster);
    motorOne.follow(motorZero);
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
    int speed = (rpm * 66 * 4096) / (18 * 60 * 10);
    motorZero.set(ControlMode.Velocity, speed);
    SmartDashboard.putNumber("Ticks Per Decisec", motorZero.getSelectedSensorVelocity());
    SmartDashboard.putNumber("RPM", (motorZero.getSelectedSensorVelocity() * 60 * 18 * 10) / (4096 * 66));
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    int rpm = 3975;
    int speed = (rpm * 66 * 4096) / (18 * 60 * 10 * 4);
    if (controller.getAButton()) {
      motorZero.set(ControlMode.Velocity, speed);
    } else  {
      motorZero.set(0);
    }
    if (controller.getTriggerAxis(Hand.kRight) > controller.getTriggerAxis(Hand.kLeft)) {
      motorTwo.set(controller.getTriggerAxis(Hand.kRight ) * 1);
      controller.setRumble(RumbleType.kLeftRumble, controller.getTriggerAxis(Hand.kRight));
    } else if (controller.getTriggerAxis(Hand.kLeft) > controller.getTriggerAxis(Hand.kRight)) {
      motorTwo.set(-controller.getTriggerAxis(Hand.kLeft) * 1);
      controller.setRumble(RumbleType.kLeftRumble, controller.getTriggerAxis(Hand.kLeft));
    } else {
      motorTwo.set(0);
      controller.setRumble(RumbleType.kLeftRumble, 0);
    }
    SmartDashboard.putNumber("Ticks Per Decisec", motorZero.getSelectedSensorVelocity());
    SmartDashboard.putNumber("RPM", (motorZero.getSelectedSensorVelocity() * 60 * 18 * 10 * 4) / (4096 * 66));
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
