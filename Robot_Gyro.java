// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final WPI_TalonSRX m_leftDrive = new WPI_TalonSRX(0);
  private final WPI_TalonSRX m_rightDrive = new WPI_TalonSRX(1);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
  private final Joystick m_stick = new Joystick(0);
  private final Timer m_timer = new Timer();
  AHRS ahrs;

  double kp = 0.05;
  double ki = 0;
  double kd = 0;
  private final PIDController PID = new PIDController(kp, ki, kd);


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftDrive.configFactoryDefault();
    m_rightDrive.configFactoryDefault();//初始化
    m_rightDrive.setInverted(true);
    ahrs = new AHRS(SerialPort.Port.kMXP);
    ahrs.enableLogging(true);
    ahrs.calibrate();
    ahrs.reset();
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
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      m_robotDrive.arcadeDrive(0.2, 0); 
    } else {
      m_robotDrive.stopMotor();
    // m_robotDrive.stopMotor(); // stop robot
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber( "IMU_YAW", ahrs.getYaw());
    SmartDashboard.putNumber( "IMU_Pitch", ahrs.getPitch());
    SmartDashboard.putNumber( "IMU_Roll", ahrs.getRoll());
    SmartDashboard.putNumber( "IMU_TotalYaw", ahrs.getAngle());
    SmartDashboard.putNumber( "Velocity_X", ahrs.getVelocityX());
    SmartDashboard.putNumber( "IMU_Accel_X", ahrs.getWorldLinearAccelX());
    
    //m_robotDrive.tankDrive(m_stick.getY(), m_stick.getX());
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
