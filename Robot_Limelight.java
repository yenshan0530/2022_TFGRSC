// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final WPI_TalonSRX m_leftDrive = new WPI_TalonSRX(0);
  private final WPI_TalonSRX m_rightDrive = new WPI_TalonSRX(3);
  private final WPI_TalonSRX m_leftDrive3 = new WPI_TalonSRX(15);

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
  private final Joystick m_stick = new Joystick(0);
  private final Timer m_timer = new Timer();
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightDrive.setInverted(true);
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
    /*if (m_timer.get() < 2.0) {
      m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }*/
    

  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    /*m_robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX());
    //read values periodically*/

    double right_command = 0.4d, left_command = 0.4d;
   
    double KpAim = -0.05d;
    double KpDistance = -0.1d;
    double min_aim_command = 0.05d;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);



    //if (m_stick.getRawButton(3))
    //{
      double sum1 = 0, sum2 = 0;  
      int cnt = 0;  
      while(++cnt <= 50) {  
              double heading_error = -x;
              double distance_error = -y;
              double steering_adjust = 0.0f;

              if (x > 1.0)
                  steering_adjust = KpAim * heading_error;// - min_aim_command;
              else if (x < -1.0)
                  steering_adjust = KpAim * heading_error;// + min_aim_command;

              double distance_adjust = KpDistance * distance_error;
              
              double tmp1 = 0.4 + steering_adjust;// + distance_adjust;
              if(tmp1 > 1) tmp1 = 1;
              if(tmp1 < -1) tmp1 = -1;
              double tmp2 = 0.4 - steering_adjust;// + distance_adjust;          
              if(tmp2 > 1) tmp2 = 1;
              if(tmp2 < -1) tmp2 = -1; 
              sum1 += tmp1;
              sum2 += tmp2;
      }
      right_command = sum1 / 50;
      left_command = sum2 / 50;
      SmartDashboard.putNumber("left_command", left_command);
      SmartDashboard.putNumber("right_command", right_command);
      m_robotDrive.tankDrive(right_command * 0.4, left_command * 0.4);
    //}
    }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    m_robotDrive.arcadeDrive(m_stick.getRawAxis(5) * 0.5, m_stick.getRawAxis(4) * 0.5); //y, x
  }


}
