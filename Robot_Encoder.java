// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final TalonSRX tal_master = new TalonSRX(0);//phoenix tuner/lifeboat
  private final TalonSRX tal_follwe = new TalonSRX(3);//phoenix tuner/lifeboat
  private final TalonSRX tal = new TalonSRX(13);
  private final Joystick m_stick = new Joystick(0);//driver station

  double  kVelP = 10;
  double  kVelI = 0.0;
  double  kVelD = 0.003;
  private final PIDController VelController = new PIDController(kVelP, kVelI, kVelD);

  final boolean kDiscontinueity = true;
  final int kTimeoutMs = 30;
  //declare configuration time
  final int k_End_0 = 910;
  //start from 80 degree(80 / 360 *4096) 
  final int k_End_1 = 1137;
  //ent at 100 degree(100 / 360 *4096)

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    tal_master.configFactoryDefault();
    //default master motor
    tal_follwe.configFactoryDefault();
    //default follow motor
    tal_follwe.follow(tal_master);
    //let tal_follow to follow tal_master 
    tal_follwe.setInverted(true);
    //reverse tal_master rotation direction
    tal_master.setInverted(false);
    //reverse tal_ rotation direction
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {   
  }
 int mode = 2;
 //1 for relative encoder
 //2 for relative encoder with PIDController 
 //3 for absolute encoder
  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    if(mode == 1){
      tal_master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
      //set encoder mode as relative
      tal_master.setSelectedSensorPosition(0);
      //reset current position to zero
    }
    else if(mode == 2){
      tal_master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
      //set encoder mode as relative
      tal_master.setSelectedSensorPosition(0);
      //reset current position to zero
    }
    else if(mode == 3){
      tal_master.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,kTimeoutMs);
      //set encoder mode as absolute
      initQuadraturePos();
    }
  }


  public void initQuadraturePos(){
    int pulseWidth = tal_master.getSensorCollection().getPulseWidthPosition();
    //get absolute position
    if(kDiscontinueity){
      int newcenter;
      newcenter = (k_End_0 + k_End_1) /2;
      //calculate new centor
      newcenter = newcenter % 4096;
      //make sure position of newcenter doesn't exceed 4095
      pulseWidth = pulseWidth - newcenter;
      //caculate new position for new center
    }
    pulseWidth = pulseWidth % 4096;
    //make sure new position doesn't exceed 4095
    tal_master.getSensorCollection().setQuadraturePosition(pulseWidth,kTimeoutMs);
    //set new center
  }


  private int ToDeg(double units){
    double deg = units * 360.0 / 4096.0;
    int ideg = (int) deg;
    return ideg;
  }


  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
        if(mode == 1){
        double pos = tal_master.getSelectedSensorPosition();
        double vel = tal_master.getSelectedSensorVelocity();
        if(m_stick.getRawButton(1)){
          tal_master.set(ControlMode.PercentOutput,0.5);
        }
        if(m_stick.getRawButton(2)){
          tal_master.set(ControlMode.PercentOutput,0 );
        }
      SmartDashboard.putNumber("pos",pos);
      SmartDashboard.putNumber("vel",vel);
      }
    else if(mode == 2){
      double vel = tal_master.getSelectedSensorVelocity();
      double targetvel = 1200;
      if(m_stick.getRawButton(1)){
        //if(vel < targetvel){
          tal_master.set(ControlMode.PercentOutput,(VelController.calculate(vel,targetvel))/4096/20);
          SmartDashboard.putNumber("vel",vel);
          SmartDashboard.putNumber("test",VelController.calculate(vel,targetvel));
        //}
      }
      if(m_stick.getRawButton(2)){
        tal_master.set(ControlMode.PercentOutput,0);
      }
    }
    else if(mode == 3){
      if(m_stick.getRawButton(1)){
        initQuadraturePos();
      }
      if(Math.abs(m_stick.getRawAxis(1))>0.1){
        tal_master.set(ControlMode.PercentOutput,m_stick.getRawAxis(1) * 0.3);
      }
      int pulseWidthWithoutOverflow = tal_master.getSensorCollection().getPulseWidthPosition();
      pulseWidthWithoutOverflow = pulseWidthWithoutOverflow % 4096;
      SmartDashboard.putNumber("pulseWidthWithoutOverflow",pulseWidthWithoutOverflow);
      SmartDashboard.putNumber("pulseWidthWithoutOverflowDeg",ToDeg(pulseWidthWithoutOverflow));
    }
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
