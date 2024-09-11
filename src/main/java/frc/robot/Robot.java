// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.intake_shoot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  //private intake_shoot shoot = new intake_shoot();
  Thread m_visionThread;

     VictorSPX leftshooter = new VictorSPX(29);
     VictorSPX rightshooter = new VictorSPX(28);
     VictorSPX leftintake = new VictorSPX(24); // 23
     VictorSPX rightintake = new VictorSPX(23); // 24

    double pretime = Timer.getFPGATimestamp();
    double nowtime = Timer.getFPGATimestamp();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    //get usbcamera from CameraServer
     UsbCamera camera = CameraServer.startAutomaticCapture(0);
     //set the resolutio
     camera.setResolution(640, 480);
     camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
     //get a CVSink to capture mats from the camera
     CvSink cvSink = CameraServer.getVideo();
     //setup a CVSource to send images to Dashboard
     CvSource outputStream = CameraServer.putVideo("rectangle", 640, 480);
    }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override                                                                                                                    
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    // m_robotContainer.setIntakecmd();
    


  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    if(nowtime - pretime < 2.0){
        rightshooter.set(VictorSPXControlMode.PercentOutput, -1);
        leftshooter.set(VictorSPXControlMode.PercentOutput, 1);
        rightintake.set(VictorSPXControlMode.PercentOutput, 0);
        leftintake.set(VictorSPXControlMode.PercentOutput, 0);
        nowtime = Timer.getFPGATimestamp();
    }
    if (nowtime - pretime > 1.5 && nowtime - pretime < 3.0) {
        rightshooter.set(VictorSPXControlMode.PercentOutput, -1);
        leftshooter.set(VictorSPXControlMode.PercentOutput, 1);
        rightintake.set(VictorSPXControlMode.PercentOutput, -1);
        leftintake.set(VictorSPXControlMode.PercentOutput, 1);
        nowtime = Timer.getFPGATimestamp();
    }
    if (nowtime - pretime > 3.0) {
        rightshooter.set(VictorSPXControlMode.PercentOutput, 0);
        leftshooter.set(VictorSPXControlMode.PercentOutput, 0);
        rightintake.set(VictorSPXControlMode.PercentOutput, 0);
        leftintake.set(VictorSPXControlMode.PercentOutput, 0);
        nowtime = Timer.getFPGATimestamp();
    }
  }   

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    
  }
}
