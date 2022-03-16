// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.AutoStates.STATE;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;

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

  private Drivetrain m_drivetrain = new Drivetrain();
  // private Shooter m_shooter = new Shooter(); //UNCOMMENT FOR SHOOTER CONTROL
  private AutoStates m_autoStates = new AutoStates(m_drivetrain);

  @Log(name = "Autonomous")
  private SendableChooser<AutoStates.STATE> m_chooser = new SendableChooser<>();

  @Override
  public void robotInit() { // necessary to be initialized before logger
    Logger.configureLoggingAndConfig(this, false);

    // set chosen auton routines here
    // ensure that corresponds to a valid INIT state
    m_chooser.setDefaultOption("Go Straight", AutoStates.STATE.GO_STRAIGHT);
    m_chooser.addOption("Shooting Auton Middle", AutoStates.STATE.TRAJ_GRABBALL_INIT);

    LiveWindow.disableAllTelemetry(); //disable livewindow telemetry, causes robot lag
  }

  @Override
  public void robotPeriodic() {
    Logger.updateEntries();
    m_drivetrain.periodic();
    m_autoStates.runStateMachine();
    System.out.println(AutoStates.getState().name());
  }

  @Override
  public void autonomousInit() {
    AutoStates.autonomousInit = true;
    AutoStates.setState(m_chooser.getSelected());
  }

  @Override
  public void autonomousPeriodic() {
    AutoStates.autonomousPeriodic = true;
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    m_drivetrain.drive();
    // m_shooter.shooterTeleop(); //UNCOMMENT FOR SHOOTER CONTROL
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
