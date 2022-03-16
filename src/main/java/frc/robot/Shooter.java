package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public class Shooter {
    private Solenoid m_intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
    private TalonSRX m_intakeMotor = new TalonSRX(Constants.Motors.kIntakeMotor);
    private TalonSRX m_towerMotor = new TalonSRX(Constants.Motors.kTowerMotor);
    private TalonSRX m_shooterMotor = new TalonSRX(Constants.Motors.kShooterMotor);

    private static DigitalInput m_breakBeamBottom = new DigitalInput(Constants.Sensors.kBreakBeamDIO);
    private Timer shootTimeout = new Timer();

    private boolean isTimerActive = false;
    private boolean isExtended = false;
    private boolean isCurrentlyShooting = false;

    private BangBangController m_shooterVelocityBB = new BangBangController();
    private XboxController operatorJoy = Constants.IO.m_operatorJoy;

    public Shooter() {
        m_shooterMotor.configFactoryDefault();
    }

    // shooter controlling logic that is ran in teleop
    public void shooterTeleop() {
        // extend intake, activate intake motors and tower motors to feed 
        // into shot position
        // until DIO is wired, relies on operator to release bumper when
        // in position
        if (operatorJoy.getRightBumperPressed()) {
            if (!isExtended) { //if not extended, extend
                extend();
            }
            
            intake();
        }

        if (operatorJoy.getAButton() || isCurrentlyShooting) {
            isCurrentlyShooting = true; //ensures operator only has to press button once to shoot
            if (!isTimerActive) {
                isTimerActive = true;
                shootTimeout.reset();
                shootTimeout.start();
            } else {
                if (shootTimeout.get() >= Constants.Motors.kShootTimeout) {
                    shootTimeout.stop();
                    isTimerActive = false;
                    isCurrentlyShooting = false;
                }
            }
        }
    }

    // enable/disable solenoid, should not be ran periodically
    private void extend() {
        if (!isExtended) {
            m_intakeSolenoid.set(true);
            isExtended = true;
        }  else {
            m_intakeSolenoid.set(false);
            isExtended = false;
        }
    }

    // TODO, verify positive or negative intakes
    public void intake() {
        m_intakeMotor.set(ControlMode.PercentOutput, Constants.Motors.kIntakePower);
    }

    // TODO, verify positive or negative goes the direction we want
    public void towerFeed() {
        m_towerMotor.set(ControlMode.PercentOutput, Constants.Motors.kTowerPower);
    }

    public void shoot() {
        m_shooterVelocityBB.setTolerance(Constants.Motors.kTargetShooterRPMTolerance);
        double targetOut = m_shooterVelocityBB.calculate(getShooterRPM(), Constants.Motors.kTargetShooterRPM);

        // using a bang bang controller for optimal response times, P accuracy isn't necessary
        // clamp to percent output, but really doesn't matter since on/off
        m_shooterMotor.set(ControlMode.PercentOutput, MathUtil.clamp(targetOut, 0, 1.0));
        if (m_shooterVelocityBB.atSetpoint()) {
            towerFeed(); //shoot
        }
    }

    // returns if the dio is "active" or not
    public static boolean isBottomCargoLoaded() {
        return m_breakBeamBottom.get();
    }

    // TODO, math
    public double getShooterRPM() {
        return m_shooterMotor.getSelectedSensorVelocity();
    }
}
