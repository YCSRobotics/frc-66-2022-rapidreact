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
    private Solenoid m_intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.Solenoids.kIntakeSolenoid);
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

    private Timer m_timer = new Timer();

    public Shooter() {
        m_shooterMotor.configFactoryDefault();
    }

    // shooter controlling logic that is ran in teleop
    public void shooterTeleop() {
        // extend intake, activate intake motors and tower motors to feed 
        // into shot position
        // until DIO is wired, relies on operator to release bumper when
        // in position
        if (operatorJoy.getRightBumper()) {
            m_intakeSolenoid.set(true);
            
            intake(Constants.Motors.kIntakePower);
        } else if (operatorJoy.getLeftBumper()) {
            intake(-Constants.Motors.kIntakePower);
        } else {
            intake(0.0);
            m_intakeSolenoid.set(false);
        }
        
        if (operatorJoy.getAButton()) {
            if (!isCurrentlyShooting) {
                isCurrentlyShooting = true;
                m_timer.reset();
                m_timer.start();
            }

            if (m_timer.get() > 1) {
                shoot(0.9);
                towerFeed(Constants.Motors.kTowerPower);
            } else {
                shoot(0.9);
            }
        } else {
            shoot(0.0);
            towerFeed(0.0);
            isCurrentlyShooting = false;
        }
    }

    // TODO, verify positive or negative intakes
    public void intake(double power) {
        m_intakeMotor.set(ControlMode.PercentOutput, power);
    }

    // TODO, verify positive or negative goes the direction we want
    public void towerFeed(double power) {
        m_towerMotor.set(ControlMode.PercentOutput, power);
    }

    public void shoot(double power) {
        m_shooterMotor.set(ControlMode.PercentOutput, power);
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
