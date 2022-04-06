package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Shooter implements Loggable {
    private Solenoid m_intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.Solenoids.kIntakeSolenoid);
    private TalonSRX m_intakeMotor = new TalonSRX(Constants.Motors.kIntakeMotor);
    private WPI_TalonSRX m_towerMotor = new WPI_TalonSRX(Constants.Motors.kTowerMotor);
    private WPI_TalonSRX m_shooterMotor = new WPI_TalonSRX(Constants.Motors.kShooterMotor);
    
    private static DigitalInput m_breakBeamBottom = new DigitalInput(Constants.Sensors.kBreakBeamDIO);
    private Timer shootTimeout = new Timer();

    private boolean isTimerActive = false;
    private boolean isExtended = false;
    private boolean isCurrentlyShooting = false;

    public static double kVelocityDebug = 500; // only for debug

    private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(Constants.Sensors.kP_shtr, 0.001, 0.001);

    private XboxController operatorJoy = Constants.IO.m_operatorJoy;

    private Timer m_timer = new Timer();
    private InterpolatingTreeMap<Double, Number> m_lookupTable = new InterpolatingTreeMap<>();

    private PhotonCamera m_gloworm = new PhotonCamera("gloworm");

    public Shooter() {
        // configure shooter motor for closed loop control
        m_shooterMotor.configFactoryDefault();
        m_shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);

        // convert our map into an interpolatingtreemap
        for (double key: Constants.kLookupTable.keySet()) {
            m_lookupTable.put(key, Constants.kLookupTable.get(key));
        }

        // Set Sensor Phase
        m_shooterMotor.setSensorPhase(Constants.Sensors.kShooterOneInverted);

        // Config peak and nominal outputs
        m_shooterMotor.configNominalOutputForward(0, Constants.Sensors.kTimeoutMs);
        m_shooterMotor.configNominalOutputReverse(0, Constants.Sensors.kTimeoutMs);
        m_shooterMotor.configPeakOutputForward(1);
        m_shooterMotor.configPeakOutputReverse(-1);

        // Config Velocity closed loop gains
        m_shooterMotor.config_kP(Constants.Sensors.kPIDLoopIdx, Constants.Sensors.kP_shtr);
        m_shooterMotor.config_kI(Constants.Sensors.kPIDLoopIdx, Constants.Sensors.kI_shtr);
        m_shooterMotor.config_kD(Constants.Sensors.kPIDLoopIdx, Constants.Sensors.kD_shtr);
        m_shooterMotor.config_kF(Constants.Sensors.kPIDLoopIdx, Constants.Sensors.kFF_shtr);
    }

    // shooter controlling logic that is ran in teleop
    public void shooterTeleop() {
        // extend and begin intaking
        // retract if no longer intaking
        if (operatorJoy.getRightBumper()) {
            extend(true);
            
            intake(Constants.Motors.kIntakePower);
        } else if (operatorJoy.getLeftBumper()) {
            intake(-Constants.Motors.kIntakePower);
        } else {
            intake(0.0);
            extend(false);
        }

        // manual tower feed control
        if (!isCurrentlyShooting && Math.abs(operatorJoy.getRightTriggerAxis()) > 0.1) {
            towerFeed(Constants.Motors.kTowerPower);
        } else if (!isCurrentlyShooting && Math.abs(operatorJoy.getLeftTriggerAxis()) > 0.1) {
            towerFeed(-Constants.Motors.kTowerPower);
            shoot(-0.9);
        } else if (!isCurrentlyShooting && Math.abs(operatorJoy.getRightTriggerAxis()) <= 0.1) {
            towerFeed(0.0);
            shoot(0.0);
        }

        m_shooterMotor.config_kP(0, Constants.Sensors.kP_shtr);

        // start shooting motor and wait X seconds
        // then activate tower to shoot
        if (operatorJoy.getAButton()) {
            if (!isCurrentlyShooting) {
                isCurrentlyShooting = true;
                m_timer.reset();
                m_timer.start();
            }
        
            // wait for 1 second and then shoot
            if (m_timer.get() > 1) {
                if (isValidTarget()) {
                    var distance = getDistanceToTarget();

                    var power = calculateOptimalShootPower(distance);

                    //shoot(power); // comment me when tuning velocity values
                    shoot(kVelocityDebug); //uses debug value to tune velocity per x distance
                } else {
                    //shoot(0.9); //fallback shoot value
                    shoot(kVelocityDebug);
                }

                towerFeed(Constants.Motors.kTowerPower);
            } else {
                shoot(kVelocityDebug);
            }
        } else {
            shoot(0.0);
            isCurrentlyShooting = false;
        }
    }

    // useful only for debugging, do not use this in code
    @Config(tabName = "Driver", name = "Config P")
    public void configShootFeedforward(double kP) {
        Constants.Sensors.kP_shtr = kP;
    }

    // useful only for debugging, do not use this in code
    @Config(tabName = "Driver", name = "Config Vel")
    public void configVelocityFallback(double velocity) {
        this.kVelocityDebug = velocity;
    }

    public void extend(boolean value) {
        m_intakeSolenoid.set(value);
    }

    public void intake(double power) {
        m_intakeMotor.set(ControlMode.PercentOutput, power);
    }

    public void towerFeed(double power) {
        m_towerMotor.set(ControlMode.PercentOutput, power);
    }

    public void shoot(double velocity) {
        //m_shooterMotor.set(ControlMode.PercentOutput, velocity); //currently using powe
        
        var output = m_feedforward.calculate(getShooterRPM(), velocity, 0.001);
        System.out.println("Voltage Output: " + output);

        if (velocity != 0.0) {
            m_shooterMotor.setVoltage(output);
        } else {
            m_shooterMotor.setVoltage(0);
        }
        
     }

    public double calculateOptimalShootPower(double distance) {
        System.out.println("Optimal Power" + m_lookupTable.get(distance));
        return m_lookupTable.get(distance);
    }

    @Log.BooleanBox(name = "Target Detected", tabName = "Driver", width = 2, height = 5)
    public boolean isValidTarget() {
        try {
            return m_gloworm.getLatestResult().hasTargets();
        } catch (NullPointerException ex) {
            return false;
        }
    }

    @Log(name = "Target Distance", tabName = "Driver", width = 3, height = 2)
    public double getDistanceToTarget() {
        // assumes camera is facing straight forward
        // may need to tune pitch for angled camera
        // all values in meters
        var currentTarget = m_gloworm.getLatestResult();
        var distance = -1.0;
    
        if (currentTarget.hasTargets()) {
            distance = PhotonUtils.calculateDistanceToTargetMeters(
                1.219, // camera height: 48 inches up where camera is
                2.565, // target height: 8 foot five inches up
                Units.degreesToRadians(43),
                Units.degreesToRadians(currentTarget.getBestTarget().getPitch()) 
            );
        }
        
        return distance;
    }
    // returns if the dio is "active" or not
    public static boolean isBottomCargoLoaded() {
        return m_breakBeamBottom.get();
    }

    // TODO 
    @Log (tabName = "Driver", name = "Shooter RPM")
    public double getShooterRPM() {
        return m_shooterMotor.getSelectedSensorVelocity();
    }
}
