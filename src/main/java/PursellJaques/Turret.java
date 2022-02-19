package PursellJaques;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.commands.TurnAngleCommand;

/**
 * A class to manage the turret
 */
public class Turret {
    // Instance Variables
    public WPI_TalonFX topShooterMotor, bottomShooterMotor, turnShooterMotor;
    public final double TICKS_TO_DEGREES_CONSTANT = 1 * (1 / 2048) * (40 / 1) * ( 360 / 1);
    // Convert from ticks to motor rotations (1 motor rotation per 2048 ticks) to turret rotations 
    // (1 turret rotation per 40 motor rotations) degrees (360 degrees per rotation)
    public final double DEGREES_TO_TICKS_CONSTANT = 1 / TICKS_TO_DEGREES_CONSTANT; // Inverse of other number
    public double maxTurretAngle; // The maximum angle the turret can turn in one direction (with 0 being forward)

    public Turret(int topShooterMotorCanID, int bottomShooterMotorCanID, int turnShooterMotorCanID, double maxTurretAngle){
        // Bottom Shooter Motor
        bottomShooterMotor = new WPI_TalonFX(bottomShooterMotorCanID);
        TalonFXConfiguration bottomShooterMotorConfig = new TalonFXConfiguration();
        bottomShooterMotorConfig.slot0.kP = Constants.SHOOTER_KP;
        bottomShooterMotorConfig.slot0.kI = Constants.SHOOTER_KI;
        bottomShooterMotorConfig.slot0.kD = Constants.SHOOTER_KD;
        bottomShooterMotor.configAllSettings(bottomShooterMotorConfig);
        bottomShooterMotor.selectProfileSlot(0, 0);
        bottomShooterMotor.setNeutralMode(NeutralMode.Brake);
        // Top Shooter Motor
        topShooterMotor = new WPI_TalonFX(topShooterMotorCanID);
        TalonFXConfiguration topShooterMotorConfig = new TalonFXConfiguration();
        topShooterMotorConfig.slot0.kP = Constants.SHOOTER_KP;
        topShooterMotorConfig.slot0.kI = Constants.SHOOTER_KI;
        topShooterMotorConfig.slot0.kD = Constants.SHOOTER_KD;
        topShooterMotor.configAllSettings(topShooterMotorConfig);
        topShooterMotor.selectProfileSlot(0, 0);
        topShooterMotor.setNeutralMode(NeutralMode.Brake);
        // turnShooterMotor
        turnShooterMotor = new WPI_TalonFX(turnShooterMotorCanID);
        TalonFXConfiguration turnShooterMotorConfig = new TalonFXConfiguration();
        turnShooterMotorConfig.slot0.kP = Constants.TURN_SHOOTER_MOTOR_KP;
        turnShooterMotorConfig.slot0.kI = Constants.TURN_SHOOTER_MOTOR_KI;
        turnShooterMotorConfig.slot0.kD = Constants.TURN_SHOOTER_MOTOR_KD;
        turnShooterMotor.configAllSettings(turnShooterMotorConfig);
        turnShooterMotor.selectProfileSlot(0, 0);
        turnShooterMotor.setNeutralMode(NeutralMode.Brake);
        // Set the shooter to have its current position as its zero point
        this.setAngleZeroPoint();
        this.maxTurretAngle = maxTurretAngle;
    }

    /**
     * Set the shooter's current position to be the position of 0 degrees
     * The code is set up so that 0 degrees needs to be directly forward
     */
    public void setAngleZeroPoint(){
        turnShooterMotor.setSelectedSensorPosition(0.0);
    }

    /**
     * Set the speeds of the shooting motors to specified values
     */
    public void setShooterSpeeds(double topMotorSpeed, double bottomMotorSpeed){
        topShooterMotor.set(ControlMode.Velocity, topMotorSpeed);
        bottomShooterMotor.set(ControlMode.Velocity, bottomMotorSpeed);
    }

    /**
     * Set the absolute angle of the turret (180 to -180)
     * If the angle is outside the range of the turret, the turret stops at its maximum value
     */
    public void setShooterAbsoluteAngle(double targetAngle){
        // Check to see if target angle is outside the turrets maximum
        if(Math.abs(targetAngle) > maxTurretAngle){
            // Set it to be the extreme value closest to the wanted value
            targetAngle = targetAngle / Math.abs(targetAngle) * maxTurretAngle;
        }
        turnShooterMotor.set(ControlMode.Position, targetAngle * DEGREES_TO_TICKS_CONSTANT);

    }

    /**
     * Sets the shooter to an angle relative to its current position
     * @param targetAngle the angle to go to, relative to the current orientation
     */
    public void setShooterRelativeAngle(double targetAngle){
        double currentAngle = turnShooterMotor.getSelectedSensorPosition() * TICKS_TO_DEGREES_CONSTANT;
        double targetAbsoluteAngle = currentAngle + targetAngle;
        setShooterAbsoluteAngle(targetAbsoluteAngle);
    }

    /**
     * Display metrics to smart dashboard
     */
    public void displayMetricsToSmartDashboard(){
        SmartDashboard.putNumber("Top Shooter Motor Speed", topShooterMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Boottom Shooter Motor Speed", bottomShooterMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Turret Angle", turnShooterMotor.getSelectedSensorPosition() * TICKS_TO_DEGREES_CONSTANT);
    }

    /**
     * Stop the shooter motors
     */
    public void stopShooterMotors(){
        topShooterMotor.stopMotor();
        bottomShooterMotor.stopMotor();
    }

    /**
     * Stop the angle motor
     */
    public void stopAngleMotor(){
        turnShooterMotor.stopMotor();
    }
}
