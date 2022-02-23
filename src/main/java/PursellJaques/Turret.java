package PursellJaques;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.TurnAngleCommand;

/**
 * A class to manage the turret
 */
public class Turret {
    // Instance Variables
    public WPI_TalonFX topShooterMotor, bottomShooterMotor, turnShooterMotor;
    public double TICKS_TO_DEGREES_CONSTANT =  2.790178571e-4; //1 * (1 / 2048) * (1 / 45) * (1 / 14) * ( 360 / 1);
    // Convert from ticks to motor rotations (1 motor rotation per 2048 ticks) to turret rotations 
    // (1 turret rotation per 45 motor rotations) degrees (360 degrees per rotation)
    public double DEGREES_TO_TICKS_CONSTANT = 3584; // Inverse of other number
    public double maxTurretAngle; // The maximum angle the turret can turn in one direction (with 0 being forward)
    public double turretZeroConstant; // The constant that tells the turret angle motor where 0 is
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

        System.out.println("TICKS TO DEGREES: " + TICKS_TO_DEGREES_CONSTANT);
        System.out.println("DEGREES TO TICKS: " + DEGREES_TO_TICKS_CONSTANT);
    }

    /**
     * Set the shooter's current position to be the position of 0 degrees
     * The code is set up so that 0 degrees needs to be directly forward
     */
    public void setAngleZeroPoint(){
        turretZeroConstant = Double.parseDouble(RobotContainer.alignmentConstants.getProperty("TURRET", "0.0"));
    }

    /**
     * Get the constant that sets the current turret position to 0
     * @return The constant that sets the current turret positio to be 0
     */
    public double getZeroAngleConstant(){
        return -1 * turnShooterMotor.getSelectedSensorPosition();
    }

    /**
     * Set the speeds of the shooting motors to specified values
     */
    public void setShooterSpeeds(double topMotorSpeed, double bottomMotorSpeed){
        topShooterMotor.set(ControlMode.Velocity, topMotorSpeed * -1);
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
            targetAngle = Math.signum(targetAngle) * maxTurretAngle;
        }
        // Power Turret + offset of the zero coonstant
        turnShooterMotor.set(ControlMode.Position, targetAngle * DEGREES_TO_TICKS_CONSTANT);
        SmartDashboard.putNumber("Current Shooter Pos in Ticks", turnShooterMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Target Shooter Pos in Ticks", targetAngle * DEGREES_TO_TICKS_CONSTANT + turretZeroConstant);
    }

    /**
     * Sets the shooter to an angle relative to its current position
     * @param targetAngle the angle to go to, relative to the current orientation
     */
    public void setShooterRelativeAngle(double targetAngle){
        double currentAngle = turnShooterMotor.getSelectedSensorPosition() * TICKS_TO_DEGREES_CONSTANT;
        double targetAbsoluteAngle = currentAngle + targetAngle;
        setShooterAbsoluteAngle(targetAbsoluteAngle);

        SmartDashboard.putNumber("TARGET ABSOLUTE ANGLE", targetAbsoluteAngle);
    }

    /**
     *  Sets the shooter to an angle relative to its current position while ignoring
     * small inputs
     * @param targetAngle the angle to go to, relv=ative to the current orientation
     * @param deadZone the max input size to ignore
     */
    public void setCurrentShooterRelativeAngleWithDeadZone(double targetAngle, double deadZone){
        if(Math.abs(targetAngle) > deadZone){
            setShooterRelativeAngle(targetAngle);
        }
        else{
            turnShooterMotor.stopMotor();
        }
    }

    /**
     * Display metrics to smart dashboard
     */
    public void displayMetricsToSmartDashboard(){
        // SmartDashboard.putNumber("Top Shooter Motor Speed", topShooterMotor.getSelectedSensorVelocity());
        // SmartDashboard.putNumber("Boottom Shooter Motor Speed", bottomShooterMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Turret Angle", getCurrentAngle());
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

    public double getCurrentAngle(){
        return (turnShooterMotor.getSelectedSensorPosition() - turretZeroConstant) * TICKS_TO_DEGREES_CONSTANT ;

    }

    public void putAlignmentConstant() {
        RobotContainer.alignmentConstants.put("TURRET", turnShooterMotor.getSelectedSensorPosition());
    }
   
}
