// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package PursellJaques;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class FalconFXSwerveModule {
    // Instance Variables
    // Drive Variables
    public WPI_TalonFX driveMotor;

    // Angle Variables
    public double angleAlignmentConstant; 
    public WPI_TalonSRX angleMotor;
    public PIDController anglePIDController;
    public AnalogInput angleEncoder; // device that maps -0.5 rotations of a shaft to -5.0 volts and 0.5 rotations to +5.0 volts
    
    // Vectors
    public Vector position;
    public Vector perpendicular;

    // Test vars
    

    // Name
    public String name;

    /**
     * 
     * @param driveMotorCID The Can ID of the drive motor
     * @param angleMotorCID The Can ID of the angle motor
     * @param encoderChannel The channel # for the angle encoder
     * @param position The vector describing the position of the swerve module relative to the center of rotation (center of the bot)
     */
    public FalconFXSwerveModule(int driveMotorCID, int angleMotorCID, int encoderChannel, Vector position, String name){
        // Drive Variables
        driveMotor = new WPI_TalonFX(driveMotorCID);
        TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
        driveMotorConfig.slot0.kP = Constants.FALCON_KP;
        driveMotorConfig.slot0.kI = Constants.FALCON_KI;
        driveMotorConfig.slot0.kD = Constants.FALCON_KD;
        driveMotor.configAllSettings(driveMotorConfig);
        driveMotor.selectProfileSlot(0, 0); // 0 for slot0 and 0 again to specify primary closed loop PID

        // Angle Variables
        angleMotor = new WPI_TalonSRX(angleMotorCID);
        anglePIDController = new PIDController(Constants.ANGLE_KP, Constants.ANGLE_KI, Constants.ANGLE_KD);
        angleEncoder = new AnalogInput(encoderChannel);
        anglePIDController.enableContinuousInput(-0.5, 0.5); // Makes the pid work with a non continuous value. Max and min are in rotations 
        angleAlignmentConstant = Double.parseDouble(RobotContainer.alignmentConstants.getProperty(name, "0.0"));
        System.out.println("THE ANGLE ALIGNMENT CONSTANTS ARE: " + angleAlignmentConstant);
        
        // Vectors
        this.position = position;
        try {
            this.perpendicular = this.position.getPerpendicular();
        } catch (Exception e) {
            System.out.println("ERROR IN CREATING PERPENDICULARD VECTOR");
            e.printStackTrace();
        }
        this.perpendicular.scalarMultiplication(-1);
        System.out.println("PERP VECTOR: " + this.perpendicular);

        // Name
        this.name = name;
    }

    /**
     * 
     * @param input the vector describing the modules motion
     */
    public void drive(Vector input){
        // Break vector into r, theta
        double r = input.getMagnitude(); // Power to give to motor from [0 to 1] 
        double theta = input.getTheta(); // Angle in degrees from [-180, 180] where 0 is directly ahead (^)

        // Drive angle motor
        double currentAnglePosition = (((angleEncoder.getVoltage() + angleAlignmentConstant) % 5) - 2.5) / 5;  // Will return a value between [-0.5, 0.5] rotations
        /*
        We don't know where the shaft encoder will start in its cycle of -5V to +5V
        This line ensures that when the code starts whatever value that angleEncoder gives
        will be shifted half a rotation in the counter-clockwise direction and bound by [-0.5, 0.5]
        */
        double thetaToRotations = theta / 360; // Will return a value between [-0.5, 0.5] rotations
        
        // Scale magnitude by Cos(difference between current and target) so that it powers less when
        // facing the wrong direction
        double angle_difference = (thetaToRotations - currentAnglePosition) * 2 * Math.PI; // Difference of angles in radians
        r *= Math.cos(angle_difference);

        // Drive speed motor with output r
        driveMotor.set(ControlMode.Velocity, r * Constants.FALCON_MAX_SPEED);
        angleMotor.set(anglePIDController.calculate(currentAnglePosition, thetaToRotations)); // - thetaToRotations because encoder has negative in the clockwise direction

        // Printouts
        // System.out.println("(" + r * Constants.FALCON_MAX_SPEED + ", " + driveMotor.getSelectedSensorVelocity() + ")");
        // System.out.println("Current Rotation: " + currentAnglePosition + ", Target Rotation: " + thetaToRotations + ", Theta: " + theta);

    }

    /**
     * 
     * @param x left/right [-1, 1]
     * @param y is up/down [-1, 1]
     * @param z CCW/CW [-1, 1] 
     */
    public void driveFromJoystickInput(double x, double y, double z){
        Vector translationVector = new Vector(x, y);
        Vector driveVector = translationVector.add(perpendicular.scalarMultiplication(z));

        // Drive motor using the calculated vector 
        this.drive(driveVector);
    }

    public void stopMotion(){
        driveMotor.stopMotor();
        angleMotor.stopMotor();
    }

    /**
     * Returns a necesary value for the module to work. Make sure all motors are in their zero position
     */
    public double calculateAngleConstant(){
        
        return 2.5 - angleEncoder.getVoltage();
    }
}
