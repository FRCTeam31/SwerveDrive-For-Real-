// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package PursellJaques;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;

/** Add your docs here. */
public class NEOSwerveModule {
    // Instance Variables
    // Drive Variables
    public CANSparkMax driveMotor;
    public SparkMaxPIDController drivePIDController;
    public RelativeEncoder driveEncoder; 

    // Angle Variables
    public WPI_TalonSRX angleMotor;
    public PIDController anglePIDController;
    public AnalogInput angleEncoder; // Angle encode that measures roations from 
    
    // Vectors
    public Vector position;
    public Vector perpendicular;

    // Test vars
    public double c; 

    /**
     * 
     * @param driveMotorCID The Can ID of the drive motor
     * @param angleMotorCID The Can ID of the angle motor
     * @param encoderChannel The channel # for the angle encoder
     * @param position The vector describing the position of the swerve module relative to the center of rotation (center of the bot)
     */
    public NEOSwerveModule(int driveMotorCID, int angleMotorCID, int encoderChannel, Vector position){
        // Drive Variables
        driveMotor = new CANSparkMax(driveMotorCID, MotorType.kBrushless);
        drivePIDController = driveMotor.getPIDController();
        driveEncoder = driveMotor.getEncoder();
        drivePIDController.setP(Constants.DRIVE_KP);
        drivePIDController.setI(Constants.DRIVE_KI);
        drivePIDController.setD(Constants.DRIVE_KD);
        drivePIDController.setOutputRange(-1, 1);

        // Angle Variables
        angleMotor = new WPI_TalonSRX(angleMotorCID);
        anglePIDController = new PIDController(Constants.ANGLE_KP, Constants.ANGLE_KI, Constants.ANGLE_KD);
        angleEncoder = new AnalogInput(encoderChannel);
        anglePIDController.enableContinuousInput(-0.5, 0.5); // Makes the pid work with a non continuous value. Max and min are in rotations 
        
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
    }

    /**
     * 
     * @param input the vector describing the modules motion
     */
    public void drive(Vector input){
        // Break vector into r, theta
        double r = input.getMagnitude(); // Power to give to motor from [0 to 1] 
        double theta = input.getTheta(); // Angle in degrees from [-180, 180] where 0 is directly ahead (^)

        // Drive speed motor with output r
        drivePIDController.setReference(r * Constants.MAX_RPM, CANSparkMax.ControlType.kVelocity);

        // Drive angle motor with output r
        double currentAnglePosition = (((angleEncoder.getVoltage() + c) % 5) - 2.5) / 5; // I know!! This should be +- 0.5
        double thetaToRotations = theta / 360; // Should return a value between [-0.5, 0.5] rotations
        angleMotor.set(anglePIDController.calculate(currentAnglePosition, -1 * thetaToRotations)); // - thetaToRotations because encoder has negative in the clockwise direction

        // Printouts
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

    /**
     * Reset necesary values for the module to work. Make sure all motors are in their zero position
     */
    public void reset(){
        // Reset for Drive motor

        // Reset for Angle motor
        // sets current value to be 0. Run this when motors are set up in their zero position
        c = 2.5 - angleEncoder.getVoltage();
    }
}
