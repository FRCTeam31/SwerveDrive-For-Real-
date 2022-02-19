package PursellJaques;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


/** Add your docs here. */
public class N_PID {
    // Instance variables
    
    // Pid Constants
    private double kP;
    private double kI;
    private double kD;
    
    // Calculation Values
    private long prevTime;
    private double prevError;
    private double integralAcc;

    // Constructor
    public N_PID(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        prevTime = System.currentTimeMillis();
    }

    // Resets values [prevTime, integralAcc, prevPosition]. Recomended before each new process use of PID
    public void reset(double error){
        prevTime = System.currentTimeMillis();
        integralAcc = 0;
        prevError = error;
    }

    // Runs pid
    public double run(double current, double target){
        // Calculate current status values
        double deltaTime = System.currentTimeMillis() - prevTime;
        double error = target - current;
        double deltaError = error - prevError; 

        // Calculate Integral
        integralAcc += deltaTime * error; 
        // Calculate Derivative
        double derivative = deltaError / deltaTime;

        // Calculate output with PID equation
        double output = kP * error + kI * integralAcc + kD * derivative;

        // Update values
        prevError = error;
        prevTime = System.currentTimeMillis();
        
        return output;
    }
}
