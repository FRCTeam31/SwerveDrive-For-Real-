// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package PursellJaques;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants;

/** Add your docs here. */
public class ArcadeDrive {
    // Instance Variables
    FalconFXSwerveModule[] leftSide;
    FalconFXSwerveModule[] rightSide;



    public ArcadeDrive(FalconFXSwerveModule[] leftSide, FalconFXSwerveModule[] rightSide){
        this.leftSide = leftSide;
        this.rightSide = rightSide;
    }

    public void drive(double drivePower, double turnPower){
        // Calculate Motor Power
        double leftPower = drivePower + turnPower;
        double rightPower = drivePower - turnPower;
        
        if(Math.abs(leftPower) > 1 || Math.abs(rightPower) > 1){
            // Normalize power
            if(Math.abs(leftPower) > Math.abs(rightPower)){
                leftPower /= Math.abs(leftPower);
                rightPower /= Math.abs(leftPower);
            }
            else{
                leftPower /= Math.abs(rightPower);
                rightPower /= Math.abs(rightPower);
            }
        }
        // Power Motors
        for(FalconFXSwerveModule module: leftSide){
            module.driveMotor.set(ControlMode.Velocity, leftPower * Constants.FALCON_MAX_SPEED);
        }
        for(FalconFXSwerveModule module: rightSide){
            module.driveMotor.set(ControlMode.Velocity, rightPower * Constants.FALCON_MAX_SPEED);
        }
    }

    public void stopMotors(){
        for(FalconFXSwerveModule module: leftSide){
            module.stopMotion();
        }

        for(FalconFXSwerveModule module: rightSide){
            module.stopMotion();
        }

    }
}
