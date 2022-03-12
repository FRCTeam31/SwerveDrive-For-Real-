// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package PursellJaques;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/** Add your docs here.
 * A class to control the climbing mechanism
 */
public class ClimbingMechanism {
    // Instance Vairables
    public WPI_TalonSRX climbingMotor;

    /**
     * @param climbingMotorCANID the CAN ID of the climbing motor
     */
    public ClimbingMechanism(int climbingMotorCANID){
        climbingMotor = new WPI_TalonSRX(climbingMotorCANID);
        climbingMotor.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * 
     * @param power [-1, 1] power to give the motor
     */
    public void powerClimbMotor(double power){
        climbingMotor.set(power);
    }

}
