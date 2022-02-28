// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package PursellJaques;

import frc.robot.RobotContainer;

/** Add your docs here. */
public class SwerveDrive {
    // Instance variables
    public NEOSwerveModule[] swerveModules;

    /**
     * 
     * @param swerveModules List of swervedrive modules
     */
    public SwerveDrive(NEOSwerveModule[] swerveModules){
        this.swerveModules = swerveModules;
    }

    /**
     *  Reset necesary values of the swerve drive. Make sure the robots motor's are in position zero
     */
    public void reset(){
        for(NEOSwerveModule swerveModule: swerveModules){
            swerveModule.reset();
        }
        RobotContainer.navX.reset();
    }

    /**
     * 
     * @param x left/right [-1, 1]
     * @param y is forward/back [1, -1]
     * @param z CCW/CW [-1, 1] 
     */
    public void drive(double x, double y, double z){
        Vector[] driveVectors = new Vector[swerveModules.length];
        int index = 0;

        // Determine max vector length
        // System.out.println("(" + x + ", " + y + ", " + z + ")");

        double maxLength = 1;
        for(NEOSwerveModule swerveModule: swerveModules){
            Vector translationVector = new Vector(x, y);
            Vector rotational = swerveModule.perpendicular.scalarMultiplication(z);
            Vector driveVector = translationVector.add(rotational);
            driveVectors[index] = driveVector;

            if(driveVector.getMagnitude() > maxLength){
                maxLength = driveVector.getMagnitude();
            }

            index ++;

            // Printouts

            // System.out.println("Translational Vector: " + translationVector + ", Rotational Vector: " + rotational);
            // System.out.println("Translational Vector: " + translationVector);
            // System.out.println("Rotational Vector: " + rotational);
            // System.out.println("Drive vector: " + driveVector);
        }

        // Drive modules with scaled vectors

        index = 0;
        for(NEOSwerveModule swerveModule: swerveModules){
           Vector driveVector = driveVectors[index];
           driveVector = driveVector.scalarMultiplication(1 / maxLength);

            swerveModule.drive(driveVector);
            index ++;
        }
    }

    public void fieldOrientedDrive(double x, double y, double z){
        Vector translationVector = new Vector(x, y);
        double magnitude = translationVector.getMagnitude();
        double vectorAngle = translationVector.getTheta();
        double newAngle = vectorAngle - RobotContainer.navX.getAngle();

        double newX = magnitude * Math.sin(newAngle * Math.PI / 180);
        double newY = magnitude * Math.cos(newAngle * Math.PI / 180);

        // System.out.println("(" + newX + ", " + newY + ", " + z + ", " + RobotContainer.navX.getAngle() + ")");

        this.drive(newX, newY, z);
        // System.out.println("IM INSIDE THE FOD");
    }
}
