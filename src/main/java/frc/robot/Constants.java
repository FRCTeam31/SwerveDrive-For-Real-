// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import PursellJaques.Vector;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // ROBOT MOTOR SETUP
    //FALCONS:
    //1---2
    //3---4
    //5 = bottom shooter motor, 6 = top shooter motor, 7 = turret angle motor
    // FalconSRX
    //8-9
    // 10-11
    // 12-13

    // ##############################################################
    // HARDWARE CONSTANTS
    // ##############################################################

    // Swerve Module Constants
    // Angle PID
    public static double ANGLE_KP = 7;
    public static double ANGLE_KI = 1;
    public static double ANGLE_KD = 0;
    // NEO drive PID
    public static double DRIVE_KP = 0.0002;
    public static double DRIVE_KI = 0;
    public static double DRIVE_KD = 0;
    // Falcon drive pid
    public static double FALCON_KP = 0.2;
    public static double FALCON_KI = 0;
    public static double FALCON_KD = 0; 
    // The filepath on the RIO for the wheel alignment constants
    public static String WHEEL_ALIGNMENT_FILE_PATH = "home/lvuser/WheelConstants";
    
    // Misc
    public static int MAX_RPM = 5000;
    // this is the maximum number of integrated encoder ticks per 100ms that the Falcon500 can be driven to
    public static int FALCON_MAX_SPEED = 24000; 

    // Swerve Module values
    // Front Left
    public static int FL_DMCID = 1;
    public static int FL_AMCID = 10;
    public static int FL_EC = 0;
    public static Vector FL_P = new Vector(-1, 1);
    // Front Right
    public static int FR_DMCID = 2;
    public static int FR_AMCID = 11;
    public static int FR_EC = 1;
    public static Vector FR_P = new Vector(1, 1);
    // Back Left
    public static int BL_DMCID = 3;
    public static int BL_AMCID = 12;
    public static int BL_EC = 2;
    public static Vector BL_P = new Vector(-1, -1);
    // Back Right
    public static int BR_DMCID = 4;
    public static int BR_AMCID = 13;
    public static int BR_EC = 3;
    public static Vector BR_P = new Vector(1, -1);
    // Test Module
    public static int TM_DMCID = 6;
    public static int TM_AMCID = 5;
    public static int TM_EC = 0;
    public static Vector TM_P = new Vector(0, 1);
    // Test Back Right
    public static int TBR_DMCID = 6;
    public static int TBR_AMCID = 4;
    public static int TBR_EC = 1;
    public static Vector TBR_P = new Vector(0.68, -0.73);
    // Test Front Right
    public static int TFR_DMCID = 3;
    public static int TFR_AMCID = 7;
    public static int TFR_EC = 2;
    public static Vector TFR_P = new Vector(0.68, 0.73);
    // Test Middle Left
    public static int TML_DMCID = 1;
    public static int TML_AMCID = 2;
    public static int TML_EC = 0;
    public static Vector TML_P = new Vector(-0.84, -0.16);

    // Shooter Constants
    public static int BOTTOM_SHOOTER_CAN_ID = 5; // 15 or 14
    public static int TOP_SHOOTER_CAN_ID = 6;
    public static int TURRET_ANGLE_MOTOR_CAN_ID = 7;
    public static  double TURRET_MAX_ANGLE = 90; // Maximum angle the turret can spin in degrees, 0 is directly forward
    public static double SHOOTER_KP = 0.2;
    public static double SHOOTER_KI = 0.001;
    public static double SHOOTER_KD = 0; 
    public static  double TURN_SHOOTER_MOTOR_KP = 0.5;
    public static  double TURN_SHOOTER_MOTOR_KI = 0.00001;
    public static  double TURN_SHOOTER_MOTOR_KD = 0.01;
    public static double TURRET_ANGLE_MOTOR_KP = 1;
    public static double TURRET_ANGLE_MOTOR_KI = 1;
    public static double TURRET_ANGLE_MOTOR_KD = 1;
    public static double[] TREE_MAP_KEYS =    {-17.0, 0};
    public static double[] TOP_MOTOR_TREE_MAP_VALUES = {0.0, 1};
    public static double[] BOTTOM_MOTOR_TREE_VALUES = {0.0, 1};

    // Joystick Constants
    public static int JOYSTICK1_PORT = 1;
    public static int JOYSTICK2_PORT = 2;

    // Vision Profile Constants
    public static int BALL_PROFILE;
    public static int RED_BALL_PROFILE = 3;
    public static int BLUE_BALL_PROFILE = 2;
    public static int GOAL_PROFILE = 1;

    // Intake Motor Constants
    public static int INTAKE_MOTOR_CID = 9;

    // ##############################################################
    // COMMAND CONSTANTS
    // ##############################################################

    // Track ball command constants (OUTDATED COMMAND)
    public static double TRACK_BALL_COMMAND_DISTANCE_P = 0.01;
    public static double TRACK_BALL_COMMAND_DISTANCE_I = 0.0;
    public static double TRACK_BALL_COMMAND_DISTANCE_D = 0;
    public static int TRACK_BALL_COMMAND_DISTANCE_INVERTER = 1;

    public static double TRACK_BALL_COMMAND_ANGLE_P = 0.01;
    public static double TRACK_BALL_COMMAND_ANGLE_I = 0.0;
    public static double TRACK_BALL_COMMAND_ANGLE_D = 0;
    public static int TRACK_BALL_COMMAND_ANGLE_INVERTER = -1;

    public static double SAFE_TURN_RATE = 0.3;
    public static double TARGET_TY = -15;
    public static double ANGLE_TOLERANCE = 3;
    public static double Y_TOLERANCE = 1;
    public static int CHECKCOUNT = 10;

    // Aim at target command Constants
    public static  double GOAL_AIM_TOLERANCE = 0.05;
    public static  double GOAL_CENTER_TIMER = 10;
    public static  double GOAL_AIMING_KP = 0.01;
    public static  double GOAL_AIMING_KI = 0;
    public static  double GOAL_AIMING_KD = 0;

    // Field Orientated Drive With Targeting Command Constants
    public static  double FODWTKP = 1;
    public static  double FODWTKI = 0;
    public static  double FODWTKD = 0;
    public static  double FODWT_F = 0;

    // Turn To Angle Command Constants
    public static double TTA_KP = 1;
    public static double TTA_KI = 0;
    public static double TTA_KD = 0;
    public static double TTA_ERROR = 2; // Degrees of tolerance
    public static double TTA_MAX_COUNTER = 0; // Number of command loops the robot needs to be inside the tolerance

    // Ball intake command constants
    public static double BALL_SUCK_SPEED = 0.3;
    public static double SAFE_INTAKE_SPEED = 0.3;
    public static double AUTO_BALL_INTAKE_DRIVE_TIME = 222;
    public static double AUTO_BALL_INTAKE_SUCK_TIME = 666;

    // Drive Distance Command Constants
    public static double DISTANCE_TO_FIRST_BALL_TICKS = 0;
    public static  double DDC_KP = 1;
    public static  double DDC_KI = 0;
    public static  double DDC_KD = 0;
    public static  int DDC_TOLERANCE = 0; // Tolerance as a percentage of the target distance
    public static  double DDC_MAX_COUNT = 0; // Number of command loops the bot needs to be inside target 
    
    // Track Ball With Pixy Command Constants
    public static  double TRACK_BALL_WITH_PIXY_COMMAND_ANGLE_KP = 1;
    public static  double TRACK_BALL_WITH_PIXY_COMMAND_ANGLE_KI = 0;
    public static  double TRACK_BALL_WITH_PIXY_COMMAND_ANGLE_KD = 0;
    public static  double TRACK_BALL_WITH_PIXY_COMMAND_DISTANCE_KP = 1;
    public static  double TRACK_BALL_WITH_PIXY_COMMAND_DISTANCE_KI = 0;
    public static  double TRACK_BALL_WITH_PIXY_COMMAND_DISTANCE_ANGLE_KD = 0;
    public static  int TRACK_BALL_WITH_PIXY_COMMAND_ANGLE_TOLERANCE = 0; // Acceptable angle
    public static  double TRACK_BALL_WITH_PIXY_COMMAND_TARGET_TY = 0; // Target distance measurement
    public static  int TRACK_BALL_WITH_PIXY_COMMAND_TY_TOLERANCE = 0; // Acceptable distance tolerance as a percentage of the target distance measurement
    public static  int TRACK_BALL_WITH_PIXY_COMMAND_MAX_COUNT = 0;



    
}
