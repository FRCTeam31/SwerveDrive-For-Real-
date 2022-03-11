// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Properties;
import java.util.ArrayList;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.ObjectInput;
import java.io.ObjectInputStream;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import PursellJaques.ArcadeDrive;
import PursellJaques.ClimbingMechanism;
import PursellJaques.FalconFXSwerveDrive;
import PursellJaques.FalconFXSwerveModule;
import PursellJaques.InterpolatingTreeMap;
import PursellJaques.PixyVisionSystem;
import PursellJaques.Turret;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.commands.AimAtTargetCommand;
import frc.robot.commands.AimTurretTowardsTargetZach;
import frc.robot.commands.AutoBallPickupCommand;
import frc.robot.commands.AutonomousCommandGroupSides;
import frc.robot.commands.BallSuckCommand;
import frc.robot.commands.CalibrateSwerve;
import frc.robot.commands.CancelAllDriveBaseCommandsCommand;
import frc.robot.commands.CancelAllShooterCommandsCommand;
import frc.robot.commands.ClimbingCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.DriveToDistanceWithMotionProfileCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FullAutonParalellCommandGroup;
import frc.robot.commands.RaiseIntakeCommand;
import frc.robot.commands.SetShooterSpeedCommand;
import frc.robot.commands.ShootBallCommand;
import frc.robot.commands.ShootLowGoalCommand;
import frc.robot.commands.SwerveModuleTestCommand;
import frc.robot.commands.TeleopBallIntakeCommand;
import frc.robot.commands.TeleopTurnTurretCommand;
import frc.robot.commands.TrackBallCommand;
import frc.robot.commands.TrackBallWithPixyCommand;
import frc.robot.commands.TurnAngleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.SerialPort;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Instance Variables
  
  // Joystick
  public static Joystick joystick, joystick2;
  public static JoystickButton button1;
  public static JoystickButton button2;
  public static JoystickButton button3;
  public static JoystickButton button4;
  public static JoystickButton button5;
  public static JoystickButton button6;
  public static JoystickButton button7;
  public static JoystickButton button8;
  public static JoystickButton button9;
  public static JoystickButton button10;

  // Swerve Variables
  public static FalconFXSwerveModule frontLeft, frontRight, backLeft, backRight, testModule, testBackRight,
    testFrontRight, testMiddleLeft;
  public static FalconFXSwerveDrive swerveDrive;
  public static Properties alignmentConstants;

  // Climbing Variables
  public static ClimbingMechanism climbingMechanism;

  // Differiental Drive Variables
  public static ArcadeDrive arcadeDrive;
  
  // Shooter Variables
  public static Turret turret;  
  public static InterpolatingTreeMap topMotorTreeMap;
  public static InterpolatingTreeMap bottomMotorTreeMap;
  
  // Vision Variables
  public static NetworkTable limelightTable;
  public static PixyVisionSystem pixy;

  // Intake Variables
  public static WPI_TalonSRX intakeMotor;
  public static WPI_TalonSRX raiseIntakeMotor;

  // NavX
  public static AHRS navX = new AHRS(SerialPort.Port.kUSB);

  // Alliance Signature

  // Commands
  public static DriveCommand driveCommand;
  public static TrackBallCommand trackBallCommand;
  public static ShootBallCommand shootBallCommand;
  public static TeleopBallIntakeCommand teleopBallIntakeCommand;
  public static CalibrateSwerve configureWheelAlignments;
  public static TrackBallWithPixyCommand trackBallWithPixyCommand;
  public static SetShooterSpeedCommand setShooterSpeedCommand;
  public static ArrayList<Command> shooterCommands;
  public static ArrayList<Command> driveBaseCommands;
  public static AimTurretTowardsTargetZach aimTurretTowardsTargetZach;
  public static TeleopTurnTurretCommand teleopTurnTurretCommand;
  public static RaiseIntakeCommand raiseIntakeCommand;
  public static BallSuckCommand ballSuckCommand;
  public static AutoBallPickupCommand autoBallPickupCommand;
  public static DriveToDistanceWithMotionProfileCommand driveToDistanceWithMotionProfileCommand;
  public static TurnAngleCommand turnAngleCommand;
  public static ShootLowGoalCommand shootLowGoalCommand;
  public CancelAllDriveBaseCommandsCommand cancelAllDriveBaseCommandsCommand;
  public CancelAllShooterCommandsCommand cancelAllShooterCommandsCommand;
  public ClimbingCommand climbingCommand;
  public static FullAutonParalellCommandGroup fullAutonParalellCommandGroup;
  // Test Commands
  // SwerveModuleTestCommand swerveModuleTestCommand;

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * 
   * @throws IOException
   * @throws ClassNotFoundException
   */
  public RobotContainer() {
    // Create swerve modules
    try {
      alignmentConstants = new Properties();
      alignmentConstants.load(new FileInputStream(Constants.ALIGNMENT_FILE_PATH));
      System.out.println("LOADED ALIGN CONSTANTS");
    } catch (Exception e) {
      e.printStackTrace();
      System.out.println("\nHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHh\n\nHHHHHHHHHHH\nHAttempting to calibrate swerve drive will result in a crash");
    }

    frontLeft = new FalconFXSwerveModule(Constants.FL_DMCID, Constants.FL_AMCID, Constants.FL_EC, Constants.FL_P,
        "front left");
    frontRight = new FalconFXSwerveModule(Constants.FR_DMCID, Constants.FR_AMCID, Constants.FR_EC, Constants.FR_P,
        "front right");
    backLeft = new FalconFXSwerveModule(Constants.BL_DMCID, Constants.BL_AMCID, Constants.BL_EC, Constants.BL_P,
        "back left");
    backRight = new FalconFXSwerveModule(Constants.BR_DMCID, Constants.BR_AMCID, Constants.BR_EC, Constants.BR_P,
        " back right");
    // testModule= new FalconFXSwerveModule(Constants.TM_DMCID, Constants.TM_AMCID,
    // Constants.TM_EC, Constants.TM_P);
    // testBackRight = new FalconFXSwerveModule(Constants.TBR_DMCID,
    // Constants.TBR_AMCID, Constants.TBR_EC, Constants.TBR_P);
    // testFrontRight = new FalconFXSwerveModule(Constants.TFR_DMCID,
    // Constants.TFR_AMCID, Constants.TFR_EC, Constants.TFR_P);
    // testMiddleLeft = new FalconFXSwerveModule(Constants.TML_DMCID,
    // Constants.TML_AMCID, Constants.TML_EC, Constants.TML_P);

    // Create FalconFXSwerveDrive
    FalconFXSwerveModule[] swerveModules = {frontRight, frontLeft, backLeft, backRight};
    swerveDrive = new FalconFXSwerveDrive(swerveModules);

    //  Create Differential Drive
    FalconFXSwerveModule[] leftDifferentialDrive = {frontLeft, backLeft};
    FalconFXSwerveModule[] rightDifferentialDrive = {frontRight, backRight};
    arcadeDrive = new ArcadeDrive(leftDifferentialDrive, rightDifferentialDrive);

    // Joystick Controls
    joystick = new Joystick(Constants.JOYSTICK1_PORT);
    joystick2 = new Joystick(2);
    button1 = new JoystickButton(joystick, 1);
    button2 = new JoystickButton(joystick, 2);
    button3 = new JoystickButton(joystick, 3);
    button4 = new JoystickButton(joystick, 4);
    button5 = new JoystickButton(joystick, 5);
    button6 = new JoystickButton(joystick, 6);
    button7 = new JoystickButton(joystick, 7);
    button8 = new JoystickButton(joystick, 8);
    button9 = new JoystickButton(joystick, 9);
    button10 = new JoystickButton(joystick, 10);

    // Shooter
    turret = new Turret(Constants.TOP_SHOOTER_CAN_ID, Constants.BOTTOM_SHOOTER_CAN_ID, Constants.TURRET_ANGLE_MOTOR_CAN_ID, Constants.TURRET_MAX_ANGLE);
    // turret.putAlignmentConstant(); uncomment when the turret needs to be reset to zero
    bottomMotorTreeMap = new InterpolatingTreeMap(Constants.TREE_MAP_KEYS[0], Constants.BOTTOM_MOTOR_TREE_VALUES[0]);
    topMotorTreeMap = new InterpolatingTreeMap(Constants.TREE_MAP_KEYS[0], Constants.TOP_MOTOR_TREE_MAP_VALUES[0]);
    for (int i = 0; i < Constants.TOP_MOTOR_TREE_MAP_VALUES.length; i++) {
      bottomMotorTreeMap.put(Constants.TREE_MAP_KEYS[i], Constants.BOTTOM_MOTOR_TREE_VALUES[i]);
      topMotorTreeMap.put(Constants.TREE_MAP_KEYS[i], Constants.TOP_MOTOR_TREE_MAP_VALUES[i]);
    }

    // Climbing Mechanism
    climbingMechanism = new ClimbingMechanism(Constants.CLIMBING_MOTOR_CAN_ID);

    // Intake Motor
    intakeMotor = new WPI_TalonSRX(Constants.INTAKE_MOTOR_CID);
    raiseIntakeMotor = new WPI_TalonSRX(Constants.RAISE_INTAKE_MOTOR_CID);

    // Vision systems
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    pixy = new PixyVisionSystem();

    // // Limelight Controls
    if (DriverStation.isFMSAttached()) {
      if (DriverStation.getAlliance() == Alliance.Blue) {
        // On Blue Alliance
        Constants.BALL_PROFILE = Constants.BLUE_BALL_PROFILE;
        System.out.println("BLUE TEAM");
      } else if (DriverStation.getAlliance() == Alliance.Red) {
        // On Red Alliance
        Constants.BALL_PROFILE = Constants.RED_BALL_PROFILE;
        System.out.println("RED TEAM");
      } else {
        // INVALID; we guess blue
        Constants.BALL_PROFILE = Constants.BLUE_BALL_PROFILE;
      }
    } else {
      // FMS Not attached; we guess blue
      Constants.BALL_PROFILE = Constants.BLUE_BALL_PROFILE;
    }

    // Commands
    cancelAllDriveBaseCommandsCommand = new CancelAllDriveBaseCommandsCommand();
    cancelAllShooterCommandsCommand = new CancelAllShooterCommandsCommand();
    driveBaseCommands = new ArrayList<Command>();
    shooterCommands = new ArrayList<Command>();

    driveCommand = new DriveCommand();
    driveBaseCommands.add(driveCommand);    
    shootBallCommand = new ShootBallCommand(1000);
    shooterCommands.add(shootBallCommand);
    teleopBallIntakeCommand = new TeleopBallIntakeCommand();
    configureWheelAlignments = new CalibrateSwerve();
    trackBallWithPixyCommand = new TrackBallWithPixyCommand(Constants.BALL_PROFILE);
    driveBaseCommands.add(trackBallWithPixyCommand);
    shooterCommands.add(trackBallWithPixyCommand);
    setShooterSpeedCommand = new SetShooterSpeedCommand();
    shooterCommands.add(setShooterSpeedCommand);
    aimTurretTowardsTargetZach = new AimTurretTowardsTargetZach();
    shooterCommands.add(aimTurretTowardsTargetZach);
    raiseIntakeCommand = new RaiseIntakeCommand();
    teleopTurnTurretCommand = new TeleopTurnTurretCommand();
    shooterCommands.add(teleopTurnTurretCommand);
    shootLowGoalCommand = new ShootLowGoalCommand();
    shooterCommands.add(shootLowGoalCommand);
    driveToDistanceWithMotionProfileCommand = new DriveToDistanceWithMotionProfileCommand(1, 3);
    driveBaseCommands.add(driveToDistanceWithMotionProfileCommand);
    turnAngleCommand = new TurnAngleCommand(45, 4);
    driveBaseCommands.add(turnAngleCommand);
    fullAutonParalellCommandGroup = new FullAutonParalellCommandGroup();
    driveBaseCommands.add(fullAutonParalellCommandGroup);
    climbingCommand = new ClimbingCommand();

    // Test Commands
    // swerveModuleTestCommand = new SwerveModuleTestCommand(testModule);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    button1.whenPressed(driveCommand);
    // button2.whenPressed(teleopBallIntakeCommand);
    button2.whenPressed(teleopTurnTurretCommand);
    // button3.whenPressed(shootBallCommand);
    button3.whenPressed(configureWheelAlignments);
    button4.whenPressed(setShooterSpeedCommand);
    button5.whenPressed(aimTurretTowardsTargetZach);  
    button6.whenPressed(cancelAllDriveBaseCommandsCommand);
    button7.whenPressed(cancelAllShooterCommandsCommand);
    button8.whenPressed(turnAngleCommand);
    // button9.whenPressed(shootLowGoalCommand);
    button9.whenPressed(trackBallWithPixyCommand);
    button10.whenPressed(fullAutonParalellCommandGroup);
    
    
  }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new FullAutonParalellCommandGroup();
    }

  /**
   * Turrn of the Drive command
   */
 

  /**
   * Schedule the drive Command
   */
  public static void turnOnDriveCommands() {
    driveCommand.schedule();
  }

  /**
   * Cancels all commands except the one passed as the parameter (assuming that all command are inside of commandList)
   */
  public static void cancelAllShooterCommands() {
    for (int i = 0; i < shooterCommands.size(); i++) {
      shooterCommands.get(i).cancel();
    }
  }

  public static void cancelAllDriveBaseCommands(){
    for (int i = 0; i < driveBaseCommands.size(); i++){
      driveBaseCommands.get(i).cancel();
    }
  }
}
