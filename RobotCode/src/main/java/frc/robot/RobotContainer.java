// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveWithXbox;
import frc.robot.commands.PathFollower;
import frc.robot.commands.PathGenerator;
import frc.robot.commands.TestPathFollower;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final Drivetrain drivetrain;

  private final DriveWithXbox driveWithXbox;

  //private final PathGenerator pathGenerator;
  private final PathFollower pathFollower;
  //private final TestPathFollower testPathFollower;
  private final PathEQ pathEQ; 


  public XboxController xbox = new XboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    
    drivetrain = new Drivetrain();

    pathEQ = new PathEQ(Constants.autoCoordinates, true);

    //Teleop commands
    driveWithXbox = new DriveWithXbox(drivetrain, xbox, false);
    driveWithXbox.addRequirements(drivetrain);
    drivetrain.setDefaultCommand(driveWithXbox);

    //pathGenerator = new PathGenerator();

    pathFollower = new PathFollower(drivetrain, pathEQ, 0.2, 0.15, 5);
    //testPathFollower = new TestPathFollower(drivetrain, pathEQ, 0.1, 0.05);
    
    

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return pathFollower;
    //return testPathFollower;
  }
}
