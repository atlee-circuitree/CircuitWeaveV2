

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.PathEQ;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.Motors;
import frc.robot.subsystems.Drivetrain.SwerveModule;

import java.lang.Math;

public class PathFollower extends CommandBase {

  private final Drivetrain drivetrain;
  
  private PathEQ pathEQ;
  private double slope;

  private double targetUValue = 0;
  private double[] targetPoint = new double[2];
  private double uIncrement = 0.5;

  private double forward = 0;
  private double strafe = 0;
  private double rotation = 0;

  private double speedMod = 1;
  private double tolerance = 0.1;

  private boolean isFinished = false;
 
  public PathFollower(Drivetrain dt, PathEQ pathEquation, double speed, double pointTolerance) {
    
    drivetrain = dt;
    pathEQ = pathEquation;
    speedMod = speed;
    tolerance = pointTolerance;

    addRequirements(drivetrain);

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    //Auto calculations

    

    //If the target u value is greater than the final u value, the robot has finished moving
    if(targetUValue > pathEQ.getFinalUValue()){
      isFinished = true;
    }


    //Find new target value
    targetPoint = pathEQ.solve(targetUValue);
    //If we have reached the current target X value
    if(targetPoint[0] - tolerance < drivetrain.getOdometryX() && drivetrain.getOdometryX() < targetPoint[0] + tolerance){
      //And the current target Y value
      if(targetPoint[1] - tolerance < drivetrain.getOdometryY() && drivetrain.getOdometryY() < targetPoint[1] + tolerance){
        //Set the next target u value and point
        targetUValue = targetUValue + uIncrement;
        targetPoint = pathEQ.solve(targetUValue);
      }
    }


    //Calculate the slope of the line passing through our current position and the target position
    double[] currentPos = {drivetrain.getAbsOdometryX(), drivetrain.getAbsOdometryY()};
    slope = pathEQ.slope(currentPos, pathEQ.solve(targetUValue));

    //Convert that slope to X/Y movement speeds
    //If the slope is purely vertical, the slope var will either be + or - infinity
    if(slope == Double.POSITIVE_INFINITY || slope == Double.NEGATIVE_INFINITY){
    
      //Figure out whether we want to go straight up or down
      //If the target Y value is ahead of the current Y value, we go forward
      if(targetPoint[1] > currentPos[1]){
        forward = 1;
        strafe = 0;
      }
      //Otherwise, we go backwards
      else{
        forward = -1;
        strafe = 0;
      }
    
    }

    //If the path runs entirely horizontal, the slope will likely be a very small number
    //This is due to tiny errors in my coeffecients, since I am rounding them
    else if(Math.abs(slope) < 0.001){
      //Figure out whether we want to go straight left or straight right
      //If the target X value is to the right of the current X value, we go to the right
      if(targetPoint[0] > currentPos[0]){
        forward = 0;
        strafe = 1;
      }
      //Otherwise, we go to the left
      else{
        forward = 0;
        strafe = -1;
      }
    }

    //If the |slope| > 1, then we know that the forward value has to be 1 or -1 depending if the slope is +/- (respectively) 
    else if(Math.abs(slope) >= 1){
      //If the target X value is to the right of the current X value, we go to the right
      if(targetPoint[0] > currentPos[0]){
        //This will give either 1 if slope > 1 or -1 if slope < -1
        forward = Math.abs(slope)/slope;
        strafe = 1/slope;
      }
      //Otherwise we go to the left
      else{
        forward = Math.abs(slope)/slope;
        strafe = -1/slope;
      }
    }

    //If the |slope| < 1, then we know that the strafe value has to be 1 or -1 depending if the slope is +/- (respectively) 
    else if(Math.abs(slope) < 1){
      //If the target X value is to the right of the current X value, we go to the right
      if(targetPoint[0] > currentPos[0]){
        //This will give either a positive value if slope > 1 or a negative value if slope < -1
        forward = (Math.abs(slope)/slope)*(1/slope);
        strafe = 1;
      }
      //Otherwise we go to the left
      else{
        forward = (Math.abs(slope)/slope)*(1/slope);
        strafe = -1;
      }
    }

    //This block should never be triggered, but just in case
    else{
      forward = 0;
      strafe = 0;
    }

    double[] fsrArray = {forward, strafe, rotation};
    SmartDashboard.putNumberArray("FWD, STR, ROT", fsrArray);



    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    SmartDashboard.putString("Breakpoint 1", "tripped");

    SmartDashboard.putNumberArray("TargetPoint", targetPoint);

    

    SmartDashboard.putNumber("speedMod", speedMod);

    SmartDashboard.putNumberArray("current Pos", currentPos);

    SmartDashboard.putBoolean("Is Finished", isFinished);

    SmartDashboard.putNumber("Target U Value", targetUValue);

    SmartDashboard.putNumber("getFinalUValue", pathEQ.getFinalUValue());

    SmartDashboard.putNumberArray("xCoefs", pathEQ.getLastRowofXCoefs());

    SmartDashboard.putNumber("Slope", slope);




    //Regular Teleop from here on out


    //Modify target values for field orientation (temp used to save calculations before original forward and strafe values are modified)
    double temp = forward * Math.cos(-drivetrain.getNavXOutputRadians()) + strafe * Math.sin(-drivetrain.getNavXOutputRadians()); 
    strafe = -forward * Math.sin(-drivetrain.getNavXOutputRadians()) + strafe * Math.cos(-drivetrain.getNavXOutputRadians()); 
    forward = temp;

    
    //Do some math to calculate the angles/sppeds needed to meet the target vectors
    //I don't have enough space or brainpower to say what A,B,C and D actually represent, but the swerve documentation does it well 
    double A = strafe - (rotation * (Constants.wheelbase/Constants.drivetrainRadius));
    double B = strafe + (rotation * (Constants.wheelbase/Constants.drivetrainRadius));
    double C = forward - (rotation * (Constants.trackwidth/Constants.drivetrainRadius));
    double D = forward + (rotation * (Constants.trackwidth/Constants.drivetrainRadius));

    //Calculates module speeds
    double frontLeftSpeed = Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2));
    double frontRightSpeed = Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2));
    double rearLeftSpeed = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2));
    double rearRightSpeed = Math.sqrt(Math.pow(A, 2) + Math.pow(D, 2));

    //Normalizes speeds (makes sure that none are > 1)
    double max = frontLeftSpeed;
    if(max < frontRightSpeed){
      max = frontRightSpeed;
    }
    if(max < rearLeftSpeed){
      max = rearLeftSpeed;
    } 
    if(max < rearRightSpeed){
      max = rearRightSpeed;
    }
    if(max > 1){
      frontLeftSpeed = frontLeftSpeed / max;
      frontRightSpeed = frontRightSpeed / max;
      rearLeftSpeed = rearLeftSpeed / max;
      rearRightSpeed = rearRightSpeed / max;
    }

    //Make SURE the robot stops whenthe joysticks are 0
    if(forward == 0 && strafe == 0 && rotation == 0){
      drivetrain.rotateMotor(Motors.FRONT_LEFT_DRV, 0);
      drivetrain.rotateMotor(Motors.FRONT_RIGHT_DRV, 0);
      drivetrain.rotateMotor(Motors.REAR_LEFT_DRV, 0);
      drivetrain.rotateMotor(Motors.REAR_RIGHT_DRV, 0);

      drivetrain.rotateModule(SwerveModule.FRONT_LEFT, Math.atan2(B, C)*(180/Math.PI), 0);
      drivetrain.rotateModule(SwerveModule.FRONT_RIGHT, Math.atan2(B, D)*(180/Math.PI), 0);
      drivetrain.rotateModule(SwerveModule.REAR_LEFT, Math.atan2(A, C)*(180/Math.PI), 0);
      drivetrain.rotateModule(SwerveModule.REAR_RIGHT, Math.atan2(A, D)*(180/Math.PI), 0);
    }
    else{
      //Set angles for modules (change speed mod later if needed)
      drivetrain.rotateModule(SwerveModule.FRONT_LEFT, Math.atan2(B, C)*(180/Math.PI), 1);
      drivetrain.rotateModule(SwerveModule.FRONT_RIGHT, Math.atan2(B, D)*(180/Math.PI), 1);
      drivetrain.rotateModule(SwerveModule.REAR_LEFT, Math.atan2(A, C)*(180/Math.PI), 1);
      drivetrain.rotateModule(SwerveModule.REAR_RIGHT, Math.atan2(A, D)*(180/Math.PI), 1);

      //Set speeds for modules
      drivetrain.rotateMotor(Motors.FRONT_LEFT_DRV, frontLeftSpeed * speedMod);
      drivetrain.rotateMotor(Motors.FRONT_RIGHT_DRV, frontRightSpeed * speedMod);
      drivetrain.rotateMotor(Motors.REAR_LEFT_DRV, rearLeftSpeed * speedMod);
      drivetrain.rotateMotor(Motors.REAR_RIGHT_DRV, rearRightSpeed * speedMod);
    }

    //Show important values on dashboard



    //Reset gyro button
    /*
    if(xbox.getBackButtonPressed()){
      drivetrain.zeroNavXYaw();
      drivetrain.resetOdometry(new Pose2d(new Translation2d(0, new Rotation2d(0)), new Rotation2d(0)));
    }
    */
  }  

  @Override
  public void end(boolean interrupted){

    forward = 0;
    strafe = 0;
    rotation = 0;

    drivetrain.rotateMotor(Motors.FRONT_LEFT_DRV, 0);
    drivetrain.rotateMotor(Motors.FRONT_RIGHT_DRV, 0);
    drivetrain.rotateMotor(Motors.REAR_LEFT_DRV, 0);
    drivetrain.rotateMotor(Motors.REAR_RIGHT_DRV, 0);

    drivetrain.rotateModule(SwerveModule.FRONT_LEFT, Math.atan2(0, 0)*(180/Math.PI), 0);
    drivetrain.rotateModule(SwerveModule.FRONT_RIGHT, Math.atan2(0, 0)*(180/Math.PI), 0);
    drivetrain.rotateModule(SwerveModule.REAR_LEFT, Math.atan2(0, 0)*(180/Math.PI), 0);
    drivetrain.rotateModule(SwerveModule.REAR_RIGHT, Math.atan2(0, 0)*(180/Math.PI), 0);

  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}