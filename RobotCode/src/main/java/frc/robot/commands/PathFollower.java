

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
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
  private double slopeSign;
  private double rise;
  private double run;

  private SlewRateLimiter slopeSmoother = new SlewRateLimiter(1);
  private SlewRateLimiter riseSmoother = new SlewRateLimiter(0.001);
  private SlewRateLimiter runSmoother = new SlewRateLimiter(0.001);

  private double targetUValue = 0;
  private double[] targetPoint = new double[2];
  private double uIncrement;

  private double forward = 0;
  private double strafe = 0;
  private double rotation = 0;

  private double speedMod;
  private double tolerance;

  private boolean isFinished = false;
 
  public PathFollower(Drivetrain dt, PathEQ pathEquation, double speed, double pointTolerance) {
    
    drivetrain = dt;
    pathEQ = pathEquation;
    speedMod = speed;
    tolerance = pointTolerance;

    addRequirements(drivetrain);

  }

  @Override
  public void initialize() {

    targetUValue = 0;
    targetPoint = new double[2];
    uIncrement = 0.1;

    forward = 0;
    strafe = 0;
    rotation = 0;

    isFinished = false;

  }

  @Override
  public void execute() {

    //Auto calculations

    double[] currentPos = {drivetrain.getRoundedOdometryY(), -drivetrain.getRoundedOdometryX()};
    targetPoint = pathEQ.solve(targetUValue);
    
    //With SlewRateLimiter
    //slope = slopeSmoother.calculate(pathEQ.slope(targetPoint, currentPos));
    //rise = riseSmoother.calculate(pathEQ.slopeRiseRun(targetPoint, currentPos)[1]);
    //run = runSmoother.calculate(pathEQ.slopeRiseRun(targetPoint, currentPos)[0]);

    //Raw values
    //slope = pathEQ.slope(targetPoint, currentPos);
    //rise = pathEQ.slopeRiseRun(targetPoint, currentPos)[1];
    //run = pathEQ.slopeRiseRun(targetPoint, currentPos)[0];

    slope = pathEQ.slope(targetPoint, currentPos);
    rise = slope;
    run = 1;
  
    //Moving Forward
    if(targetPoint[1] > currentPos[1]){
      
      forward = Math.abs(rise);

      if(slope >= 0){
        strafe = Math.abs(run);
      }
      else{
        strafe = Math.abs(run) * -1;
      }
      //strafe = 0;
    }

    //Moving Backward
    else if(targetPoint[1] < currentPos[1]){
      
      forward = Math.abs(rise) * -1;

      if(slope >= 0){
        strafe = Math.abs(run) * -1;
      }
      else{
        strafe = Math.abs(run);
      }
    }

    //Normalize speeds and catch exception slopes
    if(slope == Double.POSITIVE_INFINITY || slope == Double.NEGATIVE_INFINITY){
      
      if(targetPoint[1] > currentPos[1]){
        forward = 1;
        strafe = 0;
      }
      else{
        forward = -1;
        strafe = 0;
      }
    
    }
    else{

      if(Math.abs(forward) > Math.abs(strafe)){
        forward = 1 * (Math.abs(forward)/forward);
        strafe = strafe/Math.abs(forward);
      }
      else{
        strafe = 1 * (Math.abs(strafe)/strafe);
        forward = forward/Math.abs(strafe);
      }

    }

    forward = -forward;
    strafe = strafe;


    /* OLD CODE

    //If the target u value is greater than the final u value, the robot has finished moving
    if(targetUValue > pathEQ.getFinalUValue()){
      isFinished = true;
    }


    //Calculate the slope of the line passing through our current position and the target position
    double[] currentPos = {drivetrain.getRoundedOdometryY(), drivetrain.getRoundedOdometryX()};
    slope = pathEQ.slope(currentPos, pathEQ.solve(targetUValue));
    slopeSign = Math.abs(slope)/slope;

    //Find new target value
    targetPoint = pathEQ.solve(targetUValue);
    //If we have reached the current target X value
    if(targetPoint[0] - tolerance < currentPos[0] && currentPos[0] < targetPoint[0] + tolerance){
      //And the current target Y value
      if(targetPoint[1] - tolerance < currentPos[1] && currentPos[1] < targetPoint[1] + tolerance){
        //Set the next target u value and point
        targetUValue = targetUValue + uIncrement;
        targetPoint = pathEQ.solve(targetUValue);
      }
    }

    



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


    //If the |slope| > 1, then we know that the forward value has to be 1 or -1, and the strafe value has to be < 1 or > -1
    else if(Math.abs(slope) >= 1){
      //If the target Y value is above the current one, we go up and to the left/right depending on the slope sign
      if(targetPoint[1] > currentPos[1]){
        forward = 1;
        strafe = (Math.abs(1/slope)) * slopeSign;
      }
      //Otherwise we go down and to the right/left
      else{
        forward = -1;
        strafe = (Math.abs(1/slope)) * -slopeSign;
      }
    }

    //If the |slope| < 1, then we know that the strafe value has to be 1 or -1 and the fwd value has to be < 1 or > -1
    else if(Math.abs(slope) < 1){
      //If the target Y value is above the current one, we go up and to the left/right depending on the slope sign 
      if(targetPoint[1] > currentPos[1]){
        forward = Math.abs(1/slope);
        strafe = 1 * slopeSign;
      }
      //Otherwise we go to the left
      else{
        forward = -Math.abs(1/slope);
        strafe = -1 * slopeSign;
      }
    }

    //This block should never be triggered, but just in case
    else{
      forward = 0;
      strafe = 0;
    }

    //invert Y speed
    forward = forward * -1;

    */

    double[] fsrArray = {forward, strafe, rotation};
    SmartDashboard.putNumberArray("FWD, STR, ROT", fsrArray);



    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    SmartDashboard.putString("Breakpoint 1", "tripped");

    SmartDashboard.putNumberArray("TargetPoint", targetPoint);
    SmartDashboard.putNumber("target Pos X", targetPoint[0]);
    SmartDashboard.putNumber("target Pos Y", targetPoint[1]);

    

    SmartDashboard.putNumber("speedMod", speedMod);

    SmartDashboard.putNumberArray("current Pos", currentPos);
    SmartDashboard.putNumber("current Pos X", currentPos[0]);
    SmartDashboard.putNumber("current Pos Y", currentPos[1]);

    SmartDashboard.putBoolean("Is Finished", isFinished);

    SmartDashboard.putNumber("Target U Value", targetUValue);

    SmartDashboard.putNumber("getFinalUValue", pathEQ.getFinalUValue());

    SmartDashboard.putNumberArray("xCoefs", pathEQ.getLastRowofXCoefs());

    SmartDashboard.putNumber("Slope", slope);

    SmartDashboard.putBoolean("Ty > Cy", targetPoint[1] > currentPos[1]);

    SmartDashboard.putNumber("Rise", rise);
    SmartDashboard.putNumber("Run", run);




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


    //If we have reached the current target X value
    if(targetPoint[0] - tolerance < currentPos[0] && currentPos[0] < targetPoint[0] + tolerance){
      //And the current target Y value
      if(targetPoint[1] - tolerance < currentPos[1] && currentPos[1] < targetPoint[1] + tolerance){
        //Set the next target u value
        targetUValue = targetUValue + uIncrement;
      }
    }
    //If the target u value is greater than the final u value, the robot has finished moving
    if(targetUValue > pathEQ.getFinalUValue()){
      isFinished = true;
    }


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