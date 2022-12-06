

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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
  
  private PIDController xPID = new PIDController(Constants.xControllerP, Constants.xControllerI, Constants.xControllerD);
  private PIDController yPID = new PIDController(Constants.yControllerP, Constants.yControllerI, Constants.yControllerD);
  private ProfiledPIDController zPID = new ProfiledPIDController(
    Constants.zControllerP, Constants.zControllerI, Constants.zControllerD, new Constraints(1.0, 1.0));

  private HolonomicDriveController driveController = new HolonomicDriveController(xPID, yPID, zPID);
  
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

    //double[] currentPos = {drivetrain.getRoundedOdometryY(), -drivetrain.getRoundedOdometryX()};
    //targetPoint = pathEQ.solve(targetUValue);
    
    Pose2d currentPos = new Pose2d(drivetrain.getRoundedOdometryY(), -drivetrain.getRoundedOdometryX(),
    new Rotation2d(Math.toRadians(drivetrain.getRoundedOdometryZ())));

    //Change rotation2d references to actual target value once overlay is added
    Pose2d targetPos = new Pose2d(pathEQ.solve(targetUValue)[0], pathEQ.solve(targetUValue)[1],
    new Rotation2d(0));

    ChassisSpeeds chassisSpeeds = driveController.calculate(currentPos, targetPos, 1.0, new Rotation2d(0));

    SwerveModuleState[] calculatedStates = Constants.driveKinematics.toSwerveModuleStates(chassisSpeeds);
  

    /* OLD CODE
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


    */

    double[] fsrArray = {forward, strafe, rotation};
    SmartDashboard.putNumberArray("FWD, STR, ROT", fsrArray);



    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    SmartDashboard.putString("Breakpoint 1", "tripped");

    SmartDashboard.putNumberArray("TargetPoint", targetPoint);
    SmartDashboard.putNumber("target Pos X", targetPos.getX());
    SmartDashboard.putNumber("target Pos Y", targetPos.getY());

    

    SmartDashboard.putNumber("speedMod", speedMod);

    //SmartDashboard.putNumberArray("current Pos", currentPos);
    SmartDashboard.putNumber("current Pos X", currentPos.getX());
    SmartDashboard.putNumber("current Pos Y", currentPos.getY());

    SmartDashboard.putBoolean("Is Finished", isFinished);

    SmartDashboard.putNumber("Target U Value", targetUValue);

    SmartDashboard.putNumber("getFinalUValue", pathEQ.getFinalUValue());

    SmartDashboard.putNumberArray("xCoefs", pathEQ.getLastRowofXCoefs());

    SmartDashboard.putNumber("Slope", slope);

    //SmartDashboard.putBoolean("Ty > Cy", targetPoint[1] > currentPos[1]);

    SmartDashboard.putNumber("Rise", rise);
    SmartDashboard.putNumber("Run", run);




    //Regular Teleop from here on out
    /*

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
    */


    //Calculates module speeds
    double frontLeftSpeed = calculatedStates[0].speedMetersPerSecond;
    double frontRightSpeed = calculatedStates[1].speedMetersPerSecond;
    double rearLeftSpeed = calculatedStates[2].speedMetersPerSecond;
    double rearRightSpeed = calculatedStates[3].speedMetersPerSecond;

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

      drivetrain.rotateModule(SwerveModule.FRONT_LEFT, calculatedStates[0].angle.getDegrees(), 0);
      drivetrain.rotateModule(SwerveModule.FRONT_RIGHT, calculatedStates[1].angle.getDegrees(), 0);
      drivetrain.rotateModule(SwerveModule.REAR_LEFT, calculatedStates[2].angle.getDegrees(), 0);
      drivetrain.rotateModule(SwerveModule.REAR_RIGHT, calculatedStates[3].angle.getDegrees(), 0);
    }
    else{
      //Set angles for modules (change speed mod later if needed)
      drivetrain.rotateModule(SwerveModule.FRONT_LEFT, calculatedStates[0].angle.getDegrees(), 1);
      drivetrain.rotateModule(SwerveModule.FRONT_RIGHT, calculatedStates[1].angle.getDegrees(), 1);
      drivetrain.rotateModule(SwerveModule.REAR_LEFT, calculatedStates[2].angle.getDegrees(), 1);
      drivetrain.rotateModule(SwerveModule.REAR_RIGHT, calculatedStates[3].angle.getDegrees(), 1);

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
    if(targetPos.getX() - tolerance < currentPos.getX() && currentPos.getX() < targetPos.getX() + tolerance){
      //And the current target Y value
      if(targetPos.getY() - tolerance < currentPos.getY() && currentPos.getY() < targetPos.getY() + tolerance){
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