// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PathFollower extends CommandBase {
  
  double[][] coords = {{1,1}, {2,2}, {4,-3}, {3,4}};
  
  int matrixDims = 4*(coords.length-1);
  SimpleMatrix megaMatrix = new SimpleMatrix(matrixDims, matrixDims);

  int pow = 0;
  int coef = 1;
  int matOfset;

  public PathFollower() {
    
    //megaMatrix is called megaMatrix for a reason

    //Fill 1st row
    for(int i = 0; i < matrixDims; i++){
      
      if(i == 0){
        megaMatrix.set(0, i, 0);
      }
      else{
        megaMatrix.set(0, i, Math.pow(coords[0][0], pow)*coef);
        pow++;
        coef++;
      }
      
    }

    

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
