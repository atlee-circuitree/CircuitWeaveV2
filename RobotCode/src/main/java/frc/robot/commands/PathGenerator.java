// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.net.URI;
import java.net.URISyntaxException;
import java.awt.Desktop;
import java.io.IOException;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PathGenerator extends CommandBase {
  
  double[][] coords = {{1,1}, {2,3}, {3,-2}, {4,5}};
  double beginningSlope = 0;
  double endingSlope = -1;

  int subscript = 0;
  int pow = 0;
  int coef = 1;
  int matrixOffset = 0;

  //A really big matrix. Contains all of the function constraints and modified x values
  SimpleMatrix megaMatrix = new SimpleMatrix((coords.length-1)*4, (coords.length-1)*4);
  
  //Holds the y values and specified slopes. It is used with megaMatrix to solve for the coeffecients
  SimpleMatrix yMatrix = new SimpleMatrix((coords.length-1)*4, 1);
  
  //Holds all of the calculated coeffecients of each function
  SimpleMatrix coefMatrix = new SimpleMatrix(1, (coords.length-1)*4);

  
  //Used to order the matrix rows on the sim SmartDashboard (they come up in alphabetical order)
  String[] theAlphabet = {"A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "X", "Y", "Z"};
  
  public PathGenerator() {


    //// FILL MEGAMATRIX START (x matrix) ////////////////////////////////////////////////////////////////////////////////////
    
   
    //Make sure that the starting point of the path has the correct slope (fill line 1)
    matrixOffset = 1;
    for(int i = 0; i < 3; i++){
      megaMatrix.set(0, i + matrixOffset, coef * Math.pow(coords[subscript][0], pow));
      pow++;
      coef++;
    }
    pow = 0;
    coef = 0;
    matrixOffset = 0;
    


    //Fill middle functions
    //Makes sure that all functions not touching the overall start/endpoints of the path
    //have matching 1st/2nd derivatives at the points they touch,
    //as well as making sure that they pass through those said points

    for(int bigLoop = 0; bigLoop < coords.length-2; bigLoop++){

      //Used to determine which row to put numbers on
      int n = bigLoop * 4;

      //Make sure that the current function touches its first endpoint
      for(int i = 0; i < 4; i++){
        megaMatrix.set(n+1, i + matrixOffset, Math.pow(coords[subscript][0], pow));
        pow++;
      }
      pow = 0;

      //Make sure that the current function touches its second endpoint
      subscript++;
      for(int i = 0; i < 4; i++){
        megaMatrix.set(n+2, i + matrixOffset, Math.pow(coords[subscript][0], pow));
        pow++;
      }
      pow = -1;

      //Make sure that both funcitons touching the 2nd endpoint have equal 1st derivatives
      //There are 2 "parts" to this line, with the 2nd being the inverse of the 1st
      int invert = 1;
      int inlineOffset = 0;
      for(int i = 0; i < 2; i++){
        for(int j = 0; j < 4; j++){
          megaMatrix.set(n+3, j + inlineOffset + matrixOffset, coef * Math.pow(coords[subscript][0], pow) * invert);
          pow++;
          coef++;
        }
        pow = -1;
        coef = 0;
        invert = -1;
        inlineOffset = 4;
      }
      invert = 1;
      inlineOffset = 0;
      pow = 0;
      coef = 0;

      //Make sure that both funcitons touching the 2nd endpoint have equal 2nd derivatives
      //Again, there are 2 "parts" to this line, with the 2nd being the inverse of the 1st
      for(int i = 0; i < 2; i++){
        megaMatrix.set(n+4, inlineOffset + matrixOffset, 0);
        megaMatrix.set(n+4, inlineOffset + 1 + matrixOffset, 0);
        megaMatrix.set(n+4, inlineOffset + 2 + matrixOffset, 2 * invert);
        megaMatrix.set(n+4, inlineOffset + 3 + matrixOffset, 6 * coords[subscript][0] * invert);

        inlineOffset = 4;
        invert = -1;
      }

      //Then repeat for all middle functions, but shift all of these matrix fillers right 4 rows
      matrixOffset = matrixOffset + 4;

    }

    
    
    //We still have 3 lines to go, each correspoding to the last function

    //Make sure that the last function touches its first endpoint
    for(int i = 0; i < 4; i++){
      megaMatrix.set(megaMatrix.numRows()-3, i + matrixOffset, Math.pow(coords[subscript][0], pow));
      pow++;
    }
    pow = 0;

    //Make sure that the last function touches the endpoint for the whole path
    subscript++;
    for(int i = 0; i < 4; i++){
      megaMatrix.set(megaMatrix.numRows()-2, i + matrixOffset, Math.pow(coords[subscript][0], pow));
      pow++;
    }
    pow = 0;
    coef = 1;

    //Make sure that the ending point of the path has the correct slope (fill last line)
    matrixOffset = matrixOffset + 1;
    for(int i = 0; i < 3; i++){
      megaMatrix.set(megaMatrix.numRows()-1, i + matrixOffset, coef * Math.pow(coords[subscript][0], pow));
      pow++;
      coef++;
    }


    //// FILL MEGAMATRIX END /////////////////////////////////////////////////////////////////////////////////////////


    
    //Now we fill the Y Matrix, which will be used to solve for the coeffecients

    //Put the 1st specified slope as the top spot
    yMatrix.set(0, 0, beginningSlope);
    
    //Manually fill the 2nd line to make the for loop more effecient
    yMatrix.set(1, 0, coords[0][1]);

    subscript = 1;

    //fill the middle function y values
    for(int i = 0; i < coords.length-2; i++){
      
      //used to determine which row to put values on
      int n = i * 4;

      yMatrix.set(n + 2, 0, coords[subscript][1]);
      yMatrix.set(n + 3, 0, 0);
      yMatrix.set(n + 4, 0, 0);
      yMatrix.set(n + 5, 0, coords[subscript][1]);

      subscript++;

    }

    //Manually fill the 2nd to last line
    yMatrix.set(yMatrix.numRows()-2, 0, coords[subscript][1]);

    //Set the 2nd specified slope as the last line
    yMatrix.set(yMatrix.numRows()-1, 0, endingSlope);



    coefMatrix = megaMatrix.invert().mult(yMatrix);




    //Put megaMatrix onto SmartDashboard
    String[] megaString = matrixToStringArray(megaMatrix);

    for(int i = 1; i < megaString.length+1; i++){
      SmartDashboard.putString(theAlphabet[i-1] + " Row " + i , megaString[i-1]);
    }


    //Put yMatrix onto SmartDashboard
    String yString = "";

    for(int i = 0; i < yMatrix.numRows(); i++){
      yString = yString + String.valueOf(yMatrix.get(i, 0)) + ", ";
    }
    SmartDashboard.putString("Y Matrix", yString);


    //Put coefMatrix onto SmartDashboard
    String coefString = "";

    for(int i = 0; i < coefMatrix.numRows(); i++){
      coefString = coefString + String.valueOf(coefMatrix.get(i, 0)) + ", ";
    }
    SmartDashboard.putString("Z Coef Matrix", coefString);




  }

  


  //Converts a SimpleMatrix to a String[]
  public String[] matrixToStringArray(SimpleMatrix matrix){

    String[] stringArray = new String[matrix.numRows()];
    
    for(int i = 0; i < matrix.numRows(); i++){
      stringArray[i] = " ";
      for(int j = 0; j < matrix.numCols(); j++){
        stringArray[i] = stringArray[i] + String.valueOf(matrix.get(i, j)) + ", ";
      }
    }
    
    return stringArray;
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
