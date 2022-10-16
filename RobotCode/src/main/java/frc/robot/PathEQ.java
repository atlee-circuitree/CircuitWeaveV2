// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* 
This will eventually become a custom object that can store a chunk of coeffecients from PathGenerator
You should also be able to take the derivative of that function and do some other useful stuff with it    
*/

public class PathEQ {

    private double[][] xCoefs;
    private double[][] yCoefs;

    String[] theAlphabet = {"A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "X", "Y", "Z"};
  

    /** 
    * @apiNote The [n][0] values of each input array should be the U values denoting where each function ends, NOT THE 1ST X/Y COEFFICIENT
    */
    public PathEQ(double[][] xCoeffecients, double[][] yCoeffecients){

        //Coefs get fed smallest-largest exponent
        xCoefs = xCoeffecients;
        yCoefs = yCoeffecients;

    }


    public void dashboardYCoefs(){
        for(int i = 0; i < yCoefs.length; i++){
            SmartDashboard.putNumberArray(String.valueOf(i), yCoefs[i]);
        }
    }


    /**
     * @param uValue Input a U value
     * @return Returns the {X,Y} output for the specified U value
    */
    public double[] solve(double uValue){

        //Figure out which chunk of coefs contains the U value that we are searching for

        double[] subXCoefs = new double[5];
        double[] subYCoefs = new double[5];

        //If the requested uValue is larger than the defined function, grab the last row of coefs
        if(uValue >= getFinalUValue()){
            subXCoefs = xCoefs[xCoefs.length-1];
            subYCoefs = yCoefs[yCoefs.length-1];
            uValue = getFinalUValue();
            //SmartDashboard.putNumber("BREAKPOINT", 1);
        }
        else{
            //Otherwise seacrh for which row of coefs contains the correct u value
            for(int i = 0; i < xCoefs.length; i++){
                if(xCoefs[i][0] >= uValue){
                    subXCoefs = xCoefs[i];
                    subYCoefs = yCoefs[i];
                    break;
                }
            }
        }

        double[] result = {0,0};
        int powCounter = 0;

        //Go through each X term and add them together
        //i starts at one for each loop bc the 0 index value in subX and subY Coefs is a u value, not a coef
        for(int i = 1; i < subXCoefs.length; i++){
            result[0] = result[0] + (subXCoefs[i] * Math.pow(uValue, powCounter));
            powCounter++;
        }
        powCounter = 0;

        //SmartDashboard.putNumber("X result", result[0]);
        
        //Go through each Y term and add them together
        for(int i = 1; i < subYCoefs.length; i++){
            result[1] = result[1] + (subYCoefs[i] * Math.pow(uValue, powCounter));
            powCounter++;
        }

        //SmartDashboard.putNumber("Y result", result[1]);
        //SmartDashboard.putNumberArray("subY coefs", subYCoefs);
        
        return result;

    }




    /**
     * @param point1 Pass in as {X1,Y1}
     * @param point2 Pass in as {X2,Y2}
     * @return The slope of the line passing through both points
     * @apiNote This method will return Double.POSITIVE_INFINITY or Double.NEGATIVE_INFINITY if the slope is undefined (vertical)
     */

    public double slope(double[] point1, double[] point2){
        return (point2[1] - point1[1]) / (point2[0] - point1[0]);
    }

    /**
     * @param point1 Pass in as {X1,Y1}
     * @param point2 Pass in as {X2,Y2}
     * @return {Run, Rise}
     */

    public double[] slopeRiseRun(double[] point1, double[] point2){
        double[] riseRunArray = {point2[0]-point1[0], point2[1]-point1[1]};
        return riseRunArray;
    }

    public double getFinalUValue(){
        return xCoefs[xCoefs.length-1][0];
    }

    public double[] getLastRowofXCoefs(){
        return xCoefs[xCoefs.length-1];
    }
    


}
