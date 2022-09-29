// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/* 
This will eventually become a custom object that can store a chunk of coeffecients from PathGenerator
You should also be able to take the derivative of that function and do some other useful stuff with it    
*/

public class PathEQ {

    //
    private double[] xCoefs;
    private double[] yCoefs;

    /** 
    * @param xCoeffecients The [n][0] values of this array should be the U values denoting where each function ends, NOT THE 1ST X COEFFICIENT
    * @param yCoeffecients The [n][0] values of this array should be the U values denoting where each function ends, NOT THE 1ST Y COEFFICIENT
    */
    public PathEQ(double[] xCoeffecients, double[] yCoeffecients){

        //Coefs get fed smallest-largest exponent
        xCoefs = xCoeffecients;
        yCoefs = yCoeffecients;

    }


    public double[] solve(double uValue){

        double[] result = {0,0};
        int powCounter = 0;

        //Go through each X term and add them together
        for(int i = 0; i < xCoefs.length; i++){
            result[0] = result[0] + Math.pow(xCoefs[i], powCounter);
            powCounter++;
        }
        powCounter = 0;
        
        //Go through each Y term and add them together
        for(int i = 0; i < yCoefs.length; i++){
            result[1] = result[1] + Math.pow(yCoefs[i], powCounter);
            powCounter++;
        }
        
        return result;

    }


}
