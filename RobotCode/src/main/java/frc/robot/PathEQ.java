// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/* 
This will eventually become a custom object that can store a chunk of coeffecients from PathGenerator
You should also be able to take the derivative of that function and do some other useful stuff with it    
*/

public class PathEQ {

    private double[][] xCoefs;
    private double[][] yCoefs;

    /** 
    * @apiNote The [n][0] values of each input array should be the U values denoting where each function ends, NOT THE 1ST X/Y COEFFICIENT
    */
    public PathEQ(double[][] xCoeffecients, double[][] yCoeffecients){

        //Coefs get fed smallest-largest exponent
        xCoefs = xCoeffecients;
        yCoefs = yCoeffecients;

    }



    /**
     * @param uValue Input a U value
     * @return Returns the {X,Y} output for the specified U value
    */
    public double[] solve(double uValue){


        //Figure out which chunk of coefs contains the U value that we are searching for

        double[] subXCoefs = new double[5];
        double[] subYCoefs = new double[5];

        for(int i = 0; i < xCoefs.length-1; i++){
            if(xCoefs[i][0] >= uValue){
                subXCoefs = xCoefs[i];
                subYCoefs = yCoefs[i];
                break;
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
        
        //Go through each Y term and add them together
        for(int i = 1; i < subYCoefs.length; i++){
            result[1] = result[1] + (subYCoefs[i] * Math.pow(uValue, powCounter));
            powCounter++;
        }
        
        return result;

    }




    /**
     * @param point1 Pass in as {X1,Y1}
     * @param point2 Pass in as {X2,Y2}
     * @return The slope of the line passing through both points
     * @apiNote This method will return Double.POSITIVE_INFINITY or Double.NEGATIVE_INFINITY if the slope is undefined (vertical)
     */

    public double slope(double[] point1, double[] point2){
        return(point2[1] - point1[1]) / (point2[0] - point1[0]);
    }


    public double getFinalUValue(){
        return xCoefs[xCoefs.length-1][0];
    }

    public double[] getLastRowofXCoefs(){
        return xCoefs[xCoefs.length-1];
    }
    


}
