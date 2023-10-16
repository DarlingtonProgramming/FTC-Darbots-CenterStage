package org.firstinspires.ftc.team4100.Darvinci.util;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

public class MultiVariableKalmanFilter{
    public double[][] x; // your initial state
    public double[][] Q; // your model covariance
    public double[][] R; // your sensor covariance
    public double[][] p; // your initial covariance guess
    public double[][] K; // your initial Kalman gain guess

    public double[][] x_previous;
    public double[][] p_previous;
    public double[][] u;
    public double[][] z;
    public double[][] A;
    public double[][] B;
    public double[][] H;
    public double[][] I;

    public MultiVariableKalmanFilter(double[][] x, double[][] Q, double[][] R, double[][] p, double[][] K, double[][] u, double[][] z, double[][] A, double[][] B, double[][] H, double[][] I) {
        this.x = x;
        this.Q = Q;
        this.R = R;
        this.p = p;
        this.K = K;
        this.x_previous = x;
        this.p_previous = p;
        this.u = u;
        this.z = z;
        this.A = A;
        this.B = B;
        this.H = H;
        this.I = I;
    }
    // make a function that returns the kalman gain
    public static double[][] calculateK(double[][] p, double[][] R) {
        return MultiVariableKalmanFilter.matrixDivision(p,(MultiVariableKalmanFilter.matrixAddition(p, R)));
    }
    // return covariance
    public double[][] getCovariance() {
        return this.p;
    }
    // return state
    public double[][] getState() {
        return this.x;
    }
    // return measurement
    public double[][] getMeasurement() {
        return this.z;
    }
    // return model covariance
    public double[][] getModelCovariances() {
        return this.Q;
    }
    // return sensor covariance
    public double[][] getSensorCovariances() {
        return this.R;
    }
    // return state covariance
    public double[][] getStateCovariance() {
        return this.p;
    }
    public static void printMatrix(double[][] matrix) {
        RealMatrix realMatrix = MatrixUtils.createRealMatrix(matrix);
        for (int i = 0; i < realMatrix.getRowDimension(); i++) {
            for (int j = 0; j < realMatrix.getColumnDimension(); j++) {
                System.out.print(realMatrix.getEntry(i, j) + " ");
            }
            System.out.println();
        }
    }
    private static double[][] matrixAddition(double[][] matrix1, double[][] matrix2) {
        RealMatrix realMatrix1 = MatrixUtils.createRealMatrix(matrix1);
        RealMatrix realMatrix2 = MatrixUtils.createRealMatrix(matrix2);
        RealMatrix resultMatrix = realMatrix1.add(realMatrix2);
        return resultMatrix.getData();
    }

    // Function to perform matrix multiplication
    private static double[][] matrixMultiplication(double[][] matrix1, double[][] matrix2) {
        RealMatrix realMatrix1 = MatrixUtils.createRealMatrix(matrix1);
        RealMatrix realMatrix2 = MatrixUtils.createRealMatrix(matrix2);
        RealMatrix resultMatrix = realMatrix1.multiply(realMatrix2);
        return resultMatrix.getData();
    }

    // Function to perform matrix subtraction
    private static double[][] matrixSubtraction(double[][] matrix1, double[][] matrix2) {
        RealMatrix realMatrix1 = MatrixUtils.createRealMatrix(matrix1);
        RealMatrix realMatrix2 = MatrixUtils.createRealMatrix(matrix2);
        RealMatrix resultMatrix = realMatrix1.subtract(realMatrix2);
        return resultMatrix.getData();
    }
    public static double[][] matrixDivision(double[][] matrixA, double[][] matrixB) {
        if (matrixA.length != matrixB.length || matrixA[0].length != matrixB[0].length) {
            throw new IllegalArgumentException("Matrices must have the same dimensions.");
        }

        int rows = matrixA.length;
        int cols = matrixA[0].length;
        double[][] resultMatrix = new double[rows][cols];

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                resultMatrix[i][j] = matrixA[i][j] / matrixB[i][j];
            }
        }

        return resultMatrix;
    }

    // Function to get the transpose of a matrix
    private double[][] transpose(double[][] matrix) {
        RealMatrix realMatrix = MatrixUtils.createRealMatrix(matrix);
        RealMatrix transposeMatrix = realMatrix.transpose();
        return transposeMatrix.getData();
    }

    // Function to get the inverse of a matrix
    private double[][] inverse(double[][] matrix) {
        RealMatrix realMatrix = MatrixUtils.createRealMatrix(matrix);
        RealMatrix inverseMatrix = MatrixUtils.inverse(realMatrix);
        return inverseMatrix.getData();
    }
    public void update(double[][] measurements) {
        // Step 1: Prediction Step
        // Predict the next state estimate based on the model (A) and control input (B)
        double[][] x_priori = matrixAddition(matrixMultiplication(A, x), matrixMultiplication(B, u));

        // Predict the error covariance matrix
        double[][] p_priori = matrixAddition(matrixMultiplication(matrixMultiplication(A, p), transpose(A)), Q);

        // Step 2: Correction Step for each sensor
        for (int i = 0; i < measurements.length; i++) {
            // Get the measurement for the current sensor
            double[][] z_i = { { measurements[i][0] } };

            // Get the sensor-specific covariance
            //double[][] R_i = R[i];

            // Calculate the Kalman gain for the current sensor
            double[][] k_i = matrixMultiplication(matrixMultiplication(p_priori, transpose(H)), inverse(matrixAddition(matrixMultiplication(matrixMultiplication(H, p_priori), transpose(H)), R)));

            // Calculate the difference between the actual measurement (z) and the predicted measurement based on the prior state estimate (x_priori)
            double[][] innovation = matrixSubtraction(z_i, matrixMultiplication(H, x_priori));

            // Update the state estimate based on the Kalman gain and innovation for the current sensor
            double[][] x_i = matrixAddition(x_priori, matrixMultiplication(k_i, innovation));

            // Update the error covariance matrix based on the Kalman gain for the current sensor
            double[][] p_i = matrixMultiplication(matrixSubtraction(I, matrixMultiplication(k_i, H)), p_priori);

            // Update the state estimate and error covariance matrix for the next iteration
            x_priori = x_i;
            p_priori = p_i;
        }

        // Update the overall state estimate and error covariance matrix for all sensors
        x = x_priori;
        p = p_priori;

        // Update the previous state estimate and error covariance matrix for the next iteration
        x_previous = x;
        p_previous = p;
    }

    // Function to get the filtered value (current state estimate)
    public double[][] getFilteredValue() {
        return x;
    }



}
