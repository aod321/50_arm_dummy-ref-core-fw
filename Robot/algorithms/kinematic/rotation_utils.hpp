#ifndef ROTATION_UTILS_HPP
#define ROTATION_UTILS_HPP

#include <cmath>

/**
 * @brief Utility class for handling rotation matrices
 * @details Implements functions to create and manipulate rotation matrices for robotic applications
 */
class RotationUtils {
public:
    /**
     * @brief Create a complete rotation matrix from the first two columns
     * @details This method follows common practice in robotics (ref: https://arxiv.org/pdf/1812.07035)
     *          1. Takes the first two columns of the rotation matrix
     *          2. Performs Gram-Schmidt orthogonalization
     *          3. Computes the third column as cross product
     * 
     * @param r11,r21,r31 First column of the rotation matrix
     * @param r12,r22,r32 Second column of the rotation matrix
     * @param R Output 3x3 rotation matrix (array of 9 elements in row-major order)
     */
    static void CreateFromTwoColumns(float r11, float r21, float r31,
                                    float r12, float r22, float r32,
                                    float* R) {
        // Store the first two columns
        R[0] = r11; R[3] = r12;
        R[1] = r21; R[4] = r22;
        R[2] = r31; R[5] = r32;
        
        // Perform Gram-Schmidt orthogonalization for the first two columns
        OrthogonalizeFirstTwoColumns(R);
        
        // Compute the third column as cross product of first two columns
        ComputeThirdColumn(R);
    }

private:
    /**
     * @brief Perform Gram-Schmidt orthogonalization on the first two columns
     * @details Ensures the first two columns are orthonormal
     * @param R Rotation matrix to be orthogonalized
     */
    static void OrthogonalizeFirstTwoColumns(float* R) {
        // Normalize first column
        float norm1 = std::sqrt(R[0] * R[0] + R[1] * R[1] + R[2] * R[2]);
        if(norm1 > 1e-6f) {
            R[0] /= norm1;
            R[1] /= norm1;
            R[2] /= norm1;
        }

        // Make second column orthogonal to first column
        float dot = R[0] * R[3] + R[1] * R[4] + R[2] * R[5];
                   
        R[3] -= dot * R[0];
        R[4] -= dot * R[1];
        R[5] -= dot * R[2];

        // Normalize second column
        float norm2 = std::sqrt(R[3] * R[3] + R[4] * R[4] + R[5] * R[5]);
        if(norm2 > 1e-6f) {
            R[3] /= norm2;
            R[4] /= norm2;
            R[5] /= norm2;
        }
    }

    /**
     * @brief Compute the third column as cross product of first two columns
     * @details R3 = R1 × R2, ensures right-handed coordinate system
     * @param R Rotation matrix whose third column will be computed
     */
    static void ComputeThirdColumn(float* R) {
        // R3 = R1 × R2 (cross product)
        R[6] = R[1] * R[5] - R[2] * R[4];
        R[7] = R[2] * R[3] - R[0] * R[5];
        R[8] = R[0] * R[4] - R[1] * R[3];
    }
};

#endif // ROTATION_UTILS_HPP 