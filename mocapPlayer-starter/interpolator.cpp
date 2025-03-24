#include "interpolator.h"
#include "motion.h"
#include "types.h"
#include <ctime>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}



// == Initalization Functions ==
// =============================
Interpolator::~Interpolator()
{
}



// == Utilitary Functions ==
// =========================
void Interpolator::MultiplyMatrix3x3(matrix3x3 M1, matrix3x3 M2, matrix3x3 M3) {
    for (int i = 0; i <= 2; i++)
        for (int j = 0; j <= 2; j++)
            M3[i][j] = 0;

    for (int i = 0; i <= 2; i++)
        for (int j = 0; j <= 2; j++)
            for (int k = 0; k <= 2; k++)
                M3[i][j] += (M1[i][k] * M2[k][j]);
}

void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}



// == Conversion Functions ==
// ==========================
void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
    // (X,Y,Z) respectively
    double radians[3] = { ConvertDegreeToRadian(angles[0]), ConvertDegreeToRadian(angles[1]), ConvertDegreeToRadian(angles[2]) };
    matrix3x3 M1M2, result;

    // Z rotation
    matrix3x3 M1 = {
        { cos(radians[2]), 	-sin(radians[2]), 	0.0 },
        { sin(radians[2]), 	cos(radians[2]), 	0.0 },
        { 0.0, 				0.0, 				1.0 }
    };

    // Y rotation
    matrix3x3 M2 = {
        { cos(radians[1]), 	0.0, 	sin(radians[1]) },
        { 0.0, 				1.0, 	0.0 },
        {-sin(radians[1]), 	0.0, 	cos(radians[1]) }
    };

    // X rotation
    matrix3x3 M3 = {
        {	1.0, 	0.0, 				0.0 },
        { 	0.0, 	cos(radians[0]), 	-sin(radians[0]) },
        {	0.0, 	sin(radians[0]), 	cos(radians[0]) }
    };
    
    // M1 * M2 * M3 = R
    MultiplyMatrix3x3(M1, M2, M1M2);
    MultiplyMatrix3x3(M1M2, M3, result);

    // Copy Output
    for (int i = 0; i <= 2; i++)
        for (int j = 0; j <= 2; j++)
            R[j + (3 * i)] = result[i][j];
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double>& q)
{
    // Euler to Rotation
    double t_matrix[9];
    Euler2Rotation(angles, t_matrix);

    // Rotaton to Quaternion
    q = Quaternion<double>::Matrix2Quaternion(t_matrix);
    q.Normalize();
}

void Interpolator::Quaternion2Euler(Quaternion<double>& q, double angles[3])
{
    // Quaternion to Rotation
    double t_matrix[9];
    q.Quaternion2Matrix(t_matrix);

    // Rotation to Euler
    Rotation2Euler(t_matrix, angles);
}



// == Interpolation Functions ==
// =============================
void Interpolator::LinearInterpolationEuler(Motion* pInputMotion, Motion* pOutputMotion, int N)
{
    // Set-up CSV
    std::ofstream outfile("LinearInterpolationEuler.txt");

    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    int startKeyframe = 0;
    clock_t startClock = clock();

    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;

        Posture* startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture* endPosture = pInputMotion->GetPosture(endKeyframe);

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        // interpolate in between
        for (int frame = 1; frame <= N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N + 1);

            // interpolate root position
            interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;

            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1 - t) + endPosture->bone_rotation[bone] * t;
                
            }
            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        startKeyframe = endKeyframe;
    }

    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));

    // debug printing
    double timeTaken = clock() - startClock;
    printf("Computation Time: %lf\n", timeTaken / (double)CLOCKS_PER_SEC);
}

void Interpolator::LinearInterpolationQuaternion(Motion* pInputMotion, Motion* pOutputMotion, int N)
{
    int inputLength = pInputMotion->GetNumFrames();
    int startKeyframe = 0;
    // quat store
    Quaternion<double> startKeyframeQuaternion, endKeyframeQuaternion;
    clock_t startClock = clock();

    // Loop through key frames
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;

        // Write to first keyframe
        Posture* startKeyframePosture = pInputMotion->GetPosture(startKeyframe);
        pOutputMotion->SetPosture(startKeyframe, *startKeyframePosture);

        // Write to last keyframe
        Posture* endKeyframePosture = pInputMotion->GetPosture(endKeyframe);
        pOutputMotion->SetPosture(endKeyframe, *endKeyframePosture);

        // Loop through inbetween frames
        for (int frameIndex = 1; frameIndex <= N; frameIndex++) 
        {
            // Time positon
            double t = 1.0 * frameIndex / (N + 1);

            // Interpolate root position
            Posture posture;
            posture.root_pos = startKeyframePosture->root_pos * (1 - t) + endKeyframePosture->root_pos * t;

            // Interpolate bone rotation
            for (int boneIndex = 0; boneIndex < MAX_BONES_IN_ASF_FILE; boneIndex++) 
            {
                Euler2Quaternion(startKeyframePosture->bone_rotation[boneIndex].p, startKeyframeQuaternion);
                Euler2Quaternion(endKeyframePosture->bone_rotation[boneIndex].p, endKeyframeQuaternion);

                // Slerp
                Quaternion<double> slerp;
                slerp = Slerp(t, startKeyframeQuaternion, endKeyframeQuaternion);

                Quaternion2Euler(slerp, posture.bone_rotation[boneIndex].p);
            }
            pOutputMotion->SetPosture(startKeyframe + frameIndex, posture);
        }
        startKeyframe = endKeyframe;
    }

    // copy remaining keyframes
    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));

    // debug printing
    double timeTaken = clock() - startClock;
    printf("Computation Time: %lf\n", timeTaken / (double)CLOCKS_PER_SEC);
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    int inputLength = pInputMotion->GetNumFrames();;
    int startKeyframe = 0; 
    // quat store
    Quaternion<double> firstKeyframeQuaternion, lastKeyframeQuaternion;

    // Start the timer
    clock_t startClock = clock();

    while (startKeyframe + N + 1 < inputLength)
    {
        //Frame P4, Start is P1
        int endKeyframe = startKeyframe + N + 1;
        int prevKeyframe, nextKeyframe;

        // Set up frame P2 and P3
        prevKeyframe = startKeyframe - N - 1;
        if (startKeyframe == 0) 
            prevKeyframe = startKeyframe;

        nextKeyframe = endKeyframe + N + 1;
        if (endKeyframe + N + 1 >= inputLength) 
            nextKeyframe = endKeyframe;

        vector* previousKeyframeRotation = pInputMotion->GetPosture(prevKeyframe)->bone_rotation;
        vector* nextKeyframeRotation = pInputMotion->GetPosture(nextKeyframe)->bone_rotation;

        // Write to first keyframe
        Posture* startKeyframePosture = pInputMotion->GetPosture(startKeyframe);
        pOutputMotion->SetPosture(startKeyframe, *startKeyframePosture);

        // Write to last keyframe
        Posture* endKeyframePosture = pInputMotion->GetPosture(endKeyframe);
        pOutputMotion->SetPosture(endKeyframe, *endKeyframePosture);

        // Loop through inbetween frames
        for (int frameIndex = 1; frameIndex <= N; frameIndex++) {
            Posture posture;

            // Interpolate root position
            double t = 1.0 * frameIndex / (N + 1);
            posture.root_pos = startKeyframePosture->root_pos * (1 - t) + endKeyframePosture->root_pos * t;

            // Interpolate bone rotation
            // Smoothing equation source: https://omaraflak.medium.com/b%C3%A9zier-interpolation-8033e9a262c2
            for (int boneIndex = 0; boneIndex < MAX_BONES_IN_ASF_FILE; boneIndex++) {
                // P1-P4
                vector firstKeyframeEuler = startKeyframePosture->bone_rotation[boneIndex];    // P1
                vector prevKeyframeEuler = previousKeyframeRotation[boneIndex];                // p2
                vector nextKeyframeEuler = nextKeyframeRotation[boneIndex];                    // P3
                vector lastKeyframeEuler = endKeyframePosture->bone_rotation[boneIndex];      // P4

                vector tempEulerResult = lastKeyframeEuler + lastKeyframeEuler - firstKeyframeEuler;

                // Adustment for smoother interpolation
                prevKeyframeEuler = firstKeyframeEuler + firstKeyframeEuler - prevKeyframeEuler;
                prevKeyframeEuler = (prevKeyframeEuler + lastKeyframeEuler) * 0.5;
                prevKeyframeEuler = firstKeyframeEuler + (prevKeyframeEuler - firstKeyframeEuler) / 3.0;

                // Adustment for smoother interpolation
                nextKeyframeEuler = (tempEulerResult + nextKeyframeEuler) * 0.5;
                nextKeyframeEuler = lastKeyframeEuler + lastKeyframeEuler - nextKeyframeEuler;
                nextKeyframeEuler = lastKeyframeEuler + (nextKeyframeEuler - lastKeyframeEuler) / 3.0;

                // Rotiation
                posture.bone_rotation[boneIndex] = DeCasteljauEuler(t, firstKeyframeEuler, prevKeyframeEuler, nextKeyframeEuler, lastKeyframeEuler);
            }
            pOutputMotion->SetPosture(startKeyframe + frameIndex, posture);
        }
        startKeyframe = endKeyframe;
    }

    // copy remaining keyframes
    for (int frameIndex = startKeyframe + 1; frameIndex < inputLength; frameIndex++)
        pOutputMotion->SetPosture(frameIndex, *(pInputMotion->GetPosture(frameIndex)));

    // debug printing
    double timeTaken = clock() - startClock;
    printf("Computation Time: %lf\n", timeTaken / (double)CLOCKS_PER_SEC);
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    int inputLength = pInputMotion->GetNumFrames();;
    int startKeyframe = 0;
    // quat store
    Quaternion<double> firstKeyframeQuaternion, lastKeyframeQuaternion;

    // Start the timer
    clock_t startClock = clock();

    while (startKeyframe + N + 1 < inputLength)
    {
        //Frame P4, Start is P1
        int endKeyframe = startKeyframe + N + 1;
        int prevKeyframe, nextKeyframe;

        // Set up frame P2 and P3
        prevKeyframe = startKeyframe - N - 1;
        if (startKeyframe == 0)
            prevKeyframe = startKeyframe;

        nextKeyframe = endKeyframe + N + 1;
        if (endKeyframe + N + 1 >= inputLength)
            nextKeyframe = endKeyframe;

        vector* previousKeyframeRotation = pInputMotion->GetPosture(prevKeyframe)->bone_rotation;
        vector* nextKeyframeRotation = pInputMotion->GetPosture(nextKeyframe)->bone_rotation;

        // Write to first keyframe
        Posture* startKeyframePosture = pInputMotion->GetPosture(startKeyframe);
        pOutputMotion->SetPosture(startKeyframe, *startKeyframePosture);

        // Write to last keyframe
        Posture* endKeyframePosture = pInputMotion->GetPosture(endKeyframe);
        pOutputMotion->SetPosture(endKeyframe, *endKeyframePosture);

        // Loop through inbetween frames
        for (int frameIndex = 1; frameIndex <= N; frameIndex++) {
            Posture posture;

            // Interpolate root position
            double t = 1.0 * frameIndex / (N + 1);
            posture.root_pos = startKeyframePosture->root_pos * (1 - t) + endKeyframePosture->root_pos * t;

            // Interpolate bone rotation
            // Smoothing equation source: https://omaraflak.medium.com/b%C3%A9zier-interpolation-8033e9a262c2
            for (int boneIndex = 0; boneIndex < MAX_BONES_IN_ASF_FILE; boneIndex++) {
                // P1
                double startKeyframeAngles[3];
                Quaternion<double> firstKeyframeQuaternion;
                startKeyframePosture->bone_rotation[boneIndex].getValue(startKeyframeAngles);
                Euler2Quaternion(startKeyframeAngles, firstKeyframeQuaternion);

                // P4
                double endKeyframeAngles[3];
                Quaternion<double> endKeyframeQuaternion;
                endKeyframePosture->bone_rotation[boneIndex].getValue(endKeyframeAngles);
                Euler2Quaternion(endKeyframeAngles, endKeyframeQuaternion);

                // P2
                double previousKeyframeAngles[3];
                Quaternion<double> previousKeyframeQuaternion;
                previousKeyframeRotation[boneIndex].getValue(previousKeyframeAngles);
                Euler2Quaternion(previousKeyframeAngles, previousKeyframeQuaternion);

                // P3
                double nextKeyframeAngles[3];
                Quaternion<double> nextKeyframeQuaternion;
                nextKeyframeRotation[boneIndex].getValue(nextKeyframeAngles);
                Euler2Quaternion(nextKeyframeAngles, nextKeyframeQuaternion);

                // Adustment for smoother interpolation
                previousKeyframeQuaternion = Double(previousKeyframeQuaternion, firstKeyframeQuaternion);
                previousKeyframeQuaternion = Slerp(0.5, previousKeyframeQuaternion, endKeyframeQuaternion);
                previousKeyframeQuaternion = Slerp(1.0 / 3.0, firstKeyframeQuaternion, previousKeyframeQuaternion);

                // Adustment for smoother interpolation
                Quaternion<double> tempQuaternion = Double(firstKeyframeQuaternion, endKeyframeQuaternion);
                nextKeyframeQuaternion = Slerp(0.5, tempQuaternion, nextKeyframeQuaternion);
                nextKeyframeQuaternion = Double(nextKeyframeQuaternion, endKeyframeQuaternion);
                nextKeyframeQuaternion = Slerp(1.0 / 3.0, endKeyframeQuaternion, nextKeyframeQuaternion);

                Quaternion<double> resultQuaternionAngle = DeCasteljauQuaternion(t, firstKeyframeQuaternion, previousKeyframeQuaternion, nextKeyframeQuaternion, endKeyframeQuaternion);

                // Conversion to Euler
                double resultEulerAngle[3];
                Quaternion2Euler(resultQuaternionAngle, resultEulerAngle);
                posture.bone_rotation[boneIndex].setValue(resultEulerAngle);
            }
            pOutputMotion->SetPosture(startKeyframe + frameIndex, posture);
        }
        startKeyframe = endKeyframe;
    }

    // copy remaining keyframes
    for (int frameIndex = startKeyframe + 1; frameIndex < inputLength; frameIndex++)
        pOutputMotion->SetPosture(frameIndex, *(pInputMotion->GetPosture(frameIndex)));

    // debug printing
    double timeTaken = clock() - startClock;
    printf("Computation Time: %lf\n", timeTaken / (double)CLOCKS_PER_SEC);
}



// == Calculation Functions ==
// ===========================
Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
  Quaternion<double> result, resultQEnd_;
  double dotProduct = qStart.Gets() * qEnd_.Gets() + qStart.Getx() * qEnd_.Getx() + qStart.Gety() * qEnd_.Gety() + qStart.Getz() * qEnd_.Getz();

  // QEnd_ Correction
  if (dotProduct >= 0) 
  {
      resultQEnd_ = qEnd_;
  }
  else 
  {
      dotProduct *= -1;
      resultQEnd_.Set(-qEnd_.Gets(), -qEnd_.Getx(), -qEnd_.Gety(), -qEnd_.Getz());
  }

  // Slerp calculation
  float theta = acosf(dotProduct);
  if (theta != 0) 
  {
      result = ((sinf((1 - t) * theta) / sinf(theta)) * qStart + (sinf(t * theta) / sinf(theta)) * resultQEnd_);
      result.Normalize();
      return result;
  }
  else 
  {
      return qEnd_;
  }
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
    Quaternion<double> result;
    double dotProduct = p.Gets() * q.Gets() + p.Getx() * q.Getx() + p.Gety() * q.Gety() + p.Getz() * q.Getz();

    // calculates rotational diffrence
    result = 2 * (dotProduct)*q - p; // res = 2(p * q) * q - p
    return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
    // 3rd Order Bezier Curve
    vector q0 = p1 * t + p0 * (1 - t);
    vector q1 = p2 * t + p1 * (1 - t);
    vector q2 = p3 * t + p2 * (1 - t);

    // 2nd Order Bezier Curve
    vector r0 = q1 * t + q0 * (1 - t);
    vector r1 = q2 * t + q1 * (1 - t);

    // 1st Order Bezier Curve
    vector result = r1 * t + r0 * (1 - t);

    return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
// 3rd Order Bezier Curve
    Quaternion<double> q0 = Slerp(t, p0, p1);
    Quaternion<double> q1 = Slerp(t, p1, p2);
    Quaternion<double> q2 = Slerp(t, p2, p3);

    // 2nd Order Bezier Curve
    Quaternion<double> r0 = Slerp(t, q0, q1);
    Quaternion<double> r1 = Slerp(t, q1, q2);

    // 1st Order Bezier Curve
    Quaternion<double> result = Slerp(t, r0, r1);
    return result;

}

