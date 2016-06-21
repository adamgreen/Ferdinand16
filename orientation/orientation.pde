/*  Copyright (C) 2016  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
import processing.serial.*;

// Can base the 3D orientation of the sensor on gyros alone, accelerometer/magnetometer
// combination or Kalman filter to fuse all 3 sources together. 
final int GYRO = 0;
final int ACCEL_MAG = 1;
final int KALMAN = 2;
final int EMBEDDED = 3;

final float g_floatThreshold = 0.00001f;

Serial                   g_serial;
HeadingSensorCalibration g_calibration;
HeadingSensor            g_headingSensor;
boolean                  g_zeroRotation = false;
float                    g_cameraAngle = 0.0f;
int                      g_samples = 0;
int                      g_lastSampleCount;
float[]                  g_rotationQuaternion = {1.0f, 0.0f, 0.0f, 0.0f};
int                      g_rotationSource = ACCEL_MAG;
int                      g_fontHeight;
PFont                    g_font;
final float              g_initVariance = 1.0E-4;
PMatrix3D                g_kalmanP = new PMatrix3D(g_initVariance,           0.0f,           0.0f,           0.0f,
                                                             0.0f, g_initVariance,           0.0f,           0.0f,
                                                             0.0f,           0.0f, g_initVariance,           0.0f, 
                                                             0.0f,           0.0f,           0.0f, g_initVariance);
final float             g_gyroThreshold = 4.0f * (1.0f / 14.375f);

// Used for calculating statistics (variance in particular) of sensors effects on
// their corresponding quaternions.
double[]                 g_gyroSum = {0.0, 0.0, 0.0, 0.0};
double[]                 g_gyroSquaredSum = {0.0, 0.0, 0.0, 0.0};
int                      g_gyroSampleCount = 0;
final int                g_gyroSamplesToCount = 100000;
double[]                 g_accelMagSum = {0.0f, 0.0f, 0.0f, 0.0f};
double[]                 g_accelMagSquaredSum = {0.0f, 0.0f, 0.0f, 0.0f};
int                      g_accelMagSampleCount = 0;
final int                g_accelMagSamplesToCount = 100000;


void setup() 
{
  size(1024, 768, OPENGL);
  fill(255, 0, 0);

  g_font = loadFont("Monaco-24.vlw");
  textFont(g_font);
  g_fontHeight = int(textAscent() + textDescent() + 0.5f);

  ConfigFile configFile = new ConfigFile(System.getenv("USER") + ".config");
  Serial port = new Serial(this, configFile.getString("compass.port"), 230400);
  g_calibration = new HeadingSensorCalibration();
  g_calibration.accelMin = configFile.getIntVector("compass.accelerometer.min");
  g_calibration.magMin = configFile.getIntVector("compass.magnetometer.min");
  g_calibration.accelMax = configFile.getIntVector("compass.accelerometer.max");
  g_calibration.magMax = configFile.getIntVector("compass.magnetometer.max");
  g_calibration.gyroCoefficientA = configFile.getFloatVector("compass.gyro.coefficient.A");
  g_calibration.gyroCoefficientB = configFile.getFloatVector("compass.gyro.coefficient.B");
  g_calibration.gyroScale = configFile.getFloatVector("compass.gyro.scale");
  
  Heading filterWidths = new Heading(16, 16, 16, 16, 16, 16, 0, 0, 0, 0);
  g_headingSensor = new HeadingSensor(port, g_calibration, filterWidths);
  g_lastSampleCount = millis();
  
  g_serial = port;
}

void draw()
{
  background(100);

  int elapsedTime = millis() - g_lastSampleCount;
  if (elapsedTime > 10000)
  {
    println(g_samples / (elapsedTime / 1000));
    g_lastSampleCount = millis();
    g_samples = 0;
  }
  
  // Convert rotation quaternion into a 4x4 rotation matrix to be used for rendering.
  PMatrix3D rotationMatrix = quaternionToMatrix(g_rotationQuaternion);
  
  // Calculating yaw/pitch/roll directly from quaternion.
  float w = g_rotationQuaternion[0];
  float x = g_rotationQuaternion[1];
  float y = g_rotationQuaternion[2];
  float z = g_rotationQuaternion[3];

  float yaw = atan2(2*(x*z+y*w), 1-2*(x*x+y*y));
  float pitch = asin(-2*(y*z-x*w));
  float roll = atan2(2*(x*y+z*w), 1-2*(x*x+z*z));

  float headingAngle = yaw;
  
  // If the user has pressed the space key, then move the camera to face the device front.
  if (g_zeroRotation)
  {
    PVector cam = new PVector(0, (height / 2.0) / tan(radians(30.0)));
    g_cameraAngle = -headingAngle;
    cam.rotate(g_cameraAngle);
    camera(cam.x + width/2.0, height/2.0, cam.y, width/2.0, height/2.0, 0, 0, 1, 0);
    g_zeroRotation = false;
  }

  // Rotate the rendered box using the calculated rotation matrix.
  pushMatrix();
    translate(width / 2, height / 2, 0);
    scale(5.0f, 5.0f, 5.0f);
    applyMatrix(rotationMatrix);
    drawBox();
  popMatrix();

  // Rotate the compass image accordingly.
  pushMatrix();
    rotate2DPlaneToFaceCamera();
    translate(width - 150, height - 150, 0);
    drawCompass(headingAngle);
  popMatrix();
  
  // Display rotation source to user.
  fill(255);
  rotate2DPlaneToFaceCamera();
  text(getRotationSourceString(), 10, g_fontHeight);
}

void rotate2DPlaneToFaceCamera()
{
  translate(width/2.0f, height/2.0f, 0.0f);
  rotateY(-g_cameraAngle);
  translate(-width/2.0f, -height/2.0f, 0.0f);
}

void drawBox()
{
  // Draw four sides of box with different colours on each.
  stroke(160);
  fill(82, 10, 242);
  beginShape(QUADS);
    vertex(-25, -10, 50);
    vertex(-25, 10, 50);
    vertex(25, 10, 50);
    vertex(25, -10, 50);
  endShape();

  fill(255);
  beginShape(QUADS);
    vertex(-25, -10, -50);
    vertex(-25, -10, 50);
    vertex(25, -10, 50);
    vertex(25, -10, -50);
  endShape();

  fill(0);
  beginShape(QUADS);
    vertex(-25, 10, -50);
    vertex(-25, 10, 50);
    vertex(25, 10, 50);
    vertex(25, 10, -50);
  endShape();

  fill(126, 209, 13);
  beginShape(QUADS);
    vertex(-25, -10, -50);
    vertex(-25, 10, -50);
    vertex(25, 10, -50);
    vertex(25, -10, -50);
  endShape();
  
  fill(209, 6, 10);
  beginShape(QUADS);
    vertex(-25, 10, -50);
    vertex(-25, 10, 50);
    vertex(-25, -10, 50);
    vertex(-25, -10, -50);
  endShape();
  
  fill(237, 255, 0);
  beginShape(QUADS);
    vertex(25, 10, -50);
    vertex(25, 10, 50);
    vertex(25, -10, 50);
    vertex(25, -10, -50);
  endShape();
}

void drawCompass(float angle)
{
  rotateX(radians(-90));

  noStroke();
  fill(255, 0, 0);
  drawCylinder(100, 100, 10, 64);

  fill(0, 0, 255);
  rotateY(angle);
  translate(2.5, 0, 50);
  box(5, 10, 100);
}

void drawCylinder(float topRadius, float bottomRadius, float tall, int sides) 
{
  float angle = 0;
  float angleIncrement = TWO_PI / sides;
  beginShape(QUAD_STRIP);
  for (int i = 0; i < sides + 1; ++i) {
    vertex(topRadius*cos(angle), 0, topRadius*sin(angle));
    vertex(bottomRadius*cos(angle), tall, bottomRadius*sin(angle));
    angle += angleIncrement;
  }
  endShape();

  // If it is not a cone, draw the circular top cap
  if (topRadius != 0) {
    angle = 0;
    beginShape(TRIANGLE_FAN);

    // Center point
    vertex(0, 0, 0);
    for (int i = 0; i < sides + 1; i++) {
      vertex(topRadius * cos(angle), 0, topRadius * sin(angle));
      angle += angleIncrement;
    }
    endShape();
  }

  // If it is not a cone, draw the circular bottom cap
  if (bottomRadius != 0) {
    angle = 0;
    beginShape(TRIANGLE_FAN);

    // Center point
    vertex(0, tall, 0);
    for (int i = 0; i < sides + 1; i++) {
      vertex(bottomRadius * cos(angle), tall, bottomRadius * sin(angle));
      angle += angleIncrement;
    }
    endShape();
  }
}

String getRotationSourceString()
{
  switch (g_rotationSource)
  {
  case GYRO:
    return "Gyro";
  case ACCEL_MAG:
    return "Accel/Mag";
  case KALMAN:
    return "Kalman";
  case EMBEDDED:
    return "Embedded";
  }
  
  return "";
}

void serialEvent(Serial port)
{
  if (g_headingSensor == null)
    return;
  if (!g_headingSensor.update())
    return;
  g_samples++;
  
  // Retrieve latest calibrated sensor readings with no filtering applied. 
  FloatHeading heading = g_headingSensor.getCurrent();
  
  // Calculate rotation based on accelerometer and magnetometer.
  float[] accelMagQuaternion = calculateAccelMagRotation(heading);
  
  // Reset local Kalman filter at the same time as the embedded device. 
  if (g_headingSensor.wasResetRequested())
  {
    g_rotationQuaternion = accelMagQuaternion;
    g_kalmanP = new PMatrix3D(g_initVariance,           0.0f,           0.0f,           0.0f,
                                        0.0f, g_initVariance,           0.0f,           0.0f,
                                        0.0f,           0.0f, g_initVariance,           0.0f, 
                                        0.0f,           0.0f,           0.0f, g_initVariance);
  }
  
  // Calculate rotation based on gyro only.
  float[] gyroQuaternion = calculateGyroRotation(heading, g_rotationQuaternion);
  
  // Calculate rotation using Kalman filter.
  float[] kalmanQuaternion = calculateKalmanRotation(heading, g_rotationQuaternion);

  // Calculate rotation using embedded device's Kalman filter.
  float[] embeddedQuaternion = calculateEmbeddedKalmanRotation(heading, kalmanQuaternion);
  
  // Select which rotation quaternion to actually use for rendering.
  switch (g_rotationSource)
  {
  case GYRO:
    g_rotationQuaternion = gyroQuaternion;
    break;
  case ACCEL_MAG:
    g_rotationQuaternion = accelMagQuaternion;
    break;
  case KALMAN:
    g_rotationQuaternion = kalmanQuaternion;
    break;
  case EMBEDDED:
    g_rotationQuaternion = embeddedQuaternion;
    break;
  }
}

float[] calculateGyroRotation(FloatHeading heading, float[] currentQuaternion)
{
  // Swizzle the axis so that gyro's axis match overall sensor setup.
  float gyroX = heading.m_gyroY;
  float gyroY = heading.m_gyroZ;
  float gyroZ = heading.m_gyroX;

/* Demonstrates gyro drift better without this fix-up being applied.
  // Ignore very small rotations as they are most likely just noise.
  if (abs(gyroX) < g_gyroThreshold)
    gyroX = 0;
  if (abs(gyroY) < g_gyroThreshold)
    gyroY = 0;
  if (abs(gyroZ) < g_gyroThreshold)
    gyroZ = 0;
*/

  // Apply gyro rates (derivatives) to quaternion.
  float timeScale = (1.0f / 100.0f) * 0.5f;
  gyroX *= timeScale;
  gyroY *= timeScale;
  gyroZ *= timeScale;
  PMatrix3D gyroMatrix = new PMatrix3D( 1.0f, -gyroX, -gyroY, -gyroZ,
                                       gyroX,   1.0f,  gyroZ, -gyroY,
                                       gyroY, -gyroZ,   1.0f,  gyroX,
                                       gyroZ,  gyroY, -gyroX, 1.0f);
  updateGyroStats(gyroMatrix);
  
  float[] resultQuaternion = new float[4];
  gyroMatrix.mult(currentQuaternion, resultQuaternion);
  quaternionNormalize(resultQuaternion);

  return resultQuaternion;
}

void updateGyroStats(PMatrix3D orig)
{
  // Try updating a quaternion of all 1's to see the type of mean/variances the
  // gyro sensor measurements cause in this calculation.
  PMatrix3D m = new PMatrix3D(orig);
  float[]   quaternion = {1.0f, 1.0f, 1.0f, 1.0f};
  float[]   updatedQuaternion = new float[4];
  m.mult(quaternion, updatedQuaternion);
  
  // Accumulate these results.
  quaternionDoubleAdd(g_gyroSum, updatedQuaternion);
  quaternionDoubleAddSquared(g_gyroSquaredSum, updatedQuaternion);
  g_gyroSampleCount++;
  
  // Calculate mean/variance once enough samples have been accumualted.
  if (g_gyroSampleCount < g_gyroSamplesToCount)
    return;
  double[] mean = new double[4];
  double[] variance = new double[4];
  quaternionStats(mean, variance, g_gyroSum, g_gyroSquaredSum, g_gyroSampleCount);
  println("Gyro Quaternion Stats");
  print("    mean:"); quaternionPrint(mean);
  print("variance:"); quaternionPrint(variance);
  
  // Prepare to start accumulating next chunk of stats.
  for (int i = 0 ; i < g_gyroSum.length ; i++)
  {
    g_gyroSum[i] = 0.0;
    g_gyroSquaredSum[i] = 0.0;
  }
  g_gyroSampleCount = 0;
}

float[] calculateAccelMagRotation(FloatHeading heading)
{
  // Setup gravity (down) and north vectors.
  // NOTE: The fields are swizzled to make the axis on the device match the axis on the screen.
  PVector down = new PVector(heading.m_accelY, heading.m_accelZ, heading.m_accelX);
  PVector north = new PVector(-heading.m_magX, heading.m_magZ, heading.m_magY);

  // Project the north vector onto the earth surface plane.
  down.normalize();
  north.sub(PVector.mult(down, north.dot(down)));
  north.normalize();

  // To create a rotation matrix, we need all 3 basis vectors so calculate the vector which
  // is orthogonal to both the down and north vectors (ie. the normalized cross product).
  PVector west = north.cross(down);
  west.normalize();
  PMatrix3D rotationMatrix = new PMatrix3D(north.x, north.y, north.z, 0.0,
                                           down.x, down.y, down.z, 0.0,
                                           west.x, west.y, west.z, 0.0,
                                           0.0, 0.0, 0.0, 1.0);
  float[] rotationQuaternion = matrixToQuaternion(rotationMatrix);
  updateAccelMagStats(rotationQuaternion);
  
  return rotationQuaternion;
}

void updateAccelMagStats(float[] q)
{
  // Accumulate these results.
  quaternionDoubleAdd(g_accelMagSum, q);
  quaternionDoubleAddSquared(g_accelMagSquaredSum, q);
  g_accelMagSampleCount++;
  
  // Calculate mean/variance once enough samples have been accumualted.
  if (g_accelMagSampleCount < g_accelMagSamplesToCount)
    return;
  double[] mean = new double[4];
  double[] variance = new double[4];
  quaternionStats(mean, variance, g_accelMagSum, g_accelMagSquaredSum, g_accelMagSampleCount);
  println("Accel/Mag Quaternion Stats");
  print("    mean:"); quaternionPrint(mean);
  print("variance:"); quaternionPrint(variance);
  
  // Prepare to start accumulating next chunk of stats.
  for (int i = 0 ; i < g_accelMagSum.length ; i++)
  {
    g_accelMagSum[i] = 0.0;
    g_accelMagSquaredSum[i] = 0.0;
  }
  g_accelMagSampleCount = 0;
}

float[] calculateKalmanRotation(FloatHeading heading, float[] currentQuaternion)
{
  // System model covariance matrices which don't change.
  final float gyroVariance = 6.5E-11;
  final float accelMagVariance = 1.0E-5;
  final PMatrix3D Q = new PMatrix3D(gyroVariance,         0.0f,         0.0f,         0.0f,
                                            0.0f, gyroVariance,         0.0f,         0.0f,
                                            0.0f,         0.0f, gyroVariance,         0.0f,
                                            0.0f,         0.0f,         0.0f, gyroVariance);
  final PMatrix3D R = new PMatrix3D(accelMagVariance,             0.0f,             0.0f,             0.0f,
                                                0.0f, accelMagVariance,             0.0f,             0.0f,
                                                0.0f,             0.0f, accelMagVariance,             0.0f,
                                                0.0f,             0.0f,             0.0f, accelMagVariance);
                                          
  // Swizzle the axis so that gyro's axis match overall sensor setup.
  float gyroX = heading.m_gyroY;
  float gyroY = heading.m_gyroZ;
  float gyroZ = heading.m_gyroX;

  // Construct matrix which applies gyro rates (derivatives) to quaternion.
  // This will be the A matrix for the system model.
  final float timeScale = (1.0f / 100.0f);
  final float scaleFactor = timeScale * 0.5f;
  gyroX *= scaleFactor;
  gyroY *= scaleFactor;
  gyroZ *= scaleFactor;
  PMatrix3D A = new PMatrix3D( 1.0f, -gyroX, -gyroY, -gyroZ,
                               gyroX,   1.0f,  gyroZ, -gyroY,
                               gyroY, -gyroZ,   1.0f,  gyroX,
                               gyroZ,  gyroY, -gyroX, 1.0f);
  
  // Calculate Kalman prediction for x and error.
  float[] xPredicted = new float[4];
  A.mult(currentQuaternion, xPredicted);
  quaternionNormalize(xPredicted);
  
  PMatrix3D PPredicted = A.get();
  PMatrix3D ATranspose = A.get();
  ATranspose.transpose();
  PPredicted.apply(g_kalmanP);
  PPredicted.apply(ATranspose);
  matrixAdd(PPredicted, Q);
  
  // Calculate the Kalman gain.
  // Simplified a bit since the H matrix is the identity matrix.
  PMatrix3D K = PPredicted.get();
  PMatrix3D temp = PPredicted.get();
  matrixAdd(temp, R);
  temp.invert();
  K.apply(temp);

  // Fetch the accelerometer/magnetometer measurements as a quaternion.
  float[] z = calculateAccelMagRotation(heading);
    
  // Flip the quaternion (q == -q for quaternions) if the angle is obtuse.
  if (quaternionDot(z, xPredicted) < 0.0f)
  {
    quaternionFlip(z);
  }
  
  // Calculate the Kalman estimates.
  // Again, simplified a bit since H is the identity matrix.
  temp = K.get();
  float[] correction = new float[4];
  quaternionSubtract(z, xPredicted);
  temp.mult(z, correction);
  quaternionAdd(xPredicted, correction);
  float[] x = xPredicted;
  
  temp = K.get();
  temp.apply(PPredicted);
  matrixSubtract(PPredicted, temp);
  g_kalmanP = PPredicted;
  
  return x;
}

float[] calculateEmbeddedKalmanRotation(FloatHeading heading, float[] localQuaternion)
{
  float[] embeddedQuaternion = g_headingSensor.getEmbeddedQuaternion();
  
  // Compare locally calculated quaternion from accelerometers/magnetometers to that calculated on embedded device.
  for (int i = 0 ; i < 4 ; i++)
  {
    float diff = abs(localQuaternion[i] - embeddedQuaternion[i]);
    if (diff > g_floatThreshold)
    {
      println("quaternion[" + i + "] diff = " + diff);
    }
  }

  return embeddedQuaternion;
}

void matrixAdd(PMatrix3D m1, PMatrix3D m2)
{
  m1.m00 += m2.m00;
  m1.m01 += m2.m01;
  m1.m02 += m2.m02;
  m1.m03 += m2.m03;
  m1.m10 += m2.m10;
  m1.m11 += m2.m11;
  m1.m12 += m2.m12;
  m1.m13 += m2.m23;
  m1.m20 += m2.m20;
  m1.m21 += m2.m21;
  m1.m22 += m2.m22;
  m1.m23 += m2.m23;
  m1.m30 += m2.m30;
  m1.m31 += m2.m31;
  m1.m32 += m2.m32;
  m1.m33 += m2.m33;
}

void matrixSubtract(PMatrix3D m1, PMatrix3D m2)
{
  m1.m00 -= m2.m00;
  m1.m01 -= m2.m01;
  m1.m02 -= m2.m02;
  m1.m03 -= m2.m03;
  m1.m10 -= m2.m10;
  m1.m11 -= m2.m11;
  m1.m12 -= m2.m12;
  m1.m13 -= m2.m23;
  m1.m20 -= m2.m20;
  m1.m21 -= m2.m21;
  m1.m22 -= m2.m22;
  m1.m23 -= m2.m23;
  m1.m30 -= m2.m30;
  m1.m31 -= m2.m31;
  m1.m32 -= m2.m32;
  m1.m33 -= m2.m33;
}

void keyPressed()
{
  char lowerKey = Character.toLowerCase(key);
  
  switch(lowerKey)
  {
  case ' ':
    g_zeroRotation = true;
    break;
  case 'g':
    g_rotationSource = GYRO;
    break;
  case 'a':
    g_rotationSource = ACCEL_MAG;
    break;
  case 'k':
    g_rotationSource = KALMAN;
    g_kalmanP = new PMatrix3D(g_initVariance,           0.0f,           0.0f,           0.0f,
                                        0.0f, g_initVariance,           0.0f,           0.0f,
                                        0.0f,           0.0f, g_initVariance,           0.0f, 
                                        0.0f,           0.0f,           0.0f, g_initVariance);
    break;
  case 'e':
    g_rotationSource = EMBEDDED;
    g_kalmanP = new PMatrix3D(g_initVariance,           0.0f,           0.0f,           0.0f,
                                        0.0f, g_initVariance,           0.0f,           0.0f,
                                        0.0f,           0.0f, g_initVariance,           0.0f, 
                                        0.0f,           0.0f,           0.0f, g_initVariance);
    // Ask the Kalman filter running on the embedded device to reset itself.
    if (g_serial != null)
    {
      g_serial.write('R');
    }
    
    break;
  }
}

