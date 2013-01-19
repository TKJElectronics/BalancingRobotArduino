//convert all axis
int minAngle = 0;
int maxAngle = 360;

void convert() {   
  /* convert the gyro x-axis */
  if (stringGyro != null) {
    // trim off any whitespace:
    stringGyro = trim(stringGyro);
    // convert to an float and map to the screen height, then save in buffer:    
    gyro[gyro.length-1] = map(float(stringGyro), minAngle, maxAngle, 0, height);
  }  
  /* convert the accelerometer y-axis */
  if (stringAcc != null) {
    // trim off any whitespace:
    stringAcc = trim(stringAcc);
    // convert to an float and map to the screen height, then save in buffer:        
    acc[acc.length-1] = map(float(stringAcc), minAngle, maxAngle, 0, height);
  }
  /* convert the kalman filter y-axis */
  if (stringKalman != null) {
    // trim off any whitespace:
    stringKalman = trim(stringKalman);
    // convert to an float and map to the screen height, then save in buffer:    
    kalman[kalman.length-1] = map(float(stringKalman), minAngle, maxAngle, 0, height);
  }
}
