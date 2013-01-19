void drawAxis() { 
  /* draw acceleromter x-axis */
  noFill();
  stroke(255,0,0); //red
  // redraw everything
  beginShape();
  for(int i = 0; i<acc.length;i++)
    vertex(i+337,height-acc[i]);  
  endShape();
  // put all data one array back
  for(int i = 1; i<acc.length;i++)
    acc[i-1] = acc[i];  
   
  /* draw gyro x-axis */
  noFill();
  stroke(0,255,0); // green
  // redraw everything
  beginShape();
  for(int i = 0; i<gyro.length;i++)
    vertex(i+337,height-gyro[i]);
  endShape();
  // put all data one array back
  for(int i = 1; i<gyro.length;i++)
    gyro[i-1] = gyro[i];
    
  /* draw kalman filter x-axis */
  noFill();
  stroke(0,0,255); // blue  
  // redraw everything
  beginShape();
  for(int i = 0; i<kalman.length;i++)
    vertex(i+337,height-kalman[i]);  
  endShape();
  // put all data one array back
  for(int i = 1; i<kalman.length;i++)
    kalman[i-1] = kalman[i];
}
