/*
This code is made for processing https://processing.org/
*/

import processing.serial.*;     // import the Processing serial library
Serial myPort;                  // The serial port
String my_port = "COM2";      	// choose your port
float xx, yy, zz;
float xx_offset, yy_offset, zz_offset;
int status = 0; // 取出第一次状态

void setup() {
  size(1024, 800, P3D);

  myPort = new Serial(this, my_port, 115200);
  myPort.bufferUntil('\n');

  smooth();
}

void draw() {
  background(0);
  noStroke();
  translate(width/2, height/2);
  pushMatrix();
  rotateX(xx);//pitch
  rotateY(zz);//yaw
  rotateZ(yy);//roll
  box(150, 50, 300);
  popMatrix();
}


void serialEvent(Serial myPort) {

  String myString = myPort.readStringUntil('\n');
  myString = trim(myString);
  float sensors[] = float(split(myString, ':'));
  
  xx = sensors[0];
  yy = sensors[1];
  zz = sensors[2];
  
  if(status == 0){
    xx_offset = xx;
    yy_offset = yy;
    zz_offset = zz;
    status = 1;
  }
  else if(status == 1)
  {
    xx = xx - xx_offset;
    yy = yy - yy_offset;
    zz = zz - zz_offset;
  }
  
  println("pitch: " + xx + " roll: " + yy + " yaw: " + zz + "\n"); //debug

}
