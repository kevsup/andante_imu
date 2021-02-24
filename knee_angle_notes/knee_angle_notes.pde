import processing.serial.*;
import processing.sound.*;

Serial myPort;        // The serial port
int serialNumber = 3;

SoundFile[] file = new SoundFile[8];
boolean[] playing = new boolean[8];

int currNote;

void setup () {
  file[0] = new SoundFile(this, "C3.wav");
  file[1] = new SoundFile(this, "D3.wav");
  file[2] = new SoundFile(this, "E3.wav");
  file[3] = new SoundFile(this, "F3.wav");
  file[4] = new SoundFile(this, "G3.wav");
  file[5] = new SoundFile(this, "A3.wav");
  file[6] = new SoundFile(this, "B3.wav");
  file[7] = new SoundFile(this, "C4.wav");
  
  // set the window size:
  size(600, 400);        

  // List all the available serial ports
  println(Serial.list());
  // Check the listed serial ports in your machine
  // and use the correct index number in Serial.list()[].

  myPort = new Serial(this, Serial.list()[serialNumber], 9600);  //

  // A serialEvent() is generated when a newline character is received :
  myPort.bufferUntil('\n');
  background(0);      // set inital background:
  
  for (int i = 0; i < playing.length; i++) {
    playing[i] = false;
  }
  
  currNote = 0;
}

void draw () {
  background(0);
}

void serialEvent (Serial myPort) {
  // get the ASCII string:
  String inString = myPort.readStringUntil('\n');
  //println(inString);
  if (inString != null) {
    float kneeAngle = float(inString);
    if (kneeAngle < 0) {
       kneeAngle = 0; 
    } else if (kneeAngle > 90) {
       kneeAngle = 90; 
    }
    int numNotes = file.length;
    float note = map(kneeAngle, 90.0, 0.0, 0.0, numNotes - 0.1);
    int index = int(note);
    println(index);
    if (!playing[index]) {
      file[currNote].stop(); 
      file[index].play(); 
      playing[currNote] = false;
      playing[index] = true;
      currNote = index;
    }
  }
}
