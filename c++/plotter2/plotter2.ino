#include "BluetoothSerial.h"

BluetoothSerial SerialBT;


 

/* G-CODES 
 *  
 *  G1 X\d+ Y\d+\n
 *  G1 Z[01] \n     //0 Move, 1 Draw
 *  
 *  up -4000, -4000 -> 49 cm
 *  
 */

/* Geometrie: Abstand Aufhängung: 1930mm, Nullpunkt bei r1=r2=1445 
 *  h0=(1445**2-(1930/2)**2)**0.5    -> 1075
 *  rup = ((1930/2)**2+(1075-490)**2)**0.5  ->  1128
 *  perstep = (1445-1128)/4000.0 -> 0.07925
 */

float halbeBreite = 1930.0/2;
float rini = 1445.0;
float yini = sqrt(rini*rini-halbeBreite*halbeBreite);
float perstep = 0.07925;

int numAccel = 400;
//float accel[] = {1.0, 0.41421, 0.31784, 0.26795, 0.23607, 0.21342, 0.19626, 0.18268, 0.17157, 0.16228};
unsigned long stepTime = 1000/(sqrt(numAccel)-sqrt(numAccel-1));
int curSpeed = 0;


long steps[] = {0,0};
long target[] = {0,0};

// float r[] = {1445.0, 1445.0}; // Zero-pos Radien

 
const char* ssid = "tech-lab";
const char* password = "tech-lab";

const uint16_t port = 21;
const char * host = "192.168.1.108";

#define ONBOARD 2

int dirPins[] = {16,18};  // left motor (dir 0 -> longer), right motor (dir 0 -> shorter)
int stepPins[] = {17,19};
int shorter[] = {0,1};  // Direction for shortening

// Only Pins above 32 are useable for analogRead when WiFi ist active!

int enPin = 12;
int resPin = 33;
int sleepPin = 32;

int errPins[] = {36, 39};
int joyPins[] = {34,35,26};   // x,y,switch

byte penState = 0;
int dirs[] = {0,0};

void setDirs(int d0, int d1) {
  setDir(0, d0);
  setDir(1, d1);
}

void setDir(int stepper, int dir) {
  if (dir==-1) dir=0;
  dirs[stepper] = dir;
  digitalWrite(dirPins[stepper], dir==1 ? shorter[stepper]:1-shorter[stepper]);
}

long lastStep = 0;
bool motorstate = false;

void step(int stepper) {
  lastStep = millis();
  digitalWrite(stepPins[stepper], HIGH);
  delayMicroseconds(2);
  digitalWrite(stepPins[stepper], LOW);
  if (dirs[stepper]) {
    steps[stepper]++;
  } else {
    steps[stepper]--;
  }
}

void btConnect() {
  while(!SerialBT.connected(1000)) {
    Serial.println("BT not connected... trying...");
  }
}

int setPen(byte state) {
  bool ok = false;
  while (!ok) {
    btConnect();
    while(SerialBT.available()>0) {
      SerialBT.read();
    }
    SerialBT.write(state);
    for (int i=0; i<1000 && SerialBT.available()==0; i++) {
      delay(3);
    }
    if (SerialBT.available()>0) {
      byte response = SerialBT.read();
      if (response != state) {
        Serial.printf("Error! PenServer sent %d\n", response);
      } else {
        ok = true;
      }
    } else {
      Serial.println("Timeout!");
    }
  }
  penState = state;
  return true;
}

bool penUp() {
  return setPen(0);
}

bool penDown() {
  return setPen(1);
}

// y^, x>
/*void mmToSteps(float x, float y) {
	// Restrict Coordinates
	if (y<0) y=0;
	if (y>600) y=600;
	x+=halbeBreite;
	if (x<200) x=200;
	if (x>2*halbeBreite-200) x=2*halbeBreite-200;

	y=ystep-y;
	float r1 = sqrt(x*x+y*y);
	float r2 = sqrt((2*halbeBreite-x)*(2*halbeBreite-x) + y*y);
	r1 = r1-rini;
	r2 = r2-rini;
	target[0] = (long)(r1/perstep);
	target[1] = (long)(r2/perstep);	
}*/



void gotoSteps(long s1, long s2, bool slowDown) {
  if (penState == 0) {
    slowDown = true;
  }
  Serial.printf("goto %d, %d with %d", s1, s2, slowDown);
  long dsteps[2];
  long totalSteps[2];
  unsigned long dtime[2];
  unsigned long ntime[2];
  
  int dir[2];
  if (s1>steps[0]) {
    dir[0]=1;
  } else if(s1<steps[0]) {
    dir[0]=-1;
  } else {
    dir[0] = 0;
  }
  if (s2>steps[1]) {
    dir[1]=1;
  } else if(s2<steps[1]) {
    dir[1]=-1;
  } else {
    dir[1] = 0;
  }
  setDirs(dir[0], dir[1]);
  dsteps[0] = abs(s1-steps[0]);
  dsteps[1] = abs(s2-steps[1]);
  totalSteps[0] = dsteps[0];
  totalSteps[1] = dsteps[1];

  if (dsteps[0]>dsteps[1]) {
    dtime[0]=stepTime;
    if (dsteps[1]>0) {
      dtime[1]=stepTime*dsteps[0]/dsteps[1];
    } else {
      dtime[1] = stepTime*dsteps[0];
    }
  } else {
    dtime[1]=stepTime;
    if (dsteps[0]>0) {
      dtime[0]=stepTime*dsteps[1]/dsteps[0];
    } else {
      dtime[0] = stepTime*dsteps[1];
    }
  }
  Serial.print("d0=");
  Serial.print(dsteps[0]);
  Serial.print(" d1=");
  Serial.print(dsteps[1]);
  Serial.print(" dir[0]=");
  Serial.print(dir[0]);
  Serial.print(" dir[1]=");
  Serial.print(dir[1]);
  Serial.print(" dtime[0]=");
  Serial.print(dtime[0]);
  Serial.print(" dtime[1]=");
  Serial.println(dtime[1]);/**/
  
  ntime[0] = micros()+(long int)(dtime[0]*(sqrt(curSpeed+1)-sqrt(curSpeed)));
  ntime[1] = micros()+(long int)(dtime[1]*(sqrt(curSpeed+1)-sqrt(curSpeed)));
  //for (int s=0; s<2; s++)  Serial.printf("numAccel=%d, total-st=%d, dstps=%d\n", numAccel, totalSteps[s]-dsteps[s], dsteps[s]);
  while (dsteps[0]>0 || dsteps[1]>0) {
    int an = 0;
    for (int s=0; s<2; s++) {
		  if (dsteps[s]>an) an = dsteps[s];
    }

    bool stepped = false;
    for (int s=0; s<2; s++) {
      if (micros()>ntime[s] && dsteps[s]>0) {
        stepped = true;
        ntime[s]+=(long int)(dtime[s]*(sqrt(curSpeed+1)-sqrt(curSpeed)));  // Who needs precomputed values on an ESP32?
        step(s);
        dsteps[s]--;
      }
    }
    if (stepped) {
       if (slowDown && an<curSpeed) curSpeed = an;
       else if (curSpeed<(penState==1 ? numAccel : 6000)) curSpeed++;
    }
  }
  // Assure target
  if (dsteps[0]!=0 || dsteps[1]!=0) {
    Serial.println("Target not reached, correcting");
  }
  while (dsteps[0]!=0 || dsteps[1]!=0) {
    for (int s=0; s<2; s++) {
      if (dsteps[s]>0) {
        step(s);
        dsteps[s]--;
      }
    }
    delay(12);
  }

}

long getLong(char * buf, int &pos) {
  bool neg = buf[pos]=='-';
  if (neg) pos++;
  long res = 0;
  while (buf[pos]>='0' && buf[pos]<='9') {
    res*=10;
    res+=buf[pos]-'0';
    pos++;
  }
  return neg ? -res : res;
}

/*float getFloat(char * buf, int &pos) {
  bool neg = buf[pos]=='-';
  if (neg) pos++;
  long res = 0;
  
  while (true) {
    if (!(buf[pos]>='0' && buf[pos]<='9' || buf[pos]=='.')) break
	if (
    res*=10;
    res+=buf[pos]-'0';
    pos++;
  }
  return neg ? -res : res;
}*/


void processGcode() {
  char buffer[101];
  int p = 0;
  bool error = false;
//  while (true) {
    p=0;
    while (!Serial.available()) {
      yield();
       joystick();
       if (millis()-lastStep>2000 && motorstate) {
          motorState(false);
          curSpeed=0;
       }
    }
    
    error = false;
    char c;
    while (true) {  // Wait for \n
      if (Serial.available()) {
        c = Serial.read();
        if (c!=0) {
          Serial.print(c);
          if (c=='\n') {
            buffer[p]=0;
            break;
          }
          buffer[p++] = c;
          if (p>99) break;
        }
      }
    }
    if (p<100 && p>0) {
      Serial.println(buffer);
      // Parse
      if (buffer[0]=='G' && buffer[1]=='1' && buffer[2]==' '){
        if (buffer[3]=='X') {
          p=4;
          long r1 = getLong(buffer, p);
          //Serial.println(r1);
          p++;
          if (buffer[p]!='Y') {
              Serial.println("ERROR, expect Y after X");
              error = true;
          }          
          if (!error) {
            p++;
            long r2 = getLong(buffer, p);
            //Serial.println(r2);
			//mmToSteps(r1,r2);
            if (!motorstate) motorState(true);
            gotoSteps(r1, r2, buffer[p]!=0);
          }
        } else if (buffer[3]=='Z') {
          if (buffer[4]=='0') {
            bool ok = false;
            for (int i=0; i<10; i++) {
              if (penUp()) {
                ok = true;
                break;
              }
            }
            if (!ok) {
              Serial.println("penUp failed");
              error = true;
            }
          } else if (buffer[4]=='1') {
            bool ok = false;
            for (int i=0; i<10; i++) {
              if (penDown()) {
                ok = true;
                break;
              }
            }
            if (!ok) {
              Serial.println("penDown failed");
              error=true;
            }
          } else {
            error = true;
            Serial.println("ERROR, expect 0 or 1 after G1 Z");
          }
        } else {
          error = true;
          Serial.println("ERROR, expect X or Z after G1");
        }
      } else {
        error = true;
        Serial.print("ERROR, Line must start with 'G1 ' Got buffer[0]=");
        Serial.println((int)buffer[0]);
      }
    } else if (p>0) {
      error = true;
      Serial.println("ERROR, line too long! Forgot \\n?");
    }
    if (!error) {
      Serial.print("OK\n");
    }
 // }
}

void motorState(bool on) {
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, !on);

  pinMode(resPin, OUTPUT);
  digitalWrite(resPin, on);
  
  pinMode(sleepPin, OUTPUT);
  digitalWrite(sleepPin, on);
  motorstate = on;
}


void setup() {

  pinMode(ONBOARD, OUTPUT);
  Serial.begin(115200);
  Serial.println("Setting up BT");
  SerialBT.begin("WBPlotter", true); 
  Serial.println("Connecting to PenServer");
  if (SerialBT.connect("PenServer")) {
    Serial.println("BT connection OK");
  } else {
    Serial.println("BT not connected, will try again later...");
  }
  
  for (int j=0; j<2; j++) { 
    pinMode(dirPins[j],OUTPUT);
    digitalWrite(dirPins[j], shorter[j]);
  }
  for (int i=0;i<5;i++) {
    digitalWrite(ONBOARD,HIGH);
    delay(100);
    digitalWrite(ONBOARD, LOW);
    delay(100);
  }
  for (int j=0; j<2; j++) pinMode(stepPins[j], OUTPUT);
  
  pinMode(joyPins[2], INPUT_PULLUP);

  motorState(true);
 
}

long int last = 0;
long int lastMove = 0;
long int maxdelay = 20000;
long int stepdelay = maxdelay;
int stepnum = 1;


void joystick() {
  int a[2];
  a[0] = analogRead(joyPins[0]);
  a[1] = analogRead(joyPins[1]);
  int sw = digitalRead(joyPins[2]);
  
  int vx=(a[1]<500 ? -1 : (a[1]>3500 ? 1 : 0));
  int vy=(a[0]<500 ? -1 : (a[0]>3500 ? 1 : 0));


  int m0 = vx-vy;
  int m1 = -vx-vy;
  if (m0!=0 || m1!=0) {
    if (!motorstate) motorState(true);
    //Serial.printf("a[]=(%d, %d), v = (%d, %d),  m = (%d,%d) sw = %d\n", a[0], a[1], vx,vy,m0,m1,sw);
    if (millis()-lastMove<2) {
      if (stepdelay>100) {
        stepdelay = maxdelay*(sqrt(stepnum+1)-sqrt(stepnum));
        stepnum++;
      }
    } else {
      stepdelay=maxdelay;
      stepnum = 1;
    }
    delayMicroseconds(stepdelay);
    lastMove=millis();
  }
  setDirs(m0<0 ? 1:0, m1<0 ? 1:0);
  
  if (m0!=0) {
    step(0);
  }
  if (m1!=0) {
    step(1);
  }
  if (!sw) {
    //setPen(1-penState);
    steps[0]=0;
    steps[1]=0;
    last = millis();
    while(!digitalRead(joyPins[2]));
    if (millis()-last>500) {
      penDown();
      penUp();
    }
  }

}


void loop() {
  processGcode();
}
