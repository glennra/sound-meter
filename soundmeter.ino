/*
  Analog input, analog output, serial output

  Reads an analog input pin, maps the result to a range from 0 to 255 and uses
  the result to set the pulse width modulation (PWM) of an output pin.
  Also prints the results to the Serial Monitor.

  The circuit:
  - potentiometer connected to analog pin 0.
    Center pin of the potentiometer goes to the analog pin.
    side pins of the potentiometer go to +5V and ground
  - LED connected from digital pin 9 to ground

  created 29 Dec. 2008
  modified 9 Apr 2012
  by Tom Igoe

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogInOutSerial
*/

//#define Serial SerialUSB

// These constants won't change. They're used to give names to the pins used:
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
const int sensPin = A9;
const int pin_red = 3;
const int pin_grn = 5;
const int pin_blu = 6;
const int pin_led = 13;

int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)

const int analogMax = 1023;
const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;

unsigned int filter[3] = {0, 0, 0};
int n, n1, n2 = 0;

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect
  }

  pinMode(pin_red, OUTPUT);
  pinMode(pin_grn, OUTPUT);
  pinMode(pin_blu, OUTPUT);
  pinMode(pin_led, OUTPUT);

  pinMode(analogInPin, INPUT);

  //analogWriteResolution(10);
}

void HSVtoRGB(float fH, float fS, float fV, float& fR, float& fG, float& fB)
{
  float fC = fV * fS; // Chroma
  float fHPrime = fmod(fH / 60.0, 6);
  float fX = fC * (1 - fabs(fmod(fHPrime, 2) - 1));
  float fM = fV - fC;

  if (0 <= fHPrime && fHPrime < 1) {
    fR = fC;
    fG = fX;
    fB = 0;
  } else if (1 <= fHPrime && fHPrime < 2) {
    fR = fX;
    fG = fC;
    fB = 0;
  } else if (2 <= fHPrime && fHPrime < 3) {
    fR = 0;
    fG = fC;
    fB = fX;
  } else if (3 <= fHPrime && fHPrime < 4) {
    fR = 0;
    fG = fX;
    fB = fC;
  } else if (4 <= fHPrime && fHPrime < 5) {
    fR = fX;
    fG = 0;
    fB = fC;
  } else if (5 <= fHPrime && fHPrime < 6) {
    fR = fC;
    fG = 0;
    fB = fX;
  } else {
    fR = 0;
    fG = 0;
    fB = 0;
  }

  fR += fM;
  fG += fM;
  fB += fM;
}

void colourtest()
{
  //int col = map(volts, 0, 5.0, 120, 0);
  for(int col=120; col > -120; col--)
  {
    static const float S = 1.0;
    static const float V = 1.0;
    float x = col;
    if(col < 0)
      x = -x;
    float H = x;// * 0.1;
    float r, g, b;
  
    HSVtoRGB(H, S, V, r, g, b);
    const float scale = 255.0;
    analogWrite(pin_red, r * scale);
    analogWrite(pin_grn, g * scale * 0.4);
    analogWrite(pin_blu, b * scale * 0.6);
  
//    Serial.print(" col ");
//    Serial.print(col);
//    Serial.print(" rgb ");
//    Serial.print(r);
//    Serial.print(" ");
//    Serial.print(g);
//    Serial.print(" ");
//    Serial.print(b);
//  
//    Serial.println();
    delay(10);
  }
}

void soundloop() {

  unsigned long startMillis = millis(); // Start of sample window
  unsigned int peakToPeak = 0;   // peak-to-peak level

  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;

  // collect data for 50 mS
  while (millis() - startMillis < sampleWindow)
  {
    sample = analogRead(analogInPin);
    if (sample < 1024)  // toss out spurious readings
    {
      if (sample > signalMax)
      {
        signalMax = sample;  // save just the max levels
      }
      else if (sample < signalMin)
      {
        signalMin = sample;  // save just the min levels
      }
    }
  }
  peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude

  filter[n] = peakToPeak;

  n1 = n - 1;
  if (n1 < 0)
    n1 += 3;

  n2 = n - 2;
  if (n2 < 0)
    n2 += 3;
//  Serial.print(" n n1 n2 ");
//  Serial.print(n);
//  Serial.print(" ");
//  Serial.print(n1);
//  Serial.print(" ");
//  Serial.print(n2);

  unsigned int filtered = (filter[n2] + 2 * filter[n1] + filter[n])/4;
  n++;
  if (n >= 3)
    n = 0;

  unsigned int sensitivity = analogRead(sensPin);

  Serial.print(" ");
  Serial.print(peakToPeak);
  Serial.print(" ");
  Serial.print(filter[n2]);
  Serial.print(" ");
  Serial.print(filter[n1]);
  Serial.print(" ");
  Serial.print(filter[n]);
  Serial.print(" filt ");
  Serial.print(filtered);
  Serial.print(" sens ");
  Serial.print(sensitivity);

  //  //  Serial.print("audio ");
  //  //  Serial.println(sensorValue);
  //  //
  //int col = map(filtered, 0, 5.0, 120, 0);
  //int col = map(volts, 0, 5.0, 120.0, 0.0);

  
  //float volts = (peakToPeak * 5.0) / 1024.0;  // convert to volts
  //int col = constrain(map(peakToPeak, 0, 700, 120, 0), 0, 120);
  int col = constrain(map(filtered, 0, sensitivity/4, 120, 0), 0, 120);

  static const float S = 1.0;
  static const float V = 1.0;
  float H = (float)col;
  float r, g, b;

  HSVtoRGB(H, S, V, r, g, b);
  const float scale = 255.0;
  analogWrite(pin_red, r * scale);
  analogWrite(pin_grn, g * scale * 0.4);
  analogWrite(pin_blu, b * scale * 0.6);

//  Serial.print(" col ");
//  Serial.print(col);
//  Serial.print(" rgb ");
//  Serial.print(r);
//  Serial.print(" ");
//  Serial.print(g);
//  Serial.print(" ");
//  Serial.print(b);

  Serial.println();

}

void loop()
{
  //colourtest();
  soundloop();
}

