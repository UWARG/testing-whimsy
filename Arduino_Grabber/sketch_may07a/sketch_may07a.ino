#include "ppm.h"

#define DEBUG true // would I rather use bool DEBUG=true? 

#define CRANE_CHANNEL 3
#define GRABBER_CHANNEL 2
#define PPM_INPUT_PIN 3

// need 2 gpio for each (maybe more?), but has A and B pins for each.
#define GRABBER_GPIOA 4
#define GRABBER_GPIOB 5
#define CRANE_GPIOA   6
#define CRANE_GPIOB   7

// Loop interval time
const long interval = 50;
unsigned long previousMillis = 0;

void setup() {
  // put your setup code here, to run once:

  if (DEBUG) {
    Serial.begin(115200);
  }

  pinMode(GRABBER_GPIOA, OUTPUT);
  pinMode(GRABBER_GPIOB, OUTPUT);
  pinMode(CRANE_GPIOA, OUTPUT);
  pinMode(CRANE_GPIOB, OUTPUT);
  
  ppm.begin(PPM_INPUT_PIN, false);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    short grabber = ppm.read_channel(GRABBER_CHANNEL);
    short crane = ppm.read_channel(CRANE_CHANNEL);

    // logic is basically high -> low forward, low -> high backward
    // Check directions of everything, who knows if they are flipped on the controller
    if (grabber < 33) {
      digitalWrite(GRABBER_GPIOA, HIGH);
      digitalWrite(GRABBER_GPIOB, LOW);
    } else if (grabber < 67) {
      digitalWrite(GRABBER_GPIOA, LOW);
      digitalWrite(GRABBER_GPIOB, LOW);
    } else if (grabber < 100) {
      digitalWrite(GRABBER_GPIOA, LOW);
      digitalWrite(GRABBER_GPIOB, HIGH);
    }
    
    if (DEBUG) {
      Serial.print("Grabber:"); Serial.print(grabber); Serial.print(" ");
      Serial.print("Crane:");     Serias.print(crane); Serial.print(" ");
    }
  }
}
