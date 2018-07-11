void setup() {
  pinMode(13, OUTPUT); // forward
  pinMode(12, OUTPUT); // right
  delay(5000);
  turnRight(360);
}

void loop() {

}

void moveForward(int distance) { // distance is in cm
  digitalWrite(13, HIGH);
  delay(distance * 9.32);
  digitalWrite(13, LOW);
}

void turnRight(int rotation) { // rotation is in degrees
  digitalWrite(12, HIGH);
  digitalWrite(13, HIGH);
  delay(rotation * 21.87);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
}
