// these constants won't change:
const int sensorPin = A0;  // the piezo is connected to analog pin 0

int sensorValue ;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  sensorValue = analogRead(sensorPin);
  Serial.println(sensorValue);
  delay(100);
}
