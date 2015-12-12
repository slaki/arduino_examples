const int LEDpin = 3;
const int TEMPpin = A0;
const float threshold = 25.0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LEDpin, OUTPUT);
  digitalWrite(LEDpin, HIGH);
  delay(1000);
  digitalWrite(LEDpin, LOW);
  delay(250);
  digitalWrite(LEDpin, HIGH);
  digitalWrite(LEDpin, LOW);  
  Serial.println("setup completed...");
}

void loop() {
  int value = analogRead(TEMPpin);
  Serial.print("value:");
  Serial.println(value);
  Serial.print("Voltage:");
  Serial.println(value / 1024.0 * 5.0);
  float temp = ((value/1024.0) * 5.0 -.5) * 100.0;

  Serial.print("Temp: ");
  Serial.println(temp);

  if (temp<threshold)
  {
    digitalWrite(LEDpin, LOW);
  }
  else
  {
    digitalWrite(LEDpin, HIGH);
    Serial.println("ALERT");
  }

    
  delay(1000);
  
}
