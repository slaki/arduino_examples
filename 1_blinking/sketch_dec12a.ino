int pin = 3;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  
  Serial.println("setup");
}

void loop() {
  delay(1000);
  digitalWrite(pin, LOW);
  Serial.println("set LOW");
  
  delay(1000);
  digitalWrite(pin, HIGH);
  Serial.println("set HIGH");  
}
