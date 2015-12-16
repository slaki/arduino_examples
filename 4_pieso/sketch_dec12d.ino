/*
 * Author: Sandor Laki, lakis@inf.elte.hu
 *
 * Description:
 * Making noise if the temperature is above a given treshold. This is an extension of the previous examples...
 *
 * Created on 2015
 */

const int LEDpin = 3;
const int TEMPpin = A0;
const float threshold = 25.0;
const int DHTpin = 2;
float temp_dht = 0.0;
float hum_dht = 0.0;
const int TONEpin = 7;

int dht_read()
{
        // BUFFER TO RECEIVE
        uint8_t bits[5];
        uint8_t cnt = 7;
        uint8_t idx = 0;

        // EMPTY BUFFER
        for (int i=0; i< 5; i++) bits[i] = 0;

        // REQUEST SAMPLE
        pinMode(DHTpin, OUTPUT);
        digitalWrite(DHTpin, LOW);
        delay(18);
        digitalWrite(DHTpin, HIGH);
        delayMicroseconds(40);
        pinMode(DHTpin, INPUT);

        // ACKNOWLEDGE or TIMEOUT
        unsigned int loopCnt = 10000;
        while(digitalRead(DHTpin) == LOW)
                if (loopCnt-- == 0) return -1;

        loopCnt = 10000;
        while(digitalRead(DHTpin) == HIGH)
                if (loopCnt-- == 0) return -1;

        // READ OUTPUT - 40 BITS => 5 BYTES or TIMEOUT
        for (int i=0; i<40; i++)
        {
                loopCnt = 10000;
                while(digitalRead(DHTpin) == LOW)
                        if (loopCnt-- == 0) return -1;

                unsigned long t = micros();

                loopCnt = 10000;
                while(digitalRead(DHTpin) == HIGH)
                        if (loopCnt-- == 0) return -1;

                if ((micros() - t) > 40) bits[idx] |= (1 << cnt);
                if (cnt == 0)   // next byte?
                {
                        cnt = 7;    // restart at MSB
                        idx++;      // next byte!
                }
                else cnt--;
        }

        // WRITE TO RIGHT VARS
        // as bits[1] and bits[3] are allways zero they are omitted in formulas.
        hum_dht    = bits[0]; 
        temp_dht = bits[2]; 

        uint8_t sum = bits[0] + bits[2];  

        if (bits[4] != sum) return -1;
        return 0;
}

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

void alert() {
  digitalWrite(LEDpin, HIGH);
  Serial.println("ALERT");
  tone(TONEpin, 245);
  delay(250);
  noTone(TONEpin); 
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
    alert();
  }

  if (dht_read()<0) Serial.println("DHT Error");
  
  Serial.print("DHT Temp: ");
  Serial.println(temp_dht);
  
  Serial.print("DHT Humidity: ");
  Serial.println(hum_dht);
    
  delay(1000);
  
}
