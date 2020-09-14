#define PIN_LED 7
int i = 1;
void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  digitalWrite(PIN_LED, 0);
  delay(1000);
  digitalWrite(PIN_LED, 1);

}

void loop() {
  while(1)
  {   if (i <= 5)
        {
          digitalWrite(PIN_LED, 0);
          delay(100);
          digitalWrite(PIN_LED, 1);
          delay(100);
          i += 1;
          Serial.println("+1");
        }  
      else
        {
           digitalWrite(PIN_LED, 1);
        }
      
      
  }
}
