#include <SoftwareSerial.h>
SoftwareSerial hc06(8,9);
String cmd = "";
int error = 0;
const int base_pwm = 128;

void setup()
{
  //Initialize Serial Monitor
  Serial.begin(9600);
  hc06.begin(9600);

}

void loop()
{ 
  // Get error from bluetooth
  while(hc06.available()>0){
    cmd += (char)hc06.read();
  }
  if (cmd != ""){
    error = cmd.toInt();
    Serial.print("Error: ");
    Serial.println(error);
    cmd = "";
  }






  

  delay(33);
}
