#include <SoftwareSerial.h>
#define PWMSpeed0 11
#define PWMSpeed1 10
#define PinIN1 7
#define PinIN2 6
#define PinIN3 5
#define PinIN4 4
#define POT A0

SoftwareSerial hc06(8,9);
String cmd = "";
int error = 0;
int mode = 0; // 0 inactive, 1 FWD, 2 TURN
int abs_error = 0;
int SpeedControl0 = 100;
int SpeedControl1 = 100;
const int base_speed = 100;
int sim_error = 0;

void setup()
{
  //Initialize Serial Monitor
  Serial.begin(9600);
  hc06.begin(9600);
  pinMode(PWMSpeed0, OUTPUT);
  pinMode(PWMSpeed1, OUTPUT);
  pinMode(PinIN1, OUTPUT);
  pinMode(PinIN2, OUTPUT);
  pinMode(PinIN3, OUTPUT);
  pinMode(PinIN4, OUTPUT);
}

void loop()
{ 
  Serial.println("\n\n\n\n");
  error, mode = BT();
  // Serial.print("Error: ");
  // Serial.println(error);
  Serial.print("Modo: ");
  Serial.println(mode);
  // error = POT_ERROR();  // Simulamos un error por medio de un potenciometro
  delay(40); // Delay para lograr leer los datos del string de bluchu
  if (mode == 1){
    FWD(error);
  }else if(mode == 2){
    TURN(error);
  }else OFF();
  // FWD(error); // Programa de direccion
  
}

int POT_ERROR(){ // Simulacion de un error con un potenciometro
  sim_error = analogRead(POT);  // Obtenemos la lectura del voltaje del potencionmetro con ADC
  if(sim_error < 470){ // Si esta en la primera mitad
    sim_error = map(sim_error,470,0,0,-255);  // Generamos valores negativos al simular el error
    
  }else if(sim_error > 550){  // Si esta en la segunda mitad
    sim_error = map(sim_error,550,1023,0,255);  // Generamos valores positivos al simular el error
    
  }else { // Si el error es demasiado pequeno no se hace nada
    sim_error = 0;
  }
  return sim_error; // regresamos el error
}

void OFF(){
  digitalWrite (PinIN1, LOW);  // Establece direccion de giro motor derecho 
  digitalWrite (PinIN2, LOW); // Establece direccion de giro motor derecho
  digitalWrite (PinIN3, LOW);  // Establece direcccion de giro motor izquierdo
  digitalWrite (PinIN4, LOW);
}
int BT(){ // Get error from bluetooth
  while(hc06.available()>0){
    cmd += (char)hc06.read();
  }
  if (cmd != ""){
    // Serial.print("Command: ");
    // Serial.println(cmd);
    mode = cmd[0] - '0';
    error = cmd.substring(2).toInt();
    // Serial.print("Error: ");
    // Serial.println(error);
    // Serial.print("Modo: ");
    // Serial.println(mode);
    cmd = "";
  }
  return error, mode;
}

void FWD(int error){  // Avanzamos el vehiculo
  digitalWrite (PinIN1, LOW);  // Establece direccion de giro motor derecho 
  digitalWrite (PinIN2, HIGH); // Establece direccion de giro motor derecho
  digitalWrite (PinIN3, LOW);  // Establece direcccion de giro motor izquierdo
  digitalWrite (PinIN4, HIGH); // Establece dirccion de giro motor izquierdo
  SPEED_CTRL(error); // Modificamos la velocidad de los motores basado en el error
}

void TURN(int error){
  int nuevo_error = 0;
  int Speed_Check0 = 0;
  if(error > 5){ // Checamos si el angulo es positivo o negativo y configuramos los motores acorde 
  digitalWrite(PinIN1, LOW); 
  digitalWrite(PinIN2, HIGH);
  digitalWrite(PinIN3, HIGH);
  digitalWrite(PinIN4, LOW);
  }else if (error < -5){
  digitalWrite(PinIN1, HIGH);
  digitalWrite(PinIN2, LOW);
  digitalWrite(PinIN3, LOW);
  digitalWrite(PinIN4, HIGH);
  }

  abs_error = abs(error);
  nuevo_error = map(abs_error,90,0,0,90);
  SpeedControl0 = base_speed;
  SpeedControl0 = base_speed - nuevo_error;

  if(SpeedControl0 < 0){ // Evitamos los valores negativos de PWM
      SpeedControl0 = 0;
  }
  if(SpeedControl0 > 255){  // Evitamos valores mayores a 255
      SpeedControl0 = 255;
  } 
  if (SpeedControl0 < 25) { // Evitar que los motores esten zumbando a baja velocidad
    SpeedControl0 = 0;
  }
  // Panel de Validacion de datos
  Serial.println("Error");
  Serial.println(error);
  Serial.println("Nuevo error");
  Serial.println(nuevo_error);
  Serial.println("Speed 0");
  Serial.println(SpeedControl0);
  Serial.println("Control0");
  Serial.println(Speed_Check0);
  analogWrite (PWMSpeed0, SpeedControl0);
  analogWrite (PWMSpeed1, SpeedControl0);

}

void SPEED_CTRL(int error){
  int nuevo_error = 0;
  int Speed_Check0 = 0;
  int Speed_Check1 = 0;
  if(error > 5){   // Medimos la magnitud del error
    abs_error = abs(error);  // Obtenemos el valor absoluto del error
    nuevo_error = map(abs_error,0,100,50,150); // Remapeamos el error a un valor de PWM
    Speed_Check0 = base_speed - nuevo_error;  // Restamos a la velocidad del motor la magnitud del error
    SpeedControl0 = base_speed - nuevo_error;
    if(SpeedControl0<0){ // Evitamos los valores negativos de PWM
      SpeedControl0 = 0;
    }
    if(SpeedControl0>= 255){  // Evitamos valores mayores a 255
      SpeedControl0 = 255;
    } 
  }else if(error < -5){  // Medimos la magnitud del error
    abs_error = abs(error); // Obtenemos el valor absoluto del error
    nuevo_error = map(abs_error,0,200,0,255);  // Remapeamos el error a un valor de PWM
    Speed_Check1 = base_speed - nuevo_error;  // Restamos a la velocidad del motor la magnitud del error
    SpeedControl1 = base_speed - nuevo_error;
    if(SpeedControl1<0){  // Evitamos los valores negativos de PWM
      SpeedControl1 = 0;
    }
    if(SpeedControl1>= 255){  // Evitamos valores mayores a 255
      SpeedControl1 = 255;
    }
  }else {
    SpeedControl1 = base_speed;  // Regresamos a la velocidad base
    SpeedControl0 = base_speed;
  }

  if (SpeedControl0 < 70) { // Evitar que los motores esten zumbando a baja velocidad
    SpeedControl0 = 0;
  }
  if (SpeedControl1 < 70) {
    SpeedControl1 = 0;
  }
  
  //Panel de Validacion de datos
  Serial.println("Error");
  Serial.println(error);
  Serial.println("Nuevo error");
  Serial.println(nuevo_error);
  Serial.println("Speed 0");
  Serial.println(SpeedControl0);
  Serial.println("Speed 1");
  Serial.println(SpeedControl1);
  Serial.println("Control1");
  Serial.println(Speed_Check0);
  Serial.println("Control2");
  Serial.println(Speed_Check1);
  
  analogWrite (PWMSpeed0, SpeedControl0); // Establecemos velocidad de los motores 
  analogWrite (PWMSpeed1, SpeedControl1); 
}
