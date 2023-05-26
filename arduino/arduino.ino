#include <SoftwareSerial.h>
#define PWMSpeed0 11  // Velocidad motor derecha
#define PWMSpeed1 10    // Velocidad Motor izquierda
// Reversa izquierda == PinIN1 HIGH, PinIN2 LOW
// Avanza izquierda == PinIN1 LOW, PinIN2 HIGH
// Reversa derecha == PinIN3 HIGH, PinIN4 LOW
// Avanza derecha == PinIN3 LOW, PinIN4 HIGH
#define PinIN1 7    
#define PinIN2 6 
#define PinIN3 5
#define PinIN4 4
#define POT A0

SoftwareSerial hc06(8,9);
String cmd = "";
int error = 0;    // Error que recibimos
                      // Error positivo: a la izquierda de la linea de trayectoria
                      // Error negatico; a la derecha de la linea de trayectoria

int mode = 0;  // 0 inactive, 1 FWD, 2 TURN
int abs_error = 0;    // Valor absoluto del error que recibimos
int SpeedControl0 = 100;    // Velocidad de motor derecho
int SpeedControl1 = 100;    // Velocidad de motor izquierdo
const int base_speed = 120;   // Velocidad de motores base
int sim_error = 0;    // Error simulado por potenciometro 
const int error_threshold = 0;    // Barrera minima para reaccionar al error
const int buzz_threshold = 80;    // Barrera para evitar el zumbido de los motores por bajo voltaje o corriente
const int max_error_ang = 90;   // Error maximo de angulo que se puede recibir
const int motor_offset = 10;    // Offset para hacer que los motores giren a la misma velocidad
const int max_error_line = 100;    // Error maximo que se recibe de la linea
const int min_error_line = 1;   // Error minimo que se recibe de la linea
const int strike_speed = 160;   // Velocidad para realizar el golpe a la pelota

void setup()
{
  Serial.begin(9600);   // Iniciamos monitor Serial
  hc06.begin(9600);   // Iniciamos la coneccion bluchu
  // Definimos los PINS de control de velocidad como PINS de salida
  pinMode(PWMSpeed0, OUTPUT);   
  pinMode(PWMSpeed1, OUTPUT);
  // Definimos los PINS de contorl de direccion de motores como PINS de salida 
  pinMode(PinIN1, OUTPUT);
  pinMode(PinIN2, OUTPUT);
  pinMode(PinIN3, OUTPUT);
  pinMode(PinIN4, OUTPUT);
}

void loop()
{ 
  Serial.println("\n\n");   // Generamos un espacio para diferentes iteraciones de debug
  error, mode = BT();   // Obtenemos los valores de modo y error del modulo bluchu
  // Serial.print("Error: "); 
  // Serial.println(error);   // Visualizamos el error que estamos recibiendo
  Serial.print("Modo de Operacion: ");
  Serial.println(mode);   // Visualizamos el modo que estamos recibiendo
  // error = POT_ERROR();  // Simulamos un error por medio de un potenciometro
  delay(40); // Delay para lograr leer los datos del string de bluchu
  if (mode == 1){
    FWD(error);   // Avanzamos el coche modulando la velocidad dependiendo del error
  }else if(mode == 2){
    TURN_SINGLE();    // Giramos el coche en un sola dirreccion hasta que quedemos apuntando a la direccion correcta 
  }else if(mode == 3){
    STRIKE();
  }else {
    OFF();    // Apagamos los motores y ponemos la velocidad que reciben en 0 
  }
}

int POT_ERROR(){ // Simulacion de un error con un potenciometro
  sim_error = analogRead(POT);  // Obtenemos la lectura del voltaje del potencionmetro con ADC
  if(sim_error < 470){ // Si esta en el cuadrante derecho del potenciometro
    sim_error = map(sim_error,470,0,0,-255);  // Generamos valores negativos al simular el error
    
  }else if(sim_error > 550){  // Si esta en cuadrante izquierdo del potenciometro
    sim_error = map(sim_error,550,1023,0,255);  // Generamos valores positivos al simular el error
    
  }else { // Si el valor del error simulado es demasiado pequeno se envia un error 0
          // Simula un error de estado estacionario
    sim_error = 0;
  }
  return sim_error; // Regresamos el error
}

void OFF(){ // Funcion 
  // Apagamos todos los motores
  digitalWrite (PinIN1, LOW); 
  digitalWrite (PinIN2, LOW);
  digitalWrite (PinIN3, LOW);
  digitalWrite (PinIN4, LOW);
  // Ponemos la velocidad de los motores en 0 
  analogWrite (PWMSpeed0, 0);
  analogWrite (PWMSpeed1, 0);
}

int BT(){ // Obtenemos el modo de funcionamiento y el error del bluchu
  while(hc06.available()>0){    // Buscamos que el bluchu este disponible
    cmd += (char)hc06.read();   // Leemos los caracteres del bluchu
  }
  if (cmd != ""){ 
    // Serial.print("Command: "); 
    // Serial.println(cmd);   // Verificamos el string que estamos recibiendo
    mode = cmd[0] - '0';    // Separamos el string en modo
    error = cmd.substring(2).toInt();   // Separamos el stiring en error
    // Serial.print("Error: ");
    // Serial.println(error);   // Verificamos que estamos recibiendo el error
    // Serial.print("Modo: "); 
    // Serial.println(mode);    // Verificamos que estamos recibiendo el modo
    cmd = "";   // Limpiamos la memoria del comando
  }
  return error, mode;   // Retornamos el modo y el error que se recibo del bluchu
}

void FWD(int error){  // Avanzamos el vehiculo dependiedo del error que se recibe

  // Avanza izquierda == PinIN1 LOW, PinIN2 HIGH
  digitalWrite (PinIN1, LOW);  
  digitalWrite (PinIN2, HIGH); 

  // Avanza derecha == PinIN3 LOW, PinIN4 HIGH
  digitalWrite (PinIN3, LOW);  
  digitalWrite (PinIN4, HIGH); 

  SPEED_CTRL(error); // Modificamos la velocidad de los motores basado en el error recibido
}

void TURN(int error){   // Giramos el coche en la dirreccion que disminuya el error de angulo
  int nuevo_error = 0;    // Variable temporal para el remapeo del error 
  int Speed_Check0 = 0;   // Variable temporal para visualizar la Ley de Control

  if(error > error_threshold){    // Checamos si el angulo es positivo 
// Giro en sentido negativo
  // Avanza izquierda == PinIN1 LOW, PinIN2 HIGH
  digitalWrite(PinIN1, LOW); 
  digitalWrite(PinIN2, HIGH);
  // Reversa derecha == PinIN3 HIGH, PinIN4 LOW
  digitalWrite(PinIN3, HIGH);
  digitalWrite(PinIN4, LOW);
  
  }else if (error < -error_threshold){    // Checamos si el angulo es negativo
// Giro en sentido positivo
  // Reversa izquierda == PinIN1 HIGH, PinIN2 LOW
  digitalWrite(PinIN1, HIGH);
  digitalWrite(PinIN2, LOW);
  // Avanza derecha == PinIN3 LOW, PinIN4 HIGH
  digitalWrite(PinIN3, LOW);
  digitalWrite(PinIN4, HIGH);
  }
  
  abs_error = abs(error);   // Obtenemos el valor absoluto del error
  nuevo_error = map(abs_error,0,90,base_speed-buzz_threshold,0);// map(variable, error minimo, error maximo, valor restado minimo, valor restado maximo)
  SpeedControl0 = base_speed - nuevo_error;

  if(SpeedControl0 < 0){ // Evitamos los valores negativos de PWM
      SpeedControl0 = 0;
  }
  if(SpeedControl0 > 255){  // Evitamos valores mayores a 255
      SpeedControl0 = 255;
  } 
  if (SpeedControl0 < buzz_threshold) { // Evitar que los motores esten zumbando a baja velocidad
    SpeedControl0 = 0;
  }
  // Panel de Validacion de datos
  Serial.print("Error ");
  Serial.println(error);
  Serial.print("Error Remapeado: ");
  Serial.println(nuevo_error);
  Serial.print("Velocidad de los Motores:");
  Serial.println(SpeedControl0);
  Serial.print("Ley de Control: ");
  Serial.println(Speed_Check0);
  analogWrite (PWMSpeed0, SpeedControl0);
  analogWrite (PWMSpeed1, SpeedControl0 - motor_offset);
}

void TURN_SINGLE(){
  if (error > 0){
// Avanza izquierda == PinIN1 LOW, PinIN2 HIGH
    digitalWrite(PinIN1, LOW); 
    digitalWrite(PinIN2, HIGH);
// Reversa derecha == PinIN3 HIGH, PinIN4 LOW
    digitalWrite(PinIN3, HIGH);
    digitalWrite(PinIN4, LOW);
  } else {
// Reversa izquierda == PinIN1 HIGH, PinIN2 LOW
    digitalWrite(PinIN1, HIGH); 
    digitalWrite(PinIN2, LOW);
// Avanza derecha == PinIN3 LOW, PinIN4 HIGH
    digitalWrite(PinIN3, LOW);
    digitalWrite(PinIN4, HIGH);
  }
  SpeedControl0 = buzz_threshold + motor_offset;    //  Aseguramos que los motores funcionen al asignarles valores de PWM mayores a los que generan zumbidos en los motores
  Serial.print("Error");
  Serial.println(error);    // Verificamos el error que estamos recibiendo
  Serial.print("Velocidad de los Motores:");
  Serial.println(SpeedControl0);    // Verificamos la velocidad que estamos enviando a los motores
  analogWrite (PWMSpeed0, 100);   // Enviamos el PWM al motor derecho
  analogWrite (PWMSpeed1, 100);   // Enviamos el PWM al motor izquierdo
  delay(100);   // tiempo rotacion
  OFF();    // Apagamos los motores
  mode = 0;   // Asignamos el modo de operacion a 0
  delay(100);   // Tiempo de espera
}

void SPEED_CTRL(int error){   // Funcion que contorla la velocidad de los motores cuando el coche va en linea recta dependiendo del error
  int nuevo_error = 0;
  int Speed_Check0 = 0;
  int Speed_Check1 = 0;

  if(error > error_threshold || error < -error_threshold){   // Medimos que la magnitud del error sea mayor al la barrera de error minimo que se reconoce
    // Como se tiene un error del la posicion del coche respecto al spline, cuando se esta por encima del spline el error es positivo,
    // cuando se esta por debajo del spline el error es negativo, esto permite que se sume el error a la velocidad del motor izquierdo
    // y se reste el error a la velocidad del motor derecho, cuando el error cambie de signo, tambien cambiaria las operaciones que estamos realizando, 
    // haciendo que la compensacion de trayectoria se realiza de manera correcta en ambos casos.

    SpeedControl0 = base_speed - error;   // Se le resta el error a la velocidad del motor derecho
    SpeedControl1 = base_speed + error - motor_offset;    // Se le suma el error a la velocidad del motor izquierdo tomando en cuenta el offset
                                                          // de velocidad entre los motores

    if(SpeedControl0<0){ // Evitamos los valores negativos de PWM en el motor dercho
      SpeedControl0 = 0;
    }
    if(SpeedControl0>= 255){  // Evitamos valores mayores a 255 de PWM en el motor derecho
      SpeedControl0 = 255;
    } 
    if(SpeedControl1<0){  // Evitamos los valores negativos de PWM en el motor izquierdo
      SpeedControl1 = 0;
    }
    if(SpeedControl1>= 255){  // Evitamos valores mayores a 255 de PWM en el motor izquierdo
      SpeedControl1 = 255;
    }
  } else {    // En el caso que el error sea tan pequeno que sea "insignificante", el coche avanza en linea recta
    SpeedControl1 = base_speed  - motor_offset;    // Compensamos la diferencia de velocidad que se tiene entre los motores
    SpeedControl0 = base_speed;
  }

  //   abs_error = abs(error);  // Obtenemos el valor absoluto del error
  //   nuevo_error = map(abs_error,min_error_line,max_error_line,0,base_speed-buzz_threshold); // Remapeamos el error a un valor de PWM
  //   Speed_Check0 = base_speed - nuevo_error;  // Restamos a la velocidad del motor la magnitud del error
  //   SpeedControl0 = base_speed - abs_error;
  //   SpeedControl1 = base_speed + abs_error - motor_offset;
  //   if(SpeedControl0<0){ // Evitamos los valores negativos de PWM
  //     SpeedControl0 = 0;
  //   }
  //   if(SpeedControl0>= 255){  // Evitamos valores mayores a 255
  //     SpeedControl0 = 255;
  //   } 
  //   if(SpeedControl1<0){  // Evitamos los valores negativos de PWM
  //     SpeedControl1 = 0;
  //   }
  //   if(SpeedControl1>= 255){  // Evitamos valores mayores a 255
  //     SpeedControl1 = 255;
  //   }

  // }else if(error < -error_threshold){  // Medimos la magnitud del error
  //   abs_error = abs(error); // Obtenemos el valor absoluto del error
  //   nuevo_error = map(abs_error,min_error_line,max_error_line,0,base_speed-buzz_threshold);  // Remapeamos el error a un valor de PWM
  //   Speed_Check1 = base_speed - nuevo_error - motor_offset;  // Restamos a la velocidad del motor la magnitud del error
  //   SpeedControl1 = base_speed - abs_error - motor_offset;
  //   SpeedControl0 = base_speed + abs_error;

  //   if(SpeedControl1<0){  // Evitamos los valores negativos de PWM
  //     SpeedControl1 = 0;
  //   }
  //   if(SpeedControl1>= 255){  // Evitamos valores mayores a 255
  //     SpeedControl1 = 255;
  //   }
  //   if(SpeedControl0<0){ // Evitamos los valores negativos de PWM
  //     SpeedControl0 = 0;
  //   }
  //   if(SpeedControl0>= 255){  // Evitamos valores mayores a 255
  //     SpeedControl0 = 255;
  //   } 
  // }else {
  //   SpeedControl1 = base_speed - motor_offset;  // Regresamos a la velocidad base
  //   SpeedControl0 = base_speed ;
  // }

  if (SpeedControl0 < buzz_threshold ) { // Evitar que los motores esten zumbando a baja velocidad
    SpeedControl0 = 0;
  }
  if (SpeedControl1 < buzz_threshold - motor_offset) {
    SpeedControl1 = 0;
    // Debido a una ocasional fuga de corriente al motor izquierdo, "sigue funcionando aunque le mandemos un PWM de 0",
    // se apaga el motor izquierdo a partir de los PINS de control
    digitalWrite(PinIN1, LOW); 
    digitalWrite(PinIN2, LOW);
  }else{
    // Debido a un ocasional caso en el que los motores no quieren volver a funcionar cuando se les apaga, se agrego esta linea
    // para forzar a que vuelvan a encender
    digitalWrite(PinIN1, LOW); 
    digitalWrite(PinIN2, HIGH);
  }
  
  //Panel de Validacion de datos
  Serial.print("Error");
  Serial.println(error);
  Serial.print("Error Remapeado");
  Serial.println(nuevo_error);
  Serial.print("Velocidad Derecha ");
  Serial.println(SpeedControl0);
  Serial.print("Velocidad Izquierda ");
  Serial.println(SpeedControl1);
  Serial.print("Ley de Control Derecha: ");
  Serial.println(Speed_Check0);
  Serial.print("Ley de Control Izquierda: ");
  Serial.println(Speed_Check1);
  analogWrite (PWMSpeed0, SpeedControl0); // Establecemos velocidad del motor derecho
  analogWrite (PWMSpeed1, SpeedControl1 - motor_offset);    // Establecemos velocidad del motor izquierdo
  delay(100); // tiempo rotacion
  OFF();
  mode = 0;
  delay(100);
}
void STRIKE(){
   // Avanza izquierda == PinIN1 LOW, PinIN2 HIGH
  digitalWrite (PinIN1, LOW);  
  digitalWrite (PinIN2, HIGH); 

  // Avanza derecha == PinIN3 LOW, PinIN4 HIGH
  digitalWrite (PinIN3, LOW);  
  digitalWrite (PinIN4, HIGH); 
  analogWrite (PWMSpeed0, strike_speed); // Establecemos velocidad del motor derecho
  analogWrite (PWMSpeed1, strike_speed - motor_offset);   // Establecemos velocidad del motor izquierdo


}
