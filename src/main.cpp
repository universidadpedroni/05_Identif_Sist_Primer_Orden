#include <Arduino.h>
#include <blink.h>
#include <SoftwareSerial.h>
/*
Simple sistema de adquisición de datos para determinar la respuesta temporal de un sistema de primer orden.
El sistema consiste en una resistencia en serie y un capacitor a masa. Un simple filtro RC.
Circuito:
entrada ------ | RESISTENCIA | ------ salida
                                |
                            _________
                            CAPACITOR
                            ¯¯¯¯|¯¯¯¯¯
                                |
                                |
                                GND      

Ejemplo: Si R = 100 K y C = 1uF, Tau = R*C = 0.1s, Ts = 0.4s
K=1

Se entiende por "sistema" a este filtro pasabajos.
El Arduino es usado solamente como generador de señal y adquisición de datos.
*/                                
/*

Para comparar los datos con la FdT teórica:
s=tf('s');
R=100e3;
C=1e-6;
G=1/(R*C*s+1);
ts=4*R*C;
[y,t]=step(G,1);


// IDENTIFICACION
yss=4.91;
yss_098=0.98*yss
tss=0.43;
Ts=tss/4;
Ks=yss/5

Gs=Ks/(Ts*s+1)
[y1,t1]=step(Gs,1);

plot(d(:,1)-1,d(:,2),d(:,1)-1,d(:,3));
hold on
plot(t,5 * y)
grid

plot(t1,5 * y1)
** en d se guardan los datos que salen por el puerto serie. Lo mejor es guardar 1 seg. 

*/

SoftwareSerial mySerial(10,11); // Rx,Tx

void Telemetria(unsigned long interval, float t, float y, float u);
float PulsoEntrada(unsigned long interval, float u);

int pinEntrada = A0;  // En vez de usar un  escalón, se usa un pulso de período largo. 
int pinSalida  = A2;  // Pin de lectura
int pinGND = A4;      // Pin usado como GND del circuito.

blink parpadeo(LED_BUILTIN);

void setup() {
  delay(5000);
  pinMode(pinEntrada,OUTPUT);
  digitalWrite(pinEntrada,LOW);
  pinMode(pinGND,OUTPUT);
  digitalWrite(pinGND,LOW);
  Serial.begin(115200);
  mySerial.begin(115200);
  mySerial.print('y');
  mySerial.print(',');
  mySerial.println('u');
  
  parpadeo.init();
  
  delay(1000);

}

void loop() 
{
  static long millisInicial = millis();
  static float entrada = 0;  // Vin del sistema
  
  // Se actualiza el valor de entrada del sistema.
  entrada == 5 ? digitalWrite(pinEntrada,HIGH): digitalWrite(pinEntrada, LOW);
  
  // Se adquiere la señal de salida del sistema.
  float y = 5 * (float)analogRead(pinSalida) / 1023; // Vout
  
  // Se adquiere el instante de tiempo para el cual se toma la muestra de la señal
  float t = (float) (millis() - millisInicial) / 1000;
  
  // Se envian los datos adquiridos por el puerto serie.
  Telemetria(10, t, y, entrada);

  // Se actualiza el valor de la entrada para formar el tren de pulsos.
  entrada = PulsoEntrada(1000, entrada);
  
  // La prueba de vida del Arduino.
  parpadeo.update(250);
  

}

float PulsoEntrada(unsigned long interval, float u)
{
  
	static unsigned long previousMillis = 0;        // will store last time LED was updated
	unsigned long currentMillis = millis();
		
	if(currentMillis - previousMillis > interval) 
	{
		previousMillis = currentMillis;
    u == 0? u = 5 : u = 0;
		
	}
  return u;
    
}


void Telemetria(unsigned long interval, float t, float y, float u)
{
  
	static unsigned long previousMillis = 0;        // will store last time LED was updated
	unsigned long currentMillis = millis();
		
	if(currentMillis - previousMillis > interval) 
	{
		previousMillis = currentMillis;
    
    Serial.print(t);
    Serial.print("\t");
    Serial.print(y);
    Serial.print("\t");
    Serial.print(u);
    Serial.println();
		//mySerial.print(t);
    //mySerial.print("\t");
    mySerial.print(y);
    mySerial.print("\t");
    mySerial.print(u);
    mySerial.println();
	}
    
}