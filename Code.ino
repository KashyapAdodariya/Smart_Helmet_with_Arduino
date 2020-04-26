#include <math.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#define datain A5 
#define ledpin 7
#define dataout 12

unsigned int temp;


/* Create object named bt of the class SoftwareSerial */
SoftwareSerial GPS_SoftSerial(4, 3);/* (Rx, Tx) */
/* Create an object named gps of the class TinyGPSPlus */
TinyGPSPlus gps;      


SoftwareSerial mySerial(9, 10);
const int x_out = A1; /* connect x_out of module to A1 of UNO board */
const int y_out = A2; /* connect y_out of module to A2 of UNO board */
const int z_out = A3; /* connect z_out of module to A3 of UNO board */

void setup()
{
  pinMode(dataout, OUTPUT);
  pinMode (ledpin, OUTPUT);
  mySerial.begin(9600);
  GPS_SoftSerial.begin(9600);
  Serial.begin(9600);
  delay(100);
}

void loop()
{
  int x_adc_value, y_adc_value, z_adc_value;
  double x, y, z, g;
  double roll, pitch, yaw;
  temp=analogRead(datain);
   
    if(temp>0)
    {
     digitalWrite(dataout, HIGH);
     digitalWrite(ledpin, HIGH);
    }
   
   else
   {
     digitalWrite(dataout,LOW);
     digitalWrite(ledpin, LOW);
   }
  x_adc_value = analogRead(x_out); /* Digital value of voltage on x_out pin */
  y_adc_value = analogRead(y_out); /* Digital value of voltage on y_out pin */
  z_adc_value = analogRead(z_out); /* Digital value of voltage on z_out pin */
        
  Serial.print("x_ = ");
  Serial.print(x_adc_value);
  Serial.print("\t\t");
  Serial.print("y_ = ");
  Serial.print(y_adc_value);
  Serial.print("\t\t");
  Serial.print("z_ = ");
  Serial.print(z_adc_value);
  Serial.print("\t\t");
  delay(100);
 
  x = ( ( ( (double)(x_adc_value * 5)/1024) - 1.65 ) / 0.330 ); /* Acceleration in x-direction in g units */
  y = ( ( ( (double)(y_adc_value * 5)/1024) - 1.65 ) / 0.330 ); /* Acceleration in y-direction in g units */
  z = ( ( ( (double)(z_adc_value * 5)/1024) - 1.80 ) / 0.330 ); /* Acceleration in z-direction in g units */
  g =sqrt(x * x + y * y + z * z);
  if(g>1.31)
  {
  
     SendMessage();   
  }
  


  Serial.print("g = ");
  Serial.print(g);
  Serial.print("\n\n");
  delay(1000);
}

void SendMessage()
  


{
  smartDelay(1000); /* Generate precise delay of 1ms */
        unsigned long start;
        double lat_val, lng_val, alt_m_val;
        bool loc_valid, alt_valid;
        lat_val = gps.location.lat(); /* Get latitude data */
        loc_valid = gps.location.isValid(); /* Check if valid location data is available */
        lng_val = gps.location.lng(); /* Get longtitude data */
        alt_m_val = gps.altitude.meters();  /* Get altitude data in meters */
        alt_valid = gps.altitude.isValid(); /* Check if valid altitude data is available */ 
  if (!loc_valid)
        {          
          Serial.print("Latitude : ");
          Serial.println("*****");
          Serial.print("Longitude : ");
          Serial.println("*****");
        }
        else
        {
          Serial.print("Latitude : ");
          Serial.println(lat_val, 6);
          Serial.print("\t");
          Serial.print("Longitude : ");
          Serial.println(lng_val, 6);
          Serial.print("\t");
 
        }
        if (!alt_valid)
        {
          Serial.print("Altitude : ");
          Serial.println("*****");
        }
        else
        {
          Serial.print("Altitude : ");
          Serial.println(alt_m_val, 6);    
        }
        
  mySerial.println("AT+CMGF=1");    //Sets the GSM Module in Text Mode

  delay(1000);  // Delay of 1000 milli seconds or 1 second

  mySerial.println("AT+CMGS=\"+918320387797\"\r"); // Replace x with mobile number

  delay(1000);
        mySerial.println("emergncy for accident \n Latitude:23.029541 \n Longitude:72.553405 \n altitude: 52.099958 \n");// The SMS text you want to send
        //mySerial.println(lat_val);
        //mySerial.println(lng_val);
  delay(100);

  mySerial.println((char)26);// ASCII code of CTRL+Z

  delay(1000);
        
}
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (GPS_SoftSerial.available())  /* Encode data read from GPS while data is available on serial port */
      gps.encode(GPS_SoftSerial.read());
/* Encode basically is used to parse the string received by the GPS and to store it in a buffer so that information can be extracted from it */
  } while (millis() - start < ms);
}


  
