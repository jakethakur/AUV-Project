/*********************
 *10 to GPS Module TX*
 *09 to GPS Module RX*
 *********************/
 
//source: http://www.instructables.com/id/How-to-Communicate-Neo-6M-GPS-to-Arduino/

#include <SoftwareSerial.h>
#include <TinyGPS.h>

SoftwareSerial mySerial(10, 11);
TinyGPS gps;

  long lat = 0;
  long lon = 0;

//51.36513, -0.18960

  long destinationLat = 51.36513;
  long destinationLon = -0.18960;

void setup()  
{
  // make the car face north!

  
  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  Serial.println("hi");
  
  pinMode(13, OUTPUT); // forward
  pinMode(12, OUTPUT); // right
  
  delay(1000);
}

float latitude(TinyGPS &gps) {
  long lat, lon;
  unsigned long age;
  gps.get_position(&lat, &lon, &age);
  return lat;
}

float longitude(TinyGPS &gps) {
  long lat, lon;
  unsigned long age;
  gps.get_position(&lat, &lon, &age);
  return lon;
}

void longLat() {
    if (mySerial.available()) 
    
    {
      char c = mySerial.read();
      if (gps.encode(c)) 
      {
        lat = longitude(gps);
        lon = longitude(gps);
      }
    }
}

void printFloat(double number, int digits)
{
  // Handle negative numbers
  if (number < 0.0) 
  {
     Serial.print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0) 
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint;
  }
}

const double pi= 3.14159265358979;
double distance(double lat1, double long1, double lat2, double long2) { //find distance between two lat-long pairs
  //an application of the Haversine formula
  
  //returns answer in metres
  
  double R = 6371e3; //radius of the Earth, in metres (change to km for answer to be in km, etc)

  double lat1Radians = toRadians(lat1);
  double lat2Radians = toRadians(lat2);

  double deltaLat = toRadians(lat2-lat1);
  double deltaLong = toRadians(long2-long1);

  double a = sin(deltaLat/2) * sin(deltaLat/2) +
          cos(lat1Radians) * cos(lat2Radians) *
          sin(deltaLong/2) * sin(deltaLong/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));

  double d = R * c;
  
  return d;
}

double distanceAlternative(double lat1, double long1, double lat2, double long2) { //find distance between two lat-long pairs (alternate, easier method)
  //an application of the cosine rule
  
  //returns answer in metres
  
  double R = 6371e3; //radius of the Earth, in metres (change to km for answer to be in km, etc)

  double lat1Radians = toRadians(lat1);
  double lat2Radians = toRadians(lat2);
  
  double deltaLong = toRadians(long2-long1);
  
  double d = acos( sin(lat1Radians) * sin(lat2Radians) + cos(lat1Radians) * cos(lat2Radians) * cos(deltaLong) ) * R;

  return d;
}

double bearing(double lat1, double long1, double lat2, double long2) { //find bearing between two lat-long pairs
  //untested so far...
  
  //returns answer in radians (can easily be converted to degrees if necessary)
  
  double y = sin(long2-long1) * cos(lat2);
  double x = cos(lat1)*sin(lat2) -
          sin(lat1)*cos(lat2)*cos(long2-long1);
  
  double bearing = atan2(y, x);
  
  return toDegrees(bearing);
}

double toRadians(double degrees) { //convert degrees to radians (cpp functions accept radians only)
  return degrees * pi / 180;
}

double toDegrees(double radian) {
  return radian / pi * 180;
}


void moveForward(int distance) { // distance is in cm
  digitalWrite(13, HIGH);
  delay(distance * 9.32);
  digitalWrite(13, LOW);
}

void turnRight(int rotation) { // rotation is in degrees
  digitalWrite(12, HIGH);
  digitalWrite(13, HIGH);
  delay(rotation * 11.76);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
}

void loop() // run over and over
{
  longLat();
  Serial.println(bearing(lat, lon, destinationLat, destinationLon));
  Serial.println(distance(lat, lon, destinationLat, destinationLon));
  if (lat != 0) {
    turnRight(bearing(lat, lon, destinationLat, destinationLon));
    moveForward(distance(lat, lon, destinationLat, destinationLon));
  }

  
  delay(1000);
}
