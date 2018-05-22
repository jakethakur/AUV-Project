
//main reference: https://www.movable-type.co.uk/scripts/latlong.html
//#include <math.h>
//#include <stdlib.h>
//using namespace std;

void setup()  
{
  Serial.begin(9600);
  //both functions are slightly different to eachother (by about 10^-9) - I would assume haversine is more accurate?
  
  Serial.println(distance(36.12, -86.67, 33.94, -118.40));
  Serial.println(distanceAlternative(36.12, -86.67, 33.94, -118.40));
  
  Serial.println(bearing(36.12, -86.67, 33.94, -118.40));
}

void loop() // run over and over
{
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
  
  return bearing;
}

double toRadians(double degrees) { //convert degrees to radians (js functions accept radians only)
  return degrees * pi / 180;
}
