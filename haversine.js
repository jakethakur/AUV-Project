//an application of the Haversine formula, in order to calculate the distance between two longitude and latitudes
//https://www.movable-type.co.uk/scripts/latlong.html

//returns answer in metres

function haversine(lat1, long1, lat2, long2) {
  var R = 6371e3; // radius of the Earth, in metres

  var lat1Radians = toRadians(lat1);
  var lat2Radians = toRadians(lat2);

  var deltaLat = toRadians(lat2-lat1);
  var deltaLong = toRadians(long2-long1);

  var a = Math.sin(deltaLat/2) * Math.sin(deltaLat/2) +
          Math.cos(lat1Radians) * Math.cos(lat2Radians) *
          Math.sin(deltaLong/2) * Math.sin(deltaLong/2);
  var c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));

  var d = R * c;
  
  return d;
}

function toRadians(degrees) {
  return degrees * Math.PI / 180;
}

console.log(haversine(36.12, -86.67, 33.94, -118.40));
