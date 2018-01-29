//an application of the Haversine formula, in order to calculate the distance between two longitude and latitudes
//https://www.movable-type.co.uk/scripts/latlong.html

function haversine(lat1, long1, lat2, long2) {
  var lat1 = lat1; //first latitude
  var long1 = long1; //first longitude

  var lat2 = lat2; //second latitude
  var long2 = long2; //second longitude

  var R = 6371e3; // radius of the Earth, in metres

  var φ1 = lat1.toRadians();
  var φ2 = lat2.toRadians();

  var Δφ = (lat2-lat1).toRadians();
  var Δλ = (long2-long1).toRadians();

  var a = Math.sin(Δφ/2) * Math.sin(Δφ/2) +
          Math.cos(φ1) * Math.cos(φ2) *
          Math.sin(Δλ/2) * Math.sin(Δλ/2);
  var c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));

  var d = R * c;
}

console.log(haversine(36.12, -86.67, 33.94, -118.40));
