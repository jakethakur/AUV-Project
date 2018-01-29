//https://www.movable-type.co.uk/scripts/latlong.html

function distance(lat1, long1, lat2, long2) { //find distance between two lat-long pairs
  //an application of the Haversine formula
  
  //returns answer in metres
  
  var R = 6371e3; //radius of the Earth, in metres (change to km for answer to be in km, etc)

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

function distanceAlternative(lat1, long1, lat2, long2) { //find distance between two lat-long pairs (alternate, more obvious method)
  //an application of the cosine rule
  
  //returns answer in metres
  
  var R = 6371e3; //radius of the Earth, in metres (change to km for answer to be in km, etc)

  var lat1Radians = toRadians(lat1);
  var lat2Radians = toRadians(lat2);
  
  var deltaLong = toRadians(long2-long1);
  
  var d = Math.acos( Math.sin(lat1Radians) * Math.sin(lat2Radians) + Math.cos(lat1Radians) * Math.cos(lat2Radians) * Math.cos(deltaLong) ) * R;
}

function toRadians(degrees) { //convert degrees to radians (js functions accept radians only)
  return degrees * Math.PI / 180;
}

console.log(distance(36.12, -86.67, 33.94, -118.40));
console.log(distanceAlternative(36.12, -86.67, 33.94, -118.40));
