var openrouteservice = require('openrouteservice-js');

var Matrix = new openrouteservice.Matrix({
    api_key: "NA",
    host: "im-pittyvaich.oth-regensburg.de:22023"       //server: ors-app:8080
});


// locations has to be an array of longitudes and latitudes
// example: [[<longitude>, <latitude>], [<longitude>, <latitude>], [...], ...]
function createDistanceMatrix(locations) {

    console.log(`Creating distance matrix for ${locations.length} locations`);

    return Matrix.calculate({
        locations: locations,
        profile: "driving-car",
        sources: ['all'],
        destinations: ['all'],
        host: "http://im-pittyvaich.oth-regensburg.de:22023/ors",    // server: http:ors-app:8080
        metrics: ['distance'],
        units: 'm'
    })
    .then(resp => resp.distances)
    .catch(function(err) {
        console.log(`Error: ${err}`);
    });
}

module.exports = {
    createDistMat: createDistanceMatrix
};