const fs = require('fs');
const orsConsumer = require('./orsApiConsumer');
const defaults = require('./algorithmDefaults');


const dir = 'inputFiles/';

class InputFileClass {

    constructor(dimension, edgeWeightType, distancePerVehicle, capacity, nodes, distances, numberVeh,
                numberOfIterationsWithoutImprov, timeLimit, filename) {
        this.dimension = dimension;
        this.edgeWeightType = edgeWeightType;
        this.distancePerVehicle = distancePerVehicle;
        this.capacity = capacity;
        this.nodes = nodes;
        this.distances = distances;
        this.numberVehicles = numberVeh;
        this.numberOfIterationsWithoutImprov = numberOfIterationsWithoutImprov;
        this.timeLimit = timeLimit;
        this.filename = filename;
    }
}

class Node {
    
    constructor(algId, realId, longitude, latitude, demand, relatedPickupNode, relatedDeliveryNode) {
        this.algId = algId;
        this.realId = realId;
        this.longitude = longitude;
        this.latitude = latitude;
        this.demand = demand;
        this.relatedPickupNode = relatedPickupNode;
        this.relatedDeliveryNode = relatedDeliveryNode;
    }
}

function returnDummyData(jsonData) {
    var ids = [];
    var routes = [];
    var route1 = new Object();
    var route2 = new Object();

    var transportRequests = jsonData.transport_requests;

    for (var idx in transportRequests) {
        ids.push(transportRequests[idx].pickup.id);
        ids.push(transportRequests[idx].drop_off.id);
    }

    ids.sort((a,b) => 0.5 - Math.random());

    var sliceIdx = Math.floor(Math.random() * ids.length);       // between 

    route1.id = 1;
    route1.order = ids.slice(0, sliceIdx);

    route2.id = 2;
    route2.order = ids.slice(sliceIdx, ids.length + 1);

    routes.push(route1);
    routes.push(route2);

    return routes;
}


async function createInputFile(jsonData) {

    var filename = `cvrp_${(new Date().toJSON().slice(0,19))}`;
    var nodes = getNodes(jsonData);
    var distances = await getDistances(nodes);
    var inputFileObj = createInputFileObject(jsonData, distances, nodes, filename);

    writeToFile(inputFileObj);

    return inputFileObj;
}

function createInputFileObject(jsonData, distances, nodes, filename) {
    
    var inputFileObj = new InputFileClass();

    inputFileObj.dimension = nodes.length;
    inputFileObj.distancePerVehicle = 'distance_per_vehicle' in jsonData ? jsonData.distance_per_vehicle : defaults.algDefaults.distancePerVehicle;
    inputFileObj.edgeWeightType = 'edge_weight_type' in jsonData ? jsonData.edge_weight_type : defaults.algDefaults.edgeWeightType;
    inputFileObj.capacity = 'capacity' in jsonData ? jsonData.capacity : defaults.algDefaults.capacity;
    inputFileObj.numberOfIterationsWithoutImprov = 'options.it' in jsonData ? jsonData.options.it : defaults.algDefaults.nrOfItWithoutImprov;
    inputFileObj.timeLimit = 'options.t' in jsonData ? jsonData.options.timeLimit : -1;
    inputFileObj.numberVehicles = jsonData.options.nv;      // is required in the json request
    inputFileObj.distances = distances;
    inputFileObj.nodes = nodes;
    inputFileObj.filename = filename;

    return inputFileObj;
}


function getNodes(jsonData) {

    var nodes = [];
    var algId = 1;
    var requests = jsonData.transport_requests;
    var dimension = requests.length * 2;            // 1 request always is one pickup and one drop off node

    var depotNode = new Node(algId++, jsonData.depot.id, jsonData.depot.longitude, jsonData.depot.latitude, 0, -1, -1);

    nodes.push(depotNode);

    for (var idx in requests) {
        
        var pickup = requests[idx].pickup;
        var dropOff = requests[idx].drop_off;
        var pickUpTempIdx = algId;
        var dropOffTempIdx = algId + dimension/2

        var currentPickupNode = new Node(pickUpTempIdx, pickup.id, pickup.location.longitude, pickup.location.latitude, 1, -1, dropOffTempIdx);
        var currentDropOffNode = new Node(dropOffTempIdx, dropOff.id, dropOff.location.longitude, dropOff.location.latitude, -1, pickUpTempIdx, -1);

        nodes[pickUpTempIdx-1] = currentPickupNode;
        nodes[dropOffTempIdx-1] = currentDropOffNode;

        algId++;
    }

    return nodes;
}


async function getDistances(nodes) {
    var locations = [];

    for (var idx in nodes) {
        var longLat = [];
        
        longLat.push(nodes[idx].longitude);
        longLat.push(nodes[idx].latitude);

        locations.push(longLat);
    }

    var dist = await orsConsumer.createDistMat(locations);
    return dist;
}

function writeToFile(inputFileObject) {

    var filename = `${dir}${inputFileObject.filename}`;
    var nodes = inputFileObject.nodes;
    var distances = inputFileObject.distances;
    var fileData = "";

    fileData += (`DIMENSION : ${inputFileObject.dimension}\n`);
    fileData += (`EDGE_WEIGHT_TYPE : ${inputFileObject.edgeWeightType}\n`);
    fileData += (`DISTANCE : ${inputFileObject.distancePerVehicle}\n`);
    fileData += (`CAPACITY : ${inputFileObject.capacity}\n`);
    fileData += ('NODE_COORD_SECTION\n');
    
    for (var idx in nodes) {
        var node = nodes[idx];

        fileData += (`${node.algId} ${node.realId} ${node.longitude} ${node.latitude} ${node.relatedDeliveryNode} ${node.relatedPickupNode}\n`)
    }

    fileData += ('DEMAND_SECTION\n');

    for (var idx in nodes) {
        var node = nodes[idx];

        fileData += (`${node.algId} ${node.demand}\n`)
    }

    if (inputFileObject.edgeWeightType === 'REAL') {

        fileData += ('DISTANCE_SECTION\n');

        for (var idx in distances) {
            for (var j in distances[idx]) {

                fileData += (`${distances[idx][j]/1000} `);
            }

            fileData += ('\n');
        }
    }

    fileData += ('DEPOT_SECTION\n1\n-1\nEOF');

    fs.writeFileSync(filename, fileData);
}

module.exports = {
    createInputFile: createInputFile,
    dummy: returnDummyData,
    inputFileClass: InputFileClass,
    node: Node,
    writeInputFile: writeToFile,
    getDistances: getDistances
};