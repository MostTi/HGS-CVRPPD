const fs = require('fs');
const algPreProcessor = require('./algorithmPreprocessor');
const defaults = require('./algorithmDefaults');


var virtual = {};
var existing = {};


function createStellargraphCsvFile(inputFileObj) {
    
}

async function createInputFile(distPerVehicle, edgeWeightType, numberOfPdPairs, numberOfVehicles) {

    readBusStopFiles();

    var filename = `cvrppd_${(new Date().toJSON().slice(0,19))}`;
    var nodes = createNodes(numberOfPdPairs);
    var distances = await algPreProcessor.getDistances(nodes);
    var inputFileObj = createInputFileObject(distPerVehicle, edgeWeightType, nodes, distances, numberOfVehicles, filename);

    algPreProcessor.writeInputFile(inputFileObj);

    return inputFileObj;
}

function createInputFileObject(distPerVehicle, edgeWeightType, nodes, distances, numberOfVehicles, filename) {

    var inputFileObj = new algPreProcessor.inputFileClass();

    inputFileObj.dimension = nodes.length;
    inputFileObj.distancePerVehicle = distPerVehicle;
    inputFileObj.edgeWeightType = edgeWeightType;
    inputFileObj.capacity = defaults.algDefaults.capacity;
    inputFileObj.numberOfIterationsWithoutImprov = defaults.algDefaults.nrOfItWithoutImprov;
    inputFileObj.timeLimit = -1;
    inputFileObj.numberVehicles = numberOfVehicles;
    inputFileObj.distances = distances;
    inputFileObj.nodes = nodes;
    inputFileObj.filename = filename;

    return inputFileObj;
}

function createNodes(numberOfPdPairs) {
    
    var nodes = [];
    var pickups = [];
    var dropOffs = [];
    var algId = 1;
    var realId = 1;
    var i = 0;
    var dimension = numberOfPdPairs * 2

    while (i < numberOfPdPairs) {

        pickups.push(virtual[Math.floor(Math.random() * virtual.length)]);
        dropOffs.push(existing[Math.floor(Math.random() * existing.length)]);

        i++;
    }

    var depotNode = new algPreProcessor.node(algId++, realId++, existing[0].longitude, existing[0].latitude, 0, -1, -1);

    nodes.push(depotNode);

    i = 0;

    while (i < numberOfPdPairs) {
        var pickUpIdx = algId;
        var dropOffIdx = algId + dimension/2;

        var pickUpNode = new algPreProcessor.node(pickUpIdx, realId++, pickups[i].longitude, pickups[i].latitude, 1, -1, dropOffIdx);
        var dropOffNode = new algPreProcessor.node(dropOffIdx, realId++, dropOffs[i].longitude, dropOffs[i].latitude, -1, pickUpIdx, -1);

        nodes[pickUpIdx-1] = pickUpNode;
        nodes[dropOffIdx-1] = dropOffNode;

        algId++;
        i++;
    }

    return nodes;
}

function readBusStopFiles() {

    var virtualCsv = fs.readFileSync(`busstops/virt.csv`).toString();
    var existingCsv = fs.readFileSync(`busstops/existing.csv`).toString();

    virtual = csvToArray(virtualCsv);
    existing = csvToArray(existingCsv);    
}

function csvToArray(str, delimiter = ",") {

    const headers = str.slice(0, str.indexOf("\n")).split(delimiter);
    const rows = str.slice(str.indexOf("\n") + 1).split("\n");

    const arr = rows.map(function (row) {
        const values = row.split(delimiter);
        const e1 = headers.reduce(function (object, header, index) {
            object[header] = values[index];
            
            return object;
        }, {});

        return e1;
    });

    return arr;
}


module.exports = {
    createInputFile: createInputFile
}