const { Parser } = require('json2csv');
const { spawn } = require('child_process');
const fs = require('fs');
const converter = require('json-2-csv');

const hgsPath = "/home/linefader/Desktop/masterstuff/HGS-CVRP_updated_moves/Program/";               // path to executable TODO

function startHgs(inputFileObj, res, stellagraph) {

    var args = [];
    var solutionFileName = `solutions/newSolutionFile_${(new Date().toJSON().slice(0,19))}.json`;

    args.push(`/home/linefader/Desktop/masterstuff/simpleRestServer/inputFiles/${inputFileObj.filename}`);
    args.push(solutionFileName);
    args.push("-veh");
    args.push(inputFileObj.numberVehicles);
    args.push("-measure");
    args.push(inputFileObj.edgeWeightType)

    console.log(`${hgsPath}genvrp ${args}`);
    var algorithmProcess = spawn(hgsPath + 'genvrp', args);   

    algorithmProcess.on('error', err => {
        var message = ['Unknown internal error', err].join(': ');

        res.status(500);
        res.send({error: message});
    });

    algorithmProcess.stderr.on('data', data => {
        console.error(now() + ': ' + data.toString());

        res.status(500);
        res.send({error: data});
    });

    var solution = '';

    algorithmProcess.stdout.on('data', data => {
        console.log("HGS is running...")
        solution += data;
    });

    algorithmProcess.on('close', code => {
        console.log(`HGS status code: ${code}`);

        if (code == 0) {
            var data = readSolutionFile(solutionFileName);

            console.log("Successfully completed HGS and created Solution file!")
            console.log("HGS output:")
            console.log(solution);

            if (stellagraph) {
                console.log("Creating output file vor Stellagraph")
                var response = createStellargraphResponse(data, inputFileObj);

                res.header('Content-Type', 'text/csv');
                res.attachment("edgetable.csv");
                res.send(response);
            }
            else {
                console.log("Creating output data for the busmanagement system")
                var response = createBusmanagementResponse(data);
                
                res.status(200);
                res.send(response);
            }
        }
        else {

            console.log("HGS output:")
            console.log(solution);

            var response = {};

            response.routes = [];
            response.distance = "-1";
            response.errorMessage = getErrorMessageToCode(code);

            console.log(`Something went wrong :-( : ${response.errorMessage})`);

            res.status(200);
            res.send(response);
        }
    });
}

function getErrorMessageToCode(code) {

    if (code == 10) {
        return "No feasible solution could be created";
    }
    else if (code == 11) {
        return "Unexpected data in input file; Missing dimension or edge weight type or capacity or distance or service time";
    }
    else if (code == 12) {
        return "Number of clients is smaller or equal to 0";
    }
    else if (code == 13) {
        return "Vehicle capacity is undefined";
    }
    else if (code == 14) {
        return "Unexpected data in input file; Expected demand section";
    }
    else if (code == 15) {
        return "Unexpected data in input file; Expected distance section";
    }
    else if (code == 16) {
        return "Unexpected data in input file; Expected depot section";
    }
    else if (code == 17) {
        return "Unexpected data in input file; Expected depot index";
    }
    else if (code == 18) {
        return "Unexpected data in input file; Expected EOF";
    }
    else if (code == 19) {
        return "Impossible to open the input file";
    }
    else if (code == 20) {
        return "Distances are either to small or to big => numerical instabilities could occur";
    }
    else if (code == 21) {
        return "Demands are either to small or to big => numerical instabilities could occur";
    }
    else if (code == 22) {
        return "Number of vehicles is insufficient to service all clients";
    }
    else if (code == 23) {
        return "Best individual was deleted! This should not happen!";
    }
    else if (code == 24) {
        return "No Split solution has been propagated until the last node";
    }
}

function readSolutionFile(filename) {
    
    var data = fs.readFileSync(`${filename}`, 'utf8');
        
    var parsedData = JSON.parse(data);

    return parsedData;
}

function createStellargraphResponse(solutionData, inputFileObj) {

    var json2csv = new Parser();
    var data = [];
    var routes = solutionData.routes;
    var edgeTableEntryIdx = 0;
    var nodes = inputFileObj.nodes;
    var distances = inputFileObj.distances;
    var depot = nodes[0];

    for (var route of routes) {

        if (route.length != 0) {
            data.push(getEntry(edgeTableEntryIdx++, depot.realId, route[0], nodes, distances));

            for (var i = 0; i < route.length - 1; i++) {

                data.push(getEntry(edgeTableEntryIdx++, route[i], route[i+1], nodes, distances));
            }

            data.push(getEntry(edgeTableEntryIdx++, route[i], depot.realId, nodes, distances));
        }
    }

    var csvData = json2csv.parse(data);

    return csvData;
}

function getEntry(id, source, target, nodes, distances) {

    entry = {};

    entry.id = id;
    entry.source = source;
    entry.target = target;
    entry.weight = getDistanceBetweenTwoNodesRealIds(nodes, distances, source, target);

    return entry;
}

function getDistanceBetweenTwoNodesRealIds(nodes, distances, startRealId, destRealId) {

    var startNode = nodes.find(node => node.realId === startRealId);
    var destNode = nodes.find(node => node.realId === destRealId);

    var dist = distances[startNode.algId-1][destNode.algId-1];

    if (dist == undefined) {
        console.log(`Undefined distance between node ${startNode.realId} (lon: ${startNode.longitude}, lat: ${startNode.latitude}) and `
                    + `node ${destNode.realId} (lon: ${destNode.longitude}, lat: ${destNode.latitude}) ~~ returning 0 as distance`);

        return 0;       // TODO handle nodes
    }

    return dist;
}

function createBusmanagementResponse(data) {

    var response = {};
    var tempRoutes = data.routes;
    var routeCounter = 0;

    response.routes = [];
    
    for (var route of tempRoutes) {
        
        var realRoute = {};

        realRoute.id = routeCounter++;
        realRoute.route = route;

        response.routes.push(realRoute);
    }

    response.distance = data.cost;

    return response;
}

module.exports = {
    startHgs: startHgs
};