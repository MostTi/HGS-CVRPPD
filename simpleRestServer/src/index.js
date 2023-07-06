const express = require('express');
const cors = require('cors');
const helmet = require('helmet');
const { Validator, ValidationError } = require('express-json-validator-middleware');
const handler = require('./algorithmPreprocessor');
const requestSchema = require('./requestSchema');
const algorithm = require('./algorithmModule');
const roding = require('./rodingDataCreator');

const app = express();
const port = 3000;
const { validate } = new Validator();



app.use(cors());
app.use(helmet());
app.use(express.urlencoded({ extended: true }));
app.use(express.json());


// utc timestamp function
function now() {
    const date = new Date();

    return date.toUTCString();
}



app.post('/', validate({ body: requestSchema }), async (req, res) => {
    console.log("Success");
    console.log(req.body);
    
    var inputFileObj = await handler.createInputFile(req.body);

    console.log("Starting HGS");

    algorithm.startHgs(inputFileObj, res, false);
});

app.get('/stellagraph', async (req, res) => {
    console.log("Creating test file for stellagraph");

    var distPerVehicle = req.query.distPV;
    var edgeWeightType = req.query.edgeType;
    var numberOfPdPairs = req.query.nrOfPd;
    var numberOfVehicles = req.query.nrV;
    var error = false;

    if (distPerVehicle === undefined || edgeWeightType === undefined
        || numberOfPdPairs === undefined || numberOfVehicles === undefined) {

        error = true;
        res.writeHead(500);
        res.send("Missing query parameter");
    }
    
    if (distPerVehicle <= 0) {
        error = true;
        res.writeHead(500);
        res.end("Distance per Vehicle has to be greater than 0");
    }
    
    if (edgeWeightType !== "EUC_2D" && edgeWeightType !== "REAL") {
        error = true;
        res.status(500).send("Edge weight type has to be either 'EUC_2D' or 'REAL'");
    }

    if (numberOfPdPairs <= 0) {
        error = true;
        res.status(500).send("Number of pickup and delivery paris have to be greater than 0");
    }

    if (numberOfVehicles <= 0) {
        error = true;
        res.status(500).send("Number of vehicles has to be greater than 0");
    }

    if (!error) {
        var inputFileObj = await roding.createInputFile(distPerVehicle, edgeWeightType, numberOfPdPairs, numberOfVehicles);

        console.log("Starting HGS");

        algorithm.startHgs(inputFileObj, res, true);
    }
});

// error handler middleware for validation errors
app.use((err, req, res, next) => {

    if (error instanceof ValidationError) {
        // currently only returns the first error for security reasons
        response.status(400)
                .send(error.validationErrors);
        next();
    }
    else {
        next(error);
    }
})

const server = app.listen(port, () => {
    console.log(`HGS-REST server listening on port ${port}`);
});

server.setTimeout(3000000)