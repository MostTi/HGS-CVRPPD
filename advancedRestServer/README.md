# Node.js REST Server DEV

## Project Purpose
This REST server should help to connect the HGS algorithm solving the CVRPPD with the bus management system. Additionally test files for Stellargraph can be generated. The server is still under development and serves as a temporary solution.

## REST API
The endpoints are still kept very simple. When a request is sent, the response comes only after the algorithm is finished. This can take a long time depending on the number of nodes. For the Busmanagement System, a time limit can be specified as a termination criterion, which shortens this process.
Currently there are two REST endpoints:

| Address | Request | Description |
|---------|---------|-------------|
| localhost:22025/busmanagement | POST | Here you get after successful execution of the algorithm a JSON response containing the different calculated and optimized routes |
| localhost:22025/stellargraph | GET | Here you can download csv files containing the edge table for the calculated and optimized routes |


## Busmanagement System
### General
This REST endpoint is for generating one or more routes for the transport requests booked in the [Bus Management Portal]. A *request.json* file has to be sent in a POST request to get a *response.json* file containing the routes if the algorithm is successful.

### Request.json
**Currently only use REAL as edge_weight_type**

```
{
    "distance_per_vehicle": <number>|           // in km (default 3000)
    "edge_weight_type": <string>|               // REAL or EUC_2D (default REAL)
    "capacity": <number>|                       // vehicle capacity (default 8)
    "options": {
        "it": <number>|                         // number of iterations without improvement (default 20000) (optional)
        "t": <number>|                          // time limit (optional)
        "nv": <number>                          // number of vehicles (required)
    }|
    "depot": {                                  // default location?
        "id": <number>|
        "latitude": <number>|                   // y-coord for EUC_2D
        "longitude": <number>|                  // x-coord for EUC_2D
    }|
    "transport_requests":[
        {
            "pickup": {
                "id": <number>|
                "location": {
                    "longitude": <number>|      // x-coord for EUC_2D
                    "latitude": <number>        // y-coord for EUC_2D
                }
            }|
            "drop_off": {
                "id": <number>|
                "location": {
                    "longitude": <number>|      // x-coord for EUC_2D
                    "latitude": <number>        // y-coord for EUC_2D
                }
            }
        }|
        {
            ...
        }
    ]
}
```

### Response.json
```
// example for no routes

{
    "routes": []|
    "distance": -1|
    "errorMessage": <string>        // simple Reason why the algorithm failed
}


// example for one route

{
    "routes":[
        {
            "id": 1|
            "order": [1| 2| 3| 4]
        }
    ]|
    "distance": <number>
}


// example for multiple routes

{
    "routes": [
        {
            "id": 1|
            "order": [1| 2| 3| 4]
        }|
        {
            "id": 2|
            "order": [6| 7| 9| 8]
        }
    ]|
    "distance": <number>
}
```

## Stellargraph
This REST endpoint generates test files for [Stellargraph](https://www.stellargraph.io/). A GET request has to be sent to the server containing various query parameters. These parameters determine| for example| how many pickup and delivery pairs should be randomly selected. 

There are two different types of stops. *Virtual* and *Existing*. When selecting the pickup and delivery pairs| a virtual stop is taken as the pickup node and an existing stop is taken as the delivery node. These are chosen randomly| whereas the virtual stops are based on a weighting that increases the probability of some stops. The stops with more houses and/or POIs in a radius of 150m are weighted higher and are therefore selected with a higher probability.

### Request
A sample GET request should look like this:
```
im-pittyvaich.oth-regensburg.de:22025/stellargraph?distPV=130&edgeType=REAL&nrOfPd=10&nrV=5
```

The following query parameters exist:

| Query Parameter | Constraints | Description |
|-----------------|-------------|-------------|
| distPV | distPV $ > 0 $ | Setting the maximum distance (in km) one vehicle of the fleet can drive |
| edgeType | edgeType $ \in $ {'REAL','EUC_2D'} | Setting the type of distances; either real world distances are used or the euclidean distance; **Currently only use REAL as edgeType** |
| nrOfPd | nrOfPd $ > 0 $ | Sets the number of pickup and delivery pairs |
| nrV | nrV $ > 0 $ | Sets the number of vehicles serving the clients |

### Response
The response is a csv file containing an edge table representing the calculated and optimized routes. The following is an example edge table where the weight represents the distance measured in m:

| id | source | target | source_latitude | source_longitude | target_latitude | target_longitude | weight |
|-----------------|-------------|-------------|-----------------|-------------|-------------|-----------------|-------------|
|0|1|16|49.245077|12.823845|49.3113141|12.8319057|9465.8|
|1|16|40|49.3113141|12.8319057|49.28599213|12.85596543|4492.68|
|2|40|24|49.28599213|12.85596543|49.30719303|12.88005178|4830.23|
3|24|8|49.30719303|12.88005178|49.41721422|13.00905893|0
4|8|26|49.41721422|13.00905893|49.16728378|13.04939869|0
5|26|14|49.16728378|13.04939869|49.1898177|13.0438247|4107.86
6|14|9|49.1898177|13.0438247|49.211502|13.030145|3124.7
7|9|41|49.211502|13.030145|49.23647|12.984502|5609.88
8|41|36|49.23647|12.984502|49.247082|12.9708104|3706.42
9|36|15|49.247082|12.9708104|49.155783|12.923385|22876.59
10|15|18|49.155783|12.923385|49.41130503|13.02620046|0
11|18|17|49.41130503|13.02620046|49.265412|12.530922|0
12|17|28|49.265412|12.530922|49.24104001|12.58692508|5838.36
13|28|19|49.24104001|12.58692508|49.235982|12.641702|5680.57
14|19|4|49.235982|12.641702|49.29355324|12.6450557|9481.51
15|4|29|49.29355324|12.6450557|49.37105|12.635462|11878.31
16|29|10|49.37105|12.635462|49.39119109|12.70701029|7464.68
17|10|32|49.39119109|12.70701029|49.45258213|12.72005331|6499.76
18|32|12|49.45258213|12.72005331|49.4124803|12.77281596|6532.79
19|12|6|49.4124803|12.77281596|49.3770232|12.7286212|7612.27
20|6|11|49.3770232|12.7286212|49.373493|12.704918|2696.53
21|11|5|49.373493|12.704918|49.348637|12.712807|3397.66
22|5|27|49.348637|12.712807|49.316562|12.74164|5150.41
23|27|22|49.316562|12.74164|49.37090394|12.98451859|0
24|22|25|49.37090394|12.98451859|49.146775|12.342522|0
25|25|7|49.146775|12.342522|49.189155|12.402137|9755.33
26|7|23|49.189155|12.402137|49.150683|12.462535|9710.09
27|23|13|49.150683|12.462535|49.138303|12.591667|14210.04
28|13|37|49.138303|12.591667|49.149157|12.610868|2521.4
29|37|33|49.149157|12.610868|49.225563|12.772435|19476.88
30|33|1|49.225563|12.772435|49.245077|12.823845|6750.95
31|1|2|49.245077|12.823845|49.2619155|12.7654638|7595.77
32|2|30|49.2619155|12.7654638|49.41662846|12.93311756|0
33|30|3|49.41662846|12.93311756|49.355897|12.606415|0
34|3|38|49.355897|12.606415|49.3513297|12.6090133|854.16
35|38|31|49.3513297|12.6090133|49.350785|12.606042|256.06
36|31|39|49.350785|12.606042|49.210648|12.68982|19813.23
37|39|20|49.210648|12.68982|49.197638|12.7459723|4862.64
38|20|34|49.197638|12.7459723|49.20545641|12.8340559|7977.87
39|34|21|49.20545641|12.8340559|49.229367|12.85393|3550.39
40|21|35|49.229367|12.85393|49.267785|12.812167|7105.51
41|35|1|49.267785|12.812167|49.245077|12.823845|3530.5
-1|-1|-1|-1|-1|-1|-1|-1
0|1|8|49.245077|12.823845|49.41721422|13.00905893|0
1|8|9|49.41721422|13.00905893|49.211502|13.030145|0
2|9|14|49.211502|13.030145|49.1898177|13.0438247|3134.63
3|14|15|49.1898177|13.0438247|49.155783|12.923385|21438.42
4|15|1|49.155783|12.923385|49.245077|12.823845|16834.84
5|1|2|49.245077|12.823845|49.2619155|12.7654638|7595.77
6|2|16|49.2619155|12.7654638|49.3113141|12.8319057|10468.67
7|16|40|49.3113141|12.8319057|49.28599213|12.85596543|4492.68
8|40|24|49.28599213|12.85596543|49.30719303|12.88005178|4830.23
9|24|36|49.30719303|12.88005178|49.247082|12.9708104|11774.24
10|36|41|49.247082|12.9708104|49.23647|12.984502|3706.42
11|41|26|49.23647|12.984502|49.16728378|13.04939869|12477.1
12|26|30|49.16728378|13.04939869|49.41662846|12.93311756|0
13|30|25|49.41662846|12.93311756|49.146775|12.342522|0
14|25|18|49.146775|12.342522|49.41130503|13.02620046|0
15|18|17|49.41130503|13.02620046|49.265412|12.530922|0
16|17|28|49.265412|12.530922|49.24104001|12.58692508|5838.36
17|28|19|49.24104001|12.58692508|49.235982|12.641702|5680.57
18|19|4|49.235982|12.641702|49.29355324|12.6450557|9481.51
19|4|31|49.29355324|12.6450557|49.350785|12.606042|8553.36
20|31|38|49.350785|12.606042|49.3513297|12.6090133|256.06
21|38|3|49.3513297|12.6090133|49.355897|12.606415|854.16
22|3|29|49.355897|12.606415|49.37105|12.635462|3056.79
23|29|10|49.37105|12.635462|49.39119109|12.70701029|7464.68
24|10|32|49.39119109|12.70701029|49.45258213|12.72005331|6499.76
25|32|12|49.45258213|12.72005331|49.4124803|12.77281596|6532.79
26|12|6|49.4124803|12.77281596|49.3770232|12.7286212|7612.27
27|6|11|49.3770232|12.7286212|49.373493|12.704918|2696.53
28|11|5|49.373493|12.704918|49.348637|12.712807|3397.66
29|5|27|49.348637|12.712807|49.316562|12.74164|5150.41
30|27|22|49.316562|12.74164|49.37090394|12.98451859|0
31|22|7|49.37090394|12.98451859|49.189155|12.402137|0
32|7|23|49.189155|12.402137|49.150683|12.462535|9710.09
33|23|13|49.150683|12.462535|49.138303|12.591667|14210.04
34|13|37|49.138303|12.591667|49.149157|12.610868|2521.4
35|37|39|49.149157|12.610868|49.210648|12.68982|10605.77
36|39|20|49.210648|12.68982|49.197638|12.7459723|4862.64
37|20|33|49.197638|12.7459723|49.225563|12.772435|5962.3
38|33|34|49.225563|12.772435|49.20545641|12.8340559|8031.91
39|34|21|49.20545641|12.8340559|49.229367|12.85393|3550.39
40|21|35|49.229367|12.85393|49.267785|12.812167|7105.51
41|35|1|49.267785|12.812167|49.245077|12.823845|3530.5
-1|-1|-1|-1|-1|-1|-1|-1
...|...|...|...|...|...|...|...

## HGS Input and Solution Files
Here sample input and solution files of the HGS can be seen

### Input File
```
DIMENSION : 21
EDGE_WEIGHT_TYPE : REAL
DISTANCE : 130
CAPACITY : 8
NODE_COORD_SECTION
1 1 12.823845 49.245077 -1 -1
2 2 12.65677569 49.20896231 12 -1
3 4 12.94487045 49.40000015 13 -1
4 6 12.4152986 49.4526481 14 -1
5 8 12.69953235 49.25615052 15 -1
6 10 12.65578261 49.25221456 16 -1
7 12 12.95117452 49.25974351 17 -1
8 14 12.7422176 49.326114 18 -1
9 16 12.9924687 49.2077109 19 -1
10 18 12.83693575 49.18948564 20 -1
11 20 12.8425636 49.3080426 21 -1
12 3 12.735903 49.233045 -1 2
13 5 12.595737 49.165963 -1 3
14 7 12.511282 49.215503 -1 4
15 9 12.406765 49.184253 -1 5
16 11 12.680037 49.291455 -1 6
17 13 12.545893 49.297043 -1 7
18 15 12.678348 49.176742 -1 8
19 17 12.50326 49.185713 -1 9
20 19 12.559095 49.138727 -1 10
21 21 12.678348 49.176742 -1 11
DEMAND_SECTION
1 0
2 1
3 1
4 1
5 1
6 1
7 1
8 1
9 1
10 1
11 1
12 -1
13 -1
14 -1
15 -1
16 -1
17 -1
18 -1
19 -1
20 -1
21 -1
DISTANCE_SECTION
0 16.70937 0 54.502849999999995 13.001280000000001 20.05666 14.36649 13.4519 15.60784 8.63034 9.22555 9.92291 24.408150000000003 31.96078 40.209050000000005 15.70898 32.40151 19.307419999999997 32.119330000000005 28.04907 19.307419999999997 
16.8615 0 0 41.87495 8.311639999999999 7.4287600000000005 30.25027 18.9674 31.37377 15.844610000000001 20.30878 9.6727 9.38406 16.99256 25.24084 11.518559999999999 19.77361 5.38955 17.15111 13.02498 5.38955 
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 
48.96007 41.748400000000004 0 0 38.91177 35.57086 59.85225 36.23135 69.37313 53.843959999999996 46.62926 47.672050000000006 49.44718 35.47068 39.98451 33.37973 23.115569999999998 45.40096 45.318580000000004 51.23787 45.40096 
13.001280000000001 8.07071 0 38.91177 0 6.02583 26.29207 12.68485 28.587490000000003 15.60443 16.35058 4.56011 15.7695 23.32213 31.570400000000003 7.36934 18.37068 11.72328 23.48067 19.41042 11.72328 
20.414669999999997 7.30221 0 35.57086 6.02583 0 33.80343 17.14463 34.92694 19.397779999999997 23.861939999999997 13.22586 15.001 22.553630000000002 30.80189 5.6816 13.469520000000001 10.954780000000001 22.712169999999997 18.64192 10.954780000000001 
14.36649 30.12074 0 59.85298 24.938119999999998 33.46803 0 23.18837 13.67865 18.027459999999998 11.280389999999999 23.24395 37.81952 45.372150000000005 53.620419999999996 26.60189 45.81288 32.71879 45.530699999999996 41.460440000000006 32.71879 
13.4519 18.65516 0 36.23135 12.68485 17.14463 23.157790000000002 0 29.03811 22.112 10.40156 11.77837 26.35395 33.90657 42.15483999999999 11.46303 25.15484 21.25321 34.06512 29.99486 21.25321 
15.60784 30.917630000000003 0 68.71111 28.587490000000003 34.26492 13.67865 29.03811 0 17.89459 21.029709999999998 25.50912 38.61641 46.16904 54.41731 31.295189999999998 46.60977 29.5974 46.327589999999994 42.25733 29.5974 
8.63034 15.35884 0 53.15232 15.60443 18.706139999999998 18.027459999999998 22.209259999999997 17.86496 0 16.9623 11.214120000000001 23.05763 30.610259999999997 38.85853 22.795939999999998 31.05098 17.232310000000002 30.7688 26.69855 17.232310000000002 
9.22555 19.92125 0 46.62926 14.73864 23.268549999999998 11.2814 10.40156 21.000970000000002 16.93357 0 13.044469999999999 27.62004 35.17267 43.42094 16.40241 35.61339 22.519299999999998 35.33121 31.26096 22.519299999999998 
9.92291 9.33535 0 47.12883 4.56011 12.68264 23.21371 11.93084 25.50912 11.214120000000001 13.27221 0 17.03413 24.586759999999998 32.83504 9.813 25.02749 11.933399999999999 24.74531 20.67505 11.933399999999999 
23.800720000000002 8.42326 0 49.40986 15.846549999999999 14.96367 37.18948 25.90662 38.31299 22.783830000000002 27.24799 16.611919999999998 0 11.49498 20.75693 19.05347 18.86819 9.64075 10.02583 4.71272 9.64075 
31.505290000000002 17.29169 0 35.47068 23.551119999999997 22.66824 44.894059999999996 33.6112 46.01757 30.48841 34.95257 24.31649 12.514040000000001 0 10.97649 26.75804 14.850670000000001 19.03356 5.38462 11.30391 19.03356 
39.60949 25.395880000000002 0 39.98451 31.65532 30.77244 52.99826 41.71539 54.12176 38.5926 43.05677 32.42069 20.61823 10.99489 0 34.86224 24.3798 27.13775 13.488809999999999 19.40811 27.13775 
15.70898 11.39201 0 33.37973 7.36934 5.6816 26.60117 11.46303 35.84561 23.48758 16.65968 9.813 19.090799999999998 26.64343 34.8917 0 16.70372 15.04458 26.80197 22.731720000000003 15.04458 
32.75951 21.249959999999998 0 23.115569999999998 18.37068 13.469520000000001 46.14828 25.15484 47.27179 31.742630000000002 36.20679 25.57071 19.17447 14.850670000000001 24.39639 16.70372 0 22.99183 16.30667 22.22596 22.99183 
19.77937 5.82692 0 45.388510000000004 11.8252 10.94232 33.16814 21.885270000000002 29.56746 17.19331 23.226650000000003 12.59057 9.64075 18.73495 26.983220000000003 15.03212 22.62799 0 18.8935 13.28167 0 
31.450770000000002 17.23717 0 39.336580000000005 23.496599999999997 22.61372 44.83954 33.55667 45.96304 30.433880000000002 34.898050000000005 24.26197 10.02583 4.36556 13.627510000000001 26.70352 16.22108 18.979029999999998 0 8.81571 18.979029999999998 
27.44164 12.06417 0 45.25587 19.48746 18.604580000000002 40.830400000000004 29.54754 41.95391 26.42475 30.88891 20.252830000000003 4.71272 10.28485 19.546799999999998 22.69439 22.14038 13.28167 8.81571 0 13.28167 
19.77937 5.82692 0 45.388510000000004 11.8252 10.94232 33.16814 21.885270000000002 29.56746 17.19331 23.226650000000003 12.59057 9.64075 18.73495 26.983220000000003 15.03212 22.62799 0 18.8935 13.28167 0 
DEPOT_SECTION
1
-1
EOF
```

### Output File
```
{
"result": [{
"iteration": 0,
"routes": [[16,40,24,8,26,14,9,41,36,15,18,17,28,19,4,29,10,32,12,6,11,5,27,22,25,7,23,13,37,33],[2,30,3,38,31,39,20,34,21,35],[]],
"cost": 250,
"time": 0.14367
},
{
"iteration": 500,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 0.523557
},
{
"iteration": 1000,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 0.957536
},
{
"iteration": 1500,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 1.39804
},
{
"iteration": 2000,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 1.81623
},
{
"iteration": 2500,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 2.19025
},
{
"iteration": 3000,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 2.58317
},
{
"iteration": 3500,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 2.99666
},
{
"iteration": 4000,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 3.38127
},
{
"iteration": 4500,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 3.7547
},
{
"iteration": 5000,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 4.14427
},
{
"iteration": 5500,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 4.52567
},
{
"iteration": 6000,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 4.90826
},
{
"iteration": 6500,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 5.28492
},
{
"iteration": 7000,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 5.65456
},
{
"iteration": 7500,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 6.02051
},
{
"iteration": 8000,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 6.39204
},
{
"iteration": 8500,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 6.77015
},
{
"iteration": 9000,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 7.14386
},
{
"iteration": 9500,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 7.52655
},
{
"iteration": 10000,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 7.88621
},
{
"iteration": 10500,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 8.26374
},
{
"iteration": 11000,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 8.64908
},
{
"iteration": 11500,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 9.02206
},
{
"iteration": 12000,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 9.37934
},
{
"iteration": 12500,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 9.74191
},
{
"iteration": 13000,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 10.1093
},
{
"iteration": 13500,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 10.4801
},
{
"iteration": 14000,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 10.8678
},
{
"iteration": 14500,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 11.245
},
{
"iteration": 15000,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 11.6082
},
{
"iteration": 15500,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 11.996
},
{
"iteration": 16000,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 12.3676
},
{
"iteration": 16500,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 12.7324
},
{
"iteration": 17000,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 13.1023
},
{
"iteration": 17500,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 13.4774
},
{
"iteration": 18000,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 13.838
},
{
"iteration": 18500,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 14.2193
},
{
"iteration": 19000,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 14.6039
},
{
"iteration": 19500,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 14.9794
},
{
"iteration": 20000,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 15.3557
},
{
"iteration": -1,
"routes": [[8,9,14,15],[2,16,40,24,36,41,26,30,25,18,17,28,19,4,31,38,3,29,10,32,12,6,11,5,27,22,7,23,13,37,39,20,33,34,21,35],[]],
"cost": 241,
"time": 15.6595
}]}
```

## Known Issues
Some virtual busstops aren't located on a road.. Thats why the distance to this stops can't be determined and gets set to 0.