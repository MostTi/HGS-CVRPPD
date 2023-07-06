const requestSchema = {
    "type": "object",
    "properties": {
        "distance_per_vehicle": {
            "type": "integer"
        },
        "edge_weight_type": {
            "type": "string"
        },
        "capacity": {
            "type": "integer"
        },
        "options": {
            "type": "object",
            "properties": {
                "it": {
                    "type": "integer"
                },
                "t": {
                    "type": "integer"
                },
                "nv": {
                    "type": "integer"
                }
            },
            "required": [
                "nv"
            ]
        },
        "depot": {
            "type": "object",
            "properties": {
                "id": {
                    "type": "integer"
                },
                "longitude": {
                    "type": "number"
                },
                "latitude": {
                    "type": "number"
                }
            },
            "required": [
                "id",
                "longitude",
                "latitude"
            ]
        },
        "transport_requests": {
            "type": "array",
            "items": [
                {
                    "type": "object",
                    "properties": {
                        "pickup": {
                            "type": "object",
                            "properties": {
                                "id": {
                                    "type": "integer"
                                },
                                "location": {
                                    "type": "object",
                                    "properties": {
                                        "longitude": {
                                            "type": "number"
                                        },
                                        "latitude": {
                                            "type": "number"
                                        }
                                    },
                                    "required": [
                                        "longitude",
                                        "latitude"
                                    ]
                                }
                            },
                            "required": [
                                "id",
                                "location"
                            ]
                        },
                        "drop_off": {
                            "type": "object",
                            "properties": {
                                "id": {
                                    "type": "integer"
                                },
                                "location": {
                                    "type": "object",
                                    "properties": {
                                        "longitude": {
                                            "type": "number"
                                        },
                                        "latitude": {
                                            "type": "number"
                                        }
                                    },
                                    "required": [
                                        "longitude",
                                        "latitude"
                                    ]
                                }
                            },
                            "required": [
                                "id",
                                "location"
                            ]
                        }
                    },
                    "required": [
                        "pickup",
                        "drop_off"
                    ]
                }
            ]
        }
    },
    "required": [
        "options",
        "depot",
        "transport_requests"
    ]
}