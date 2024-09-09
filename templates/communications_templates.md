This document is designed to clarify the communication API between Robots and Server.

## Communications Robot-Server:

#### 1. Start Mission:
```json
{
    "command": "start_mission",
    "mission_details": {
        "orientation": "<initial_orientation>",
        "position": {
            "x": 0.0,
            "y": 0.0
        }
    },
    "timestamp": "ISO 8601"
}
```

#### 2. End Mission:
```json
{
    "command": "end_mission",
    "timestamp": "ISO 8601"
}
```

#### 3. Update:
```json
{
    "command": "update",
    "status": "<robot_status>",
    "position": {
        "x": 0.0,
        "y": 0.0
    },
    "timestamp": "ISO 8601"
}
```

#### 4. Return to Base:
```json
{
    "command": "return_to_base",
    "timestamp": "ISO 8601"
}
```

#### 5. Update Controller Code:
```json
{
    "command": "update_controller_code",
    "code": "<new_code>",
    "timestamp": "ISO 8601"
}
```

#### 6. Notify Robots to Communicate:
```json
{
    "command": "P2P",
    "timestamp": "ISO 8601"
}
```

## Communication Robot-Robot:

#### 1. Find the Furthest Robot:
```json
{
    "command": "find_furthest",
    "relative_point": {
        "x": 0.0,
        "y": 0.0
    },
    "timestamp": "ISO 8601"
}
```
