# API Contracts

## Position

`POST /api/v1/position`

### Description

Sends the target positions of the blocks to the API service. Upon receiving the payload, the API service initiates motion planning by invoking the vision service. The API service returns a response immediately upon recording the positions without waiting for motion planning to complete.

### Trigger Behavior & Status Codes

* **`204 No Content`**: Returned when the target block positions are successfully recorded and motion planning has been initiated.
* **`400 Bad Request`**: Returned when the request payload is malformed or invalid.
* **`409 Conflict`**: Returned when a position request is received while a motion planning process is actively in progress.

### Request Payload

```typescript
{
  "blocks": Array<{
    "color": "red" | "green" | "blue"; // Lowercase string
    "posX": number;                    // Left-to-right grid coordinate; 0-indexed from leftmost
    "posY": number;                    // Top-to-bottom grid coordinate; 0-indexed from topmost
    "posZ": number;                    // Layer height coordinate; 0-indexed from lowest layer
  }>
}
```

---

## Position Status

`GET /api/v1/position/status`

### Description

Retrieves the processing status of the most recent position request.

### Trigger Behavior & Status Codes

* **`200 OK`**: Returned when the status query completes successfully, regardless of whether processing is pending, succeeded, or failed.

### Response Payload

```typescript
{
  "status": "pending" | "running" | "succeeded" | "failed";
  "message": string;
}
```

---

## Motion Start

`POST /api/v1/motion/start`

### Description

Commands the API service to execute robot motion based on the completed motion plan.

### Trigger Behavior & Status Codes

* **`204 No Content`**: Returned when the motion execution is successfully initiated.
* **`409 Conflict`**: Returned when motion execution cannot be started because motion planning is incomplete, failed, or no position request has been submitted.

---

## Motion Stop

`POST /api/v1/motion/stop`

### Description

Commands the API service to immediately halt active robot motion.

### Trigger Behavior & Status Codes

* **`204 No Content`**: Returned when the stop command is successfully received and motion is halted.
* **`409 Conflict`**: Returned when a stop command is received but no robot motion is currently active.

---

## Motion Status

`GET /api/v1/motion/status`

### Description

Retrieves the execution status of the active or most recent robot motion.

### Trigger Behavior & Status Codes

* **`200 OK`**: Returned when the status query completes successfully.

### Response Payload

```typescript
{
  "status": "pending" | "running" | "succeeded" | "failed";
  "message": string;
}
```
