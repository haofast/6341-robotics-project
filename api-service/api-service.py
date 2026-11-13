from enum import Enum
from typing import List, Set, Tuple
from fastapi import FastAPI, HTTPException, status
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
import httpx  # For communicating with the vision-service HTTP endpoints
from pydantic import BaseModel, Field, model_validator

app = FastAPI(
    title="Robotic Arm Block Assembly API",
    version="1.3.0",
    description="API service managing block positions, vision/motion planning, and robot execution.",
)

# Configuration for external Vision Service / Robot Backend
VISION_SERVICE_URL = "http://localhost:8001"  # Placeholder URL

# Exception Handler
@app.exception_handler(RequestValidationError)
async def validation_exception_handler(request, exc: RequestValidationError):
    # Returns 400 instead of FastAPI default of 422.
    clean_errors = []
    for error in exc.errors():
        error_dict = dict(error)
        # Convert any non-serializable objects in 'ctx' (like ValueError instances) to strings
        if "ctx" in error_dict:
            error_dict["ctx"] = {k: str(v) for k, v in error_dict["ctx"].items()}
        clean_errors.append(error_dict)

    return JSONResponse(
        status_code=status.HTTP_400_BAD_REQUEST,
        content={
            "error": "Bad Request",
            "message": "The request payload is malformed or invalid.",
            "details": clean_errors
        },
    )

# --- Enums & Models ---

class BlockColor(str, Enum):
    red = "red"
    green = "green"
    blue = "blue"

class Block(BaseModel):
    color: BlockColor
    posX: int = Field(..., description="Left-to-right grid coordinate; 0-indexed (0-5)")
    posY: int = Field(..., description="Layer height coordinate; 0-indexed")
    posZ: int = Field(..., description="Top-to-bottom grid coordinate; 0-indexed (0-2)")
    

    @model_validator(mode="after")
    def validate_block_rules(self) -> "Block":
        # 1. Check 3x3 grid bounds for posX and posY
        if not (0 <= self.posX <= 5) or not (0 <= self.posZ <= 2):
            raise ValueError(f'''Grid coordinates posX ({self.posX}) must be between 0 and 5\n
                                 Grid coordinates posY ({self.posY}) must be between 0 and 2.''')
        
        # 2. Check posZ is non-negative
        if self.posY < 0:
            raise ValueError(f"Layer height posZ ({self.posZ}) must be non-negative.")
        
        return self

class PositionRequest(BaseModel):
    blocks: List[Block]

    @model_validator(mode="after")
    def validate_global_block_rules(self) -> "PositionRequest":
        coordinates: Set[Tuple[int, int, int]] = set()
        grid_by_layer = {}

        for block in self.blocks:
            coord = (block.posX, block.posY, block.posZ)
            
            # 3. Check for duplicate coordinates
            if coord in coordinates:
                raise ValueError(f"Duplicate coordinate detected at posX={block.posX}, posY={block.posY}, posZ={block.posZ}.")
            coordinates.add(coord)

            if block.posY not in grid_by_layer:
                grid_by_layer[block.posY] = set()
            grid_by_layer[block.posY].add((block.posX, block.posZ))

        # 4. Gravity / Stacking validation: a block at posZ > 0 must have a block beneath it
        for y in sorted(grid_by_layer.keys()):
            if y > 0:
                lower_layer = grid_by_layer.get(y - 1, set())
                for (x, z) in grid_by_layer[y]:
                    if (x, z) not in lower_layer:
                        raise ValueError(
                            f"Invalid stack configuration: Block at ({x}, {z}, layer {y}) "
                            f"does not have a supporting block directly beneath it at layer {y - 1}."
                        )
                        
        return self

class StatusEnum(str, Enum):
    pending = "pending"
    running = "running"
    succeeded = "succeeded"
    failed = "failed"

class StatusResponse(BaseModel):
    status: StatusEnum
    message: str

# --- Helper Placeholder Functions for Vision / Robot Service ---

async def placeholder_trigger_motion_planning(blocks: List[Block]):
    """
    Placeholder: Calls the vision-service endpoint to initiate MoveIt 2 motion planning[cite: 1].
    """
    # async with httpx.AsyncClient() as client:
    #     try:
    #         response = await client.post(f"{VISION_SERVICE_URL}/api/v1/vision/plan", json={"blocks": [b.dict() for b in blocks]})
    #         response.raise_for_status()
    #     except httpx.HTTPError as e:
    #         pass
    pass

async def placeholder_get_motion_plan_status() -> StatusResponse:
    """
    Placeholder: Queries the vision-service endpoint to check on the status 
    of motion planning[cite: 1].
    """
    # async with httpx.AsyncClient() as client:
    #     response = await client.get(f"{VISION_SERVICE_URL}/api/v1/vision/status")
    #     return StatusResponse(**response.json())
    
    return StatusResponse(status=StatusEnum.succeeded, message="Motion plan successfully generated by vision-service.")

async def placeholder_start_robot_motion():
    """Placeholder: Commands vision-service/ROS2 backend to execute the planned robot motion[cite: 1]."""
    # async with httpx.AsyncClient() as client:
    #     await client.post(f"{VISION_SERVICE_URL}/api/v1/motion/execute")
    pass

async def placeholder_stop_robot_motion():
    """Placeholder: Commands vision-service/ROS2 backend to immediately halt robot motion[cite: 1]."""
    # async with httpx.AsyncClient() as client:
    #     await client.post(f"{VISION_SERVICE_URL}/api/v1/motion/halt")
    pass

async def placeholder_get_execution_status() -> StatusResponse:
    """
    Placeholder: Queries the robot/vision-service endpoint to retrieve the execution 
    status of active or recent robot motion.
    """
    # async with httpx.AsyncClient() as client:
    #     response = await client.get(f"{VISION_SERVICE_URL}/api/v1/motion/status")
    #     return StatusResponse(**response.json())
    
    return StatusResponse(status=StatusEnum.succeeded, message="Robot motion finished or idle.")


# --- Endpoints ---

@app.post("/api/v1/position", status_code=status.HTTP_204_NO_CONTENT)
async def post_position(payload: PositionRequest):
    """
    Sends target positions of blocks, triggering motion planning[cite: 1].
    """

    # Check if motion planning is already running or pending via vision service
    motion_plan_status = await placeholder_get_motion_plan_status()
    if motion_plan_status.status in [StatusEnum.pending, StatusEnum.running]:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail="A position request or motion planning process is actively in progress."
        )
    
    # Trigger background/asynchronous call to the vision-service
    await placeholder_trigger_motion_planning(payload.blocks)
    
    return None

@app.get("/api/v1/position/status", response_model=StatusResponse, status_code=status.HTTP_200_OK)
async def get_motion_plan_status():
    """
    Retrieves the processing status of the most recent position request from the vision-service[cite: 1].
    """
    return await placeholder_get_motion_plan_status()

@app.post("/api/v1/motion/start", status_code=status.HTTP_204_NO_CONTENT)
async def start_motion():
    """
    Commands the API service to execute robot motion based on the completed motion plan[cite: 1].
    """
    motion_plan_status = await placeholder_get_motion_plan_status()
    execution_status = await placeholder_get_execution_status()
    
    # Check if motion planning is incomplete, failed, or missing
    if motion_plan_status.status != StatusEnum.succeeded:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail="Motion execution cannot be started because motion planning is incomplete, failed, or missing."
        )
    
    # Check if robot motion is already active
    if execution_status.status in [StatusEnum.pending, StatusEnum.running]:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail="Robot motion is already active."
        )

    await placeholder_start_robot_motion()
    
    return None

@app.post("/api/v1/motion/stop", status_code=status.HTTP_204_NO_CONTENT)
async def stop_motion():
    """
    Immediately halts active robot motion[cite: 1].
    """
    execution_status = await placeholder_get_execution_status()
    
    if execution_status.status not in [StatusEnum.pending, StatusEnum.running]:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail="No robot motion is currently active to stop."
        )

    await placeholder_stop_robot_motion()

    return None

@app.get("/api/v1/motion/status", response_model=StatusResponse, status_code=status.HTTP_200_OK)
async def get_execution_status():
    """
    Retrieves the execution status of the active or most recent robot motion from the robot backend[cite: 1].
    """
    return await placeholder_get_execution_status()