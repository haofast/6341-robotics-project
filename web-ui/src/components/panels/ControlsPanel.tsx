import { useSnapshot } from "valtio";
import { Console } from "../common/Console";
import { Button } from "../ui/button";
import { Card } from "../ui/card";
import { workspaceState } from "@/state/workspace";
import { getMotionStatus, getPositionStatus, postMotionStart, postMotionStop, postPosition, type GetMotionStatusResponse, type GetPositionStatusResponse, type PostPositionRequest } from "@/utilities/api";
import { BlockUtil } from "@/utilities/block";
import { useState } from "react";

export function ControlsPanel() {
  const workspaceSnap = useSnapshot(workspaceState);

  const [positionStatus, setPositionStatus] = useState<GetPositionStatusResponse>();
  const [motionStatus, setMotionStatus] = useState<GetMotionStatusResponse>();

  const setPositionDisabled = (
    positionStatus?.status === "running"
    || motionStatus?.status === "running"
  );

  const startMotionDisabled = (
    positionStatus?.status !== "succeeded"
    || motionStatus?.status === "running"
  );

  const stopMotionDisabled = (
    positionStatus?.status !== "succeeded"
    || motionStatus?.status !== "running"
  );

  function buildPositionRequest(): PostPositionRequest {
    return {
      blocks: workspaceSnap.placedBlocks.map((block) => ({
        color: BlockUtil.getColorString(block),
        posX: block.position.x,
        posY: block.position.y,
        posZ: block.position.z,
      })),
    };
  }

  async function uploadPosition() {
    await postPosition(buildPositionRequest());
    setPositionStatus(await getPositionStatus());
    pollPositionStatus();
  }

  async function startMotion() {
    await postMotionStart();
    setMotionStatus(await getMotionStatus());
    pollMotionStatus();
  }

  async function stopMotion() {
    await postMotionStop();
    setMotionStatus(await getMotionStatus());
  }

  async function pollPositionStatus() {
    const response = await getPositionStatus();
    setPositionStatus(response);
    if (response.status === "succeeded" || response.status === "failed") return;
    setTimeout(pollPositionStatus, 2000);
  }

  async function pollMotionStatus() {
    const response = await getMotionStatus();
    setMotionStatus(response);
    if (response.status === "succeeded" || response.status === "failed") return;
    setTimeout(pollMotionStatus, 2000);
  }

  return (
    <Card className="fixed top-6 right-6 bottom-6 w-80 p-4 overflow-y-auto bg-background/75 backdrop-blur">
      <div className="space-y-2">
        <div className="text-[10px] font-light tracking-widest">
          POSITION CONSOLE
        </div>
        <Console logs={[
          {
            type: "info",
            timestamp: new Date(),
            text: "end effector at (0,0,0)",
          },
        ]} />
      </div>
      <hr />
      <div className="space-y-2">
        <div className="text-[10px] font-light tracking-widest">
          LOG CONSOLE
        </div>
        <Console logs={[
          {
            type: "info",
            timestamp: new Date(),
            text: "positioning uploaded",
          },
        ]} />
      </div>
      <hr />
      <div className="space-y-2">
        <div className="text-[10px] font-light tracking-widest">
          CONTROLS
        </div>
        <div className="flex flex-row gap-2">
          <Button
            className="flex-grow"
            variant="default"
            onClick={uploadPosition}
            disabled={setPositionDisabled}
          >
            Set Position
          </Button>
        </div>
        <div className="flex flex-row gap-2">
          <Button
            className="flex-grow"
            variant="default"
            onClick={startMotion}
            disabled={startMotionDisabled}
          >
            Start
          </Button>
          <Button
            className="flex-grow"
            variant="outline"
            onClick={stopMotion}
            disabled={stopMotionDisabled}
          >
            Stop
          </Button>
        </div>
      </div>
    </Card>
  );
}
