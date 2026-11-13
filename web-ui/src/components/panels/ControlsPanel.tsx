import { useSnapshot } from "valtio";
import { Console } from "../common/Console";
import { Button } from "../ui/button";
import { Card } from "../ui/card";
import { workspaceState } from "@/state/workspace";
import { getMotionStatus, getPositionStatus, postMotionStart, postMotionStop, postPosition, type GetMotionStatusResponse, type GetPositionStatusResponse, type PostPositionRequest } from "@/utilities/api";
import { BlockUtil } from "@/utilities/block";
import { useState } from "react";
import { WorkspaceUtils } from "@/utilities/workspace";
import { toast } from "../ui/toast";
import { addLogConsoleEntry, logConsoleState } from "@/state/log-console";
import { positionConsoleState } from "@/state/position-console";

export function ControlsPanel() {
  const workspaceSnap = useSnapshot(workspaceState);
  const positionConsoleSnap = useSnapshot(positionConsoleState);
  const logConsoleSnap = useSnapshot(logConsoleState);

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

  function logSuccess(title: string, description: string) {
    toast.add({ type: "success", title, description });
    addLogConsoleEntry({ type: "error", text: description });
  }

  function logError(title: string, description: string) {
    toast.add({ type: "error", title, description });
    addLogConsoleEntry({ type: "error", text: description });
  }

  async function uploadPosition() {
    if (!WorkspaceUtils.isValidConfiguration()) {
      return logError("Configuration not uploaded", "Blocks cannot be overlapping or floating!");
    }

    await postPosition(buildPositionRequest())
      .then(() => logSuccess("Request succeeded", "Position uploaded"))
      .then(getPositionStatus)
      .then(setPositionStatus)
      .then(pollPositionStatus)
      .catch((e) => logError("Request failed", e.message));
  }

  async function startMotion() {
    await postMotionStart()
      .then(() => logSuccess("Request succeeded", "Motion started"))
      .then(getMotionStatus)
      .then(setMotionStatus)
      .then(pollMotionStatus)
      .catch((e) => logError("Request failed", e.message));
  }

  async function stopMotion() {
    await postMotionStop()
      .then(() => logSuccess("Request succeeded", "Motion stopped"))
      .then(getMotionStatus)
      .then(setMotionStatus)
      .catch((e) => logError("Request failed", e.message));
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
        <Console entries={positionConsoleSnap.entries} />
      </div>
      <hr />
      <div className="space-y-2">
        <div className="text-[10px] font-light tracking-widest">
          LOG CONSOLE
        </div>
        <Console entries={logConsoleSnap.entries} />
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
