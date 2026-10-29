import { Card } from "../ui/card";
import { workspaceState } from "@/state/workspace";
import { useSnapshot } from "valtio";
import { BlockUtil } from "@/utilities/block";

export function WorkspaceInfoPanel() {
  const { selectedBlock } = useSnapshot(workspaceState)

  return (
    <>
      {selectedBlock && (
        <Card className="fixed p-4 w-80 bottom-6 left-6 bg-background/75 backdrop-blur">
          <div className="space-y-2">
            <>
              <div className="text-[10px] font-light tracking-widest">
                SELECTED BLOCK
              </div>
              <div className="text-muted-foreground font-mono">
                <div>id: {selectedBlock.id}</div>
                <div>position: {JSON.stringify(selectedBlock.position)}</div>
                <div>color: {selectedBlock.color}</div>
                <div>overlapping: {BlockUtil.isOverlapping(selectedBlock).toString()}</div>
                <div>floating: {BlockUtil.isFloating(selectedBlock).toString()}</div>
                <div>valid position: {BlockUtil.isInValidPosition(selectedBlock).toString()}</div>
              </div>
            </>
          </div>
          
        </Card>
      )}
    </>
  )
}
