import { isBlockOverlapping, isBlockFloating, isBlockPositionValid } from "@/utilities/workspace";
import { Card } from "./ui/card";
import { workspaceSnap } from "@/state/workspace";
import { useSnapshot } from "valtio";

export function WorkspacePanel() {
  const { selectedBlock } = useSnapshot(workspaceSnap)

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
                <div>overlapping: {isBlockOverlapping(selectedBlock).toString()}</div>
                <div>floating: {isBlockFloating(selectedBlock).toString()}</div>
                <div>valid position: {isBlockPositionValid(selectedBlock).toString()}</div>
              </div>
            </>
          </div>
          
        </Card>
      )}
    </>
  )
}