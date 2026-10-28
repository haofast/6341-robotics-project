import { BroomSparklesIcon, DicesIcon, PlusIcon, ScanBoxIcon } from "lucide-react";
import { MAX_BLOCKS_PER_COLOR, addBlock, resetWorkspace, workspaceSnap } from "@/state/workspace";
import { randomizeBlockPositions } from "@/utilities/workspace";
import { COLOR } from "@/constants/color";
import { Button } from "./ui/button";
import { Card } from "./ui/card";
import { useSnapshot } from "valtio";

export function ToolPanel() {
  return (
    <Card className="fixed w-min flex flex-col gap-1 top-6 left-6 right-6 p-1 bg-background/75 backdrop-blur">
      <AddBlockButton iconColor="text-red-600" blockColor={COLOR.RED} />
      <AddBlockButton iconColor="text-green-600" blockColor={COLOR.GREEN} />
      <AddBlockButton iconColor="text-blue-600" blockColor={COLOR.BLUE} />
      <Button className="relative p-5" variant="ghost" size="icon" onClick={randomizeBlockPositions}>
        <DicesIcon className="size-6 text-muted-foreground" />
      </Button>
      <Button className="relative p-5" variant="ghost" size="icon" onClick={resetWorkspace}>
        <BroomSparklesIcon className="size-6 text-muted-foreground" />
      </Button>
    </Card>
  )
}

function AddBlockButton({
  iconColor,
  blockColor,
}: {
  iconColor: string,
  blockColor: string,
}) {
  const { placedBlocks } = useSnapshot(workspaceSnap)
  const numberOffPlacedBlocksOfColor = placedBlocks.filter((b) => b.color === blockColor).length
  const numberOfBlocksOfColorRemaining = MAX_BLOCKS_PER_COLOR[blockColor] - numberOffPlacedBlocksOfColor

  return (
    <Button
      className="relative p-5"
      variant="ghost"
      size="icon"
      onClick={() => addBlock({ color: blockColor })}
      disabled={numberOfBlocksOfColorRemaining === 0}
    >
      <PlusIcon className="absolute top-0 right-0 text-muted-foreground" />
      <ScanBoxIcon className={`size-6 ${iconColor}`} />
      <div className="absolute bottom-1 left-1 text-muted-foreground font-bold">
        {numberOfBlocksOfColorRemaining}
      </div>
    </Button>
  )
}
