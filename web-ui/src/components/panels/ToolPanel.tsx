import { BroomSparklesIcon, DicesIcon, PlusIcon, ScanBoxIcon } from "lucide-react";
import { COLOR } from "@/constants/color";
import { Card } from "../ui/card";
import { WorkspaceUtils } from "@/utilities/workspace";
import { ToolButton } from "../common/ToolButton";
import { workspaceState, MAX_BLOCKS_PER_COLOR } from "@/state/workspace";
import { useSnapshot } from "valtio";

interface AddBlockToolButtonProps {
  iconColor: string,
  blockColor: string,
}

export function ToolPanel() {
  return (
    <Card className="fixed w-min flex flex-col gap-1 top-6 left-6 right-6 p-1 bg-background/75 backdrop-blur">
      <AddBlockToolButton iconColor="text-red-600" blockColor={COLOR.RED} />
      <AddBlockToolButton iconColor="text-green-600" blockColor={COLOR.GREEN} />
      <AddBlockToolButton iconColor="text-blue-600" blockColor={COLOR.BLUE} />
      <ToolButton mainIcon={DicesIcon} action={() => WorkspaceUtils.scatterBlocks()} />
      <ToolButton mainIcon={BroomSparklesIcon} action={() => WorkspaceUtils.clear()} />
    </Card>
  )
}

export function AddBlockToolButton({ iconColor, blockColor }: AddBlockToolButtonProps) {
  const { placedBlocks } = useSnapshot(workspaceState)
  const numberOffPlacedBlocksOfColor = placedBlocks.filter((b) => b.color === blockColor).length
  const numberOfBlocksOfColorRemaining = MAX_BLOCKS_PER_COLOR[blockColor] - numberOffPlacedBlocksOfColor

  return (
    <ToolButton
      mainIcon={ScanBoxIcon}
      accentIcon={PlusIcon}
      iconColor={iconColor}
      disabled={numberOfBlocksOfColorRemaining === 0}
      badgeCount={numberOfBlocksOfColorRemaining}
      action={() => WorkspaceUtils.addBlock({ color: blockColor })}
    />
  )
}
