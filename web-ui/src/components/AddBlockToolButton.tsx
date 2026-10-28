import { workspaceState, MAX_BLOCKS_PER_COLOR } from "@/state/workspace"
import { ScanBoxIcon, PlusIcon } from "lucide-react"
import { useSnapshot } from "valtio"
import { ToolButton } from "./ToolButton"
import { WorkspaceUtils } from "@/utilities/workspace"

interface AddBlockToolButtonProps {
  iconColor: string,
  blockColor: string,
}

export function AddBlockToolButton({
  iconColor,
  blockColor,
}: AddBlockToolButtonProps) {
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
