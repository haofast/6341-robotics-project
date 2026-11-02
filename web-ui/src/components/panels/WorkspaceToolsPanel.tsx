import { BroomSparklesIcon, DicesIcon, PlusIcon, ScanBoxIcon } from "lucide-react";
import { COLOR } from "@/constants/color";
import { Card } from "../ui/card";
import { WorkspaceUtils } from "@/utilities/workspace";
import { IconButton } from "../common/IconButton";
import { workspaceState, MAX_BLOCKS_PER_COLOR } from "@/state/workspace";
import { useSnapshot } from "valtio";

interface AddBlockButtonProps {
  iconColor: string,
  blockColor: string,
}

export function WorkspaceToolsPanel() {
  return (
    <Card className="fixed w-min flex flex-col gap-1 top-6 left-6 right-6 p-1 bg-background/75 backdrop-blur">
      <AddBlockButton iconColor="text-red-600" blockColor={COLOR.RED} />
      <AddBlockButton iconColor="text-green-600" blockColor={COLOR.GREEN} />
      <AddBlockButton iconColor="text-blue-600" blockColor={COLOR.BLUE} />
      <IconButton mainIcon={DicesIcon} action={() => WorkspaceUtils.scatterBlocks()} />
      <IconButton mainIcon={BroomSparklesIcon} action={() => WorkspaceUtils.clear()} />
    </Card>
  );
}

export function AddBlockButton({
  iconColor,
  blockColor,
}: AddBlockButtonProps) {
  const { placedBlocks } = useSnapshot(workspaceState);
  const numberOffPlacedBlocksOfColor = placedBlocks.filter((b) => b.color === blockColor).length;
  const numberOfBlocksOfColorRemaining = MAX_BLOCKS_PER_COLOR[blockColor] - numberOffPlacedBlocksOfColor;

  return (
    <IconButton
      mainIcon={ScanBoxIcon}
      accentIcon={PlusIcon}
      iconColor={iconColor}
      disabled={numberOfBlocksOfColorRemaining === 0}
      badgeCount={numberOfBlocksOfColorRemaining}
      action={() => WorkspaceUtils.addBlock({ color: blockColor })}
    />
  );
}
