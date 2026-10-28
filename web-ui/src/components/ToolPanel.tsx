import { BroomSparklesIcon, DicesIcon } from "lucide-react";
import { COLOR } from "@/constants/color";
import { Card } from "./ui/card";
import { WorkspaceUtils } from "@/utilities/workspace";
import { AddBlockToolButton } from "./AddBlockToolButton";
import { ToolButton } from "./ToolButton";

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
