import { workspaceState } from "@/state/workspace";
import type { Block } from "@/types/common";
import { D, PositionUtil } from "./position";
import { COLOR } from "@/constants/color";

export class BlockUtil {

  public static applyGravity(block: Block) {
    while (this.isFloating(block)) block.position[D.superior]--;
  }

  public static isOverlapping(block: Block) {
    return workspaceState.placedBlocks.some(b => (
      b.id !== block.id && PositionUtil.isSame(block.position, b.position)
    ));
  }
  
  public static isFloating(block: Block) {
    return !PositionUtil.isOnFloor(block.position) && !workspaceState.placedBlocks.some(b => (
      b.id !== block.id && PositionUtil.isOnTop(block.position, b.position)
    ));
  }
  
  public static isInValidPosition(block: Block) {
    return !this.isOverlapping(block) && !this.isFloating(block);
  }

  public static getColorString(block: Block) {
    for (const [colorName, colorCode] of Object.entries(COLOR)) {
      if (block.color === colorCode) return colorName.toLowerCase();
    }
    return "";
  }
}
