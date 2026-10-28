import { workspaceState } from "@/state/workspace";
import type { Block } from "@/types/common";
import { D, PositionUtil } from "./position";

export class BlockUtil {

  public static applyGravity(block: Block) {
    while (this.isFloating(block)) block.position[D.superior]--
  }

  public static isOverlapping(block: Block) {
    return workspaceState.placedBlocks.some(b => (
      b.id !== block.id && PositionUtil.isSame(block.position, b.position)
    ))
  }
  
  public static isFloating(block: Block) {
    return !PositionUtil.isOnFloor(block.position) && !workspaceState.placedBlocks.some(b => (
      b.id !== block.id && PositionUtil.isOnTop(block.position, b.position)
    ))
  }
  
  public static isInValidPosition(block: Block) {
    return this.isOverlapping(block) || this.isFloating(block)
  }
}
