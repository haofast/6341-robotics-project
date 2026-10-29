import { workspaceState } from "@/state/workspace";
import { BlockUtil } from "./block";
import type { Block, Position } from "@/types/common";
import { D, PositionUtil } from "./position";
import { v4 as uuid } from 'uuid';

export class WorkspaceUtils {

  public static clear() {
    workspaceState.placedBlocks = []
    workspaceState.selectedBlock = null
  }

  public static addBlock(block: Omit<Block, "id" | "position">) {
    const newBlock = { id: uuid(), position: WorkspaceUtils.getNextFreePosition(), ...block }
    workspaceState.placedBlocks.push(newBlock);
    workspaceState.selectedBlock = newBlock;
  }

  public static updateBlockPosition(block: Block, newPosition: Position) {
    workspaceState.placedBlocks.find((b) => b.id === block.id)!.position = newPosition;
  }

  public static setSelectedBlock(blockId: string | null) {
    workspaceState.selectedBlock = workspaceState.placedBlocks.find((b) => b.id === blockId) ?? null
  }

  public static scatterBlocks() {
    this.randomizeBlockPositions()
    this.applyGravityToBlocksPositions()
  }

  private static randomizeBlockPositions() {
    workspaceState.placedBlocks.forEach((block) => {
      block.position = this.getRandomFreePosition()
    })
  }

  private static applyGravityToBlocksPositions() {
    for (let height = 1; height < workspaceState.dimensions.height; height++) {
      this.getBlocksAtHeight(height).forEach((block) => BlockUtil.applyGravity(block));
    }
  }

  public static getNextFreePosition() {
    for (const position of this.getCoordinates()) {
      if (this.isPositionFree(position)) return position
    }

    throw new Error("no free positions left")
  }

  private static getRandomFreePosition() {
    for (let attempts = 0; attempts < 1000; attempts++) {
      const position = this.getRandomPosition();
      if (this.isPositionFree(position)) return position;
    }

    throw new Error("no free positions left");
  }

  private static getRandomPosition() {
    const { width, height, depth } = workspaceState.dimensions
    const lateral = Math.floor(Math.random() * width)
    const superior = Math.floor(Math.random() * height)
    const anterior = Math.floor(Math.random() * depth)
    return PositionUtil.dPosToPos({ lateral, superior, anterior })
  }

  private static isPositionFree(position: Position) {
    return !workspaceState.placedBlocks.some((block) => (
      PositionUtil.isSame(block.position, position)
    ))
  }

  private static getBlocksAtHeight(height: number) {
    return workspaceState.placedBlocks.filter((block) => (
      block.position[D.superior] === height
    ))
  }

  private static * getCoordinates() {
    const { width, height, depth } = workspaceState.dimensions

    for (let superior = 0; superior < height; superior++) {
      for (let anterior = 0; anterior < depth; anterior++) {
        for (let lateral = 0; lateral < width; lateral++) {
          yield PositionUtil.dPosToPos({ lateral, superior, anterior })
        }
      }
    }
  }
}
