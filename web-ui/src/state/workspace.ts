import { COLOR } from "@/constants/color";
import type { Block, Dimensions, Position } from "@/types/workspace"
import { findNextFreePosition, getRandomFreePosition, isBlockFloating, isPositionOccupied } from "@/utilities/workspace";
import { v4 as uuid } from 'uuid';
import { proxy } from "valtio"

export const MAX_BLOCKS_PER_COLOR = {
  [COLOR.RED]: 5,
  [COLOR.GREEN]: 5,
  [COLOR.BLUE]: 5
}

const DEFAULT_WORKSPACE_DIMENSIONS: Dimensions = [
  6, // Width
  5, // Height
  6, // Depth
]

const INITIAL_PLACED_BLOCKS = [
  { id: uuid(), position: { x: 0, y: 0, z: 0 }, color: COLOR.BLUE },
  { id: uuid(), position: { x: 1, y: 0, z: 0 }, color: COLOR.RED },
  { id: uuid(), position: { x: 0, y: 0, z: 1 }, color: COLOR.GREEN },
]

export interface WorkspaceSnap {
  dimensions: Dimensions
  placedBlocks: Block[]
  selectedBlock: Block | null
}

export const workspaceSnap = proxy<WorkspaceSnap>({
  dimensions: DEFAULT_WORKSPACE_DIMENSIONS,
  placedBlocks: INITIAL_PLACED_BLOCKS,
  selectedBlock: null
})

export function resetWorkspace() {
  workspaceSnap.placedBlocks = []
  setSelectedBlock(null)
}

export function randomizeBlockPositions() {
  workspaceSnap.placedBlocks.forEach((b) => b.position = getRandomFreePosition())
  applyBlockGravity()
}

export function applyBlockGravity() {
  for (let i = 1; i < workspaceSnap.dimensions[1]; i++) {
    getBlocksAtHeight(i).forEach(block => {
      while (isBlockFloating(block)) block.position.y--;
    });
  }
}

export function getBlocksAtHeight(height: number) {
  return workspaceSnap.placedBlocks.filter((b) => b.position.y === height)
}

export function getBlock(blockId: string | null) {
  return workspaceSnap.placedBlocks.find((b) => b.id === blockId) ?? null
}

export function addBlock(block: Pick<Block, "color">) {
  const newBlock = { id: uuid(), position: findNextFreePosition(), ...block }
  workspaceSnap.placedBlocks.push(newBlock);
  setSelectedBlock(newBlock.id)
}

export function removeBlock(blockId: string) {
  workspaceSnap.placedBlocks = workspaceSnap.placedBlocks.filter((b) => b.id !== blockId);
  if (workspaceSnap.selectedBlock?.id === blockId) setSelectedBlock(null)
}

export function updateBlockPosition(blockId: string, position: Position) {
  getBlock(blockId)!.position = position
}

export function setSelectedBlock(blockId: string | null) {
  workspaceSnap.selectedBlock = getBlock(blockId)
}
