import { COLOR } from "@/constants/color";
import type { Block, Dimensions } from "@/types/common"
import { v4 as uuid } from 'uuid';
import { proxy } from "valtio"

export const MAX_BLOCKS_PER_COLOR = {
  [COLOR.RED]: 5,
  [COLOR.GREEN]: 5,
  [COLOR.BLUE]: 5
}

const DEFAULT_WORKSPACE_DIMENSIONS: Dimensions = {
  width: 6,
  height: 5,
  depth: 6,
}

const INITIAL_PLACED_BLOCKS = [
  { id: uuid(), position: { x: 0, y: 0, z: 0 }, color: COLOR.BLUE },
  { id: uuid(), position: { x: 1, y: 0, z: 0 }, color: COLOR.RED },
  { id: uuid(), position: { x: 0, y: 0, z: 1 }, color: COLOR.GREEN },
]

export interface WorkspaceState {
  dimensions: Dimensions
  placedBlocks: Block[]
  selectedBlock: Block | null
}

export const workspaceState = proxy<WorkspaceState>({
  dimensions: DEFAULT_WORKSPACE_DIMENSIONS,
  placedBlocks: INITIAL_PLACED_BLOCKS,
  selectedBlock: null,
})
