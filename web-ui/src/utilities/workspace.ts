import { workspaceSnap } from "@/state/workspace";
import type { Block, Position } from "@/types/workspace";

export function getRandomFreePosition() {
  const [width, height, depth] = workspaceSnap.dimensions;
  const maxAttempts = 1000; // Maximum number of attempts to find a free position
  let attempts = 0;

  while (attempts < maxAttempts) {
    const position: Position = {
      x: Math.floor(Math.random() * width),
      y: Math.floor(Math.random() * height),
      z: Math.floor(Math.random() * depth)
    };

    if (!isPositionOccupied(position)) {
      return position;
    }

    console.log("position retry")
    attempts++;
  }

  throw new Error("no free positions left");
}

export function findNextFreePosition() {
  const [width, height, depth] = workspaceSnap.dimensions

  for (let j = 0; j < height; j++) {
    for (let k = 0; k < depth; k++) {
      for (let i = 0; i < width; i++) {
        if (!isPositionOccupied({ x: i, y: j, z: k })) {
          return { x: i, y: j, z: k }
        }
      }
    }
  }

  throw new Error("no free positions left")
}

export function isPositionOccupied(position: Position) {
  return workspaceSnap.placedBlocks.some(b => (
    b.position.x === position.x
    && b.position.z === position.z
    && b.position.y === position.y
  ))
}

export function isBlockOverlapping(block: Block) {
  return workspaceSnap.placedBlocks.some(b => (
    b.id !== block.id
    && b.position.x === block.position.x
    && b.position.z === block.position.z
    && b.position.y === block.position.y
  ))
}

export function isBlockFloating(block: Block) {
  if (block.position.y === 0) return false;

  return !workspaceSnap.placedBlocks.some(b => (
    b.id !== block.id
    && b.position.x === block.position.x
    && b.position.z === block.position.z
    && b.position.y === block.position.y - 1
  ))
}

export function isBlockPositionValid(block: Block) {
  return !isBlockFloating(block) && !isBlockOverlapping(block)
}

