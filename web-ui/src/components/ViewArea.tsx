import { useState } from "react";
import { GridSpace } from "./GridSpace";
import type { Block, GridDimensions } from '../types/common';

export function ViewArea() {
  const dimensions: GridDimensions = [8, 5, 8];

  const [blocks, setBlocks] = useState<Block[]>([
    { id: "1", position: { x: 0, y: 0, z: 0 }, color: '#3b82f6' },
    { id: "2", position: { x: 1, y: 0, z: 0 }, color: '#ef4444' },
    { id: "3", position: { x: 0, y: 1, z: 0 }, color: '#10b981' },
  ]);

  // Handle the position update and update state
  function onBlockUpdate(id: string, newPos: { x: number; y: number; z: number }) {
    setBlocks((prevBlocks) =>
      prevBlocks.map((block) =>
        block.id === id ? { ...block, ...newPos } : block
      )
    );
  }

  return (
    <div className="w-full flex flex-row items-center justify-center">
      <GridSpace
        dimensions={dimensions}
        blocks={blocks}
        onBlockUpdate={onBlockUpdate}
      />
    </div>
  );
}