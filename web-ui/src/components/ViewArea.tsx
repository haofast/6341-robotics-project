import { useState } from "react";
import { GridSpace, type Block, type GridDimensions } from "./GridSpace";

export function ViewArea() {
  const dimensions: GridDimensions = [10, 5, 10];

  const [blocks] = useState<Block[]>([
    { id: "1", x: 0, y: 0, z: 0, color: '#3b82f6' },
    { id: "2", x: 1, y: 0, z: 0, color: '#ef4444' },
    { id: "3", x: 0, y: 1, z: 0, color: '#10b981' },
  ]);

  return (
    <div className="w-full flex flex-row items-center justify-center">
      <GridSpace dimensions={dimensions} blocks={blocks} />
    </div>
  )
}