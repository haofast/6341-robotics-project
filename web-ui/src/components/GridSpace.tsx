import React, { useState } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Grid } from '@react-three/drei';
import { DraggableBlock } from './DraggableBlock';
import type { Block, Position, GridDimensions } from '../types/common';
import * as THREE from 'three';

export interface GridSpaceProps {
  dimensions: GridDimensions;
  blocks: Block[];
  onBlockUpdate: (id: string, newPos: Position) => void;
  selectedBlockId?: string | null;
}

export const GridSpace: React.FC<GridSpaceProps> = ({
  dimensions,
  blocks,
  onBlockUpdate,
  selectedBlockId: externalSelectedId,
}) => {
  const [selectedBlock, setSelectedBlock] = useState<string | null>(null);
  const selectedId = externalSelectedId !== undefined ? externalSelectedId : selectedBlock;
  const [width, height, depth] = dimensions;

  function handleCanvasClick(event: React.PointerEvent<HTMLDivElement>) {
    if (event.target === event.currentTarget) {
      setSelectedBlock(null);
    }
  }

  return (
    <Canvas
      camera={{ position: [width * 1.5, height * 1.5, depth * 1.5], fov: 50 }}
      onPointerDown={handleCanvasClick}
      shadows
    >
      <ambientLight intensity={0.6} />
      <directionalLight
        position={[15, 20, 15]}
        intensity={1.2}
        castShadow
        shadow-mapSize-width={2048}
        shadow-mapSize-height={2048}
      />

      <OrbitControls
        makeDefault
        target={[width / 2, height / 2, depth / 2]}
      />

      <Grid
        position={[width / 2, -0.01, depth / 2]}
        args={[width, depth]}
        cellSize={1}
        cellThickness={1}
        cellColor="#4a4b50"
        sectionSize={5}
        sectionThickness={1.5}
        sectionColor="#8a8b90"
        fadeDistance={50}
      />

      <lineSegments position={[width / 2, height / 2, depth / 2]}>
        <edgesGeometry args={[new THREE.BoxGeometry(width, height, depth)]} />
        <lineBasicMaterial color="#ffffff" opacity={0.25} transparent />
      </lineSegments>

      {blocks.map((block) => (
        <DraggableBlock
          key={block.id}
          block={block}
          isSelected={selectedId === block.id}
          dimensions={dimensions}
          onSelect={setSelectedBlock}
          onUpdate={onBlockUpdate}
        />
      ))}
    </Canvas>
  );
};
