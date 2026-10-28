import React from 'react';
import { useSnapshot } from 'valtio';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Grid } from '@react-three/drei';
import { workspaceState } from '@/state/workspace';
import { DraggableBlock } from './DraggableBlock';
import * as THREE from 'three';
import { WorkspaceUtils } from '@/utilities/workspace';


export const GridSpace: React.FC = () => {
  const { placedBlocks, dimensions } = useSnapshot(workspaceState)
  const { width, height, depth } = dimensions;

  function handleCanvasClick(event: React.PointerEvent<HTMLDivElement>) {
    if (event.target === event.currentTarget) {
      WorkspaceUtils.setSelectedBlock(null);
    }
  }

  return (
    <Canvas
      camera={{ position: [width * 2, height * 2, depth * 2], fov: 50}}
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

      {placedBlocks.map((block) => (
        <DraggableBlock
          key={block.id}
          block={block}
        />
      ))}
    </Canvas>
  );
};
