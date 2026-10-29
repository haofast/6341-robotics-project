import React, { useRef, useEffect } from 'react';
import * as THREE from 'three';
import { TransformControls } from '@react-three/drei';
import type { TransformControls as TransformControlsImpl } from 'three-stdlib';
import type { Block, Position } from '../../types/common';
import { workspaceState } from '@/state/workspace';
import { useSnapshot } from 'valtio';
import { WorkspaceUtils } from '@/utilities/workspace';
import { PositionUtil } from '@/utilities/position';

interface DraggableBlockProps {
  block: Block;
}

export const GridBlock: React.FC<DraggableBlockProps> = ({ block }) => {
  const { dimensions, selectedBlock } = useSnapshot(workspaceState)
  const meshRef = useRef<THREE.Mesh>(null!);
  const transformRef = useRef<TransformControlsImpl>(null);

  function getAdjustedBlockPos(position: Position) {
    const positionVector = PositionUtil.posTo3DVec(position)
    positionVector[0] += 0.5
    positionVector[1] += 0.5
    positionVector[2] += 0.5
    return positionVector
  }

  function handleObjectChange() {
    if (meshRef.current) {
      const meshRefPosition = meshRef.current.position;
      const { width, height, depth } = dimensions;

      const lateral = Math.max(0, Math.min(width - 1, Math.round(meshRefPosition.x - 0.5)));
      const superior = Math.max(0, Math.min(height - 1, Math.round(meshRefPosition.y - 0.5)));
      const anterior = Math.max(0, Math.min(depth - 1, Math.round(meshRefPosition.z - 0.5)));

      const dPosition = { lateral, superior, anterior }
      const position = PositionUtil.dPosToPos(dPosition)

      WorkspaceUtils.updateBlockPosition(block, position)
      meshRefPosition.set(...getAdjustedBlockPos(position))
    }
  }

  useEffect(() => {
    meshRef.current?.position.set(...getAdjustedBlockPos(block.position));
  }, [block.position]);

  return (
    <>
      <mesh
        ref={meshRef}
        position={getAdjustedBlockPos(block.position)}
        onClick={() => WorkspaceUtils.setSelectedBlock(block.id)}
        castShadow
        receiveShadow
      >
        <boxGeometry args={[1, 1, 1]} />
        <meshStandardMaterial
          color={block.color}
          roughness={0.3}
          metalness={0.1}
        />
      </mesh>

      {block.id === selectedBlock?.id && (
        <TransformControls
          ref={transformRef}
          object={meshRef}
          mode="translate"
          translationSnap={1}
          onObjectChange={handleObjectChange}
        />
      )}
    </>
  );
};
