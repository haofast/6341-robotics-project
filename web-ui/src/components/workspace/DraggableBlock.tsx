import React, { useRef, useEffect } from 'react';
import * as THREE from 'three';
import { TransformControls } from '@react-three/drei';
import type { TransformControls as TransformControlsImpl } from 'three-stdlib';
import type { Block } from '../../types/workspace';
import { workspaceSnap, setSelectedBlock, updateBlockPosition } from '@/state/workspace';
import { useSnapshot } from 'valtio';

interface DraggableBlockProps {
  block: Block;
}

export const DraggableBlock: React.FC<DraggableBlockProps> = ({ block }) => {
  const { dimensions, selectedBlock } = useSnapshot(workspaceSnap)
  const meshRef = useRef<THREE.Mesh>(null!);
  const transformRef = useRef<TransformControlsImpl>(null);
  const [maxX, maxY, maxZ] = dimensions;

  function getBlockPosition(): [number, number, number] {
    return [
      block.position.x + 0.5,
      block.position.y + 0.5,
      block.position.z + 0.5,
    ]
  }

  function handleObjectChange() {
    if (meshRef.current) {
      const pos = meshRef.current.position;
      let logicalX = Math.max(0, Math.min(maxX - 1, Math.round(pos.x - 0.5)));
      let logicalY = Math.max(0, Math.min(maxY - 1, Math.round(pos.y - 0.5)));
      let logicalZ = Math.max(0, Math.min(maxZ - 1, Math.round(pos.z - 0.5)));
      pos.set(logicalX + 0.5, logicalY + 0.5, logicalZ + 0.5);
      updateBlockPosition(block.id, { x: logicalX, y: logicalY, z: logicalZ });
    }
  }

  useEffect(() => {
    meshRef.current?.position.set(...getBlockPosition());
  }, [block.position]);

  return (
    <>
      <mesh
        ref={meshRef}
        position={getBlockPosition()}
        onClick={() => setSelectedBlock(block.id)}
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
