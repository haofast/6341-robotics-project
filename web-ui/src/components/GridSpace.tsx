import { Canvas } from '@react-three/fiber';
import { OrbitControls, Grid } from '@react-three/drei';

export interface GridSpaceProps {
  dimensions: GridDimensions;
  blocks: Block[];
}

export type GridDimensions = [
  width: number,
  height: number,
  depth: number,
];

export interface Block {
  id: string;
  x: number;
  y: number;
  z: number;
  color: string;
}

export function GridSpace({ dimensions, blocks }: GridSpaceProps) {
  const [maxX, maxY, maxZ] = dimensions;

  return (
    <Canvas camera={{ position: [15, 15, 15], fov: 50 }}>
      {/* Ambient and directional lighting */}
      <ambientLight intensity={0.7} />
      <directionalLight position={[10, 20, 10]} intensity={1} />

      {/* Camera Controls */}
      <OrbitControls makeDefault />

      {/* Floor Grid centered relative to dimensions */}
      <Grid 
        position={[maxX / 2 - 0.5, 0, maxZ / 2 - 0.5]} 
        args={[maxX, maxZ]} 
        cellSize={1} 
        cellColor="gray" 
        sectionColor="black" 
      />

      {/* Render Placed Cubes */}
      {blocks.map((block) => (
        <mesh 
          key={block.id} 
          position={[
            block.x + 0.5, 
            block.y + 0.5, 
            block.z + 0.5
          ]}
        >
          <boxGeometry args={[1, 1, 1]} />
          <meshStandardMaterial color={block.color} />
        </mesh>
      ))}
    </Canvas>
  );
};
