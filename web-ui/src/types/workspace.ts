export type Dimensions = [
  width: number,
  height: number,
  depth: number,
];

export interface Position {
  x: number;
  y: number;
  z: number;
}

export interface Block {
  id: string;
  position: Position;
  color: string;
  label?: string;
}
