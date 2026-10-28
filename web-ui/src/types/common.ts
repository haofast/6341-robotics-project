export type Dimensions = {
  width: number,
  height: number,
  depth: number,
}

export interface Position {
  x: number;
  y: number;
  z: number;
}

export interface DPosition {
  lateral: number;
  superior: number;
  anterior: number;
}

export interface Block {
  id: string;
  position: Position;
  color: string;
  label?: string;
}
