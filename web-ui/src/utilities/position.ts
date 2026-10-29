import type { DPosition, Position } from "@/types/common";

export const D = {
  lateral: "x",
  superior: "z",
  anterior: "y",
} as const

export const D_ = {
  x: "lateral",
  y: "anterior",
  z: "superior",
} as const

export class PositionUtil {

  public static dPosToPos(dPosition: DPosition): Position {
    return {
      x: dPosition[D_.x],
      y: dPosition[D_.y],
      z: dPosition[D_.z],
    }
  }

  public static posTo3DVec(p: Position): [number, number, number] {
    return [
      p[D.lateral],
      p[D.superior],
      p[D.anterior],
    ]
  }

  public static isSame(a: Position, b: Position) {
    return (
      a.x === b.x
      && a.y === b.y
      && a.z === b.z
    )
  }

  public static isOnTop(above: Position, below: Position) {
    return (
      above[D.lateral] === below[D.lateral]
      && above[D.anterior] === below[D.anterior]
      && above[D.superior] - 1 === below[D.superior]
    )
  }

  public static isOnFloor(p: Position) {
    return p[D.superior] === 0
  }
}
