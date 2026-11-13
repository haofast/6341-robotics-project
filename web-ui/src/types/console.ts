export interface ConsoleState {
  entries: ConsoleEntry[]
}

export interface ConsoleEntry {
  type: "info" | "warning" | "error"
  timestamp: Date,
  text: string
}
