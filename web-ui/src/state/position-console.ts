import type { ConsoleEntry, ConsoleState } from "@/types/console";
import { proxy } from "valtio";

export const positionConsoleState = proxy<ConsoleState>({
  entries: [],
});

export function addPositionConsoleEntry(entry: Omit<ConsoleEntry, "timestamp">) {
  positionConsoleState.entries.push({ timestamp: new Date, ...entry });
}
