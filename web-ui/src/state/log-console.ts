import type { ConsoleEntry, ConsoleState } from "@/types/console";
import { proxy } from "valtio";

export const logConsoleState = proxy<ConsoleState>({
  entries: [],
});

export function addLogConsoleEntry(entry: Omit<ConsoleEntry, "timestamp">) {
  logConsoleState.entries.push({ timestamp: new Date, ...entry });
}
