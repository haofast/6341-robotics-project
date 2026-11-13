import type { ConsoleEntry } from "@/types/console";

interface ConsoleProps {
  entries: ConsoleEntry[]
}

export function Console({ entries }: ConsoleProps) {
  return (
    <div className="overflow-auto w-full h-50 p-2 font-mono text-[11px] bg-black text-white">
      {entries.map((entry, index) => (
        <div key={index}>
          {`> [${entry.timestamp.toLocaleTimeString()}] ${entry.text}`}
        </div>
      ))}
    </div>
  );
}
