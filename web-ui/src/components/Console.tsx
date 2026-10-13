interface ConsoleProps {
  logs: {
    type: "info" | "warning" | "error"
    timestamp: Date,
    text: string,
  }[]
}

export function Console({ logs }: ConsoleProps) {
  return (
    <div className="overflow-auto w-full h-50 p-2 font-mono text-[11px] bg-black text-white">
      {logs.map((log, index) => (
        <div key={index}>
          {`> [${log.timestamp.toLocaleTimeString()}] ${log.text}`}
        </div>
      ))}
    </div>
  )
}