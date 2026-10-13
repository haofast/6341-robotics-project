import { SidePanel } from "./components/SidePanel"
import { SampleViewArea } from "./components/SampleViewArea"

export function App() {
  return (
    <div className="flex min-h-svh max-h-svh p-6">
      <SampleViewArea />
      <SidePanel />
    </div>
  )
}

export default App
