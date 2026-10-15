import { SidePanel } from "./components/SidePanel"
import { ViewArea } from "./components/ViewArea"

export function App() {
  return (
    <div className="flex min-h-svh max-h-svh p-6">
      <ViewArea />
      <SidePanel />
    </div>
  )
}

export default App
